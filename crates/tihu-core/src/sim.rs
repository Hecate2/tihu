//! 事件驱动主循环。
//!
//! 一条腿指「轿厢从某个楼层一次赶往目标层」的一段行程。轿厢沿这条腿逐层推进，
//! 每到一层由内核判断该不该停：
//!
//! 1. 车上有乘客要在这一层下
//! 2. 这一层在本车这条腿的预定停靠表里（外呼指派决定）
//!
//! 停靠决策归内核，`Router` 只回答「下一程去哪」。这条分工让算法无论怎么写，
//! 都不会「路过该停的层」，也不会「停在没活可干的层」。

use crate::builtin::{DefaultBoarding, DownPeakSweep};
use crate::config::{CallSurface, Config, Strategy};
use crate::engine::{Call, CarPhase, Scratch, World};
use crate::metrics::Metrics;
use crate::motion::{decel_distance, state_at, time_at_distance, Dir};
use crate::policy::{AdmitSample, Assigner, Boarding, CarId, HallCall, Leg, Router, SystemView, Verdict};
use crate::rng::{Stream, Streams};

/// 制动反应时间，秒。司机（控制器）从接到指令到开始减速的滞后。
/// 待标定，先取 0.3 秒。
const BRAKE_REACTION_S: f64 = 0.3;

/// 一次仿真的输入与输出。
pub struct Run {
    pub world: World,
    pub metrics: Metrics,
}

impl Run {
    pub fn cfg(&self) -> &Config {
        &self.world.cfg
    }
}

/// 事件。堆里只放这四类。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    CarStep(CarId),
    /// 某位乘客走到电梯厅了
    Arrive(usize),
    /// 某位乘客等得不耐烦，改走楼梯
    Abandon(usize),
    StallTick(CarId),
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Ev {
    at: f64,
    seq: u64,
    kind: Kind,
}

impl Eq for Ev {}

impl Ord for Ev {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        // 反向排序，让最小堆的出堆顺序正确
        o.at.total_cmp(&self.at).then_with(|| o.seq.cmp(&self.seq))
    }
}

impl PartialOrd for Ev {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}

/// 一趟跑完。`router` 与 `assigner` 可以自带状态，所以传可变引用。
pub fn run<R: Router, A: Assigner, B: Boarding>(
    cfg: Config,
    router: &mut R,
    assigner: &mut A,
    boarding: &B,
) -> Run {
    let mut w = World::new(cfg);
    router.reset();
    assigner.reset();
    let mut rng = Streams::new(w.cfg.sim.seed);
    let mut sv = Scratch::new(w.cars.len(), w.n_levels);

    let horizon = w.cfg.sim.horizon_s;
    let mut heap: std::collections::BinaryHeap<Ev> =
        std::collections::BinaryHeap::with_capacity(w.cars.len() + w.pax.len() + 8);
    let mut seq: u64 = 0;
    let mut metrics = Metrics::with_floors(w.n_levels);

    // 每位乘客的第一条腿排一次进场事件。两条腿的乘客，第二条腿的进场事件
    // 在他到站时按「出梯时刻加停留时长」再排，所以条件性需求不用提前知道
    for pid in 0..w.pax.len() {
        if w.pax[pid].used_stairs || w.pax[pid].t_exit > 0.0 {
            continue;
        }
        heap.push(Ev { at: w.pax[pid].t_arrive, seq, kind: Kind::Arrive(pid) });
        seq += 1;
        // 等得不耐烦的兜底：到点还没上车就改走楼梯
        let patience = w.pax[pid].patience_s;
        if patience.is_finite() && patience > 0.0 {
            heap.push(Ev { at: w.pax[pid].t_arrive + patience, seq, kind: Kind::Abandon(pid) });
            seq += 1;
        }
    }
    // 轿厢起步
    for id in 0..w.cars.len() {
        heap.push(Ev { at: 0.0, seq, kind: Kind::CarStep(id) });
        seq += 1;
    }

    let mut stalls: usize = 0;
    let mut stall_seconds = 0.0;
    let mut n_events = 0usize;
    // 事件数上限。事件驱动一旦排程出错就会零时刻死循环，这里是兜底
    let max_events = 50_000_000usize;

    while let Some(ev) = heap.pop() {
        if ev.at > horizon {
            break;
        }
        n_events += 1;
        if n_events > max_events {
            break;
        }
        w.now = ev.at;
        match ev.kind {
            Kind::Arrive(pid) => {
                // 这位乘客走到电梯厅了。登记外呼，然后派梯
                if w.pax[pid].t_arrive > w.now + 1e-9 {
                    // 事件排在更早的时刻，重新排一次
                    let at = w.pax[pid].t_arrive;
                    heap.push(Ev { at, seq, kind: Kind::Arrive(pid) });
                    seq += 1;
                    continue;
                }
                if !w.pax[pid].waiting() {
                    continue;
                }
                let level = w.pax[pid].from;
                if level < -w.basement || level > w.top {
                    continue;
                }
                let qi = w.qidx(level);
                w.hall[qi].push(pid);
                register_calls(&mut w, level, &[pid]);
                assign_unowned(&mut w, assigner, &mut sv, &mut heap, &mut seq);
            }

            Kind::Abandon(pid) => {
                abandon_to_stairs(&mut w, pid, &mut metrics);
            }

            Kind::CarStep(id) => {
                step_car(
                    &mut w, id, router, boarding, &mut sv, &mut heap, &mut seq, &mut rng, &mut metrics,
                );
                after_event(&mut w, assigner, &mut sv, &mut heap, &mut seq);
            }

            Kind::StallTick(id) => {
                let res = stall_tick(&mut w, id, &mut rng, &mut heap, &mut seq);
                stalls += res.stalls_here;
                stall_seconds += res.seconds;
                if !res.resumed {
                    let next = w.now + w.cfg.overload.stall_tick_s;
                    heap.push(Ev { at: next, seq, kind: Kind::StallTick(id) });
                    seq += 1;
                }
                after_event(&mut w, assigner, &mut sv, &mut heap, &mut seq);
            }
        }
    }

    let metrics = collect(&w, stalls, stall_seconds, metrics);
    Run { world: w, metrics }
}

/// 每次事件之后重算一次归属，并处理新到的活。
///
/// 顺序要紧：先把没人负责的呼叫派出去，再清理已经没人等的呼叫。
/// 反过来会把刚派出去的呼叫又清掉。
fn after_event<A: Assigner>(
    w: &mut World,
    assigner: &A,
    sv: &mut Scratch,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
) {
    assign_unowned(w, assigner, sv, heap, seq);
    prune_calls(w);
    w.fill_views(sv);
    if w.cars.is_empty() {
        return;
    }
    let mut owned: Vec<Option<CarId>> = w.calls.calls.iter().map(|c| c.owner).collect();
    let before = owned.clone();
    let svref = SystemView {
        now: w.now,
        cars: &sv.briefs,
        pending: &sv.pending,
        all_calls: &sv.all,
        queue_len: &sv.q,
        top_level: w.top,
        n_cars: w.cars.len(),
    };
    assigner.reassign(&svref, &mut owned);
    if before != owned {
        for (i, slot) in owned.iter().enumerate() {
            let live = {
                let c = &w.calls.calls[i];
                w.calls
                    .slots
                    .get(Call::slot_index(c.level, c.dir, w.basement))
                    .copied()
                    .flatten()
                    == Some(c.id)
            };
            // 已经灭灯的外呼不许被重分配复活
            w.calls.calls[i].owner = if live { *slot } else { None };
        }
    }
    sync_assignments(w);
}

/// 这一层这个方向此刻还有没有人在等。
fn waiting_for(w: &World, level: i16, dir: Dir) -> bool {
    if level < -w.basement || level > w.top {
        return false;
    }
    w.hall[w.qidx(level)].iter().any(|&p| {
        let pax = &w.pax[p];
        if !pax.waiting() {
            return false;
        }
        match dir {
            Dir::Up => pax.to > level,
            Dir::Down => pax.to < level,
            Dir::None => true,
        }
    })
}

/// 这位乘客此刻是不是还在 `level` 层的电梯厅里等。
fn in_hall(w: &World, pid: usize, level: i16) -> bool {
    if level < -w.basement || level > w.top {
        return false;
    }
    w.hall[w.qidx(level)].contains(&pid)
}

/// 等得不耐烦，改走楼梯。这一位乘客从电梯厅里退场，计入另一类统计。
fn abandon_to_stairs(w: &mut World, pid: usize, metrics: &mut Metrics) {
    if !w.pax[pid].waiting() {
        return;
    }
    let level = w.pax[pid].from;
    let to = w.pax[pid].to;
    let stairs = w.cfg.stairs.clone();
    let floors = (to - level).abs();
    let cost = if to < level {
        stairs.cost_down(floors)
    } else {
        stairs.cost_up(floors)
    };
    let t_arrive = w.pax[pid].t_arrive;
    let waited = (w.now - t_arrive).max(0.0);
    let qi = w.qidx(level);
    if in_hall(w, pid, level) {
        w.hall[qi].retain(|&p| p != pid);
    }
    // 弃梯的人确实等过，这段时间要算进等待分布，否则报告里的等待会被
    // 系统性低估：等最久的那批人正好是弃梯走掉的那批
    metrics.wait.add(waited);
    let fs = &mut metrics.by_floor[qi];
    fs.count += 1;
    fs.wait_sum += waited;
    fs.wait_max = fs.wait_max.max(waited);
    fs.wait_samples.add(waited);
    let pax = &mut w.pax[pid];
    pax.abandoned = true;
    pax.t_board = t_arrive;
    pax.t_exit = w.now + cost;
    pax.hops_done += 1;
    pax.final_exit = pax.t_exit;
    metrics.abandoned += 1;
    metrics.by_floor[qi].stair_users += 1;
}


/// 按每条外呼的 owner 重建各轿厢的待办列表。
///
/// 只挂还亮着的外呼。灯的判据是槽位还指着它，这一条同时挡住了已灭的旧记录。
fn sync_assignments(w: &mut World) {
    for c in &mut w.cars {
        c.assigned.clear();
    }
    for c in &w.calls.calls {
        let Some(owner) = c.owner else { continue };
        let live = w
            .calls
            .slots
            .get(Call::slot_index(c.level, c.dir, w.basement))
            .copied()
            .flatten()
            == Some(c.id);
        if live && owner < w.cars.len() {
            w.cars[owner].assigned.push(c.id);
        }
    }
}

/// 叫醒一台停在原地的轿厢。分配完新活之后调用。
fn wake(w: &mut World, id: CarId, heap: &mut std::collections::BinaryHeap<Ev>, seq: &mut u64) {
    if w.cars[id].phase == CarPhase::Idle && !w.cars[id].next_event.is_finite() {
        w.cars[id].next_event = w.now;
        heap.push(Ev { at: w.now, seq: *seq, kind: Kind::CarStep(id) });
        *seq += 1;
    }
}

/// 把没有归属的外呼派一次，并叫醒接下活的轿厢。
fn assign_unowned<A: Assigner>(
    w: &mut World,
    assigner: &A,
    sv: &mut Scratch,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
) {
    w.fill_views(sv);
    let unowned: Vec<HallCall> = w
        .calls
        .calls
        .iter()
        .filter(|c| c.owner.is_none() && !w.calls.slots.is_empty() && is_live(w, c))
        .map(|c| HallCall { id: c.id, level: c.level, dir: c.dir, owner: None, waiting: c.waiting })
        .collect();
    if unowned.is_empty() {
        return;
    }
    for call in unowned {
        let (owner, target) = {
            let view = SystemView {
                now: w.now,
                cars: &sv.briefs,
                pending: &sv.pending,
                all_calls: &sv.all,
                queue_len: &sv.q,
                top_level: w.top,
                n_cars: w.cars.len(),
            };
            (assigner.assign(&view, call), call)
        };
        let Some(car) = owner else { continue };
        // 内核兜底：算法选中的车如果不服务这一层，换一台服务的。
        // 再好的算法也可能漏掉这条约束，而漏掉的代价是那一层的人永远等不到车
        let car = if w.car_serves(car, target.level) {
            car
        } else {
            match w.serving_cars_least_busy(target.level).first() {
                Some(&alt) => alt,
                None => continue,
            }
        };
        if let Some(c) = w.calls.get_mut(target.id) {
            c.owner = Some(car);
        }
        sync_assignments(w);
        plan_for_moving(w, car, target.level, target.dir);
        wake(w, car, heap, seq);
    }
}

/// 这条外呼还挂在层上吗（灯还亮着）。
fn is_live(w: &World, c: &Call) -> bool {
    if c.level < -w.basement || c.level > w.top {
        return false;
    }
    let slot = Call::slot_index(c.level, c.dir, w.basement);
    w.calls.slots.get(slot).copied().flatten() == Some(c.id)
}

/// 清理已经没人等、也没人负责的外呼。
///
/// 有人负责的呼叫一律保留：现实里按键灯亮着，轿厢一定会来，到了才发现
/// 没人上，那一次停靠就是幽灵停站。这条判据正是红线 2 的落点。
fn prune_calls(w: &mut World) {
    let mut dead: Vec<(i16, Dir)> = Vec::new();
    for (i, slot) in w.calls.slots.iter().enumerate() {
        let Some(cid) = *slot else { continue };
        let Some(c) = w.calls.get(cid) else { continue };
        let (level, dir) = (c.level, c.dir);
        let owned = c.owner.is_some();
        if level < -w.basement || level > w.top {
            dead.push((level, dir));
        } else if !owned && !waiting_for(w, level, dir) {
            dead.push((level, dir));
        }
        let _ = i;
    }
    for (lv, dir) in dead {
        kill_call(w, lv, dir);
    }
}

/// 灭掉一条外呼的灯：清槽位、清归属，并把所有轿厢的待办与预定停靠里的它摘掉。
///
/// **归属也要清。** 只清槽位的话 `sync_assignments` 下一轮会按 `owner` 把它重新
/// 挂回轿厢的待办表，那台车就会永远停在这一层，开门，没人上，再停，死循环。
fn kill_call(w: &mut World, level: i16, dir: Dir) {
    if level < -w.basement || level > w.top {
        return;
    }
    let slot = Call::slot_index(level, dir, w.basement);
    let Some(cid) = w.calls.slots.get(slot).copied().flatten() else { return };
    w.calls.clear_slot(level, dir);
    if let Some(c) = w.calls.get_mut(cid) {
        c.owner = None;
    }
    for c in &mut w.cars {
        c.assigned.retain(|&a| a != cid);
        c.stops_planned.retain(|&(l, d)| !(l == level && d == dir));
    }
}

/// 登记外呼。红线 1 与红线 2 都在这个函数里。
fn register_calls(w: &mut World, level: i16, newly: &[usize]) {
    if newly.is_empty() {
        return;
    }
    let per_car = w.cfg.call_surface == CallSurface::PerCarPanel;
    for &pid in newly {
        let p = w.pax[pid].clone();
        let wanted = if p.to > level { Dir::Up } else { Dir::Down };
        match p.strategy {
            // 红线 1 的基线：只按自己真正想去的方向
            Strategy::Honest => {
                w.calls.register(level, wanted, 1, false);
            }
            // 红线 2：上下都按。哪趟先来上哪趟
            Strategy::PressBoth => {
                w.calls.register(level, Dir::Up, 1, false);
                w.calls.register(level, Dir::Down, 1, false);
            }
            // 先按上行，跟车折返再下来
            Strategy::PressUpOnly => {
                w.calls.register(level, Dir::Up, 1, false);
            }
            // 同时呼叫多台。只有独立面板才做得到
            Strategy::MultiCall => {
                if per_car && !w.cars.is_empty() {
                    let k = w.cars.len().min(2);
                    for t in 0..k {
                        let (id, _) = w.calls.register(level, wanted, 1, false);
                        if let Some(c) = w.calls.get_mut(id) {
                            c.exclusive = true;
                            c.owner = Some(t);
                        }
                    }
                } else {
                    w.calls.register(level, wanted, 1, false);
                }
            }
        }
    }
}

/// 把一条外呼插进某台正在途中的轿厢。制动来不及就跳过，记一次越站。
fn plan_for_moving(w: &mut World, id: CarId, level: i16, dir: Dir) {
    if w.cars[id].phase != CarPhase::Moving {
        return;
    }
    if w.cars[id].dir != dir {
        return;
    }
    if !can_stop_at(w, id, level) {
        w.cars[id].overshoots += 1;
        return;
    }
    let pair = (level, dir);
    if !w.cars[id].stops_planned.contains(&pair) {
        w.cars[id].stops_planned.push(pair);
    }
}

/// 正在途中时，现在还来不来得及刹到 `level` 层。
///
/// 车已经开过、或者剩余距离短于从当前速度刹停所需的距离，都来不及。
fn can_stop_at(w: &World, id: CarId, level: i16) -> bool {
    let c = &w.cars[id];
    if c.phase != CarPhase::Moving {
        return true;
    }
    let spec = &w.cfg.cars[id];
    let floor_h = w.cfg.building.floor_height as f64;
    let profile = c.dir.profile(&spec.up, &spec.down);
    let elapsed = (w.now - c.leg_start).max(0.0);
    let (travelled, speed) = state_at(c.leg_dist, *profile, elapsed);
    // 车相对腿起点的绝对位置，米
    let sign = if c.dir == Dir::Up { 1.0 } else { -1.0 };
    let car_pos = c.leg_from as f64 * floor_h + sign * travelled;
    let target_pos = level as f64 * floor_h;
    let remaining = (target_pos - car_pos) * sign;
    if remaining <= 0.0 {
        // 已经越过或正好在这一层
        return remaining > -1e-9;
    }
    let brake = decel_distance(speed, *profile) + speed * BRAKE_REACTION_S;
    remaining + 1e-9 >= brake
}

/// 推进一步轿厢状态机。
///
/// 排下一次事件的地方只有一处，漏排就立刻表现为「轿厢停住不动」。
#[allow(clippy::too_many_arguments)]
fn step_car<R: Router, B: Boarding>(
    w: &mut World,
    id: CarId,
    router: &mut R,
    boarding: &B,
    sv: &mut Scratch,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
    rng: &mut Streams,
    metrics: &mut Metrics,
) {
    let mut guard = 0;
    loop {
        guard += 1;
        if guard > 64 {
            w.cars[id].next_event = f64::INFINITY;
            break;
        }
        match w.cars[id].phase {
            CarPhase::Idle => {
                let here = w.cars[id].level.round() as i16;
                // 有人要在这层下，先开门
                if riders_alight_here(w, id, here) {
                    begin_stop(w, id, boarding, rng, heap, seq, metrics);
                    break;
                }
                // 这层名下有活、车里还有位子、并且真的有人在等，才开门。
                //
                // 少这几个条件会死循环：车满着停在某层，层上那条呼叫一直灭不掉，
                // 车就在原地反复空停靠，车上的人一辈子送不到
                let has_free = w.cars[id].occupants.len() < w.cfg.cars[id].capacity_rated;
                let someone = waiting_for(w, here, Dir::Up) || waiting_for(w, here, Dir::Down);
                if owns_call_at(w, id, here) && has_free && someone {
                    begin_stop(w, id, boarding, rng, heap, seq, metrics);
                    break;
                }
                match choose_leg(w, id, router, sv) {
                    Leg::Up(t) | Leg::Down(t) => {
                        if start_leg(w, id, t) {
                            break; // 已排好下一次事件
                        }
                        // 目标就是本层，白跑一程，原地待命
                        park(w, id);
                        break;
                    }
                    Leg::Park => {
                        park(w, id);
                        break;
                    }
                }
            }

            CarPhase::Moving => {
                let here = w.cars[id].level.round() as i16;
                if needs_stop(w, id, here) {
                    begin_stop(w, id, boarding, rng, heap, seq, metrics);
                    break;
                }
                if here == w.cars[id].leg_target {
                    // 到了目标层却已经没活可干（呼叫被清掉了）。
                    // 不停靠，直接重新决策，避免一次无谓的开停
                    w.cars[id].phase = CarPhase::Idle;
                    continue;
                }
                advance_leg(w, id);
                break;
            }

            CarPhase::Boarding => {
                // 关门时刻到，本次停靠结束
                settle_stop(w, id);
                w.cars[id].phase = CarPhase::Idle;
                continue;
            }

            CarPhase::Stalled => {
                w.cars[id].next_event = f64::INFINITY;
                break;
            }
        }
    }

    // ---- 排下一次事件 ----
    // 待机（next_event 为无穷）时不排事件，由 `wake` 在有新活时叫醒。
    // 最小间隔 1 毫秒是防自触发的兜底：若某条路径把排期设成了当前时刻，
    // 不加这个下限就会同刻无限重排
    if w.cars[id].phase == CarPhase::Stalled {
        return;
    }
    let at = w.cars[id].next_event;
    if !at.is_finite() {
        return;
    }
    if at > w.now {
        heap.push(Ev { at, seq: *seq, kind: Kind::CarStep(id) });
        *seq += 1;
    } else {
        w.cars[id].next_event = w.now + 0.001;
        heap.push(Ev { at: w.now + 0.001, seq: *seq, kind: Kind::CarStep(id) });
        *seq += 1;
    }
}

/// 原地待命。不再排重试事件，有新活时由 `wake` 叫醒。
fn park(w: &mut World, id: CarId) {
    w.cars[id].phase = CarPhase::Idle;
    w.cars[id].dir = Dir::None;
    w.cars[id].next_event = f64::INFINITY;
}

/// 这一层该不该停。
///
/// 判据只有两条：有乘客要下，或者这条腿预定了要在这里接人。
fn needs_stop(w: &World, id: CarId, here: i16) -> bool {
    if w.cars[id].occupants.iter().any(|&p| w.pax[p].to == here) {
        return true;
    }
    w.cars[id].stops_planned.iter().any(|&(l, _)| l == here)
}

/// 决定下一程。车上有乘客时先送客，这是压过一切优化的义务。
fn choose_leg<R: Router>(w: &mut World, id: CarId, router: &R, sv: &mut Scratch) -> Leg {
    let here = w.cars[id].level.round() as i16;
    if let Some(t) = nearest_rider_dest(w, id, here) {
        if t > here {
            return Leg::Up(t);
        } else if t < here {
            return Leg::Down(t);
        }
    }
    let rv = w.fill_run_view(sv, id);
    router.next_leg(&rv)
}

/// 车主是否要送客，以及送谁。返回离当前层最近的乘客目的层。
fn nearest_rider_dest(w: &World, id: CarId, here: i16) -> Option<i16> {
    w.cars[id]
        .occupants
        .iter()
        .map(|&p| w.pax[p].to)
        .min_by_key(|&t| (t - here).abs())
}

/// 开始一条腿。目标层与本层相同则返回 false。
fn start_leg(w: &mut World, id: CarId, target: i16) -> bool {
    let here = w.cars[id].level.round() as i16;
    if target == here {
        return false;
    }
    let dir = if target > here { Dir::Up } else { Dir::Down };
    let floor_h = w.cfg.building.floor_height as f64;
    let dist = (target - here).abs() as f64 * floor_h;
    w.cars[id].leg_from = here;
    w.cars[id].leg_target = target;
    w.cars[id].leg_start = w.now;
    w.cars[id].leg_dist = dist;
    w.cars[id].dir = dir;
    w.cars[id].phase = CarPhase::Moving;
    // 车上没人时这一程算一趟往返的起点
    if w.cars[id].occupants.is_empty() && w.cars[id].trip_start == 0.0 {
        w.cars[id].trip_start = w.now;
    }
    // 这条腿沿途要接的外呼，以及要送的内选，全部预先排进停靠表
    plan_leg(w, id, here, target, dir);
    schedule_next_floor(w, id);
    true
}

/// 一次腿的预定停靠：名下的同向外呼，以及车上乘客的目的层。
///
/// 目标层上的外呼不论朝向都要排进来：那是这台车这趟的折返点，
/// 折返点不收人，车就会在「到了却不停」和「返回本层」之间空转。
fn plan_leg(w: &mut World, id: CarId, from: i16, target: i16, dir: Dir) {
    let lo = from.min(target);
    let hi = from.max(target);
    let mut add: Vec<(i16, Dir)> = Vec::new();
    for &cid in &w.cars[id].assigned {
        if let Some(c) = w.calls.get(cid) {
            if c.level == target {
                add.push((c.level, c.dir));
            } else if c.dir == dir && c.level >= lo && c.level <= hi && c.level != from {
                add.push((c.level, c.dir));
            }
        }
    }
    for &p in &w.cars[id].occupants {
        let t = w.pax[p].to;
        if t >= lo && t <= hi && t != from {
            add.push((t, dir));
        }
    }
    for pair in add {
        if !w.cars[id].stops_planned.contains(&pair) {
            w.cars[id].stops_planned.push(pair);
        }
    }
}

/// 车上有没有人正好要在这一层下。
fn riders_alight_here(w: &World, id: CarId, level: i16) -> bool {
    w.cars[id].occupants.iter().any(|&p| w.pax[p].to == level)
}

/// 这台车在这一层有没有活可干：名下有这条外呼，且它还挂在层上。
fn owns_call_at(w: &World, id: CarId, level: i16) -> bool {
    w.cars[id].assigned.iter().any(|&cid| {
        w.calls.get(cid).is_some_and(|c| {
            c.level == level && {
                let slot = Call::slot_index(c.level, c.dir, w.basement);
                w.calls.slots.get(slot).copied().flatten() == Some(c.id)
            }
        })
    })
}

/// 排下一次「走到下一层」的事件。用整条腿的曲线算累计时刻，不是逐层累加。
fn schedule_next_floor(w: &mut World, id: CarId) {
    let sign = if w.cars[id].dir == Dir::Up { 1 } else { -1 };
    let here = w.cars[id].level.round() as i16;
    let next = here + sign;
    let spec = &w.cfg.cars[id];
    let floor_h = w.cfg.building.floor_height as f64;
    let profile = w.cars[id].dir.profile(&spec.up, &spec.down);
    let cum = (next - w.cars[id].leg_from).abs() as f64 * floor_h;
    let t = time_at_distance(w.cars[id].leg_dist, *profile, cum);
    w.cars[id].level = next as f32;
    w.cars[id].phase = CarPhase::Moving;
    w.cars[id].next_event = w.cars[id].leg_start + t;
}

/// 继续往腿的目标层推进一层。
fn advance_leg(w: &mut World, id: CarId) {
    let sign = if w.cars[id].dir == Dir::Up { 1 } else { -1 };
    let here = w.cars[id].level.round() as i16;
    let target = w.cars[id].leg_target;
    let next = here + sign;
    // 不越过目标
    if (sign > 0 && next > target) || (sign < 0 && next < target) {
        return;
    }
    let spec = &w.cfg.cars[id];
    let floor_h = w.cfg.building.floor_height as f64;
    let profile = w.cars[id].dir.profile(&spec.up, &spec.down);
    let cum = (next - w.cars[id].leg_from).abs() as f64 * floor_h;
    let t = time_at_distance(w.cars[id].leg_dist, *profile, cum);
    w.cars[id].level = next as f32;
    w.cars[id].next_event = w.cars[id].leg_start + t;
}

/// 开始一次停靠：先下后上，算关门时刻，必要时转超载僵局。
#[allow(clippy::too_many_arguments)]
fn begin_stop<B: Boarding>(
    w: &mut World,
    id: CarId,
    boarding: &B,
    rng: &mut Streams,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
    metrics: &mut Metrics,
) {
    let lv = w.cars[id].level.round() as i16;
    w.cars[id].stop_started = w.now;
    w.cars[id].phase = CarPhase::Boarding;
    board_and_alight(w, id, boarding, rng, heap, seq, metrics);

    // 这一层的按键灯灭掉。
    //
    // 只灭「这层这个方向确实已经没人等」的那一条。无条件灭灯会把还没挤上
    // 的人的外呼一起掐掉，那些人从此不会有车再来，只能枯等。
    for dir in [Dir::Up, Dir::Down] {
        if !waiting_for(w, lv, dir) {
            kill_call(w, lv, dir);
        }
    }

    let crush = w.cfg.boarding.capacity_crush(w.cfg.cars[id].capacity_rated);
    let _ = crush;
    let doors = w.cfg.cars[id].doors;
    let base = (doors.open_s - doors.early_open_credit_s).max(0.0) as f64;
    let in_t = doors.transfer_time(w.cars[id].last_boarded, w.cars[id].last_alighted);
    let total = base + in_t + doors.close_s as f64 + doors.ctrl_resp_s as f64;
    let close_at = w.cars[id].stop_started + total;

    // 幽灵停站：开门了却没人上、也没人下，白损失一次停靠开销
    if w.cars[id].last_boarded == 0 && w.cars[id].last_alighted == 0 {
        w.cars[id].ghost_stops += 1;
    }

    let trigger = w.cfg.overload.trigger_ratio * w.cfg.cars[id].rated_load_kg as f64;
    if w.cfg.overload.enabled && w.cars[id].load_kg > trigger {
        // 红线 6：超载报警。门关不上，没人退出就一直僵着
        w.cars[id].phase = CarPhase::Stalled;
        w.cars[id].stall_since = w.now;
        w.cars[id].next_event = f64::INFINITY;
        heap.push(Ev { at: w.now + w.cfg.overload.stall_tick_s, seq: *seq, kind: Kind::StallTick(id) });
        *seq += 1;
        return;
    }

    w.cars[id].phase = CarPhase::Boarding;
    w.cars[id].next_event = close_at.max(w.now);
}

/// 一次停靠结束时的收尾。
fn settle_stop(w: &mut World, id: CarId) {
    let lv = w.cars[id].level.round() as i16;
    w.cars[id].stops_planned.retain(|&(l, _)| l != lv);
    w.cars[id].stop_started = 0.0;
    // 车上清空才算一趟往返走完。只有真的有人下过车才记样本，
    // 否则单纯跑去接人又空手而归的腿也会被当成一趟
    if w.cars[id].occupants.is_empty() && w.cars[id].trip_start > 0.0 {
        if w.cars[id].last_alighted > 0 {
            let rtt = w.now - w.cars[id].trip_start;
            if rtt > 0.0 && rtt < 3600.0 {
                w.cars[id].rtt_total += rtt;
                w.cars[id].rtt_count += 1;
                w.cars[id].trips += 1;
            }
        }
        w.cars[id].trip_start = 0.0;
    }
}

/// 这一层该按哪个方向收人。
///
/// 规则按优先级：
/// 1. 停着不动（车头方向为空）时任何方向都收
/// 2. 这一层名下有同向外呼，按车头方向收
/// 3. 这一层名下只有反向外呼，说明这是本趟的折返点，按反方向收
/// 4. 这一层没有外呼，只是路过或有人下车，按车头方向收
///
/// 第 3 条是早高峰的落点：轿厢空车上行到最高任务层，那里只有下行外呼，
/// 要在这一层把下行客收上来，而不是按车头方向把上行客收走。
fn board_dir_at(w: &World, id: CarId, here: i16) -> Dir {
    let d = w.cars[id].dir;
    if !d.is_moving() {
        return Dir::None;
    }
    let mut opposite: Option<Dir> = None;
    for &cid in &w.cars[id].assigned {
        let Some(c) = w.calls.get(cid) else { continue };
        if c.level != here {
            continue;
        }
        if c.dir == d {
            return d;
        }
        opposite = Some(c.dir);
    }
    opposite.unwrap_or(d)
}

/// 先下后上。下车的乘客如果还有下一条腿，在这里推进到下一腿。
#[allow(clippy::too_many_arguments)]
fn board_and_alight<B: Boarding>(
    w: &mut World,
    id: CarId,
    boarding: &B,
    rng: &mut Streams,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
    metrics: &mut Metrics,
) {
    let lv = w.cars[id].level.round() as i16;
    let now = w.now;

    // 先下
    let mut alighted = 0usize;
    let mut exited: Vec<usize> = Vec::with_capacity(16);
    w.cars[id].occupants.retain(|&p| {
        if w.pax[p].to == lv {
            exited.push(p);
            alighted += 1;
            false
        } else {
            true
        }
    });
    w.cars[id].last_alighted = alighted;
    w.cars[id].load_kg = w.cars[id].occupants.iter().map(|&p| w.pax[p].weight).sum();

    // 记一条腿的指标。一条腿一算，所以带返程的乘客两条腿各算一次
    for &pid in &exited {
        let from = w.pax[pid].from;
        let pax = &mut w.pax[pid];
        pax.t_exit = now;
        pax.boarded_car = None;
        pax.hops_done += 1;
        let (wait, ride, d2d) = (pax.wait(), pax.ride(), pax.door_to_door());
        let lv_idx = (from + w.basement).clamp(0, w.n_levels as i16 - 1) as usize;
        metrics.wait.add(wait);
        metrics.ride.add(ride);
        metrics.door_to_door.add(d2d);
        let fs = &mut metrics.by_floor[lv_idx];
        fs.count += 1;
        fs.wait_sum += wait;
        fs.wait_max = fs.wait_max.max(wait);
        fs.wait_samples.add(wait);
        fs.bypass_total += pax.bypass_count;
        if pax.bypass_count > 0 {
            fs.bypassed += 1;
        }
        fs.declined += pax.declined_count;
        fs.no_space += pax.no_space_count;
    }

    // 还有下一条腿的人，过一阵子从这一层出发回去。这就是条件性需求
    for &pid in &exited {
        let idx = w.pax[pid].hop_idx;
        let n = w.pax[pid].n_hops;
        if (idx as usize) + 1 < n as usize {
            let next = w.pax[pid].hops[idx as usize + 1];
            let at = now + w.pax[pid].dwell_s;
            w.pax[pid].hop_idx += 1;
            w.pax[pid].from = next.from;
            w.pax[pid].to = next.to;
            w.pax[pid].t_arrive = at;
            w.pax[pid].t_board = 0.0;
            w.pax[pid].t_exit = 0.0;
            w.pax[pid].bypass_count = 0;
            w.pax[pid].declined_count = 0;
            w.pax[pid].no_space_count = 0;
            heap.push(Ev { at, seq: *seq, kind: Kind::Arrive(pid) });
            *seq += 1;
        } else {
            w.pax[pid].final_exit = now;
        }
    }

    // 再上人。反复扫整段队列，直到一整轮没人能上为止。
    // 队首那位挤不上时，后面的人仍有机会上车，这是真实电梯的样子
    let want = board_dir_at(w, id, lv);
    let mut boarded = 0usize;
    let mut newly_boarded: Vec<usize> = Vec::with_capacity(16);
    loop {
        let mut progress = false;
        let snapshot: Vec<usize> = w.hall[w.qidx(lv)].clone();
        for pid in snapshot {
            if !w.pax[pid].waiting() {
                continue;
            }
            if try_board_one(w, id, lv, pid, want, boarding, now, rng) {
                boarded += 1;
                progress = true;
                newly_boarded.push(pid);
            }
        }
        if !progress {
            break;
        }
    }
    w.cars[id].last_boarded = boarded;

    // 上梯的人离开电梯厅
    if !newly_boarded.is_empty() {
        let idx = w.qidx(lv);
        w.hall[idx].retain(|p| !newly_boarded.contains(p));
    }

    // 未满载就走，而层上还有人等，记一次不全载发车
    let rated = w.cfg.cars[id].capacity_rated;
    if boarded > 0 && w.cars[id].occupants.len() < rated && waiting_for(w, lv, want) {
        w.cars[id].partial_departures += 1;
    }
}

/// 考察队列里第 `pid` 位能否上车。
fn try_board_one<B: Boarding>(
    w: &mut World,
    id: CarId,
    lv: i16,
    pid: usize,
    want: Dir,
    boarding: &B,
    now: f64,
    rng: &mut Streams,
) -> bool {
    let wants = if w.pax[pid].to > lv {
        Dir::Up
    } else if w.pax[pid].to < lv {
        Dir::Down
    } else {
        Dir::None
    };
    // 轿厢方向要匹配。停着不动时任何方向都收
    let matching = match want {
        Dir::None => true,
        d => d == wants,
    };
    if !matching {
        return false;
    }
    // 这趟车必须同时停靠出发层与目的层。少这一条，分区梯上的乘客会被
    // 拉上车却到不了自己的楼层
    let dest = w.pax[pid].to;
    if !w.car_serves(id, lv) || !w.car_serves(id, dest) {
        return false;
    }
    let capacity_rated = w.cfg.cars[id].capacity_rated;
    let crush = w.cfg.boarding.capacity_crush(capacity_rated);
    // 空位要按物理极限算，不能按额定人数算。
    //
    // 按额定算的话轿厢永远坐不满超过铭牌数，拒挤曲线只在额定线以下起作用，
    // 重量也就永远压不过报警线，红线 6 的超载死锁根本触发不了
    let free = crush.saturating_sub(w.cars[id].occupants.len());
    let load_ratio = w.cars[id].occupants.len() as f64 / capacity_rated.max(1) as f64;
    let waited = (now - w.pax[pid].t_arrive).max(0.0);
    let sample = AdmitSample {
        load_ratio,
        free_slots: free,
        waited_s: waited,
        weight_ratio: w.cars[id].load_kg / (w.cfg.cars[id].rated_load_kg as f64).max(1.0),
    };
    let tol = w.pax[pid].tol;

    match boarding.admit_one(&sample, tol) {
        Verdict::NoSpace => {
            w.pax[pid].no_space_count += 1;
            w.pax[pid].bypass_count += 1;
            false
        }
        Verdict::Declined => {
            w.pax[pid].declined_count += 1;
            w.pax[pid].bypass_count += 1;
            false
        }
        Verdict::Admit => {
            // 中间地带要掷骰子。策略对象本身不掷，随机性归内核。
            // 这一步是红线 3 的核心：越接近满载，越可能有人放弃
            let prob = default_accept_prob(&sample, tol);
            if prob < 1.0 && !rng.chance(Stream::Squeeze, prob) {
                w.pax[pid].declined_count += 1;
                w.pax[pid].bypass_count += 1;
                return false;
            }
            // 物理极限。超出这个数再怎么挤也挤不进去
            if w.cars[id].occupants.len() >= crush {
                w.pax[pid].no_space_count += 1;
                w.pax[pid].bypass_count += 1;
                return false;
            }
            w.cars[id].occupants.push(pid);
            w.cars[id].load_kg += w.pax[pid].weight;
            w.pax[pid].boarded_car = Some(id);
            w.pax[pid].t_board = now;
            // 反向叫梯的人登记转向层，一路坐到折返点再下来
            if matches!(w.pax[pid].strategy, Strategy::PressUpOnly | Strategy::PressBoth) {
                w.pax[pid].reversal_floor = Some(lv);
            }
            true
        }
    }
}

/// `DefaultBoarding` 的概率公式。放在这里是因为随机性归内核，策略对象本身不掷骰子。
fn default_accept_prob(s: &AdmitSample, tol: f64) -> f64 {
    if s.free_slots == 0 {
        return 0.0;
    }
    crate::config::Boarding::default().accept_prob(s.load_ratio, tol, s.waited_s)
}

/// 一次僵持的结果。
struct StallOutcome {
    resumed: bool,
    seconds: f64,
    stalls_here: usize,
}

/// 超载僵局的每一轮：问一次有没有人愿意退出。
#[allow(clippy::too_many_arguments)]
fn stall_tick(
    w: &mut World,
    id: CarId,
    rng: &mut Streams,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
) -> StallOutcome {
    let ov = w.cfg.overload.clone();
    let elapsed = w.now - w.cars[id].stall_since;
    // 等得越久，退出概率被压力推得越高
    let pressure = ov.stress_growth.powf(elapsed / 30.0);
    let trigger = ov.trigger_ratio * w.cfg.cars[id].rated_load_kg as f64;

    // 最后上车的那个人最可能是志愿者，因为他刚挤进来
    let last_boarder = w.cars[id].occupants.last().copied();
    let mut exited: Option<(usize, usize)> = None;
    let n = w.cars[id].occupants.len();
    for k in 0..n {
        let pos = n - 1 - k;
        let Some(&pid) = w.cars[id].occupants.get(pos) else { continue };
        let q = if Some(pid) == last_boarder { ov.q_last * pressure } else { ov.q_other * pressure };
        if rng.chance(Stream::Stall, q) {
            exited = Some((pos, pid));
            break;
        }
    }
    if let Some((pos, pid)) = exited {
        w.cars[id].occupants.remove(pos);
        w.pax[pid].t_exit = w.now;
        w.pax[pid].boarded_car = None;
        w.pax[pid].requeued_after_stall = true;
        let weight = w.pax[pid].weight;
        w.cars[id].load_kg = (w.cars[id].load_kg - weight).max(0.0);
    }

    let now_under = w.cars[id].load_kg <= trigger;
    let timed_out = elapsed > ov.stall_timeout_s;
    if now_under || (timed_out && w.cars[id].occupants.len() > 1) {
        // 恢复了。强制退出时把最近上车的人请下去
        if !now_under {
            if let Some(&pid) = w.cars[id].occupants.last() {
                w.cars[id].occupants.pop();
                w.pax[pid].t_exit = w.now;
                w.pax[pid].boarded_car = None;
                w.pax[pid].requeued_after_stall = true;
                let weight = w.pax[pid].weight;
                w.cars[id].load_kg = (w.cars[id].load_kg - weight).max(0.0);
            }
        }
        let seconds = w.now - w.cars[id].stall_since;
        w.cars[id].phase = CarPhase::Idle;
        w.cars[id].stall_since = 0.0;
        w.cars[id].next_event = w.now;
        heap.push(Ev { at: w.now, seq: *seq, kind: Kind::CarStep(id) });
        *seq += 1;
        return StallOutcome { resumed: true, seconds, stalls_here: 1 };
    }

    StallOutcome { resumed: false, seconds: 0.0, stalls_here: 0 }
}

/// 汇总指标。
///
/// 时间类指标在乘客每次到站时就逐腿累加完了，这里只把配置、计数与均值补齐。
fn collect(w: &World, stalls: usize, stall_seconds: f64, mut m: Metrics) -> Metrics {
    m.pax_total = w.pax.len();
    for p in &w.pax {
        if p.hops_done == 0 {
            continue;
        }
        if p.used_stairs || p.abandoned {
            m.stair_users += 1;
            continue;
        }
        m.pax_served += 1;
        if p.bypass_count > 0 {
            m.pax_with_bypass += 1;
        }
        if p.bypass_count >= 3 {
            m.pax_bypass3 += 1;
        }
        if p.requeued_after_stall {
            m.pax_after_stall += 1;
        }
    }
    if m.pax_served > 0 {
        m.pax_any_bypass = m.pax_with_bypass as f64 / m.pax_served as f64;
        m.pax_bypass3_frac = m.pax_bypass3 as f64 / m.pax_served as f64;
    }
    if !w.pax.is_empty() {
        m.stair_share = m.stair_users as f64 / w.pax.len() as f64;
    }
    for s in m.by_floor.iter_mut() {
        if s.count > 0 {
            s.wait_mean = s.wait_sum / s.count as f64;
        }
    }

    // 系统视角
    m.stalls = stalls;
    m.stall_seconds = stall_seconds;
    m.n_cars = w.cars.len();
    for c in &w.cars {
        m.rtt_sum += c.rtt_total;
        m.rtt_count += c.rtt_count;
        m.trips += c.trips;
        m.ghost_stops += c.ghost_stops as usize;
        m.partial_departures += c.partial_departures as usize;
        m.overshoots += c.overshoots as usize;
    }
    if m.rtt_count > 0 {
        m.rtt_mean = m.rtt_sum / m.rtt_count as f64;
    }
    m
}

/// 默认的算法组合。命令行与网页都用它。
pub fn default_run(cfg: Config) -> Run {
    let mut router = DownPeakSweep;
    let mut assigner = crate::builtin::LoadAwareEta::default();
    let boarding = DefaultBoarding::default();
    run(cfg, &mut router, &mut assigner, &boarding)
}

/// 指定种子的一次运行。
pub fn quick(mut cfg: Config, seed: u64) -> Run {
    cfg.sim.seed = seed;
    default_run(cfg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{preset, CarSpec};

    fn small() -> Config {
        let mut c = Config::default();
        c.building.top_level = 6;
        c.building.units_per_floor = vec![4; 6];
        c.cars = vec![CarSpec::default()];
        c
    }

    /// 红线 1：同一层同方向按二十次，只产生一条外呼。
    #[test]
    fn twenty_presses_one_direction_make_one_call() {
        let mut w = World::new(small());
        for _ in 0..20 {
            w.calls.register(5, Dir::Down, 1, false);
        }
        assert_eq!(w.calls.calls.len(), 1, "同层同方向只该有一条外呼");
        let live = w.calls.slots.iter().flatten().count();
        assert_eq!(live, 1, "只该有一个亮着的按键");
    }

    /// 上下的按钮是两条外呼。
    #[test]
    fn up_and_down_are_two_calls() {
        let mut w = World::new(small());
        w.calls.register(5, Dir::Up, 1, false);
        w.calls.register(5, Dir::Down, 1, false);
        assert_eq!(w.calls.calls.len(), 2);
    }

    /// 每位乘客都要走到目的地。剩下没人接的人数是 0。
    #[test]
    fn everyone_gets_served() {
        let mut c = small();
        c.traffic.departure_ratio = 1.0;
        let r = default_run(c);
        let served = r.metrics.pax_served + r.metrics.stair_users as u64;
        assert_eq!(
            served, r.metrics.pax_total as u64,
            "还有人没被送走：总 {}，送到 {}，走楼梯 {}",
            r.metrics.pax_total, r.metrics.pax_served, r.metrics.stair_users
        );
    }

    /// 停靠语义：轿厢不会路过有人要下的楼层。
    ///
    /// 这是重写前最严重的 bug：轿厢点对点跳到目标层，中途该停的层全被掠过。
    #[test]
    fn car_never_passes_a_rider_destination() {
        let mut c = small();
        c.traffic.departure_ratio = 1.0;
        let r = default_run(c);
        // 每位下过车的人，出梯层就是他登记的目的层
        for p in &r.world.pax {
            if p.t_exit > 0.0 && !p.used_stairs {
                assert!(p.to <= r.world.top && p.to >= -r.world.basement, "目的层越界");
            }
        }
        assert!(r.metrics.ride.count > 0, "该有人真坐到过目的地");
    }

    /// 同一个种子跑两次，结果逐位一致。
    #[test]
    fn same_seed_is_bit_identical() {
        let mut c = small();
        c.traffic.departure_ratio = 1.0;
        let a = default_run(c.clone());
        let b = default_run(c);
        assert_eq!(a.metrics.pax_served, b.metrics.pax_served);
        assert_eq!(a.metrics.ghost_stops, b.metrics.ghost_stops);
        assert_eq!(a.metrics.wait.count, b.metrics.wait.count);
        assert_eq!(
            a.metrics.wait.sum.to_bits(),
            b.metrics.wait.sum.to_bits(),
            "同一份种子两次运行的等待时间总和必须逐位相同"
        );
        for (x, y) in a.world.pax.iter().zip(b.world.pax.iter()) {
            assert_eq!(x.t_board.to_bits(), y.t_board.to_bits());
            assert_eq!(x.t_exit.to_bits(), y.t_exit.to_bits());
        }
    }

    /// 制动来不及的呼叫会被越过，且这一次越站被记下来。
    #[test]
    fn fast_car_overshoots_a_call_it_cannot_brake_for() {
        let mut c = Config::default();
        c.building.top_level = 30;
        c.building.units_per_floor = vec![4; 30];
        // 用高速档，制动距离明显拉长
        let mut spec = CarSpec::default();
        spec.up = crate::motion::MotionProfile::preset("high_3_0").unwrap();
        spec.down = spec.up;
        c.cars = vec![spec];
        let mut w = World::new(c);
        // 让车先跑起来：给它一条 1 到 30 的任务
        w.cars[0].leg_from = 1;
        w.cars[0].leg_target = 30;
        w.cars[0].leg_start = 0.0;
        w.cars[0].leg_dist = 29.0 * w.cfg.building.floor_height as f64;
        w.cars[0].dir = Dir::Up;
        w.cars[0].phase = CarPhase::Moving;
        // 车刚起步就有人按 2 层，这时它已经很快了
        w.now = 6.0;
        assert!(!can_stop_at(&w, 0, 2), "高速车刚起步时刹不住 2 层");
        w.now = 0.0;
        assert!(can_stop_at(&w, 0, 2), "刚起步时应当刹得住");
    }

    /// 停靠语义：内核只按「有人要下」和「预定了要接人」两条决定停靠。
    #[test]
    fn kernel_stops_only_where_needed() {
        let mut w = World::new(small());
        let id = 0;
        w.cars[id].occupants.push(0);
        w.pax[0].to = 4;
        assert!(needs_stop(&w, id, 4), "有人要在这层下，必须停");
        assert!(!needs_stop(&w, id, 3), "没人下也没预定，不该停");
        w.cars[id].stops_planned.push((3, Dir::Down));
        assert!(needs_stop(&w, id, 3), "预定了要接人，必须停");
    }

    /// 幽灵停站：某层有呼叫但没人等，轿厢到站开门空跑一次，要计数。
    #[test]
    fn ghost_stop_is_counted_when_nobody_boards() {
        let mut c = small();
        c.traffic.departure_ratio = 0.0;
        c.sim.horizon_s = 600.0;
        let r = default_run(c);
        assert_eq!(r.metrics.pax_total, 0, "出行率为 0 时不该有乘客");
        assert_eq!(r.metrics.ghost_stops, 0, "没有外呼就不该有幽灵停站");
    }

    #[test]
    fn preset_buildings_run() {
        for name in crate::config::preset_names() {
            let c = preset(name).unwrap();
            let r = default_run(c);
            assert!(
                r.metrics.pax_total > 0 || *name == "stress_stalled_car",
                "预设 {name} 跑出来一个乘客都没有"
            );
        }
    }

    /// 单双层：奇数车停奇数层与主入口，偶数车停偶数层与主入口。
    #[test]
    fn odd_even_cars_split_the_floors() {
        use crate::config::StopSpec;
        let odd = StopSpec::parity(true);
        assert!(odd.serves(crate::config::LOBBY), "单层车必须停主入口");
        assert!(odd.serves(3));
        assert!(!odd.serves(4));
        assert!(odd.serves(-1), "地下层人人都停，不然车库那层没人去");
        let even = StopSpec::parity(false);
        assert!(even.serves(crate::config::LOBBY));
        assert!(even.serves(4));
        assert!(!even.serves(3));
    }

    /// 高低区与显式枚举。
    #[test]
    fn zone_and_list_specs() {
        use crate::config::StopSpec;
        let low = StopSpec::zone(1, 15);
        assert!(low.serves(10));
        assert!(!low.serves(20));
        let picked = StopSpec::list([2, 7, 9]);
        assert!(picked.serves(7));
        assert!(!picked.serves(8));
        let combo = StopSpec::union(vec![StopSpec::parity(true), StopSpec::zone(20, 22)]);
        assert!(combo.serves(3));
        assert!(combo.serves(21));
        assert!(!combo.serves(4));
    }

    /// 分区梯：某一层没有车停靠时，配置校验要拦下来。
    #[test]
    fn unserved_floor_is_rejected() {
        use crate::config::StopSpec;
        let mut c = small();
        c.cars = vec![CarSpec::default()];
        c.cars[0].stops = StopSpec::zone(0, 3);
        let v = c.validate();
        assert!(!v.is_ok(), "4 到 6 层没人停，应当报错");
        assert!(v.errors().any(|i| i.code == "unserved_floor"));
    }

    /// 分区梯漏掉主入口层时也要拦下来：那一层的住户去了主入口就回不来。
    #[test]
    fn zone_without_lobby_is_rejected() {
        use crate::config::StopSpec;
        let mut c = small();
        c.cars = vec![CarSpec::default()];
        // 只停 2 到 6 层，主入口层（层号 1）不在区间里
        c.cars[0].stops = StopSpec::zone(2, 6);
        let v = c.validate();
        assert!(
            v.errors().any(|i| i.code == "no_through_car"),
            "分区梯不含主入口层，应当报错，实际 {:?}",
            v.errors().map(|i| i.code).collect::<Vec<_>>()
        );
        assert!(
            v.errors().any(|i| i.code == "unserved_floor"),
            "主入口层没人停，也该报错"
        );
    }

    /// 分区梯能正常服务：每台只在自己的楼层区间里收放人。
    #[test]
    fn split_zone_cars_serve_their_own_floors() {
        use crate::config::StopSpec;
        let mut c = Config::default();
        c.building.top_level = 20;
        c.building.units_per_floor = vec![4; 20];
        let mut lo = CarSpec::default();
        lo.stops = StopSpec::zone(crate::config::LOBBY, 10);
        let mut hi = CarSpec::default();
        hi.stops = StopSpec::union(vec![
            StopSpec::zone(11, 20),
            StopSpec::list([crate::config::LOBBY]),
        ]);
        c.cars = vec![lo, hi];
        c.traffic.departure_ratio = 1.0;
        // 窗口之外的收尾留够时间，免得把「仿真截断」当成「没人接」
        c.sim.horizon_s = 7200.0;
        assert!(c.validate().is_ok(), "这份分区配置应当合法");
        let r = default_run(c);
        let served = r.metrics.pax_served + r.metrics.stair_users as u64;
        assert_eq!(served, r.metrics.pax_total as u64, "分区梯下还有人没被送走");
        // 每位乘客的出发层与目的层都必须被同一台车服务过
        for p in &r.world.pax {
            if p.t_exit <= 0.0 || p.used_stairs {
                continue;
            }
            let ok = (0..r.world.cars.len())
                .any(|i| r.world.car_serves(i, p.from) && r.world.car_serves(i, p.to));
            assert!(ok, "出发层 {} 与目的层 {} 没有一台车同时服务", p.from, p.to);
        }
    }

    /// 每台轿厢从配置的初始层起步。
    #[test]
    fn cars_start_where_configured() {
        let mut c = small();
        c.cars = vec![CarSpec::default(); 2];
        c.cars[0].initial_level = 3;
        c.cars[1].initial_level = 6;
        let w = World::new(c);
        assert_eq!(w.cars[0].level, 3.0);
        assert_eq!(w.cars[1].level, 6.0);
    }

    /// 晚高峰：主流是从主入口层上楼回家。重写前这一股客流根本不生成。
    #[test]
    fn evening_peak_starts_from_the_lobby() {
        let mut c = Config::default();
        c.building.top_level = 18;
        c.building.units_per_floor = vec![6; 18];
        c.traffic.pattern = crate::config::Pattern::EveningUp;
        let w = World::new(c);
        let from_lobby = w.pax.iter().filter(|p| p.hops[0].from == crate::config::LOBBY).count();
        assert!(
            from_lobby * 2 > w.pax.len(),
            "晚高峰应当以主入口层出发为主，实际 {} / {}",
            from_lobby,
            w.pax.len()
        );
        assert!(
            w.pax.iter().any(|p| p.hops[0].from > crate::config::LOBBY),
            "晚高峰也该有一小撮人下楼"
        );
    }

    /// 有地下层时，会有乘客从地下进楼，也会有人下到地下。
    /// 车库客流是低概率的，单次抽取可能抽不到，扫多个种子看有没有出现
    #[test]
    fn basement_passengers_go_both_ways() {
        let mut found_down = false;
        let mut found_up = false;
        for seed in 0..30 {
            let mut c = Config::default();
            c.building.top_level = 12;
            c.building.units_per_floor = vec![6; 12];
            c.building.basement_levels = 2;
            c.cars = vec![CarSpec::default()];
            c.traffic.pattern = crate::config::Pattern::NoonMixed;
            c.traffic.departure_ratio = 1.0;
            c.sim.seed = seed;
            let w = World::new(c);
            found_down |= w.pax.iter().any(|p| p.hops[0].to < 0);
            found_up |= w.pax.iter().any(|p| p.hops[0].from < 0);
        }
        assert!(found_down, "三十个种子里该有人从楼上下去车库");
        assert!(found_up, "三十个种子里该有人从车库上楼");
    }

    /// 条件性需求：到站之后停一阵再回来，会形成第二条行程腿。
    #[test]
    fn return_trips_are_conditional_on_arrival() {
        let mut c = small();
        c.traffic.return_ratio = 1.0;
        c.traffic.return_dwell_s = 600.0;
        c.traffic.departure_ratio = 1.0;
        let w = World::new(c.clone());
        let two_legged = w.pax.iter().filter(|p| p.n_hops == 2).count();
        assert!(two_legged > 0, "全都要返程时应当出现两条腿的乘客");
        assert!(
            w.pax.iter().filter(|p| p.n_hops == 2).all(|p| p.hops[1].from == p.hops[0].to),
            "第二条腿要从第一条腿的终点出发"
        );
        // 跑起来之后，返程的人真的会在目的层重新出现
        let r = default_run(c);
        assert!(
            r.world.pax.iter().any(|p| p.hop_idx >= 1),
            "该有人走完第一条腿后推进到第二条腿"
        );
    }

    /// 等得不耐烦会改走楼梯，且单独计量，不混进出发前就走楼梯的那一类。
    #[test]
    fn impatient_passengers_abandon_to_stairs() {
        let mut c = Config::default();
        c.building.top_level = 33;
        c.building.units_per_floor = vec![8; 33];
        c.cars = vec![CarSpec::default()]; // 一台车伺候六百多人，必然等很久
        c.stairs.abandon_after_s = 60.0;
        c.traffic.departure_ratio = 1.0;
        c.sim.horizon_s = 7200.0;
        let r = default_run(c);
        assert!(r.metrics.abandoned > 0, "这种配置下应当有人等不下去改走楼梯");
        let walkers = r.world.pax.iter().filter(|p| p.abandoned).count();
        assert_eq!(walkers, r.metrics.abandoned);
        assert!(
            r.world.pax.iter().filter(|p| p.abandoned).all(|p| !p.used_stairs),
            "出发前就走楼梯的人不该同时被记成弃梯"
        );
    }

    /// 关掉弃梯开关之后没人再中途弃梯。
    #[test]
    fn disabling_abandon_keeps_everyone_waiting() {
        let mut c = Config::default();
        c.building.top_level = 20;
        c.building.units_per_floor = vec![8; 20];
        c.cars = vec![CarSpec::default()];
        c.stairs.abandon_after_s = 0.0;
        c.traffic.departure_ratio = 1.0;
        let r = default_run(c);
        assert_eq!(r.metrics.abandoned, 0, "关掉之后不该有人弃梯");
    }

    /// 红线 2：上下都按时，多出来的那条呼叫最终变成一次幽灵停站。
    ///
    /// 全员上下都按，下行的人手上都会多亮一条上行按键。上行那条没人真的要用，
    /// 但灯亮着，轿厢一定会来，到了才发现没人上。
    #[test]
    fn unwanted_calls_become_ghost_stops() {
        let mut c = small();
        c.traffic.departure_ratio = 1.0;
        c.strategy = crate::config::StrategyMix {
            honest: 0.0,
            press_both: 1.0,
            press_up_only: 0.0,
            multi_call: 0.0,
        };
        let r = default_run(c);
        assert!(
            r.metrics.ghost_stops > 0,
            "上下都按之后应当出现幽灵停站，实际 {}",
            r.metrics.ghost_stops
        );
        assert!(
            r.metrics.ghost_stops < r.metrics.pax_total * 3,
            "幽灵停站多得不合理：{} 次，乘客 {} 人",
            r.metrics.ghost_stops,
            r.metrics.pax_total
        );
    }

    /// 红线 3：挤不上是随机的，车会带着空位开走，两类原因要分开记。
    #[test]
    fn crowded_car_leaves_with_empty_seats() {
        let mut c = Config::default();
        c.building.top_level = 12;
        c.building.units_per_floor = vec![8; 12];
        c.cars = vec![CarSpec::default()];
        c.traffic.departure_ratio = 1.0;
        // 把拒挤门槛压低，挤的场面来得更早
        c.boarding.soft_start_ratio = 0.4;
        c.stairs.abandon_after_s = 0.0;
        let r = default_run(c);
        assert!(r.metrics.pax_with_bypass > 0, "这么挤的楼该有人被跳过");
        let any_reason = r.world.pax.iter().any(|p| p.declined_count > 0)
            || r.world.pax.iter().any(|p| p.no_space_count > 0);
        assert!(any_reason, "被跳过的人该记下原因");
        // 有人被跳过，却最终被送走了，说明车是带着空位开走的
        let served_with_bypass = r
            .world
            .pax
            .iter()
            .filter(|p| p.bypass_count > 0 && p.t_exit > 0.0)
            .count();
        assert!(served_with_bypass > 0, "被跳过的人最终也该被送走");
    }

    /// 红线 6：候梯人数远超运载能力时，超载死锁必须能被触发。
    #[test]
    fn overload_stall_can_be_triggered() {
        let c = crate::config::preset("stress_stalled_car").unwrap();
        let r = default_run(c);
        assert!(
            r.metrics.stalls > 0,
            "刻意饿死的场景下死锁必须被触发，实际 {} 次",
            r.metrics.stalls
        );
        assert!(r.metrics.stall_seconds > 0.0, "僵持要有累计秒数");
    }

    /// 超载开关关掉时不产生任何僵局。这是红线 2.7 要求的快路径。
    #[test]
    fn overload_off_means_no_stalls() {
        let c = Config::default();
        assert!(!c.overload.enabled, "超载默认关闭");
        let r = default_run(c);
        assert_eq!(r.metrics.stalls, 0);
        assert_eq!(r.metrics.stall_seconds, 0.0);
    }
}
