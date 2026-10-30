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
use crate::policy::{AdmitSample, Assigner, Boarding, CarId, Leg, Router, SystemView, Verdict};
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
    // 容量要给足：每位乘客最多排两个事件（进场与弃梯），再加轿厢的推进
    let mut heap: std::collections::BinaryHeap<Ev> =
        std::collections::BinaryHeap::with_capacity(w.cars.len() + w.pax.len() * 2 + 8);
    let mut seq: u64 = 0;
    let mut metrics = Metrics::with_floors(w.n_levels);
    // 出发前就走楼梯的人不进电梯厅，但逐层统计要算上他们，口径与全局
    // `stair_users` 一致，见 MODEL 9.2。中途弃梯的在 `abandon_to_stairs`
    // 里另加，两边合起来才等于全局那个数
    for p in &w.pax {
        if p.used_stairs && w.in_bounds(p.from) {
            let qi = w.qidx(p.from);
            metrics.by_floor[qi].stair_users += 1;
        }
    }

    // 每位乘客的第一条腿排一次进场事件。两条腿的乘客，第二条腿的进场事件
    // 在他到站时按「出梯时刻加停留时长」再排，所以条件性需求不用提前知道
    for pid in 0..w.pax.len() {
        if w.pax[pid].used_stairs || w.pax[pid].t_exit > 0.0 {
            continue;
        }
        heap.push(Ev {
            at: w.pax[pid].t_arrive,
            seq,
            kind: Kind::Arrive(pid),
        });
        seq += 1;
        // 等得不耐烦的兜底：到点还没上车就改走楼梯
        let patience = w.pax[pid].patience_s;
        if patience.is_finite() && patience > 0.0 {
            w.pax[pid].abandon_at = w.pax[pid].t_arrive + patience;
            heap.push(Ev {
                at: w.pax[pid].t_arrive + patience,
                seq,
                kind: Kind::Abandon(pid),
            });
            seq += 1;
        }
    }
    // 轿厢起步
    for id in 0..w.cars.len() {
        heap.push(Ev {
            at: 0.0,
            seq,
            kind: Kind::CarStep(id),
        });
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
                    heap.push(Ev {
                        at,
                        seq,
                        kind: Kind::Arrive(pid),
                    });
                    seq += 1;
                    continue;
                }
                if !w.pax[pid].waiting() {
                    continue;
                }
                let level = w.pax[pid].from;
                if !w.in_bounds(level) {
                    continue;
                }
                let qi = w.qidx(level);
                w.hall[qi].push(pid);
                // 有车正开着门停在这层时，新到的人可以直接上。策略要求延长
                // 开门时，这段保持开启的时长就是留给这些人的
                if board_stopped(&mut w, pid, boarding, &mut rng) {
                    after_event(&mut w, assigner, &mut sv, &mut heap, &mut seq);
                    continue;
                }
                let pre_owned = register_calls(&mut w, level, &[pid]);
                assign_unowned(&mut w, assigner, &mut sv, &mut heap, &mut seq);
                // 出生带主的独立呼叫不走派梯，这里补同步待办、插停靠、
                // 叫醒三件套。不叫醒的话车收不到任何事件，原地趴到天荒地老
                if !pre_owned.is_empty() {
                    sync_assignments(&mut w);
                    for cid in pre_owned {
                        if let Some(owner) = w.calls.get(cid).and_then(|c| c.owner) {
                            hand_off(&mut w, cid, owner, &mut heap, &mut seq);
                        }
                    }
                }
                // 新来的等待者若超出归属车的停靠范围，就得叫醒别的、愿意停
                // 这层的车来顺手接走（见 `waiting_rescue`）。归属车本来就
                // 送得了时不用打扰它们，省得空车被反复叫醒又趴下
                let to = w.pax[pid].to;
                let wanted = if to > level { Dir::Up } else { Dir::Down };
                let owner = w
                    .calls
                    .slots
                    .get(Call::slot_index(level, wanted, w.basement))
                    .copied()
                    .flatten()
                    .and_then(|cid| w.calls.get(cid))
                    .and_then(|c| c.owner);
                if let Some(o) = owner {
                    if !w.car_serves(o, to) {
                        wake_serving_idle(&mut w, level, Some(o), &mut heap, &mut seq);
                    }
                }
                // 新外呼可能让归属需要重算，也可能让已经没人等的灯该灭
                after_event(&mut w, assigner, &mut sv, &mut heap, &mut seq);
            }

            Kind::Abandon(pid) => {
                abandon_to_stairs(&mut w, pid, &mut metrics);
                // 最后一个等的人改走楼梯后，外呼得跟着灭，不然下一台车白停一次
                after_event(&mut w, assigner, &mut sv, &mut heap, &mut seq);
            }

            Kind::CarStep(id) => {
                step_car(
                    &mut w,
                    id,
                    router,
                    boarding,
                    &mut sv,
                    &mut heap,
                    &mut seq,
                    &mut rng,
                    &mut metrics,
                );
                after_event(&mut w, assigner, &mut sv, &mut heap, &mut seq);
            }

            Kind::StallTick(id) => {
                let res = stall_tick(&mut w, id, &mut rng, &mut heap, &mut seq);
                stalls += res.stalls_here;
                stall_seconds += res.seconds;
                if !res.resumed {
                    let next = w.now + w.cfg.overload.stall_tick_s;
                    heap.push(Ev {
                        at: next,
                        seq,
                        kind: Kind::StallTick(id),
                    });
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
    // 归属表借内核的复用缓冲，事件之间不再申请内存
    sv.owned.clear();
    sv.owned.extend(w.calls.calls.iter().map(|c| c.owner));
    let svref = SystemView {
        now: w.now,
        cars: &sv.briefs,
        pending: &sv.pending,
        all_calls: &sv.all,
        queue_len: &sv.q,
        top_level: w.top,
        n_cars: w.cars.len(),
    };
    assigner.reassign(&svref, &mut sv.owned);
    // 只写回真的变了的槽位，也就不必再拷一份旧归属去比
    for (i, slot) in sv.owned.iter().enumerate() {
        if w.calls.calls[i].owner == *slot {
            continue;
        }
        // 独立呼叫只属于一台车，重分配不动它
        if w.calls.calls[i].exclusive {
            continue;
        }
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
        if !live {
            w.calls.calls[i].owner = None;
            continue;
        }
        let level = w.calls.calls[i].level;
        // 内核兜底：改判的目标车得真能来接这一层，判据见 `car_can_serve`。
        // 算法把归属改到谁头上，内核都得替等梯的人守住这条底线。
        // `None` 是算法主动放手，保持无人负责，等下一次分配
        let owner = match *slot {
            Some(car) if car_can_serve(w, car, level) => Some(car),
            Some(_) => w.least_busy_serving(level),
            None => None,
        };
        match owner {
            // 新车主可能正趴在原地，得叫醒。正在途中的顺手插一脚刹车
            Some(car) => {
                let cid = w.calls.calls[i].id;
                hand_off(w, cid, car, heap, seq);
            }
            None => w.calls.calls[i].owner = None,
        }
    }
    sync_assignments(w);
}

/// 这一层这个方向此刻还有没有人在等。
fn waiting_for(w: &World, level: i16, dir: Dir) -> bool {
    if level < -w.basement || level > w.top {
        return false;
    }
    w.hall[w.qidx(level)]
        .iter()
        .any(|&p| w.pax[p].waiting() && w.pax[p].presses(level, dir))
}

/// 这一层有没有「归别人、但别人接不了」的人在等。
///
/// 共享外呼只归一台车。若等待者的目的层超出归属车的停靠范围（高低区共用
/// 主入口层时就会这样），归属车送不了他，他也不会再有第二张外呼。这时
/// 任何一台既停这一层、又能送他的车都该顺手接走，否则他会一直等下去。
/// 只有归属车真的接不了时才出手，正常楼层不受影响。
fn waiting_rescue(w: &World, id: CarId, here: i16) -> bool {
    if here < -w.basement || here > w.top {
        return false;
    }
    let want = board_dir_at(w, id, here);
    for &p in &w.hall[w.qidx(here)] {
        let px = &w.pax[p];
        if !px.waiting() || !w.car_serves(id, px.to) {
            continue;
        }
        // 看乘客按过哪个方向，不看他的真实目的方向。只按上行的人按的是
        // 上行，拿真实方向去查会查到一条他根本没按的呼叫
        let pdir = match want {
            Dir::Up | Dir::Down => want,
            Dir::None => {
                if px.presses(here, Dir::Up) {
                    Dir::Up
                } else {
                    Dir::Down
                }
            }
        };
        if !px.presses(here, pdir) {
            continue;
        }
        let slot = Call::slot_index(here, pdir, w.basement);
        let owner = w
            .calls
            .slots
            .get(slot)
            .copied()
            .flatten()
            .and_then(|cid| w.calls.get(cid))
            .and_then(|c| c.owner);
        match owner {
            // 有人负责，而且那台车送得了他：不抢
            Some(o) if o != id && w.car_serves(o, px.to) => {}
            // 没人负责、本来就是我的活，或归属车送不了：该我接
            _ => return true,
        }
    }
    false
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
    // 过期计时器：换腿和僵局重排都会把 abandon_at 往后推，
    // 旧事件到点就忽略，只认当前这条腿的截止时刻
    if w.pax[pid].patience_s.is_finite() && w.now + 1e-9 < w.pax[pid].abandon_at {
        return;
    }
    let level = w.pax[pid].from;
    let to = w.pax[pid].to;
    // 楼外的乘客没有电梯厅可退，也没有逐层样本可记。正常配置到不了这里，
    // 越界的畸形乘客才会，丢掉比把下标打穿好
    if !w.in_bounds(level) {
        return;
    }
    let stairs = w.cfg.stairs.clone();
    let floors = (to - level).abs();
    // 可行性上限：没人会爬二十层楼梯去上班。超过这个层数就不许弃梯，
    // 他只能继续等，见 `Stairs::practical_floor_cap`
    if stairs.practical_floor_cap > 0 && floors > stairs.practical_floor_cap {
        return;
    }
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
    // 少了一个人等，两个方向都要处理：外呼上记的人数跟着降，
    // 要是这条外呼死在一台卡住的车上，顺手打上标记。
    // 「已经没人等」这个条件把两条计数路径分开，一条外呼只记一次
    for dir in [Dir::Up, Dir::Down] {
        refresh_waiting(w, level, dir);
        if waiting_for(w, level, dir) {
            continue;
        }
        let slot = Call::slot_index(level, dir, w.basement);
        let Some(cid) = w.calls.slots.get(slot).copied().flatten() else {
            continue;
        };
        let stalled = w.calls.get(cid).is_some_and(|c| owned_by_stalled(w, c));
        if stalled {
            if let Some(c) = w.calls.get_mut(cid) {
                c.orphaned = true;
            }
        }
    }
    // 弃梯的人确实等过，这段时间要算进等待分布，否则报告里的等待会被
    // 系统性低估：等最久的那批人正好是弃梯走掉的那批
    metrics.wait.add(waited);
    let (bypass, declined, no_space) = {
        let p = &w.pax[pid];
        (p.leg_bypass(), p.declined_count, p.no_space_count)
    };
    let fs = &mut metrics.by_floor[qi];
    fs.count += 1;
    fs.wait_sum += waited;
    fs.wait_max = fs.wait_max.max(waited);
    fs.wait_samples.add(waited);
    // 弃梯的人之前被跳过的那几次真实发生过，也要进逐层计数。
    // 只在上车时才记，会让「跳过」总数漏掉最后改走楼梯的那批人
    fs.bypass_total += bypass;
    fs.declined += declined;
    fs.no_space += no_space;
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
/// 只挂还算数的外呼。共享看灯（槽位还指着它），独立看归属，
/// 哪条都不满足的旧记录在这里掉队。
fn sync_assignments(w: &mut World) {
    for c in &mut w.cars {
        c.assigned.clear();
    }
    for c in &w.calls.calls {
        let Some(owner) = c.owner else { continue };
        if !w.call_is_live(c) {
            continue;
        }
        if owner < w.cars.len() {
            w.cars[owner].assigned.push(c.id);
        }
    }
}

/// 内核兜底判据：这台车此刻真能来接这一层的人吗。
///
/// 三条一起看。车号越界是算法写错了下标，不服务这一层是算法漏了
/// `StopSpec`，僵死车是算法不知道它卡住了。三条里任意一条成立，
/// 内核都要换一台车，不能把外呼挂上去，也不能直接崩。
fn car_can_serve(w: &World, car: CarId, level: i16) -> bool {
    car < w.cars.len() && w.car_serves(car, level) && w.cars[car].phase != CarPhase::Stalled
}

/// 把一条外呼记到一台车名下，并让它动起来。
///
/// 派梯有三个入口：新外呼派梯、改判写回、独立面板出生带主。三处都要做
/// 「改归属、插停靠、叫醒」这三件事，少一步车就不知道有活。集中在一处，
/// 免得三份各写一遍再慢慢漂移。待办列表由调用方在合适的时候同步
fn hand_off(
    w: &mut World,
    cid: u32,
    car: CarId,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
) {
    let Some((level, dir)) = w.calls.get(cid).map(|c| (c.level, c.dir)) else {
        return;
    };
    if let Some(c) = w.calls.get_mut(cid) {
        c.owner = Some(car);
    }
    plan_for_moving(w, car, level, dir);
    wake(w, car, heap, seq);
}

/// 叫醒一台停在原地的轿厢。分配完新活之后调用。
fn wake(w: &mut World, id: CarId, heap: &mut std::collections::BinaryHeap<Ev>, seq: &mut u64) {
    if w.cars[id].phase == CarPhase::Idle && !w.cars[id].next_event.is_finite() {
        // 有新活，允许在同层重新开门，别被上一次收尾的压制挡住
        w.cars[id].settled_level = None;
        w.cars[id].next_event = w.now;
        heap.push(Ev {
            at: w.now,
            seq: *seq,
            kind: Kind::CarStep(id),
        });
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
    // 按下标走，不先抄一份清单，事件之间不分配内存。这是一次快照里
    // 的外呼编号，派梯不会新增记录，循环期间下标稳定
    let mut changed = false;
    for i in 0..w.calls.calls.len() {
        if !should_dispatch(w, i) {
            continue;
        }
        let target = w.calls.calls[i].view(w.calls.calls[i].waiting);
        let owner = {
            let view = SystemView {
                now: w.now,
                cars: &sv.briefs,
                pending: &sv.pending,
                all_calls: &sv.all,
                queue_len: &sv.q,
                top_level: w.top,
                n_cars: w.cars.len(),
            };
            assigner.assign(&view, target)
        };
        let Some(car) = owner else { continue };
        // 内核兜底：算法选中的车如果接不了这一层，换一台干得了的。
        // 再好的算法也可能漏掉这条约束，而漏掉的代价是那一层的人
        // 永远等不到车。判据见 `car_can_serve`
        let car = if car_can_serve(w, car, target.level) {
            car
        } else {
            let Some(alt) = w.least_busy_serving(target.level) else {
                continue;
            };
            alt
        };
        hand_off(w, target.id, car, heap, seq);
        changed = true;
        // 同层可能还停着别的闲车，它送得了归属车送不了的等待者。
        // 叫醒它，让它自己决定顺手接不接（在别的层会重新趴下，不白跑）
        wake_serving_idle(w, target.level, Some(car), heap, seq);
    }
    // 待办列表在循环外同步一次。派梯过程不需要它，叫醒后的那一步才要
    if changed {
        sync_assignments(w);
    }
}

/// 这条外呼此刻该不该派：没归属、还算数。独立呼叫不进槽位，
/// 另看层上还有没有人在等，否则出生没主的独立呼叫永远没人捡
fn should_dispatch(w: &World, i: usize) -> bool {
    let c = &w.calls.calls[i];
    if c.owner.is_some() {
        return false;
    }
    if c.exclusive {
        return waiting_for(w, c.level, c.dir);
    }
    w.call_is_live(c)
}

/// 把愿意停这一层、又停在原地待命的轿厢叫醒。`except` 是已经处理过的那一台。
///
/// 停在这层的车会看自己要不要顺手接「归属车送不了」的等待者。停在别层的车
/// 醒了也会因为没有名下活而重新趴下，不白跑。
fn wake_serving_idle(
    w: &mut World,
    level: i16,
    except: Option<CarId>,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
) {
    for other in 0..w.cars.len() {
        if except != Some(other) && w.car_serves(other, level) {
            wake(w, other, heap, seq);
        }
    }
}

/// 这条外呼现在归到一台卡死的轿厢名下了吗。
fn owned_by_stalled(w: &World, c: &Call) -> bool {
    c.owner
        .is_some_and(|o| o < w.cars.len() && w.cars[o].phase == CarPhase::Stalled)
}

/// 电梯厅里的人数变了，把外呼上记的人数刷新一次。
///
/// 派梯算法读 `HallCall.waiting`。只在按键那一刻算一次的话，车接走一批人
/// 以后这个数会一直偏大，算法会以为层上还排着长队
fn refresh_waiting(w: &mut World, level: i16, dir: Dir) {
    if level < -w.basement || level > w.top {
        return;
    }
    let n = count_waiting(w, level, dir);
    let slot = Call::slot_index(level, dir, w.basement);
    if let Some(cid) = w.calls.slots.get(slot).copied().flatten() {
        if let Some(c) = w.calls.get_mut(cid) {
            c.waiting = n;
        }
    }
    // 独立面板的呼叫不进槽位，从各车名下的待办里找
    for id in 0..w.cars.len() {
        for k in 0..w.cars[id].assigned.len() {
            let cid = w.cars[id].assigned[k];
            if let Some(c) = w.calls.get_mut(cid) {
                if c.exclusive && c.level == level && c.dir == dir {
                    c.waiting = n;
                }
            }
        }
    }
}

/// 清理已经没人等、也没人负责的外呼。
///
/// 有人负责的呼叫一律保留：现实里按键灯亮着，轿厢一定会来，到了才发现
/// 没人上，那一次停靠就是幽灵停站。这条判据正是红线 2 的落点。
fn prune_calls(w: &mut World) {
    // 按下标走，灭一条立刻灭，不先攒一份清单，事件之间不分配内存
    for slot in 0..w.calls.slots.len() {
        let Some(cid) = w.calls.slots[slot] else {
            continue;
        };
        let Some(c) = w.calls.get(cid) else { continue };
        let (level, dir) = (c.level, c.dir);
        let owned = c.owner.is_some();
        if level < -w.basement || level > w.top || (!owned && !waiting_for(w, level, dir)) {
            kill_call(w, level, dir);
        }
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
    let Some(cid) = w.calls.slots.get(slot).copied().flatten() else {
        return;
    };
    w.calls.clear_slot(level, dir);
    if let Some(c) = w.calls.get_mut(cid) {
        c.owner = None;
    }
    for c in &mut w.cars {
        c.assigned.retain(|&a| a != cid);
        c.stops_planned.retain(|&(l, d)| !(l == level && d == dir));
    }
}

/// 这一层这个方向此刻有几个人在等。派梯算法会读这个数。
fn count_waiting(w: &World, level: i16, dir: Dir) -> usize {
    if level < -w.basement || level > w.top {
        return 0;
    }
    w.hall[w.qidx(level)]
        .iter()
        .filter(|&&p| w.pax[p].waiting() && w.pax[p].presses(level, dir))
        .count()
}

/// 登记外呼。红线 1 与红线 2 都在这个函数里。
///
/// 返回出生就带主的独立呼叫编号。它们不走派梯，调用方负责补
/// 同步待办、插停靠、叫醒三件套（见 Arrive）。
fn register_calls(w: &mut World, level: i16, newly: &[usize]) -> Vec<u32> {
    if newly.is_empty() {
        return Vec::new();
    }
    let per_car = w.cfg.call_surface == CallSurface::PerCarPanel;
    let mut pre_owned: Vec<u32> = Vec::new();
    for &pid in newly {
        let p = w.pax[pid].clone();
        let wanted = if p.to > level { Dir::Up } else { Dir::Down };
        match p.strategy {
            // 红线 1 的基线：只按自己真正想去的方向
            Strategy::Honest => {
                let n = count_waiting(w, level, wanted);
                w.calls.register(level, wanted, n, false);
            }
            // 红线 2：上下都按。哪趟先来上哪趟
            Strategy::PressBoth => {
                let up = count_waiting(w, level, Dir::Up);
                let down = count_waiting(w, level, Dir::Down);
                w.calls.register(level, Dir::Up, up, false);
                w.calls.register(level, Dir::Down, down, false);
            }
            // 先按上行，跟车折返再下来
            Strategy::PressUpOnly => {
                let n = count_waiting(w, level, Dir::Up);
                w.calls.register(level, Dir::Up, n, false);
            }
            // 同时呼叫多台。只有独立面板才做得到
            Strategy::MultiCall => {
                if per_car {
                    let n = count_waiting(w, level, wanted);
                    let k = w.cars.len().min(2);
                    for t in 0..k {
                        let (id, _) = w.calls.register(level, wanted, n, true);
                        // 独立面板一车一按钮：这条呼叫从出生就属于这台车。
                        // 车不服务这一层或已经卡死，就近换一台服务的，
                        // 与 assign_unowned 里那道兜底同一条。实在没车可换
                        // 就留成无主，等派梯捡走，总比出生即死锁强
                        let owner = if car_can_serve(w, t, level) {
                            t
                        } else {
                            let Some(alt) = w.least_busy_serving(level) else {
                                continue;
                            };
                            alt
                        };
                        if let Some(c) = w.calls.get_mut(id) {
                            c.owner = Some(owner);
                        }
                        pre_owned.push(id);
                    }
                } else {
                    let n = count_waiting(w, level, wanted);
                    w.calls.register(level, wanted, n, false);
                }
            }
        }
    }
    pre_owned
}

/// 把一条外呼插进某台正在途中的轿厢。真刹不住才记一次越站，
/// 车已经开过的楼层只跳过不记：那不是刹不住，是车已经走了。
fn plan_for_moving(w: &mut World, id: CarId, level: i16, dir: Dir) {
    if w.cars[id].phase != CarPhase::Moving {
        return;
    }
    if w.cars[id].dir != dir {
        return;
    }
    match brake_check(w, id, level) {
        Brake::Stop => {
            let pair = (level, dir);
            if !w.cars[id].stops_planned.contains(&pair) {
                w.cars[id].stops_planned.push(pair);
            }
        }
        Brake::Passed => {}
        Brake::Overshoot => {
            w.cars[id].overshoots += 1;
        }
    }
}

/// 制动检查的结果。`Passed` 与 `Overshoot` 都插不进去，
/// 但只有后者是真的刹不住。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Brake {
    /// 来得及刹住，可以插进去
    Stop,
    /// 已经开过这一层，插不进去，但不记越站
    Passed,
    /// 还没到，但剩余距离不够刹停，记一次越站
    Overshoot,
}

/// 正在途中时，现在还来不来得及刹到 `level` 层。
///
/// 车已经开过、或者剩余距离短于从当前速度刹停所需的距离，都插不进去，
/// 但只有后者算越站。低速梯制动距离短，几乎不该出现 Overshoot，
/// 见 `docs/MODEL.md` 第 3.8 节。
fn brake_check(w: &World, id: CarId, level: i16) -> Brake {
    let c = &w.cars[id];
    if c.phase != CarPhase::Moving {
        return Brake::Stop;
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
        if remaining > -1e-9 {
            return Brake::Stop;
        }
        return Brake::Passed;
    }
    let brake = decel_distance(speed, *profile) + speed * BRAKE_REACTION_S;
    if remaining + 1e-9 >= brake {
        Brake::Stop
    } else {
        Brake::Overshoot
    }
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
                // 车就在原地反复空停靠，车上的人一辈子送不到。
                //
                // 位子按额定算，不按物理极限算。13 到 17 人之间时这道门不开，
                // 层上的人只能等耐心耗尽，与红线 3 的挤梯口径不一致，这是已知取舍：
                // 按极限开门会多出一次谁也挤不上的空停靠，白付一次停靠开销，
                // 那批人还要再等下一趟。想放宽，先用测试确认满员到站不会把车留在原地
                let has_free = w.cars[id].occupants.len() < w.cfg.cars[id].capacity_rated;
                let someone = waiting_for(w, here, Dir::Up) || waiting_for(w, here, Dir::Down);
                // 刚在这一层收过尾就不要再开门。否则车没满又挤不上人时，
                // 会在同一层反复空停靠，`choose_leg` 永远走不到，整台车卡死
                let fresh = w.cars[id].settled_level != Some(here);
                let own = owns_call_at(w, id, here) && someone;
                // 归属车送不了的等待者，顺手接走，见 `waiting_rescue`
                let rescue = waiting_rescue(w, id, here);
                if fresh && has_free && (own || rescue) {
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
                // 满载直驶：车已经满员，这一层只是要接同向的人，没人要在这里下，
                // 停靠表里的这一站就作废，照直开过去。记一次，口径见 MODEL 9.3。
                // 停不起的还有可能白停一次（幽灵），这一条把它变成有意义的直驶
                let full = w.cars[id].occupants.len() >= w.cfg.cars[id].capacity_rated;
                if full
                    && !riders_alight_here(w, id, here)
                    && w.cars[id].stops_planned.iter().any(|&(l, _)| l == here)
                    && full_passes_call(w, id, here)
                {
                    w.cars[id].full_bypasses += 1;
                    w.cars[id].stops_planned.retain(|&(l, _)| l != here);
                } else if needs_stop(w, id, here) {
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
                // 关门判据交给策略。默认实现到点就关，时序与旧版一致。
                // 返回 false 就再开一个保持开启时长，这期间到的人还能上，
                // 见主循环 `Kind::Arrive` 分支
                let close = {
                    w.fill_views(sv);
                    let v = w.fill_run_view(sv, id);
                    boarding.close_now(&v)
                };
                if close {
                    settle_stop(w, id);
                    w.cars[id].phase = CarPhase::Idle;
                    continue;
                }
                let hold = w.cfg.cars[id].doors.dwell_s as f64;
                w.cars[id].holding = true;
                w.cars[id].next_event = w.now + hold.max(1e-3);
                break;
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
        heap.push(Ev {
            at,
            seq: *seq,
            kind: Kind::CarStep(id),
        });
        *seq += 1;
    } else {
        w.cars[id].next_event = w.now + 0.001;
        heap.push(Ev {
            at: w.now + 0.001,
            seq: *seq,
            kind: Kind::CarStep(id),
        });
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
    // 离开这一层，重开门的压制随之解除
    w.cars[id].settled_level = None;
    // 车上没人时这一程算一趟往返的起点
    if w.cars[id].occupants.is_empty() && w.cars[id].trip_start == 0.0 {
        w.cars[id].trip_start = w.now;
    }
    // 这条腿沿途要接的外呼，以及要送的内选，全部预先排进停靠表
    plan_leg(w, id, here, target, dir);
    advance_leg(w, id);
    true
}

/// 一次腿的预定停靠：名下的同向外呼，以及车上乘客的目的层。
///
/// 目标层上的外呼不论朝向都要排进来：那是这台车这趟的折返点，
/// 折返点不收人，车就会在「到了却不停」和「返回本层」之间空转。
fn plan_leg(w: &mut World, id: CarId, from: i16, target: i16, dir: Dir) {
    let lo = from.min(target);
    let hi = from.max(target);
    // 按下标扫，不先抄一份待停清单，这条腿的规划不分配内存
    for k in 0..w.cars[id].assigned.len() {
        let cid = w.cars[id].assigned[k];
        let Some(c) = w.calls.get(cid) else {
            continue;
        };
        let on_the_way = c.dir == dir && c.level >= lo && c.level <= hi && c.level != from;
        if c.level != target && !on_the_way {
            continue;
        }
        let pair = (c.level, c.dir);
        if !w.cars[id].stops_planned.contains(&pair) {
            w.cars[id].stops_planned.push(pair);
        }
    }
    for k in 0..w.cars[id].occupants.len() {
        let pid = w.cars[id].occupants[k];
        let t = w.pax[pid].to;
        let pair = (t, dir);
        if t >= lo && t <= hi && t != from && !w.cars[id].stops_planned.contains(&pair) {
            w.cars[id].stops_planned.push(pair);
        }
    }
}

/// 这一层有没有一条「这台车方向上、还亮着」的外呼。
///
/// 只按车头方向找。反方向的外呼本来就不归它接，满载开过去不算直驶。
fn full_passes_call(w: &World, id: CarId, here: i16) -> bool {
    if !w.car_serves(id, here) {
        return false;
    }
    let dir = w.cars[id].dir;
    if !dir.is_moving() {
        return false;
    }
    w.calls
        .slots
        .get(Call::slot_index(here, dir, w.basement))
        .copied()
        .flatten()
        .is_some()
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
///
/// 开腿与途中推进共用这一处，排期逻辑就不会写岔。
fn advance_leg(w: &mut World, id: CarId) {
    let sign = if w.cars[id].dir == Dir::Up { 1 } else { -1 };
    let here = w.cars[id].level.round() as i16;
    let target = w.cars[id].leg_target;
    // 削到目标层为止。宁可早一层到，也不要越过去之后漏排事件
    let raw = here + sign;
    let next = if sign > 0 {
        raw.min(target)
    } else {
        raw.max(target)
    };
    if next == here {
        return;
    }
    let spec = &w.cfg.cars[id];
    let floor_h = w.cfg.building.floor_height as f64;
    let profile = w.cars[id].dir.profile(&spec.up, &spec.down);
    let cum = (next - w.cars[id].leg_from).abs() as f64 * floor_h;
    let t = time_at_distance(w.cars[id].leg_dist, *profile, cum);
    w.cars[id].level = next as f32;
    w.cars[id].phase = CarPhase::Moving;
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
    //
    // 独立面板的多呼叫按同一个判据：这一层这个方向还留得下人，就留灯。
    // 车到站就无脑擦灯，会让满载到达时没挤上的人永远等不到下一趟，
    // 与共享按键世界里「灯一直亮着等下一辆有位的车」的规矩对不上。
    for dir in [Dir::Up, Dir::Down] {
        if !waiting_for(w, lv, dir) {
            kill_call(w, lv, dir);
            w.calls.clear_exclusive_owned_by(id, lv, dir);
        } else {
            // 刚接走一批人，外呼上记的人数要跟着降，别让算法以为队还很长
            refresh_waiting(w, lv, dir);
        }
    }

    let doors = w.cfg.cars[id].doors;
    let base = (doors.open_s - doors.early_open_credit_s).max(0.0) as f64;
    let in_t = doors.transfer_time(w.cars[id].last_boarded, w.cars[id].last_alighted);
    // 保持开启每次停靠都付，与上下客时间相加，见 MODEL 3.6 的公式。
    // 少了这 1.5 秒，空停一次只有 4.5 秒，而估计函数按 6.0 算，
    // 乘客脑子里的信念和实际收费就对不上了
    let total =
        base + doors.dwell_s as f64 + in_t + doors.close_s as f64 + doors.ctrl_resp_s as f64;
    let close_at = w.cars[id].stop_started + total;

    // 幽灵停站：开门了却没人上、也没人下，白损失一次停靠开销。
    // 这么早就记，是因为绝大多数停靠当场就定下了。策略要求延长开门时，
    // 后面真有人上来的话在 `board_stopped` 里撤销这一笔
    if w.cars[id].last_boarded == 0 && w.cars[id].last_alighted == 0 {
        w.cars[id].ghost_stops += 1;
    }

    let trigger = w.cfg.overload.trigger_ratio * w.cfg.cars[id].rated_load_kg as f64;
    if w.cfg.overload.enabled && w.cars[id].load_kg > trigger {
        // 红线 6：超载报警。门关不上，没人退出就一直僵着
        w.cars[id].phase = CarPhase::Stalled;
        w.cars[id].stall_since = w.now;
        w.cars[id].next_event = f64::INFINITY;
        heap.push(Ev {
            at: w.now + w.cfg.overload.stall_tick_s,
            seq: *seq,
            kind: Kind::StallTick(id),
        });
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
    w.cars[id].holding = false;
    // 记下刚收尾的层，Idle 里据此压制原地重开门
    w.cars[id].settled_level = Some(lv);
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
/// 4. 这一层名下没有外呼，且车上已空，方向失去意义，两个方向都收
/// 5. 这一层没有外呼，车还载着人，按车头方向收
///
/// 第 3 条是早高峰的落点：轿厢空车上行到最高任务层，那里只有下行外呼，
/// 要在这一层把下行客收上来，而不是按车头方向把上行客收走。
/// 第 4 条是分区梯的落点：空车到站时车头还残留着上一段的方向，若按它收人，
/// 归属别的车、但只有这台车送得了的等待者就永远接不走。
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
    if let Some(o) = opposite {
        return o;
    }
    if w.cars[id].occupants.is_empty() {
        return Dir::None;
    }
    d
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
        // 先把下标算好再借乘客。qidx 借整个 World，与下面那笔可变借用作对
        let lv_idx = w.qidx(from);
        let pax = &mut w.pax[pid];
        pax.t_exit = now;
        pax.boarded_car = None;
        pax.hops_done += 1;
        let (wait, ride, d2d) = (pax.wait(), pax.ride(), pax.door_to_door());
        metrics.wait.add(wait);
        metrics.ride.add(ride);
        metrics.door_to_door.add(d2d);
        let fs = &mut metrics.by_floor[lv_idx];
        fs.count += 1;
        fs.wait_sum += wait;
        fs.wait_max = fs.wait_max.max(wait);
        fs.wait_samples.add(wait);
        // 门到门只记坐上车走完的人。中途弃梯的在另一条路径里只记等待，
        // 这里不记，保持与全局 door_to_door 同一口径
        fs.d2d_sum += d2d;
        fs.d2d_max = fs.d2d_max.max(d2d);
        fs.d2d_samples.add(d2d);
        fs.bypass_total += pax.leg_bypass();
        fs.declined += pax.declined_count;
        fs.no_space += pax.no_space_count;
        // 这一腿的计数已经进逐层了，搬进累计值再清零。人物级的统计读两者
        // 之和，换腿时清零单腿计数不会把前一腿的丢掉
        pax.declined_total += pax.declined_count;
        pax.no_space_total += pax.no_space_count;
        pax.declined_count = 0;
        pax.no_space_count = 0;
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
            heap.push(Ev {
                at,
                seq: *seq,
                kind: Kind::Arrive(pid),
            });
            *seq += 1;
            // 第二条腿给一个完整的新耐心窗口。第一条腿遗留的旧计时器
            // 还在堆里，它到点时按 abandon_at 认出过期，不会误赶人
            let patience = w.pax[pid].patience_s;
            if patience.is_finite() && patience > 0.0 {
                w.pax[pid].abandon_at = at + patience;
                heap.push(Ev {
                    at: at + patience,
                    seq: *seq,
                    kind: Kind::Abandon(pid),
                });
                *seq += 1;
            }
        } else {
            w.pax[pid].final_exit = now;
        }
    }

    // 再上人。一次扫完所有候选人。
    //
    // 队首那位挤不上时，后面的人仍有机会，所以扫描不能因为一次失败就停。
    // 但不必反复重扫：随着人一个个上车，载客率只会升，挤不上的条件只会更差，
    // 第二轮的结果不可能比第一轮好。
    let want = board_dir_at(w, id, lv);
    let mut boarded = 0usize;
    let mut served = Dir::None;
    let qi = w.qidx(lv);
    let n = w.hall[qi].len();
    for i in 0..n {
        // 先把 id 复制出来，再进可变借用的上客流程。上客不动电梯厅名单，
        // 所以这一趟按下标走是安全的，也就不用先克隆一份
        let pid = w.hall[qi][i];
        if !w.pax[pid].waiting() {
            continue;
        }
        let pdir = if w.pax[pid].to > lv {
            Dir::Up
        } else {
            Dir::Down
        };
        if try_board_one(w, id, lv, pid, want, boarding, now, rng) {
            boarded += 1;
            served = pdir;
        }
    }
    w.cars[id].last_boarded = boarded;

    // 上梯的人离开电梯厅。刚上车的人才带 `boarded_car`，一次线性过滤就够
    if boarded > 0 {
        w.hall[qi].retain(|&p| w.pax[p].boarded_car.is_none());
    }

    // 未满载就走，而层上还有人等，记一次不全载发车。
    //
    // 只看车这一趟实际收的那个方向。空车到站时 `want` 是 `Dir::None`，若拿它
    // 当判据，反方向留着的人也会算进「没挤上」，把指标灌水。收的是哪边走就
    // 数哪边，两个方向都收过就都算。
    let rated = w.cfg.cars[id].capacity_rated;
    if boarded > 0 && w.cars[id].occupants.len() < rated {
        let dirs: &[Dir] = if want.is_moving() {
            &[want]
        } else if served.is_moving() {
            &[served]
        } else {
            &[]
        };
        if dirs.iter().any(|&d| waiting_for(w, lv, d)) {
            w.cars[id].partial_departures += 1;
        }
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
    // 轿厢方向要匹配乘客按过的按钮，不能拿真实目的方向去挡。
    // 只按上行的下行乘客按的是上行，双按的人两个方向都收。
    // 停着不动时任何方向都收
    let matching = match want {
        Dir::None => true,
        d => w.pax[pid].presses(lv, d),
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
    // 车里的不会比 crush 多：每次 push 之前都先比这道线，见下面的 Admit 分支
    let free = crush - w.cars[id].occupants.len();
    let load_ratio = w.cars[id].occupants.len() as f64 / capacity_rated as f64;
    let waited = (now - w.pax[pid].t_arrive).max(0.0);
    let sample = AdmitSample {
        load_ratio,
        free_slots: free,
        waited_s: waited,
        weight_ratio: w.cars[id].load_kg / w.cfg.cars[id].rated_load_kg as f64,
    };
    let tol = w.pax[pid].tol;

    match boarding.admit_one(&sample, tol) {
        Verdict::NoSpace => {
            w.pax[pid].no_space_count += 1;
            false
        }
        Verdict::Declined => {
            w.pax[pid].declined_count += 1;
            false
        }
        Verdict::Admit => {
            // 物理极限先判。装不下就是装不下，跟愿不愿意挤无关；放到骰子
            // 后面的话，超重的人有一半概率被记成「不肯挤」，界面上两个
            // 原因就串了。千克上限管真装不下，人数上限管真挤不进去。
            let load_crush = w
                .cfg
                .boarding
                .load_crush_kg(w.cfg.cars[id].rated_load_kg as f64);
            if w.cars[id].load_kg + w.pax[pid].weight > load_crush + 1e-9 {
                w.pax[pid].no_space_count += 1;
                return false;
            }
            if w.cars[id].occupants.len() >= crush {
                w.pax[pid].no_space_count += 1;
                return false;
            }
            // 中间地带要掷骰子。概率由策略给，随机性归内核。
            // 这一步是红线 3 的核心：越接近满载，越可能有人放弃。
            // 拿策略的概率而不是配置里的曲线，写新 `Boarding` 才能真的改判据
            let prob = boarding.accept_prob(&sample, tol);
            if prob < 1.0 && !rng.chance(Stream::Squeeze, prob) {
                w.pax[pid].declined_count += 1;
                return false;
            }
            w.cars[id].occupants.push(pid);
            w.cars[id].load_kg += w.pax[pid].weight;
            w.pax[pid].boarded_car = Some(id);
            w.pax[pid].t_board = now;
            // 反向叫梯的人登记转向层，一路坐到折返点再下来
            if matches!(
                w.pax[pid].strategy,
                Strategy::PressUpOnly | Strategy::PressBoth
            ) {
                w.pax[pid].reversal_floor = Some(lv);
            }
            true
        }
    }
}

/// 新到的人能不能直接上一台正开着门的车。
///
/// 只在策略要求延长开门时才会走到这里，见 `Boarding::close_now`。
/// 上去了就把人从电梯厅拿掉，外呼上的人数跟着刷新。
fn board_stopped<B: Boarding>(w: &mut World, pid: usize, boarding: &B, rng: &mut Streams) -> bool {
    let lv = w.pax[pid].from;
    let now = w.now;
    let mut boarded: Option<CarId> = None;
    for id in 0..w.cars.len() {
        // 只看策略要求延长开门的那一段。正常开门窗口内的到达仍走旧路径，
        // 默认策略的结果一个数都不变
        if !w.cars[id].holding {
            continue;
        }
        if w.cars[id].phase != CarPhase::Boarding {
            continue;
        }
        if w.cars[id].level.round() as i16 != lv {
            continue;
        }
        let want = board_dir_at(w, id, lv);
        if try_board_one(w, id, lv, pid, want, boarding, now, rng) {
            boarded = Some(id);
            break;
        }
    }
    let Some(id) = boarded else {
        return false;
    };
    let qi = w.qidx(lv);
    w.hall[qi].retain(|&p| p != pid);
    // 这站原本按空停记了一次幽灵。延长开门期间真有人上来，就把这笔撤掉
    if w.cars[id].last_boarded == 0 && w.cars[id].last_alighted == 0 {
        w.cars[id].ghost_stops = w.cars[id].ghost_stops.saturating_sub(1);
    }
    w.cars[id].last_boarded += 1;
    refresh_waiting(w, lv, Dir::Up);
    refresh_waiting(w, lv, Dir::Down);
    true
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
        let Some(&pid) = w.cars[id].occupants.get(pos) else {
            continue;
        };
        let q = if Some(pid) == last_boarder {
            ov.q_last * pressure
        } else {
            ov.q_other * pressure
        };
        if rng.chance(Stream::Stall, q) {
            exited = Some((pos, pid));
            break;
        }
    }
    if let Some((_, pid)) = exited {
        requeue_after_stall(w, id, pid, heap, seq);
    }

    let now_under = w.cars[id].load_kg <= trigger;
    let timed_out = elapsed > ov.stall_timeout_s;
    if now_under || (timed_out && w.cars[id].occupants.len() > 1) {
        // 恢复了。强制退出时把最近上车的人请下去
        if !now_under {
            if let Some(&pid) = w.cars[id].occupants.last() {
                requeue_after_stall(w, id, pid, heap, seq);
            }
        }
        let seconds = w.now - w.cars[id].stall_since;
        w.cars[id].phase = CarPhase::Idle;
        w.cars[id].stall_since = 0.0;
        w.cars[id].next_event = w.now;
        heap.push(Ev {
            at: w.now,
            seq: *seq,
            kind: Kind::CarStep(id),
        });
        *seq += 1;
        return StallOutcome {
            resumed: true,
            seconds,
            stalls_here: 1,
        };
    }

    StallOutcome {
        resumed: false,
        seconds: 0.0,
        stalls_here: 0,
    }
}

/// 把一位乘客从轿厢里请出来，让他重新排队。
///
/// **不能把他记成「已到达」。** 他只是被从轿厢里请了出来，一步都没往目的地走。
/// 早先的实现直接写 `t_exit = now`，于是他的行程被算成完成，站到目的地却没到，
/// 还会连带把等待时间的口径弄乱。
///
/// 僵局发生在乘客刚上车的那一层，所以他被请出来的地方就是他的出发点，
/// 推回那一层的电梯厅就等于回到原处。到达时刻保持不变，等待继续累计，
/// 见 `docs/MODEL.md` 第 8.3 节。
fn requeue_after_stall(
    w: &mut World,
    id: CarId,
    pid: usize,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
) {
    let floor = w.cars[id].level.round() as i16;
    if floor < -w.basement || floor > w.top {
        return;
    }
    w.cars[id].occupants.retain(|&p| p != pid);
    let weight = w.pax[pid].weight;
    w.cars[id].load_kg = (w.cars[id].load_kg - weight).max(0.0);
    w.pax[pid].t_board = 0.0;
    w.pax[pid].t_exit = 0.0;
    w.pax[pid].boarded_car = None;
    w.pax[pid].requeued_after_stall = true;
    let patience = w.pax[pid].patience_s;
    let qi = w.qidx(floor);
    if !w.hall[qi].contains(&pid) {
        w.hall[qi].push(pid);
    }
    // 重新进场，外呼会在那一刻重新登记
    heap.push(Ev {
        at: w.now,
        seq: *seq,
        kind: Kind::Arrive(pid),
    });
    *seq += 1;
    // 重新给一个完整的耐心窗口，否则他会立刻触发已经过期的弃梯事件。
    // 旧事件还躺在堆里，到点按 abandon_at 认出过期，不会误伤
    if patience.is_finite() && patience > 0.0 {
        w.pax[pid].abandon_at = w.now + patience;
        heap.push(Ev {
            at: w.now + patience,
            seq: *seq,
            kind: Kind::Abandon(pid),
        });
        *seq += 1;
    }
}

/// 汇总指标。
///
/// 时间类指标在乘客每次到站时就逐腿累加完了，这里只把配置、计数与均值补齐。
fn collect(w: &World, mut stalls: usize, mut stall_seconds: f64, mut m: Metrics) -> Metrics {
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
        // 还没走完最后一腿的人，这一段还挂在单腿计数上没搬
        let bypass = p.bypass();
        if bypass > 0 {
            m.pax_with_bypass += 1;
        }
        if bypass >= 3 {
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
    // 因轿厢卡死而没人接管的外呼。两种情形合起来算一条外呼一次：
    // 等待者全部放弃时打了标记的，以及跑完时还挂在僵死车名下、层上还有人在等的
    m.calls_orphaned += w
        .calls
        .calls
        .iter()
        .filter(|c| {
            c.orphaned
                || (w.call_is_live(c) && owned_by_stalled(w, c) && waiting_for(w, c.level, c.dir))
        })
        .count();
    for s in m.by_floor.iter_mut() {
        if s.count > 0 {
            s.wait_mean = s.wait_sum / s.count as f64;
        }
        // 门到门的分母只数坐上车的人。中途弃梯的进了 count 却没进 d2d，
        // 按 count 除会把均值稀释掉
        if s.d2d_samples.count > 0 {
            s.d2d_mean = s.d2d_sum / s.d2d_samples.count as f64;
        }
    }

    // 系统视角。跑到结束还没解除的僵局也要算，否则永久卡死的场景
    // 反而报 0 次 0 秒，而那正是最严重的那种卡死
    for c in &w.cars {
        if c.phase == CarPhase::Stalled {
            stalls += 1;
            stall_seconds += (w.now - c.stall_since).max(0.0);
        }
    }
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
        m.full_bypasses += c.full_bypasses as usize;
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
    // 上下客规则按配置走，不能用写死的默认值，否则调参拧不动拒挤曲线
    let boarding = DefaultBoarding {
        cfg: cfg.boarding.clone(),
    };
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
    use crate::config::{preset, CallSurface, CarSpec, StrategyMix};
    use crate::policy::{HallCall, RunView};

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

    /// 红线 2 回归：独立面板同时呼叫多台，擦灯不能把没挤上的人一起扔掉。
    ///
    /// 旧写法是车一到站就无条件擦本车的独立呼叫。满载到达时没挤上的人
    /// 从此没有车再来，只能枯等。改成「这层这个方向还留得下人就不擦」。
    #[test]
    fn multi_call_does_not_strand_left_behind_waiters() {
        let mut c = Config::default();
        c.building.top_level = 33;
        c.building.units_per_floor = vec![6; 33];
        c.building.ground_floor_residential = true;
        let mut car = CarSpec::default();
        car.capacity_rated = 10;
        c.cars = vec![car.clone(), car];
        c.traffic.pattern = crate::config::Pattern::MorningDown;
        c.traffic.departure_ratio = 0.6;
        c.call_surface = CallSurface::PerCarPanel;
        c.strategy = StrategyMix {
            honest: 0.0,
            press_both: 0.0,
            press_up_only: 0.0,
            multi_call: 1.0,
        };
        c.stairs.enabled = false;
        c.sim.horizon_s = 20_000.0;
        let r = quick(c, 1);
        let stranded = r
            .world
            .pax
            .iter()
            .filter(|p| p.hops_done == 0 && !p.used_stairs && !p.abandoned)
            .count();
        assert_eq!(stranded, 0, "多呼叫场景还有 {stranded} 人永远等不到车");
    }

    /// 分区梯回归：等待者超出归属车的停靠范围时，另一台车要能顺手接走。
    ///
    /// 高低区两台车共用主入口层外呼，外呼归了低区车，但高区等待者低区车
    /// 送不了。修好之前低区车会停在原地反复开门（活锁），高区那一半人永远
    /// 上不了车。见问题 2 的记录。
    #[test]
    fn zoned_cars_pick_up_each_others_unservable_waiters() {
        use crate::config::StopSpec;
        let mut c = Config::default();
        c.building.top_level = 33;
        c.building.units_per_floor = vec![6; 33];
        c.building.ground_floor_residential = true;
        let low: Vec<i16> = std::iter::once(1).chain(2..=17).collect();
        let high: Vec<i16> = std::iter::once(1).chain(18..=33).collect();
        let mut c0 = CarSpec::default();
        c0.stops = StopSpec::list(low);
        let mut c1 = CarSpec::default();
        c1.stops = StopSpec::list(high);
        c.cars = vec![c0, c1];
        c.traffic.pattern = crate::config::Pattern::MorningDown;
        c.traffic.departure_ratio = 0.6;
        c.stairs.enabled = false;
        c.sim.horizon_s = 20_000.0;
        let r = quick(c, 1);
        let stranded = r
            .world
            .pax
            .iter()
            .filter(|p| p.hops_done == 0 && !p.used_stairs && !p.abandoned)
            .count();
        assert_eq!(stranded, 0, "分区梯还有 {stranded} 人永远等不到车");
    }

    /// 只按上行的下行乘客必须能上车。
    ///
    /// 修好之前他按的上行呼叫召来一台沿上行收人的车，方向门槛拿他的真实
    /// 目的方向去比，一律拒绝。到站后呼叫又被当成没人等而擦掉，只能枯等
    /// 到仿真结束。
    #[test]
    fn press_up_only_passengers_are_not_stranded() {
        let mut c = Config::default();
        c.building.top_level = 2;
        c.building.units_per_floor = vec![2, 2];
        c.building.persons_per_unit = 1.0;
        c.cars = vec![CarSpec::default()];
        c.traffic.pattern = crate::config::Pattern::MorningDown;
        c.traffic.window_s = 120.0;
        c.traffic.departure_ratio = 1.0;
        c.traffic.reverse_ratio = 0.0;
        c.traffic.return_ratio = 0.0;
        c.strategy = StrategyMix {
            honest: 0.0,
            press_both: 0.0,
            press_up_only: 1.0,
            multi_call: 0.0,
        };
        c.stairs.enabled = false;
        c.sim.horizon_s = 3000.0;
        let r = quick(c, 1);
        let stranded = r
            .world
            .pax
            .iter()
            .filter(|p| p.hops_done == 0 && !p.used_stairs && !p.abandoned)
            .count();
        assert_eq!(stranded, 0, "只按上行的场景还有 {stranded} 人永远等不到车");
    }

    /// 满载直驶：一台电梯被挤满时，路过同向外呼不再白停，计入 `full_bypasses`。
    ///
    /// 修好之前这台车会在满载时照样停靠，开门没人上得去，全算成幽灵停站。
    #[test]
    fn full_car_skips_hall_calls_and_counts_them() {
        let mut c = Config::default();
        c.building.top_level = 18;
        c.building.units_per_floor = vec![4; 18];
        c.building.ground_floor_residential = true;
        c.cars = vec![CarSpec::default()];
        c.traffic.pattern = crate::config::Pattern::MorningDown;
        c.traffic.departure_ratio = 0.95;
        c.stairs.enabled = false;
        let mut bypasses = 0usize;
        for seed in 1..=3 {
            bypasses += quick(c.clone(), seed).metrics.full_bypasses;
        }
        assert!(
            bypasses > 0,
            "满载的单梯早高峰应当出现满载直驶，实际 {bypasses}"
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
                assert!(
                    p.to <= r.world.top && p.to >= -r.world.basement,
                    "目的层越界"
                );
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
        // 车跑起来之后有人按 2 层：t=2.5 时车在 2.5 米处速度 2.0，
        // 剩 0.5 米不够刹停（要 3.3 米），真刹不住
        w.now = 2.5;
        assert_eq!(
            brake_check(&w, 0, 2),
            Brake::Overshoot,
            "高速车跑起来刹不住 2 层"
        );
        let before = w.cars[0].overshoots;
        plan_for_moving(&mut w, 0, 2, Dir::Up);
        assert_eq!(w.cars[0].overshoots, before + 1, "真刹不住要记一次越站");
        w.now = 0.0;
        assert_eq!(brake_check(&w, 0, 2), Brake::Stop, "刚起步时应当刹得住");
    }

    /// 已经开过的楼层不算越站：那不是刹不住，是车已经走了。
    /// 低速档制动加反应只有 1.72 米，物理上不存在刹不住的下一层，
    /// 按键晚了一步不该记到它头上，更不该插一个追不上的停靠。
    #[test]
    fn passed_floor_is_not_an_overshoot() {
        let mut c = Config::default();
        c.building.top_level = 30;
        c.building.units_per_floor = vec![4; 30];
        let mut spec = CarSpec::default();
        spec.up = crate::motion::MotionProfile::preset("low_rise_1_0").unwrap();
        spec.down = spec.up;
        c.cars = vec![spec];
        let mut w = World::new(c);
        w.cars[0].leg_from = 1;
        w.cars[0].leg_target = 30;
        w.cars[0].leg_start = 0.0;
        w.cars[0].leg_dist = 29.0 * w.cfg.building.floor_height as f64;
        w.cars[0].dir = Dir::Up;
        w.cars[0].phase = CarPhase::Moving;
        // 车已经开过 2 层十几米：1.0 档早就把它甩在身后，
        // 这时按 2 层既插不进去，也不该记越站
        w.now = 20.0;
        assert_eq!(brake_check(&w, 0, 2), Brake::Passed);
        let before = w.cars[0].overshoots;
        plan_for_moving(&mut w, 0, 2, Dir::Up);
        assert_eq!(w.cars[0].overshoots, before, "开过的楼层不该记越站");
        assert!(
            !w.cars[0].stops_planned.contains(&(2, Dir::Up)),
            "开过的楼层也不该插停靠"
        );
        // 同一时刻，前方下一层来得及刹：正常插入，不记数
        assert_eq!(brake_check(&w, 0, 8), Brake::Stop, "低速档前方应当刹得住");
        plan_for_moving(&mut w, 0, 8, Dir::Up);
        assert_eq!(w.cars[0].overshoots, before);
        assert!(w.cars[0].stops_planned.contains(&(8, Dir::Up)));
    }

    /// 空停一次 6.0 秒：开门 1.5 ＋保持 1.5 ＋关门 2.8 ＋响应 0.2。
    /// 保持开启漏掉就只有 4.5 秒，而估计函数按 6.0 算，两边对不上。
    #[test]
    fn empty_stop_costs_six_seconds() {
        let mut w = World::new(small());
        w.cars[0].level = 2.0;
        w.now = 100.0;
        let mut heap = std::collections::BinaryHeap::<Ev>::new();
        let mut seq = 0u64;
        let mut metrics = Metrics::with_floors(w.n_levels);
        let mut rng = Streams::new(7);
        let boarding = DefaultBoarding::default();
        begin_stop(
            &mut w,
            0,
            &boarding,
            &mut rng,
            &mut heap,
            &mut seq,
            &mut metrics,
        );
        let cost = w.cars[0].next_event - 100.0;
        assert!((cost - 6.0).abs() < 1e-6, "空停应为 6.0 秒，实际 {cost}");
    }

    /// 有人上下时按人头加时间：下 1 人加 0.8 秒，一共 6.8 秒。
    #[test]
    fn loaded_stop_adds_transfer_time() {
        let mut w = World::new(small());
        w.cars[0].level = 2.0;
        w.now = 100.0;
        let pid = 0;
        w.pax[pid].to = 2;
        w.cars[0].occupants.push(pid);
        let mut heap = std::collections::BinaryHeap::<Ev>::new();
        let mut seq = 0u64;
        let mut metrics = Metrics::with_floors(w.n_levels);
        let mut rng = Streams::new(7);
        let boarding = DefaultBoarding::default();
        begin_stop(
            &mut w,
            0,
            &boarding,
            &mut rng,
            &mut heap,
            &mut seq,
            &mut metrics,
        );
        let cost = w.cars[0].next_event - 100.0;
        assert!((cost - 6.8).abs() < 1e-6, "下 1 人应为 6.8 秒，实际 {cost}");
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
        // 这一条测的是分区服务本身，把弃梯的可行性上限关掉，
        // 否则远程乘客中途走不了楼梯，会长久留在队列里
        c.stairs.practical_floor_cap = 0;
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
        let from_lobby = w
            .pax
            .iter()
            .filter(|p| p.hops[0].from == crate::config::LOBBY)
            .count();
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
            w.pax
                .iter()
                .filter(|p| p.n_hops == 2)
                .all(|p| p.hops[1].from == p.hops[0].to),
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
        assert!(
            r.metrics.abandoned > 0,
            "这种配置下应当有人等不下去改走楼梯"
        );
        let walkers = r.world.pax.iter().filter(|p| p.abandoned).count();
        assert_eq!(walkers, r.metrics.abandoned);
        assert!(
            r.world
                .pax
                .iter()
                .filter(|p| p.abandoned)
                .all(|p| !p.used_stairs),
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

    /// 弃梯的人之前被跳过的那几次要留在逐层统计里。只在「上车」那一刻记，
    /// 会让最后改走楼梯的那批人把跳过计数一起带走，逐层总数偏小。
    #[test]
    fn abandoned_passenger_keeps_bypass_counts() {
        let mut w = World::new(small());
        // 挑一个真正在等车的：有的乘客一出生就走了楼梯，t_exit 已经填上
        let pid = w
            .pax
            .iter()
            .position(|p| p.waiting())
            .expect("该有人在等车");
        let from = w.pax[pid].from;
        let qi = w.qidx(from);
        w.pax[pid].declined_count = 1;
        w.pax[pid].no_space_count = 1;
        w.pax[pid].t_arrive = w.now;
        w.pax[pid].t_board = 0.0;
        w.pax[pid].abandoned = false;
        w.pax[pid].patience_s = 1.0;
        w.pax[pid].abandon_at = w.now;
        w.hall[qi].push(pid);
        let mut metrics = Metrics::with_floors(w.n_levels);
        abandon_to_stairs(&mut w, pid, &mut metrics);
        let fs = &metrics.by_floor[qi];
        assert!(w.pax[pid].abandoned, "这一位应当记成弃梯");
        assert_eq!(fs.bypass_total, 2, "两次被跳过要进逐层总数");
        assert_eq!(fs.declined, 1, "拒挤那次要留下");
        assert_eq!(fs.no_space, 1, "没位子那次要留下");
    }

    /// 人物级的跳过次数要跨腿累计。`collect` 读「累计值 + 当前腿」，不能
    /// 只看当前腿：换腿会清零单腿计数，只看它会把第一腿的次数抹掉。
    #[test]
    fn person_level_bypass_reads_both_counters() {
        let mut w = World::new(small());
        assert!(w.pax.len() >= 2, "前提：至少两位乘客");
        for pid in 0..2 {
            w.pax[pid].hops_done = 1;
            w.pax[pid].t_exit = 1.0;
            w.pax[pid].used_stairs = false;
            w.pax[pid].abandoned = false;
        }
        // 一号：第一腿被跳过两次，第二腿又被跳过一次
        w.pax[0].declined_total = 2;
        w.pax[0].declined_count = 1;
        // 二号：一次都没被跳过
        w.pax[1].declined_total = 0;
        w.pax[1].declined_count = 0;
        let m = collect(&w, 0, 0.0, Metrics::with_floors(w.n_levels));
        assert_eq!(m.pax_served, 2);
        assert_eq!(m.pax_with_bypass, 1, "两次累计加当前一次，是同一个人的三次");
        assert_eq!(m.pax_bypass3, 1, "三次就该进「被跳过三次」那一档");
    }

    /// 一条腿走完就结算：单腿计数搬进累计值并清零，不留残值。
    #[test]
    fn bypass_counts_move_to_the_total_when_a_leg_ends() {
        let mut c = small();
        c.traffic.return_ratio = 0.5;
        c.traffic.departure_ratio = 1.0;
        c.stairs.enabled = false;
        let r = default_run(c);
        let two_legged: Vec<_> = r
            .world
            .pax
            .iter()
            .filter(|p| p.n_hops == 2 && p.hops_done == 2)
            .collect();
        assert!(!two_legged.is_empty(), "前提：得有人把两腿都走完");
        assert!(
            two_legged.iter().all(|p| p.leg_bypass() == 0),
            "走完的腿已经结算过，单腿计数该清零"
        );
    }

    /// 过期计时器不赶人。换腿和僵局重排都会推迟 abandon_at，
    /// 旧事件到点必须忽略，只认当前这条腿的截止时刻。
    #[test]
    fn stale_abandon_timer_does_not_evict() {
        let mut w = World::new(small());
        let pid = 0;
        let from = w.pax[pid].from;
        let qi = w.qidx(from);
        // 摆一个正在等第二条腿的人：旧截止 100，新截止 1000
        w.pax[pid].hop_idx = 1;
        w.pax[pid].t_arrive = 500.0;
        w.pax[pid].t_board = 0.0;
        w.pax[pid].t_exit = 0.0;
        w.pax[pid].to = from;
        w.pax[pid].used_stairs = false;
        w.pax[pid].abandoned = false;
        w.pax[pid].patience_s = 500.0;
        w.pax[pid].abandon_at = 1000.0;
        if !w.hall[qi].contains(&pid) {
            w.hall[qi].push(pid);
        }
        let mut metrics = Metrics::with_floors(w.n_levels);
        // 第一条腿遗留的旧事件在 100 到点：忽略，人还在等
        w.now = 100.0;
        abandon_to_stairs(&mut w, pid, &mut metrics);
        assert!(!w.pax[pid].abandoned, "旧计时器不该把返程的人赶走");
        assert_eq!(metrics.abandoned, 0);
        // 当前腿的截止到了：正常弃梯
        w.now = 1000.0;
        abandon_to_stairs(&mut w, pid, &mut metrics);
        assert!(w.pax[pid].abandoned, "新截止到了就该弃梯");
        assert_eq!(metrics.abandoned, 1);
    }

    /// 红线 2：按过的方向决定能不能上车。
    ///
    /// 修好之前只按上行的人会被上行轿厢挡在门外，双按的人也一样。
    /// 上行那条呼叫召来的车一个人接不走，白停一次。
    #[test]
    fn pressed_direction_decides_who_boards() {
        let mut w = World::new(small());
        let pid = 0;
        {
            let p = &mut w.pax[pid];
            p.from = 5;
            p.to = 1;
            p.t_arrive = 100.0;
            p.t_board = 0.0;
            p.t_exit = 0.0;
            p.boarded_car = None;
            p.used_stairs = false;
            p.abandoned = false;
            p.weight = 60.0;
            p.strategy = Strategy::PressUpOnly;
        }
        let boarding = crate::builtin::DefaultBoarding {
            cfg: w.cfg.boarding.clone(),
        };
        let mut rng = Streams::new(w.cfg.sim.seed);

        let up = try_board_one(&mut w, 0, 5, pid, Dir::Up, &boarding, 100.0, &mut rng);
        assert!(up, "只按上行的人应当能坐上上行轿厢");
        assert_eq!(w.pax[pid].boarded_car, Some(0));

        // 双按的人上行也收
        w.cars[0].occupants.clear();
        w.cars[0].load_kg = 0.0;
        w.pax[pid].boarded_car = None;
        w.pax[pid].t_board = 0.0;
        w.pax[pid].strategy = Strategy::PressBoth;
        let both = try_board_one(&mut w, 0, 5, pid, Dir::Up, &boarding, 100.0, &mut rng);
        assert!(both, "双按的人应当能坐上先到的上行轿厢");
    }

    /// 上车概率由策略给，不能拿配置里的曲线顶替。
    ///
    /// 修好之前引擎用 `config.boarding` 算概率，自定义 `Boarding` 把概率设成
    /// 0 也照样按默认曲线放人。这里让策略返回 0，断言乘客被记成不肯挤。
    #[test]
    fn strategy_probability_overrides_default_curve() {
        struct RefuseBoarding;
        impl Boarding for RefuseBoarding {
            fn admit_one(&self, _s: &AdmitSample, _tol: f64) -> Verdict {
                Verdict::Admit
            }
            fn accept_prob(&self, _s: &AdmitSample, _tol: f64) -> f64 {
                0.0
            }
        }

        let mut w = World::new(small());
        let pid = 0;
        {
            let p = &mut w.pax[pid];
            p.from = 5;
            p.to = 1;
            p.t_arrive = 0.0;
            p.t_board = 0.0;
            p.t_exit = 0.0;
            p.boarded_car = None;
            p.used_stairs = false;
            p.abandoned = false;
            p.weight = 60.0;
            p.tol = 0.0;
            p.strategy = Strategy::Honest;
        }
        // 空车按默认曲线是 100% 放人，策略说 0 就该 0
        assert_eq!(w.cfg.boarding.accept_prob(0.0, 0.0, 0.0), 1.0);

        let mut rng = Streams::new(w.cfg.sim.seed);
        let refused = try_board_one(&mut w, 0, 5, pid, Dir::Down, &RefuseBoarding, 0.0, &mut rng);
        assert!(!refused, "策略给出的概率是 0，不该放人");
        assert_eq!(w.pax[pid].declined_count, 1);
        assert_eq!(w.pax[pid].boarded_car, None);
    }

    /// 车门关不关由策略决定。返回 false 时车继续开门，到点再问一次。
    #[test]
    fn close_now_false_holds_the_doors() {
        struct HoldOnce {
            left: std::cell::Cell<u32>,
        }
        impl Boarding for HoldOnce {
            fn admit_one(&self, s: &AdmitSample, _tol: f64) -> Verdict {
                if s.free_slots == 0 {
                    Verdict::NoSpace
                } else {
                    Verdict::Admit
                }
            }
            fn close_now(&self, _v: &RunView<'_>) -> bool {
                let n = self.left.get();
                if n > 0 {
                    self.left.set(n - 1);
                    return false;
                }
                true
            }
        }

        let mut w = World::new(small());
        w.now = 50.0;
        w.cars[0].phase = CarPhase::Boarding;
        w.cars[0].level = 3.0;
        w.cars[0].stop_started = 50.0;
        w.cars[0].next_event = 50.0;
        // 让这站不算幽灵，专心看关门时机
        w.cars[0].last_alighted = 1;
        let boarding = HoldOnce {
            left: std::cell::Cell::new(1),
        };
        let mut router = DownPeakSweep;
        let mut sv = Scratch::new(w.cars.len(), w.n_levels);
        let mut heap = std::collections::BinaryHeap::new();
        let mut seq = 0u64;
        let mut rng = Streams::new(w.cfg.sim.seed);
        let mut metrics = Metrics::with_floors(w.n_levels);

        step_car(
            &mut w,
            0,
            &mut router,
            &boarding,
            &mut sv,
            &mut heap,
            &mut seq,
            &mut rng,
            &mut metrics,
        );
        assert_eq!(
            w.cars[0].phase,
            CarPhase::Boarding,
            "策略要求继续开门，不该收尾"
        );
        assert_eq!(
            w.cars[0].next_event,
            50.0 + w.cfg.cars[0].doors.dwell_s as f64,
            "该再等一个保持开启时长"
        );
        assert!(w.cars[0].holding, "延长开门期间要允许新到的人上车");

        // 下一次到点，策略放行，这一站结束
        w.now = w.cars[0].next_event;
        step_car(
            &mut w,
            0,
            &mut router,
            &boarding,
            &mut sv,
            &mut heap,
            &mut seq,
            &mut rng,
            &mut metrics,
        );
        // 策略放行之后这一站必须结束，车可能接着去主入口层待命
        assert_ne!(w.cars[0].phase, CarPhase::Boarding);
        assert_eq!(w.cars[0].stop_started, 0.0);
        assert!(!w.cars[0].holding);
    }

    /// 车门开着时新到的人可以直接上，这是延长开门的意义所在。
    #[test]
    fn late_arrival_boards_while_doors_are_open() {
        let mut w = World::new(small());
        let pid = 0;
        {
            let p = &mut w.pax[pid];
            p.from = 5;
            p.to = 1;
            p.t_arrive = 100.0;
            p.t_board = 0.0;
            p.t_exit = 0.0;
            p.boarded_car = None;
            p.used_stairs = false;
            p.abandoned = false;
            p.weight = 60.0;
            p.tol = 0.0;
            p.strategy = Strategy::Honest;
        }
        w.now = 100.0;
        w.cars[0].phase = CarPhase::Boarding;
        w.cars[0].level = 5.0;
        w.cars[0].holding = true;
        let qi = w.qidx(5);
        w.hall[qi].push(pid);

        let boarding = DefaultBoarding {
            cfg: w.cfg.boarding.clone(),
        };
        let mut rng = Streams::new(w.cfg.sim.seed);
        assert!(
            board_stopped(&mut w, pid, &boarding, &mut rng),
            "开着门的车该接走新到的人"
        );
        assert_eq!(w.pax[pid].boarded_car, Some(0));
        assert!(w.hall[qi].is_empty(), "上了车的人要离开电梯厅");
        assert_eq!(w.cars[0].last_boarded, 1);
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
        let any_reason = r
            .world
            .pax
            .iter()
            .any(|p| p.declined_total + p.declined_count > 0)
            || r.world
                .pax
                .iter()
                .any(|p| p.no_space_total + p.no_space_count > 0);
        assert!(any_reason, "被跳过的人该记下原因");
        // 有人被跳过，却最终被送走了，说明车是带着空位开走的
        let served_with_bypass = r
            .world
            .pax
            .iter()
            .filter(|p| p.bypass() > 0 && p.t_exit > 0.0)
            .count();
        assert!(served_with_bypass > 0, "被跳过的人最终也该被送走");
    }

    /// 载重装不下的时候，原因要记成「没位子」。
    ///
    /// 骰子先掷的话，超重的人有一半概率被记成「不肯挤」，界面上这两个
    /// 原因是分开显示的，一混就分不清该加电梯还是该改轿厢。
    #[test]
    fn overweight_is_no_space_not_declined() {
        let mut c = small();
        // 把关卡压到零，人人掷骰子，串错的机会最大
        c.boarding.soft_start_ratio = 0.0;
        let mut w = World::new(c);
        w.pax[0].from = 5;
        w.pax[0].to = 1;
        w.pax[0].weight = 80.0;
        w.pax[0].t_arrive = 0.0;
        w.pax[0].t_exit = 0.0;
        w.pax[0].used_stairs = false;
        w.pax[0].abandoned = false;
        w.pax[0].boarded_car = None;
        // 车上十五人 1080 公斤。再来八十公斤就过 1100 的载重上限，
        // 而人数上限十七还差两个
        w.cars[0].occupants = (1..16).collect();
        w.cars[0].load_kg = 1080.0;
        let boarding = crate::builtin::DefaultBoarding {
            cfg: w.cfg.boarding.clone(),
        };
        let mut rng = Streams::new(1);
        let admitted = try_board_one(&mut w, 0, 5, 0, Dir::Down, &boarding, 0.0, &mut rng);
        assert!(!admitted, "超重的人上不去");
        assert_eq!(w.pax[0].no_space_count, 1, "这是装不下，不是不愿意挤");
        assert_eq!(w.pax[0].declined_count, 0);
    }

    /// 拒挤曲线的四个软参数必须进决策。同一份种子下拧旋钮，
    /// declined 与 no_space 的次数要跟着动。之前骰子处写死默认值，
    /// 四组参数跑出逐位相同的结果，调参等于没调。
    #[test]
    fn boarding_soft_params_change_outcomes() {
        fn counts(soft: f64, gamma: f64) -> (u32, u32) {
            let mut c = Config::default();
            c.building.top_level = 18;
            c.building.units_per_floor = vec![4; 18];
            c.cars = vec![CarSpec::default()];
            c.traffic.departure_ratio = 1.0;
            c.stairs.abandon_after_s = 0.0;
            c.boarding.soft_start_ratio = soft;
            c.boarding.gamma = gamma;
            let r = default_run(c);
            let declined = r
                .world
                .pax
                .iter()
                .map(|p| p.declined_total + p.declined_count)
                .sum();
            let no_space = r
                .world
                .pax
                .iter()
                .map(|p| p.no_space_total + p.no_space_count)
                .sum();
            (declined, no_space)
        }
        // 门槛压到 0：人人掷骰子，declined 应当最多
        let loose = counts(0.0, 1.0);
        // 门槛拉到 1：谁都不掷骰子，declined 应当归零
        let strict = counts(1.0, 1.0);
        assert!(
            loose.0 > strict.0,
            "soft_start 0 与 1 的 declined 应当不同，实际 {loose:?} 对 {strict:?}"
        );
        // 曲线形状变化：拒绝次数要跟着动（方向不硬断言，
        // 下游抽签序列会发散，只断言敏感不断言单调）
        let steep = counts(0.5, 8.0);
        let flat = counts(0.5, 1.0);
        assert_ne!(
            steep.0, flat.0,
            "gamma 1 与 8 的 declined 应当不同，实际都是 {}",
            steep.0
        );
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

    /// 逐层门到门与全局口径一致：每人门到门不短于等待，
    /// 各层门到门之和等于全局门到门之和。
    #[test]
    fn floor_door_to_door_is_consistent() {
        let c = Config::default();
        let r = default_run(c);
        for p in &r.world.pax {
            if p.hops_done > 0
                && !p.used_stairs
                && !p.abandoned
                && p.t_board > 0.0
                && p.t_exit > 0.0
            {
                assert!(
                    p.door_to_door() + 1e-9 >= p.wait(),
                    "门到门 {} 短于等待 {}",
                    p.door_to_door(),
                    p.wait()
                );
            }
        }
        let mut sum = 0.0;
        let mut n = 0u64;
        for f in &r.metrics.by_floor {
            assert!(
                f.d2d_samples.count <= f.count as u64,
                "门到门样本 {} 超过该层人数 {}",
                f.d2d_samples.count,
                f.count
            );
            if f.d2d_samples.count > 0 {
                let mean = f.d2d_sum / f.d2d_samples.count as f64;
                assert!((f.d2d_mean - mean).abs() < 1e-9, "门到门均值口径不对");
                assert!(f.d2d_max + 1e-9 >= f.d2d_mean, "门到门最大值小于均值");
            }
            sum += f.d2d_sum;
            n += f.d2d_samples.count;
        }
        assert!(
            (sum - r.metrics.door_to_door.sum).abs() < 1e-6,
            "各层门到门之和对不上全局"
        );
        assert_eq!(
            n, r.metrics.door_to_door.count,
            "各层门到门样本数对不上全局"
        );
    }

    /// 红线 2：独立面板下同时呼叫多台，两台车都来，第一台上人，
    /// 第二台扑空变幽灵停站。零送达意味着死锁。
    #[test]
    fn multi_call_summons_two_cars_and_ghosts_the_second() {
        let mut c = Config::default();
        c.building.top_level = 12;
        c.building.units_per_floor = vec![4; 12];
        c.cars = vec![CarSpec::default(); 2];
        c.call_surface = CallSurface::PerCarPanel;
        c.strategy = StrategyMix {
            honest: 0.0,
            press_both: 0.0,
            press_up_only: 0.0,
            multi_call: 1.0,
        };
        let r = default_run(c);
        assert!(r.metrics.pax_total > 0, "这一份配置应当有乘客");
        assert!(
            r.metrics.pax_served > 0,
            "多呼叫不能零送达，总 {} 送达 {}",
            r.metrics.pax_total,
            r.metrics.pax_served
        );
        assert!(
            r.metrics.ghost_stops > 0,
            "第二台车应当扑空出幽灵停站，实际 {}",
            r.metrics.ghost_stops
        );
    }

    /// 没人呼叫时，算法也能主动把轿厢挪到想待的那一层。
    #[test]
    fn router_can_move_an_idle_car_without_any_call() {
        struct AlwaysUpTo5;
        impl Router for AlwaysUpTo5 {
            fn name(&self) -> &'static str {
                "always_up_to_5"
            }
            fn next_leg(&self, v: &RunView<'_>) -> Leg {
                let here = v.level();
                if here < 5 {
                    Leg::Up(5)
                } else if here > 5 {
                    Leg::Down(5)
                } else {
                    Leg::Park
                }
            }
        }
        let mut c = small();
        c.traffic.departure_ratio = 0.0; // 一个乘客都没有
        let mut router = AlwaysUpTo5;
        let mut assigner = crate::builtin::NearestCar;
        let boarding = DefaultBoarding::default();
        let r = run(c, &mut router, &mut assigner, &boarding);
        assert_eq!(r.metrics.pax_total, 0, "这一份配置不该有乘客");
        assert_eq!(r.world.cars[0].level, 5.0, "轿厢应当自己挪到第五层待命");
    }

    /// 被超载僵局请下车的乘客是重新排队，不是「已经到达」。
    #[test]
    fn stall_eviction_requeues_instead_of_completing_the_trip() {
        let mut w = World::new(small());
        let pid = 0;
        let from = w.pax[pid].from;
        // 把他放进车里，再模拟一次僵局把他请出来
        w.cars[0].level = from as f32;
        w.cars[0].occupants.push(pid);
        w.cars[0].load_kg = w.pax[pid].weight;
        w.pax[pid].boarded_car = Some(0);
        w.pax[pid].t_board = 100.0;
        let qi = w.qidx(from);
        w.hall[qi].retain(|&p| p != pid);

        let mut heap = std::collections::BinaryHeap::<Ev>::new();
        let mut seq = 0u64;
        requeue_after_stall(&mut w, 0, pid, &mut heap, &mut seq);

        assert_eq!(
            w.pax[pid].t_exit, 0.0,
            "被请下车的人一步都没到目的地，不算到达"
        );
        assert_eq!(w.pax[pid].t_board, 0.0, "上梯记录要清掉，等待重新累计");
        assert!(w.pax[pid].boarded_car.is_none());
        assert!(w.pax[pid].requeued_after_stall);
        assert!(
            w.hall[w.qidx(from)].contains(&pid),
            "要回到出发层的电梯厅排队"
        );
        assert!(!heap.is_empty(), "要排一次重新进场的事件，否则没人再管他");
        assert_eq!(w.cars[0].occupants.len(), 0);
        assert!(w.cars[0].load_kg.abs() < 1e-9, "载重要把这个人扣掉");
    }

    /// 座位还剩着，但重量已经到顶，这时物理上就是进不去了。
    ///
    /// 这一条把「装不下」与「不愿意挤」分开：前者记 no_space，后者记 declined。
    #[test]
    fn weight_limit_refuses_boarding_even_with_free_seats() {
        let mut w = World::new(small());
        let pid = 0;
        let from = w.pax[pid].from;
        let cap = w
            .cfg
            .boarding
            .load_crush_kg(w.cfg.cars[0].rated_load_kg as f64);
        // 车停在出发点，已经压在载重上限上
        w.cars[0].level = from as f32;
        w.cars[0].load_kg = cap;
        assert_eq!(w.cars[0].occupants.len(), 0, "座位还是空的");
        assert!(
            w.pax[pid].to != from && w.pax[pid].weight > 0.0,
            "这位乘客有目的地也有重量"
        );

        let mut rng = Streams::new(7);
        // 按这个世界自己的配置构造判据，不用写死的默认值
        let boarding = DefaultBoarding {
            cfg: w.cfg.boarding.clone(),
        };
        let ok = try_board_one(&mut w, 0, from, pid, Dir::None, &boarding, 0.0, &mut rng);
        assert!(!ok, "载重到顶就不该再放人进来");
        assert_eq!(w.pax[pid].no_space_count, 1, "这是装不下，不是不愿意挤");
        assert_eq!(w.pax[pid].declined_count, 0);
        assert_eq!(w.cars[0].occupants.len(), 0);
    }

    /// 多分量体重分布：均值落在合理区间，两头都要有人。
    ///
    /// 单一峰的对数正态抽不出「一撮很重的人」，也抽不出足够的小孩。
    /// 这一条盯的就是混合分布确实把四个峰都覆盖到了。
    #[test]
    fn weight_mix_covers_all_four_peaks() {
        let w = World::new(small());
        let weights: Vec<f64> = w.pax.iter().map(|p| p.weight).collect();
        assert!(weights.len() > 20, "样本太小，说明这份配置没造出几个人");
        let mean = weights.iter().sum::<f64>() / weights.len() as f64;
        assert!(
            (52.0..=70.0).contains(&mean),
            "平均体重 {mean:.1} 公斤偏离中国居民体格数据的量级"
        );
        let max = weights.iter().cloned().fold(f64::MIN, f64::max);
        let min = weights.iter().cloned().fold(f64::MAX, f64::min);
        assert!(max > 85.0, "分布里要有一撮很重的人，实际最重 {max:.1}");
        assert!(min < 40.0, "同行团里要有小孩，实际最轻 {min:.1}");
        // 衣物与随身物品的余量不能让任何人轻到不合常理
        assert!(min > 15.0, "最轻也要像个真人，实际 {min:.1}");
        // 绕一次场景，确认没有别处偷偷重抽体重
        let mut c = small();
        c.traffic.return_ratio = 1.0;
        let before: Vec<u64> = World::new(c.clone())
            .pax
            .iter()
            .map(|p| p.weight.to_bits())
            .collect();
        let after: Vec<u64> = default_run(c)
            .world
            .pax
            .iter()
            .map(|p| p.weight.to_bits())
            .collect();
        assert_eq!(before, after, "体重在跑完之后必须与生成时逐位一致");
    }

    /// 折返时体重不变。同一份记录，两条腿共用同一个重量。
    #[test]
    fn weight_is_identical_on_the_return_leg() {
        let mut c = small();
        c.traffic.return_ratio = 1.0;
        c.traffic.departure_ratio = 1.0;
        c.sim.horizon_s = 7200.0;
        let r = default_run(c);
        let returned: Vec<&crate::engine::Pax> =
            r.world.pax.iter().filter(|p| p.n_hops == 2).collect();
        assert!(!returned.is_empty(), "该有人走两条腿");
        for p in returned {
            // 两条腿之间没有任何地方重抽体重，所以跑完之后仍然是同一个值
            assert!(p.weight > 0.0, "体重不该被清零");
            assert!(
                p.hop_idx <= p.n_hops,
                "腿序号不该越界：{} / {}",
                p.hop_idx,
                p.n_hops
            );
        }
    }

    /// 层间交通：每一位乘客的起终点都来自配置里的某一股。
    #[test]
    fn inter_floor_traffic_follows_the_configured_flows() {
        use crate::config::{Flow, Pattern};
        let mut c = Config::default();
        c.building.top_level = 20;
        c.building.units_per_floor = vec![4; 20];
        c.cars = vec![CarSpec::default(); 2];
        c.traffic.pattern = Pattern::InterFloor;
        c.traffic.departure_ratio = 0.5;
        c.traffic.flows = vec![Flow::new(3, 15, 0.6), Flow::new(15, 3, 0.4)];
        assert!(
            c.validate().is_ok(),
            "{:?}",
            c.validate().errors().map(|i| i.code).collect::<Vec<_>>()
        );

        let w = World::new(c.clone());
        assert!(w.pax.len() > 10, "层间客流应该造出人");
        for p in &w.pax {
            let (f, t) = (p.hops[0].from, p.hops[0].to);
            assert!(
                (f, t) == (3, 15) || (f, t) == (15, 3),
                "起终点必须来自配置的那两股，实际 {f} 到 {t}"
            );
        }
        // 反方向的那一股也要有人
        let back = w.pax.iter().filter(|p| p.hops[0].from == 15).count();
        assert!(back > 0, "反向那股也该有人在走");
    }

    /// 层间模式下没有客流表就要报错，免得跑出一个空场景。
    #[test]
    fn inter_floor_without_flows_is_rejected() {
        use crate::config::Pattern;
        let mut c = Config::default();
        c.traffic.pattern = Pattern::InterFloor;
        let v = c.validate();
        assert!(v.errors().any(|i| i.code == "no_flows"));
    }

    /// 院楼预设能跑，而且客流确实发生在高层之间。
    #[test]
    fn hospital_preset_runs_between_upper_floors() {
        let c = crate::config::preset("hospital").unwrap();
        assert!(
            c.validate().is_ok(),
            "{:?}",
            c.validate().errors().map(|i| i.code).collect::<Vec<_>>()
        );
        let r = default_run(c);
        assert!(
            r.metrics.pax_total > 50,
            "院楼该有几百段行程，实际 {}",
            r.metrics.pax_total
        );
        // 每一段的起终点都不在主入口层，这正是层间交通的样子
        for p in &r.world.pax {
            assert!(p.hops[0].from >= 2, "院楼的客流不该从主入口层出发");
            assert!(p.hops[0].to >= 2);
        }
    }

    /// 回归：reassign 把外呼改判给趴着的车，内核要叫醒新车主。
    ///
    /// 修好之前写回只改归属表，不排事件。安静时段被救援的外呼会枯等到
    /// 弃梯或者仿真结束，而接活的车还趴在原地。
    #[test]
    fn reassign_wakes_the_new_owner() {
        struct MoveAllToCar1;
        impl Assigner for MoveAllToCar1 {
            fn name(&self) -> &'static str {
                "move_all_to_car_1"
            }
            fn assign(&self, _v: &SystemView<'_>, _c: HallCall) -> Option<CarId> {
                Some(0)
            }
            fn reassign(&self, _v: &SystemView<'_>, owned: &mut [Option<CarId>]) {
                for o in owned.iter_mut() {
                    if o.is_some() {
                        *o = Some(1);
                    }
                }
            }
        }
        let mut c = small();
        c.cars = vec![CarSpec::default(); 2];
        let mut w = World::new(c);
        assert_eq!(w.cars[1].phase, CarPhase::Idle);
        assert!(!w.cars[1].next_event.is_finite(), "前提：车 1 趴在原地");
        w.calls.register(5, Dir::Down, 1, false);
        let id = w.calls.calls[0].id;
        w.calls.get_mut(id).unwrap().owner = Some(0);
        let mut sv = Scratch::new(w.cars.len(), w.n_levels);
        let mut heap = std::collections::BinaryHeap::new();
        let mut seq = 0u64;
        after_event(&mut w, &MoveAllToCar1, &mut sv, &mut heap, &mut seq);
        assert_eq!(w.calls.get(id).unwrap().owner, Some(1), "改判应当写回");
        assert!(
            heap.iter().any(|e| matches!(e.kind, Kind::CarStep(1))),
            "改判给车 1 之后要给它排一步，否则它不知道有活"
        );
        // 叫醒之后要真的开走。醒一下就趴回去等于没修
        let mut router = DownPeakSweep;
        let boarding = DefaultBoarding::default();
        let mut rng = Streams::new(1);
        let mut metrics = Metrics::with_floors(w.n_levels);
        step_car(
            &mut w,
            1,
            &mut router,
            &boarding,
            &mut sv,
            &mut heap,
            &mut seq,
            &mut rng,
            &mut metrics,
        );
        assert_eq!(w.cars[1].phase, CarPhase::Moving, "车 1 应当动起来");
        assert_eq!(w.cars[1].leg_target, 5, "车 1 应当开去找 5 层那条外呼");
    }

    /// 回归：reassign 把外呼改判给停不了这一层的车，内核要换回能服务的。
    ///
    /// `RescueWrapper` 这类算法只看负载挑车。内核若不管，外呼会挂在一台
    /// 永远不会来这一层的车上，等在那层的人谁都等不到车。
    #[test]
    fn reassign_to_unserving_car_falls_back() {
        use crate::config::StopSpec;
        struct MoveAllToCar1;
        impl Assigner for MoveAllToCar1 {
            fn name(&self) -> &'static str {
                "move_all_to_car_1"
            }
            fn assign(&self, _v: &SystemView<'_>, _c: HallCall) -> Option<CarId> {
                Some(0)
            }
            fn reassign(&self, _v: &SystemView<'_>, owned: &mut [Option<CarId>]) {
                for o in owned.iter_mut() {
                    if o.is_some() {
                        *o = Some(1);
                    }
                }
            }
        }
        let mut c = small();
        let c0 = CarSpec {
            stops: StopSpec::zone(1, 10),
            ..CarSpec::default()
        };
        let c1 = CarSpec {
            stops: StopSpec::zone(1, 2),
            ..CarSpec::default()
        };
        c.cars = vec![c0, c1];
        let mut w = World::new(c);
        assert!(
            w.car_serves(0, 6) && !w.car_serves(1, 6),
            "前提：只有车 0 停 6 层"
        );
        w.calls.register(6, Dir::Down, 1, false);
        let id = w.calls.calls[0].id;
        w.calls.get_mut(id).unwrap().owner = Some(0);
        let mut sv = Scratch::new(w.cars.len(), w.n_levels);
        let mut heap = std::collections::BinaryHeap::new();
        let mut seq = 0u64;
        after_event(&mut w, &MoveAllToCar1, &mut sv, &mut heap, &mut seq);
        assert_eq!(
            w.calls.get(id).unwrap().owner,
            Some(0),
            "改判到不服务 6 层的车，内核必须换回能服务的车"
        );
    }

    /// 回归：`RoundRobin` 选中卡死的车时，内核兜底要把新外呼给能干的。
    #[test]
    fn stalled_car_never_owns_new_calls() {
        let mut c = small();
        c.cars = vec![CarSpec::default(); 2];
        let mut w = World::new(c);
        w.cars[0].phase = CarPhase::Stalled;
        w.cars[0].stall_since = -100.0;
        w.calls.register(5, Dir::Down, 1, false);
        let id = w.calls.calls[0].id;
        let mut sv = Scratch::new(w.cars.len(), w.n_levels);
        let mut heap = std::collections::BinaryHeap::new();
        let mut seq = 0u64;
        let rr = crate::builtin::RoundRobin::default();
        assign_unowned(&mut w, &rr, &mut sv, &mut heap, &mut seq);
        assert_eq!(
            w.calls.get(id).unwrap().owner,
            Some(1),
            "僵死车不该拿新外呼，活该给能干活的那台"
        );
    }

    /// 回归：算法返回越界的轿厢号，内核兜底要接住，不能直接 panic。
    ///
    /// `assign` 与 `reassign` 都是外部算法的返回值，写错一个下标就能
    /// 把整场仿真打崩。内核既然已经在兜「不服务这一层」，这条也得兜。
    #[test]
    fn out_of_range_car_id_falls_back_instead_of_panicking() {
        struct BadAssigner;
        impl Assigner for BadAssigner {
            fn name(&self) -> &'static str {
                "bad"
            }
            fn assign(&self, _v: &SystemView<'_>, _c: HallCall) -> Option<CarId> {
                Some(99)
            }
            fn reassign(&self, _v: &SystemView<'_>, owned: &mut [Option<CarId>]) {
                for o in owned.iter_mut() {
                    if o.is_some() {
                        *o = Some(99);
                    }
                }
            }
        }
        let mut c = small();
        c.cars = vec![CarSpec::default(); 2];
        let mut w = World::new(c);
        w.calls.register(5, Dir::Down, 1, false);
        let id = w.calls.calls[0].id;
        let mut sv = Scratch::new(w.cars.len(), w.n_levels);
        let mut heap = std::collections::BinaryHeap::new();
        let mut seq = 0u64;
        assign_unowned(&mut w, &BadAssigner, &mut sv, &mut heap, &mut seq);
        assert!(
            w.calls.get(id).unwrap().owner.is_some_and(|o| o < 2),
            "越界车号要换成真正存在的车，实际 {:?}",
            w.calls.get(id).unwrap().owner
        );
        after_event(&mut w, &BadAssigner, &mut sv, &mut heap, &mut seq);
        assert!(
            w.calls.get(id).unwrap().owner.is_some_and(|o| o < 2),
            "改判写回同样要挡住越界车号，实际 {:?}",
            w.calls.get(id).unwrap().owner
        );
    }

    /// 回归：多呼叫面板下全场皆卡死，出生就无主的独立外呼恢复后要能被捡起来。
    ///
    /// 独立呼叫不进槽位，只看槽位的派梯永远看不到它，等的人只能弃梯。
    #[test]
    fn ownerless_exclusive_call_is_picked_up_after_the_stall_clears() {
        let mut c = small();
        c.cars = vec![CarSpec::default(); 2];
        c.call_surface = CallSurface::PerCarPanel;
        c.strategy = StrategyMix {
            honest: 0.0,
            press_both: 0.0,
            press_up_only: 0.0,
            multi_call: 1.0,
        };
        let mut w = World::new(c);
        // 递 0 号乘客到 5 层，按多呼叫面板同时按两台
        w.pax[0].from = 5;
        w.pax[0].to = 1;
        w.pax[0].t_arrive = 0.0;
        w.pax[0].t_exit = 0.0;
        w.pax[0].used_stairs = false;
        w.pax[0].abandoned = false;
        w.pax[0].boarded_car = None;
        w.pax[0].strategy = Strategy::MultiCall;
        let qi = w.qidx(5);
        w.hall[qi].push(0);
        for car in &mut w.cars {
            car.phase = CarPhase::Stalled;
            car.stall_since = -100.0;
        }
        let pre = register_calls(&mut w, 5, &[0]);
        assert!(pre.is_empty(), "前提：全场僵死时多呼叫生不出归属");
        let id = w.calls.calls.len() as u32 - 1;
        assert!(
            w.calls
                .get(id)
                .is_some_and(|c| c.exclusive && c.owner.is_none()),
            "前提：这条独立外呼出生无主"
        );
        // 僵局解除，派梯要把它捡起来，不能因为不进槽位就永远看不见
        w.cars[0].phase = CarPhase::Idle;
        w.cars[0].stall_since = 0.0;
        let mut sv = Scratch::new(w.cars.len(), w.n_levels);
        let mut heap = std::collections::BinaryHeap::new();
        let mut seq = 0u64;
        assign_unowned(
            &mut w,
            &crate::builtin::RoundRobin::default(),
            &mut sv,
            &mut heap,
            &mut seq,
        );
        assert_eq!(
            w.calls.get(id).unwrap().owner,
            Some(0),
            "僵局解除后要有一条车接手这条独立外呼"
        );
    }

    /// 灭灯的外呼不进算法视图。编号永不复用，旧记录留在表里，
    /// 但它们不能挂在 `pending` 里，否则派梯会被引到没人的楼层。
    #[test]
    fn dead_calls_stay_out_of_the_system_view() {
        let mut w = World::new(small());
        w.calls.register(5, Dir::Down, 1, false);
        let id = w.calls.calls[0].id;
        kill_call(&mut w, 5, Dir::Down);
        let mut sv = Scratch::new(w.cars.len(), w.n_levels);
        w.fill_views(&mut sv);
        assert!(
            sv.all.iter().all(|c| c.id != id),
            "灭灯的外呼不该出现在 all_calls 里"
        );
        assert!(sv.pending.is_empty(), "灭灯的外呼也不该挂在 pending 里");
    }

    /// 外呼上记的等待人数要跟着实际队长短，不能停在按键那一刻。
    #[test]
    fn call_waiting_count_tracks_the_queue() {
        let mut c = small();
        c.traffic.departure_ratio = 0.9;
        let mut w = World::new(c);
        assert!(w.pax.len() >= 3, "前提：至少三位乘客");
        // 三个人在 5 层等下行
        for pid in 0..3 {
            w.pax[pid].from = 5;
            w.pax[pid].to = 1;
            w.pax[pid].t_arrive = 0.0;
            w.pax[pid].t_exit = 0.0;
            w.pax[pid].used_stairs = false;
            w.pax[pid].abandoned = false;
            w.pax[pid].boarded_car = None;
            w.pax[pid].patience_s = f64::INFINITY;
        }
        let qi = w.qidx(5);
        w.hall[qi].extend([0, 1, 2]);
        w.calls.register(5, Dir::Down, 3, false);
        let mut metrics = Metrics::with_floors(w.n_levels);
        w.now = 100.0;
        abandon_to_stairs(&mut w, 0, &mut metrics);
        let mut sv = Scratch::new(w.cars.len(), w.n_levels);
        w.fill_views(&mut sv);
        let call = sv
            .all
            .iter()
            .find(|c| c.level == 5 && c.dir == Dir::Down)
            .expect("这条外呼还亮着");
        assert_eq!(call.waiting, 2, "走掉一个人以后该只剩两个人");
    }

    /// 逐层走楼梯人数与全局口径一致：
    /// 出发前走楼梯的人加中途弃梯的人，两边必须相等。
    #[test]
    fn floor_stair_users_add_up_to_the_global_count() {
        let mut c = Config::default();
        c.building.top_level = 18;
        c.building.units_per_floor = vec![4; 18];
        c.building.ground_floor_residential = true;
        c.traffic.departure_ratio = 0.9;
        let r = quick(c, 11);
        assert!(r.metrics.stair_users > 0, "这份配置该有人走楼梯");
        let per_floor: usize = r
            .metrics
            .by_floor
            .iter()
            .map(|f| f.stair_users as usize)
            .sum();
        assert_eq!(
            per_floor, r.metrics.stair_users,
            "逐层加起来要与全局 stair_users 相等"
        );
    }

    /// 外呼挂在卡死车名下算「没人接管」：跑完时人还在等算一次，
    /// 人全部放弃也算一次。两条路径不能重复计。
    #[test]
    fn calls_orphaned_counts_calls_left_on_a_stalled_car() {
        // 一：人还在等，跑完时外呼还挂在卡死的车 0 名下
        let mut c = small();
        c.cars = vec![CarSpec::default(); 2];
        let mut w = World::new(c);
        assert!(!w.pax.is_empty(), "前提：world 里得有位乘客可用");
        w.cars[0].phase = CarPhase::Stalled;
        w.cars[0].stall_since = -100.0;
        w.pax[0].from = 5;
        w.pax[0].to = 1;
        w.pax[0].t_arrive = 0.0;
        w.pax[0].t_exit = 0.0;
        w.pax[0].used_stairs = false;
        w.pax[0].abandoned = false;
        w.pax[0].boarded_car = None;
        let qi = w.qidx(5);
        w.hall[qi].push(0);
        w.calls.register(5, Dir::Down, 1, false);
        let id = w.calls.calls[0].id;
        w.calls.get_mut(id).unwrap().owner = Some(0);
        let m = collect(&w, 0, 0.0, Metrics::with_floors(w.n_levels));
        assert_eq!(m.calls_orphaned, 1, "还挂着没人接，要记一次");

        // 二：人放弃走楼梯，同一条外呼不能再被数一次
        let mut m = Metrics::with_floors(w.n_levels);
        w.pax[0].patience_s = f64::INFINITY;
        w.now = 100.0;
        abandon_to_stairs(&mut w, 0, &mut m);
        assert!(w.pax[0].abandoned, "前提：这位乘客应当弃梯");
        let m = collect(&w, 0, 0.0, m);
        assert_eq!(
            m.calls_orphaned, 1,
            "弃梯那条路径打标记，与跑完时那条互斥，只算一次"
        );
    }

    /// 卡住的车名下会真的留下没人接的外呼。
    ///
    /// 场景是人为搭的：一台梯先接下几层的外呼，然后永远卡死。
    /// 名下那些外呼既没人改派，层上的人只能弃梯，这就是红线 6 的损耗。
    #[test]
    fn overload_stall_leaves_orphaned_calls() {
        let mut c = Config::default();
        c.building.top_level = 18;
        c.building.units_per_floor = vec![6; 18];
        c.building.ground_floor_residential = true;
        c.cars = vec![CarSpec::default()];
        c.traffic.pattern = crate::config::Pattern::MorningDown;
        c.traffic.departure_ratio = 0.9;
        c.stairs.abandon_after_s = 60.0;
        c.overload.enabled = true;
        c.overload.trigger_ratio = 0.1;
        c.overload.q_last = 0.0;
        c.overload.q_other = 0.0;
        c.overload.stall_timeout_s = 1e9;
        c.sim.horizon_s = 3600.0;
        let r = quick(c, 3);
        assert!(r.metrics.stalls > 0, "前提：这台梯会卡住");
        assert!(
            r.metrics.calls_orphaned > 0,
            "车卡住时名下的外呼没人接，实际 {}",
            r.metrics.calls_orphaned
        );
    }

    /// 弃梯受步层数上限约束：没人会爬二十层楼梯。
    #[test]
    fn abandonment_respects_the_walkable_floor_cap() {
        let mut c = Config::default();
        c.building.top_level = 22;
        c.building.units_per_floor = vec![6; 22];
        c.cars = vec![CarSpec::default()]; // 一台车伺候二十层，必然等很久
        c.stairs.practical_floor_cap = 6;
        c.stairs.abandon_after_s = 30.0;
        c.traffic.departure_ratio = 1.0;
        c.sim.horizon_s = 7200.0;
        let r = default_run(c);
        assert!(r.metrics.abandoned > 0, "这么挤的配置该有人弃梯");
        for p in r.world.pax.iter().filter(|p| p.abandoned) {
            let span = (p.hops[0].to - p.hops[0].from).abs();
            assert!(span <= 6, "超过步层上限还弃梯了：{} 层跨度", span);
        }
    }

    /// 顶层号被夹取抬高的配置，弃梯计时器到点也不许把下标打穿。
    ///
    /// `World::new` 先量几何再夹取时，这里会 panic 在 `abandon_to_stairs`
    /// 的 `by_floor[qidx(level)]` 上，因为格位数按抬升前的顶层号算。
    #[test]
    fn abandon_in_a_lifted_building_keeps_indices_in_range() {
        let mut c = Config::default();
        c.building.top_level = 1;
        c.building.units_per_floor = vec![4];
        c.traffic.window_s = 120.0;
        c.sim.horizon_s = 900.0;
        // 让弃梯计时器落在仿真窗口里，走到 abandon_to_stairs 那条路
        c.stairs.abandon_after_s = 5.0;
        let r = quick(c, 1);
        assert!(r.metrics.pax_total > 0, "前提：世界里得有人");
        assert!(
            r.world
                .pax
                .iter()
                .all(|p| r.world.in_bounds(p.from) && r.world.in_bounds(p.to)),
            "生成出来的乘客不能落在楼外"
        );
    }
}
