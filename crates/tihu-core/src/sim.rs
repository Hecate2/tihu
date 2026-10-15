//! 事件驱动主循环。
//!
//! 主循环的形状见 `docs/ARCHITECTURE.md` 第 3 节。这里是它真正跑起来的地方：
//! 一次 `run` 调用把世界推进到终止条件，然后交回指标。

use crate::builtin::{DefaultBoarding, DownPeakSweep};
use crate::config::{CallSurface, Config, Strategy};
use crate::engine::{Call, CarPhase, Scratch, World};
use crate::metrics::{FloorStats, Metrics};
use crate::motion::Dir;
use crate::policy::{AdmitSample, Assigner, Boarding, CarId, HallCall, Idle, Leg, Router, RunView, SystemView, Verdict};
use crate::rng::{Stream, Streams};

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

/// 事件。堆里只放这三类。
#[derive(Debug, Clone, Copy, PartialEq)]
enum Kind {
    CarStep(CarId),
    Arrival(u16),
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
    let mut rng = Streams::new(w.cfg.sim.seed ^ 0x5DEECE66D);
    let mut sv = Scratch::new(w.cars.len(), w.n_levels);

    let horizon = w.cfg.sim.horizon_s;
    let mut heap: std::collections::BinaryHeap<Ev> = std::collections::BinaryHeap::with_capacity(
        w.cars.len() + w.n_levels + 8,
    );
    let mut seq: u64 = 0;

    // 首批到达事件。每层排最早到的那一个，往后由 Arrival 分支自己续排。
    for lvl in 0..w.n_levels {
        let q = &w.queues[lvl];
        if q.arrived < q.waiting.len() {
            let first = q.waiting[0];
            heap.push(Ev { at: w.pax[first].t_arrive, seq, kind: Kind::Arrival(lvl as u16) });
            seq += 1;
        }
    }
    if std::env::var("TIHU_ARR").is_ok() {
        eprintln!("[init] n_levels={} 首批排入={} 堆容量={}", w.n_levels, seq, heap.capacity());
        for lvl in 0..w.n_levels {
            let q = &w.queues[lvl];
            eprintln!("  L{} 队列{} 首位到达={:?}", lvl, q.waiting.len(),
                q.waiting.first().map(|&p| w.pax[p].t_arrive));
        }
    }
    // 轿厢从大堂起步
    for id in 0..w.cars.len() {
        heap.push(Ev { at: 0.0, seq, kind: Kind::CarStep(id) });
        seq += 1;
    }

    let mut stalls: usize = 0;
    let mut stall_seconds = 0.0;
    let mut orphaned = 0usize;
    let mut ghost_stops = 0usize;
    let mut full_bypasses = 0usize;

    // 事件数上限。事件驱动一旦排程出错就会零时刻死循环，这里是兜底
    let max_events = 20_000_000usize;
    let mut n_events = 0usize;

    let mut max_time_seen = 0.0f64;
    while let Some(ev) = heap.pop() {
        if ev.at > horizon {
            eprintln!("[loop] 因超出仿真窗口结束. 事件={} 最后时刻={:.1} horizon={:.1}",
                n_events, max_time_seen, horizon);
            break;
        }
        max_time_seen = ev.at;
        n_events += 1;
        if n_events > max_events {
            eprintln!("[loop] 事件数超上限 {}，强制结束。最后时刻={:.1}", n_events, max_time_seen);
            break;
        }
        if std::env::var("TIHU_ARR").is_ok() && n_events % 10000 == 0 {
            eprintln!("[loop] 事件={} t={:.1} 堆={}", n_events, ev.at, heap.len());
        }
        w.now = ev.at;
        match ev.kind {
            Kind::Arrival(lvl) => {
                // 到达事件只负责「把这一层已经到点的人放进电梯厅」。
                // 判据是到达时刻是否已过，与队列游标无关。
                //
                // 早先的实现用游标做上界，结果是同层第二个人永远进不来：
                // 游标推进后上界与起点用了不同的量，两者错位。
                // 这里改成扫全场，代价是每层几十人的线性检查，可以接受
                let now = w.now;
                let qi = lvl as usize;
                if qi >= w.queues.len() {
                    continue;
                }
                let mut newly: Vec<usize> = Vec::with_capacity(8);
                {
                    let q = &w.queues[qi];
                    let n = q.arrived.min(q.waiting.len());
                    for k in n..q.waiting.len() {
                        let pid = q.waiting[k];
                        if w.pax[pid].t_arrive > now {
                            break;
                        }
                        newly.push(pid);
                    }
                }
                if std::env::var("TIHU_ARR").is_ok() {
                    eprintln!("[arr] t={:.1} qi={} 新到{} arrived={} 总{}",
                        now, qi, newly.len(), w.queues[qi].arrived, w.queues[qi].waiting.len());
                }
                if newly.is_empty() {
                    // 这一层的人还没到。排下一次唤醒
                    let q = &w.queues[qi];
                    if q.arrived < q.waiting.len() {
                        let next = q.waiting[q.arrived];
                        heap.push(Ev { at: w.pax[next].t_arrive, seq, kind: Kind::Arrival(lvl) });
                        seq += 1;
                    }
                    continue;
                }
                w.queues[qi].arrived += newly.len();
                for &pid in &newly {
                    w.pax[pid].called = true;
                }

                let level = lvl as i16 - w.basement;
                register_calls(&mut w, level, &newly, assigner, &mut sv, &mut full_bypasses);

                // 派梯与唤醒。顺序要紧：先派梯把活挂到某台车上，再叫醒它
                sync_assignments(&mut w);
                if std::env::var("TIHU_STEP").map(|v| v=="1").unwrap_or(true) {
                    dispatch_unowned(&mut w, assigner, &mut sv);
                }
                sync_assignments(&mut w);
                if std::env::var("TIHU_STEP").map(|v| v!="2").unwrap_or(true) {
                    wake_idle_cars(&mut w, &mut heap, &mut seq);
                }

                // 排下一次到达
                let q = &w.queues[qi];
                if q.arrived < q.waiting.len() {
                    let next = q.waiting[q.arrived];
                    heap.push(Ev { at: w.pax[next].t_arrive, seq, kind: Kind::Arrival(lvl) });
                    seq += 1;
                }
            }

            Kind::CarStep(id) => {
                let at = step_car(&mut w, id, router, assigner, boarding, &mut sv, &mut heap, &mut seq, &mut rng);
                let _ = at;
            }
            Kind::StallTick(id) => {
                // 超载僵局：问一次有没有人愿意退出
                let res = stall_tick(&mut w, id, &mut rng, &mut heap, &mut seq, assigner, &mut sv);
                stalls += res.stalls_here;
                stall_seconds += res.seconds;
                orphaned += res.orphaned;
                if !res.resumed {
                    let next = w.now + w.cfg.overload.stall_tick_s;
                    heap.push(Ev { at: next, seq, kind: Kind::StallTick(id) });
                    seq += 1;
                }
            }
        }

        // 清理已无人的外呼。
        // 判据是「这一层这个方向，真的还有人在等吗」。
        // 不能在「这一趟没人上车」时清，那样外呼会被提前丢掉，
        // 队列里明明还有人在等，轿厢却再也不会回来
        prune_calls(&mut w);
        sync_assignments(&mut w);
        dispatch_unowned(&mut w, assigner, &mut sv);

        // 每次事件之后问一次派梯要不要推翻先前的分配。
        // 这是死锁场景下两种派梯算法拉开差距的地方，见 docs/MODEL.md 第 8.4 节。
        if !w.cars.is_empty() {
            w.fill_views(&mut sv);
            let mut owned: Vec<Option<CarId>> = w.calls.calls.iter().map(|c| c.owner).collect();
            let before: Vec<Option<CarId>> = owned.clone();
            let svref = crate::policy::SystemView {
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
                    w.calls.calls[i].owner = *slot;
                }
            }
            // 每轮都按 owner 重建一次 assigned。
            // 放在无条件路径里，是因为新建外呼时 owner 从 None 变成 Some，
            // 那次变化不会被 before != owned 捕获。
            sync_assignments(&mut w);
        }
    }


    // 收尾：把还没出梯的乘客按当前状态结算
    let metrics = collect(&w, stalls, stall_seconds, orphaned, ghost_stops, full_bypasses);
    Run { world: w, metrics }
}

/// 这一层是否真的有人在等电梯。
fn waiting_here(w: &World, level: i16) -> bool {
    if level < -w.basement || level > w.top {
        return false;
    }
    let qi = w.qidx(level);
    let q = &w.queues[qi];
    let n = q.arrived.min(q.waiting.len());
    q.waiting[..n]
        .iter()
        .any(|&p| still_waiting_at(&w.pax[p], level))
}

/// 这位乘客此刻是不是还在等电梯。
///
/// 判据不能只看 `boarded_car`，因为人下车之后那个字段会被清空。
/// 真正的判据是「还没出梯、也不在车上」，也就是 `t_exit == 0`。
fn still_waiting_at(p: &crate::engine::Pax, level: i16) -> bool {
    if p.t_exit > 0.0 || p.used_stairs {
        return false;
    }
    // 目的是这一层的人不算「在等去别处」
    p.to != level
}

/// 把没有归属的外呼重新派一次。
///
/// 去重之后会出现「呼叫还在，但车主已经干完走了」的状态。
/// 这一轮补派，避免呼叫永远挂着没人理。
fn dispatch_unowned<A: Assigner>(w: &mut World, assigner: &A, sv: &mut Scratch) {
    let unowned: Vec<HallCall> = w
        .calls
        .calls
        .iter()
        .filter(|c| c.owner.is_none())
        .map(|c| HallCall { id: c.id, level: c.level, dir: c.dir, owner: None, waiting: c.waiting })
        .collect();
    if unowned.is_empty() {
        return;
    }
    w.fill_views(sv);
    for call in unowned {
        let view = SystemView {
            now: w.now,
            cars: &sv.briefs,
            pending: &sv.pending,
            all_calls: &sv.all,
            queue_len: &sv.q,
            top_level: w.top,
            n_cars: w.cars.len(),
        };
        if let Some(car) = assigner.assign(&view, call) {
            if let Some(c) = w.calls.get_mut(call.id) {
                c.owner = Some(car);
            }
            if let Some(c) = w.cars.get_mut(car) {
                if !c.assigned.contains(&call.id) {
                    c.assigned.push(call.id);
                }
            }
        }
    }
}

/// 清理那些已经没有任何人在等的外呼。
///
/// 判据是该 `(楼层, 方向)` 下没有任何未上梯、且还想往这个方向走的人。
/// 注意不能用「轿厢停过这一层」做判据，那会把还在排队的人漏掉。
fn prune_calls(w: &mut World) {
    // 只在「这一层这个方向真的没人等」时清掉槽位。
    //
    // 早先的实现每轮把没人等的槽位清掉、再由后来者重建，结果同层同方向
    // 堆出多条外呼，红线 1 的去重形同虚设，派梯会把同一条需求派成好几趟车。
    //
    // 只清槽位，不删 calls 里的记录。外呼的编号要保持稳定，
    // 轿厢的待办列表靠编号索引
    let mut dead: Vec<(i16, Dir)> = Vec::new();
    for slot_idx in 0..w.calls.slots.len() {
        let Some(cid) = w.calls.slots[slot_idx] else { continue };
        let Some(c) = w.calls.get(cid) else { continue };
        if c.level < -w.basement || c.level > w.top {
            dead.push((c.level, c.dir));
            continue;
        }
        let qi = w.qidx(c.level);
        let n = w.queues[qi].arrived.min(w.queues[qi].waiting.len());
        let wants = c.dir;
        // 判据要宽：这一层只要还有人没出梯，就保留呼叫槽位。
        //
        // 窄判据（「有没有人正等着往这个方向走」）会在人刚上梯、
        // 还没出梯的窗口里把槽位清掉，下一位到场的人又新建一条，
        // 于是同一层同一方向堆出多条外呼，去重形同虚设。
        let someone = w.queues[qi].waiting[..n].iter().any(|&p| {
            let pax = &w.pax[p];
            if pax.t_exit > 0.0 || pax.used_stairs {
                return false;
            }
            if pax.boarded_car.is_some() {
                // 已经在车上，不算「在等」，但目的地方向一致的话这条呼叫仍有效
                return wants == Dir::Up && pax.to > c.level || wants == Dir::Down && pax.to < c.level;
            }
            pax.to != c.level
        });
        if !someone {
            dead.push((c.level, c.dir));
        }
    }
    for (lv, dir) in dead {
        w.calls.clear_slot(lv, dir);
        // 从所有轿厢的待办里摘掉这条呼叫
        for c in &mut w.cars {
            if let Some(cid) = w.calls.slots[Call::slot_index(lv, dir, w.basement)] {
                c.assigned.retain(|&a| a != cid);
            }
        }
    }
    // 待办里指向已清槽位的呼叫也要摘掉
    let live: std::collections::HashSet<u32> = w.calls.slots.iter().flatten().copied().collect();
    for c in &mut w.cars {
        c.assigned.retain(|a| live.contains(a));
    }
}

/// 待机轿厢的重试间隔，秒。取两秒是为了不让轿厢真的闲下来太久，
/// 又不至于产生大量无效事件
const RETRY_INTERVAL: f64 = 2.0;

/// 全楼还有没有没被服务的乘客。
fn someone_still_waiting(w: &World) -> bool {
    w.queues.iter().any(|q| {
        let n = q.arrived.min(q.waiting.len());
        // 不给具体层号，这里只要「还有没有人没被服务」
        q.waiting[..n]
            .iter()
            .any(|&p| w.pax[p].t_exit == 0.0 && !w.pax[p].used_stairs)
    })
}

/// 按每条外呼的 owner 重建各轿厢的待办列表。
fn sync_assignments(w: &mut World) {
    for c in &mut w.cars {
        c.assigned.clear();
    }
    for c in &w.calls.calls {
        if let Some(owner) = c.owner {
            if owner < w.cars.len() {
                w.cars[owner].assigned.push(c.id);
            }
        }
    }
}

/// 叫醒所有停着的轿厢。
///
/// 刻意扫全表而不是维护待机名单。名单会漏登记，一旦漏掉那台车
/// 就永远不会再被叫醒，车里的人会卡死在轿厢里，而且不报错。
fn wake_idle_cars(
    w: &mut World,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
) {
    // 先在只读阶段算好每台车该不该被叫醒。
    // 之前把遍历写在 `w.cars[cid]` 的可变借用作用域里，闭包又去读 `w.pax`，
    // 借用冲突在 release 下会变成未定义行为，症状是事件流被悄悄截断
    let queue_anywhere = someone_still_waiting(w);
    let mut wake_list: Vec<CarId> = Vec::with_capacity(w.cars.len());
    for cid in 0..w.cars.len() {
        let c = &w.cars[cid];
        if c.phase != CarPhase::Idle || c.woken {
            continue;
        }
        let riders = !c.occupants.is_empty();
        let has_call = !c.assigned.is_empty();
        // 全楼都没人等且车是空的，就不必浪费事件
        if !riders && !has_call && !queue_anywhere {
            continue;
        }
        // 已经排了下一次就不用重复叫。待机时 next_event 是无穷，那正是该叫它的信号
        if !c.next_event.is_finite() {
            wake_list.push(cid);
        }
    }
    for cid in wake_list {
        // 排一个「立刻」事件。不要把 next_event 也设成 w.now，
        // 那会让 step_car 的第三段再排一次同刻事件，形成零时刻循环。
        // 两处都靠 next_event 判断是否已排，会互相打架
        heap.push(Ev { at: w.now, seq: *seq, kind: Kind::CarStep(cid) });
        *seq += 1;
    }
}

/// 登记外呼。红线 1 与红线 2 都在这个函数里。
#[allow(clippy::too_many_arguments)]
fn register_calls<A: Assigner>(
    w: &mut World,
    level: i16,
    newly: &[usize],
    assigner: &A,
    sv: &mut Scratch,
    _full_bypasses: &mut usize,
) {
    if newly.is_empty() {
        return;
    }
    let per_car = w.cfg.call_surface == CallSurface::PerCarPanel;
    // 按方向统计这一批新到的人要往哪走
    let mut up = 0usize;
    let mut down = 0usize;
    for &pid in newly {
        let p = &w.pax[pid];
        if p.to > level {
            up += 1;
        } else if p.to < level {
            down += 1;
        }
    }

    w.fill_views(sv);
    let (up, down) = (up, down);
    let mut fresh: Vec<(u32, bool)> = Vec::with_capacity(2);

    for &pid in newly {
        let p = w.pax[pid].clone();
        let wanted = if p.to > level { Dir::Up } else { Dir::Down };
        match p.strategy {
            // 红线 1 的基线：只按自己真正想去的方向
            Strategy::Honest => {
                fresh.push(w.calls.register(level, wanted, 1, false));
            }
            // 红线 2：上下都按。哪趟先来上哪趟
            Strategy::PressBoth => {
                fresh.push(w.calls.register(level, Dir::Up, 1, false));
                fresh.push(w.calls.register(level, Dir::Down, 1, false));
            }
            // 先按上行，跟车折返再下来
            Strategy::PressUpOnly => {
                fresh.push(w.calls.register(level, Dir::Up, 1, false));
            }
            // 同时呼叫多台。只有独立面板才做得到
            Strategy::MultiCall => {
                if per_car {
                    let k = 2usize;
                    let mut targets = Vec::with_capacity(k);
                    for t in 0..k {
                        targets.push(t % w.cars.len());
                    }
                    targets.sort_unstable();
                    targets.dedup();
                    for t in targets {
                        let (id, _) = w.calls.register(level, wanted, 1, false);
                        if let Some(c) = w.calls.get_mut(id) {
                            c.exclusive = true;
                            c.owner = Some(t);
                        }
                    }
                } else {
                    fresh.push(w.calls.register(level, wanted, 1, false));
                }
            }
        }
    }
    let _ = (up, down);

    // 派梯。对新建的外呼要派，对「存在但还没人认领的」也要派。
    // 后者是常态：同层同方向的呼叫被去重之后，后来的人不会新建外呼，
    // 若原先那台车已经把活干完，这条呼叫会一直没人负责
    w.fill_views(sv);
    for (id, _is_new) in &fresh {
        let Some(c) = w.calls.get(*id) else { continue };
        if c.owner.is_some() {
            continue;
        }
        let view = crate::policy::HallCall {
            id: c.id,
            level: c.level,
            dir: c.dir,
            owner: None,
            waiting: c.waiting,
        };
        if let Some(car) = assigner.assign(
            &crate::policy::SystemView {
                now: w.now,
                cars: &sv.briefs,
                pending: &sv.pending,
                all_calls: &sv.all,
                queue_len: &sv.q,
                top_level: w.top,
                n_cars: w.cars.len(),
            },
            view,
        ) {
            if let Some(c) = w.calls.get_mut(*id) {
                c.owner = Some(car);
            }
            if let Some(car) = w.cars.get_mut(car) {
                if !car.assigned.contains(id) {
                    car.assigned.push(*id);
                }
            }
        }
    }
}

/// 推进一步轿厢状态机。
/// 推进一步轿厢状态机。
///
/// 结构上分成三段：先做只读决策，再按阶段执行，最后统一排下一次事件。
/// 这样排期只有一处，漏排就立刻表现为「轿厢停住不动」，不会静默错乱。
#[allow(clippy::too_many_arguments)]
fn step_car<R: Router, A: Assigner, B: Boarding>(
    w: &mut World,
    id: CarId,
    router: &mut R,
    assigner: &A,
    boarding: &B,
    sv: &mut Scratch,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
    rng: &mut Streams,
) -> Option<f64> {
    let spec_up = w.cfg.cars[id].up;
    let spec_down = w.cfg.cars[id].down;
    let doors = w.cfg.cars[id].doors;
    let crush = w.cfg.boarding.capacity_crush(w.cfg.cars[id].capacity_rated);
    let floor_h = w.cfg.building.floor_height;
    let overload_on = w.cfg.overload.enabled;
    let trigger = w.cfg.overload.trigger_ratio * w.cfg.cars[id].rated_load_kg as f64;
    let phase = w.cars[id].phase;
    let has_riders = !w.cars[id].occupants.is_empty();

    // ---- 第一段：只读决策 ----
    let (must_stop, next_leg) = {
        w.fill_views(sv);
        let rv = w.fill_run_view(sv, id);
        let here = rv.level();
        // 停靠必须名副其实：有人要在这一层出去，或者这一层真的有人在等。
        // 只问算法不够：算法可能因为「名下有活」而想停，
        // 但那台车此刻在别的层，停在这里是白停
        let riders_here = rv.will_alight(here);
        let queue_here = waiting_here(w, here);
        let stop = riders_here || queue_here;
        let leg = if phase == CarPhase::Idle {
            // 停着的轿厢按优先级决定去哪：
            // 1. 车上还有人没送到，先送他们。这是电梯的义务，压过一切优化
            // 2. 名下有活就去接
            // 3. 都不要才问算法的待机策略
            //
            // 送客排在接活前面是硬要求。早先反过来写，导致车停在某层不动：
            // 名下那条「本层外呼」永远离自己最近，于是每轮都选它，
            // 而车里的人一直等着下不去。
            let rider_goal = if has_riders { nearest_rider_dest(w, here) } else { None };
            if let Some(t) = rider_goal {
                if t != here {
                    dir_leg(t, here)
                } else {
                    // 人就在这一层，该停靠服务
                    Leg::Park
                }
            } else {
                let job = rv.mine.iter().map(|c| c.level).min_by_key(|&l| (l - here).abs());
                match job {
                    Some(t) => dir_leg(t, here),
                    None => idle_leg(&rv, router),
                }
            }
        } else if phase == CarPhase::Moving {
            // 在路上。目标就是本趟的终点，不能每轮重新决策，
            // 否则轿厢永远走不到那一层
            Leg::Park
        } else if has_riders {
            // 停着但车上有乘客没送到。送客是电梯的义务，优先级高于算法优化
            match nearest_rider_dest(w, here) {
                Some(t) if t != here => dir_leg(t, here),
                _ => router.next_leg(&rv),
            }
        } else {
            router.next_leg(&rv)
        };
        if std::env::var("TIHU_IDLE").is_ok() && phase == CarPhase::Idle {
            eprintln!(
                "[idle] t={:.1} 车{} here={} stop={} leg={:?} riders={} 名下={} 全楼有人等={}",
                w.now, id, here, stop, leg, has_riders, rv.mine.len(), someone_still_waiting(w)
            );
        }
        (stop, leg)
    };


    // ---- 第二段：按阶段执行 ----
    match phase {
        CarPhase::Idle => {
            w.cars[id].dir = Dir::None;
            if must_stop {
                serve_floor(w, id, boarding, crush, &doors, overload_on, trigger, rng, heap, seq);
            } else {
                match next_leg {
                    Leg::Park => {
                        // 站着等。但要留一条退路：若全楼还有人没被服务，
                        // 排一个短间隔重试。
                        // 事件驱动里「从忙变闲」这个转移没人触发，
                        // 不排这条重试，轿厢就会永远停着，而排队的人还在等
                        if someone_still_waiting(w) {
                            w.cars[id].next_event = w.now + RETRY_INTERVAL;
                        } else {
                            w.cars[id].next_event = f64::INFINITY;
                        }
                    }
                    leg => start_trip(w, id, leg, spec_up, spec_down, floor_h),
                }
            }
        }

        CarPhase::Moving => {
            // 到站了。先把位置推到本趟终点
            if w.cars[id].trip_target != w.cars[id].level.round() as i16 {
                w.cars[id].level = w.cars[id].trip_target as f32;
            }
            let here = w.cars[id].level.round() as i16;
            // 只有「车上有人要在这一层出去」才停靠。
            // 不能把「这一层有人在等」也算进来，那样车会在这里反复空停，
            // 上不了人又走不掉，车里的人永远下不去
            let riders_here = w.cars[id].occupants.iter().any(|&p| w.pax[p].to == here);
            if riders_here {
                serve_floor(w, id, boarding, crush, &doors, overload_on, trigger, rng, heap, seq);
            } else {
                // 这一层没事，决定下一程去哪。
                // 优先级：车上乘客的门口 > 算法指定的下一程。
                // 只看算法会让车停在大堂之外的某层不动，
                // 因为算法看到的是「当前层没有停靠需求」，不是「车里还有人没送到」
                let goal = nearest_rider_dest(w, here).or_else(|| match next_leg {
                    Leg::Up(t) | Leg::Down(t) => Some(t),
                    Leg::Park => None,
                });
                match goal {
                    Some(t) if t != here => {
                        start_trip(w, id, dir_leg(t, here), spec_up, spec_down, floor_h)
                    }
                    _ => {
                        // 真的没事干了。待机。若还有人没被服务就留一条重试
                        w.cars[id].phase = CarPhase::Idle;
                        w.cars[id].dir = Dir::None;
                        w.cars[id].next_event = if someone_still_waiting(w) {
                            w.now + RETRY_INTERVAL
                        } else {
                            f64::INFINITY
                        };
                    }
                }
            }
        }

        CarPhase::Boarding => {
            // 上一次停靠没结算完，补一次
            serve_floor(w, id, boarding, crush, &doors, overload_on, trigger, rng, heap, seq);
        }

        CarPhase::Stalled => {
            w.cars[id].next_event = f64::INFINITY;
        }
    }

    // ---- 第三段：排下一次 ----
    // 待机（next_event 为无穷）时不排事件，由 wake_idle_cars 在有新活时叫醒。
    // 条件用 `>=`：`do_boarding` 后排期恰好等于当前时刻（关门完成），
    // 用 `>` 会漏排，轿厢就此停摆，车里的人永远出不去。
    // 排下一次。最小间隔 1 毫秒是防自触发的兜底：
    // 若某条路径把排期设成了当前时刻，不加这个下限就会同刻无限重排
    if std::env::var("TIHU_ARR").is_ok() && w.now < 560.0 {
        eprintln!(
            "[sched] t={:.3} 车{} phase={:?} next_event={} at<=now? {}",
            w.now, id, w.cars[id].phase, w.cars[id].next_event,
            w.cars[id].next_event <= w.now
        );
    }
    if std::env::var("TIHU_ARR").is_ok() && w.now > 700.0 && w.now < 760.0 {
        eprintln!(
            "[dec] t={:.2} 车{} phase={:?} here={} stop={} leg={:?} 名下{} 等待中={}",
            w.now, id, phase, w.cars[id].level.round(), must_stop, next_leg,
            w.cars[id].assigned.len(), someone_still_waiting(w)
        );
    }
    let at = w.cars[id].next_event;
    if at.is_finite() && at > w.now && w.cars[id].phase != CarPhase::Stalled {
        heap.push(Ev { at, seq: *seq, kind: Kind::CarStep(id) });
        *seq += 1;
    } else if at.is_finite() && at >= w.now && w.cars[id].phase != CarPhase::Stalled {
        w.cars[id].next_event = w.now + 0.001;
        heap.push(Ev { at: w.now + 0.001, seq: *seq, kind: Kind::CarStep(id) });
        *seq += 1;
    }
    None
}

/// 朝目标层出发。
fn start_trip(
    w: &mut World,
    id: CarId,
    leg: Leg,
    up: crate::motion::MotionProfile,
    down: crate::motion::MotionProfile,
    floor_h: f32,
) {
    let (dir, target) = match leg {
        Leg::Up(t) => (Dir::Up, t),
        Leg::Down(t) => (Dir::Down, t),
        Leg::Park => (Dir::None, w.cars[id].level.round() as i16),
    };
    let from = w.cars[id].level as f64;
    let to = target as f64;
    // 目标层必须真的不同。零距离的「移动」会让轿厢反复重排同一个时刻的事件，
    // 一次仿真能烧掉几十万事件而不推进任何时间
    if !dir.is_moving() || (to - from).abs() < f64::EPSILON {
        w.cars[id].dir = Dir::None;
        w.cars[id].phase = CarPhase::Idle;
        w.cars[id].next_event = if someone_still_waiting(w) {
            w.now + RETRY_INTERVAL
        } else {
            f64::INFINITY
        };
        return;
    }
    let dist = ((target as f64) - from).abs() * floor_h as f64;
    let profile = dir.profile(&up, &down);
    let time = crate::motion::traverse_time(dist, *profile).time;
    w.cars[id].dir = dir;
    w.cars[id].phase = CarPhase::Moving;
    w.cars[id].trip_target = target;
    w.cars[id].next_event = w.now + time.max(0.05);
    if w.cars[id].trip_start == 0.0 {
        w.cars[id].trip_start = w.now;
    }
}

/// 在当前层完成一次停靠：先下后上，关门排程。
#[allow(clippy::too_many_arguments)]
fn serve_floor<B: Boarding>(
    w: &mut World,
    id: CarId,
    boarding: &B,
    crush: usize,
    doors: &crate::motion::DoorTiming,
    overload_on: bool,
    trigger: f64,
    rng: &mut Streams,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
) {
    w.cars[id].phase = CarPhase::Boarding;
    do_boarding(w, id, boarding, crush, doors, overload_on, trigger, rng, heap, seq);
}

/// 车上最近的乘客要去哪一层。
fn nearest_rider_dest(w: &World, here: i16) -> Option<i16> {
    w.cars
        .iter()
        .flat_map(|c| c.occupants.iter())
        .map(|&p| w.pax[p].to)
        .min_by_key(|&t| (t - here).abs())
}

/// 目标层比当前层高就上行，低就下行。
fn dir_leg(target: i16, here: i16) -> Leg {
    if target > here {
        Leg::Up(target)
    } else if target < here {
        Leg::Down(target)
    } else {
        Leg::Park
    }
}

/// 空闲时算法的处置，转成 Leg。
fn idle_leg<R: Router>(rv: &RunView<'_>, router: &R) -> Leg {
    let here = rv.level();
    match router.idle(rv) {
        Idle::Hold => Leg::Park,
        Idle::GoTo(t) => dir_leg(t, here),
    }
}

/// Leg 里的目标层。
fn leg_target(leg: Leg, fallback: f32) -> f64 {
    match leg {
        Leg::Up(t) | Leg::Down(t) => t as f64,
        Leg::Park => fallback as f64,
    }
}

/// `DefaultBoarding` 的概率公式。放在这里是因为随机性归内核，策略对象本身不掷骰子。
fn default_accept_prob(s: &AdmitSample, tol: f64) -> f64 {
    if s.free_slots == 0 {
        return 0.0;
    }
    crate::config::Boarding::default().accept_prob(s.load_ratio, tol, s.waited_s)
}

/// 清掉这一层已经服务完的呼叫。
///
/// 判据是「还有没有人真的在等」。已经上梯的人不算，他们由 `boarded_car` 标记。
fn clear_served_calls(w: &mut World, id: CarId, level: i16) {
    let qi = w.qidx(level);
    let n = w.queues[qi].arrived.min(w.queues[qi].waiting.len());
    let still_waiting = w.queues[qi].waiting[..n]
        .iter()
        .any(|&p| w.pax[p].boarded_car.is_none() && w.pax[p].to != level);
    if still_waiting {
        return;
    }
    for dir in [Dir::Up, Dir::Down] {
        let slot = Call::slot_index(level, dir, w.basement);
        let call_id = w.calls.slots[slot];
        let Some(cid) = call_id else { continue };
        let owner = w.calls.get(cid).and_then(|c| c.owner);
        if owner != Some(id) {
            continue;
        }
        w.calls.clear_slot(level, dir);
        if let Some(car) = w.cars.get_mut(id) {
            car.assigned.retain(|&a| a != cid);
        }
    }
}

/// 一次僵持的结果。
struct StallOutcome {
    resumed: bool,
    seconds: f64,
    stalls_here: usize,
    orphaned: usize,
}

/// 超载僵局的每一轮：问一次有没有人愿意退出。
#[allow(clippy::too_many_arguments)]
fn stall_tick<A: Assigner>(
    w: &mut World,
    id: CarId,
    rng: &mut Streams,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
    assigner: &A,
    _sv: &mut Scratch,
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

    let mut orphaned = 0usize;
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
        return StallOutcome { resumed: true, seconds, stalls_here: 1, orphaned };
    }

    let _ = assigner;
    StallOutcome { resumed: false, seconds: 0.0, stalls_here: 0, orphaned }
}

/// 汇总指标。
#[allow(clippy::too_many_arguments)]
fn collect(
    w: &World,
    stalls: usize,
    stall_seconds: f64,
    orphaned: usize,
    ghost_stops: usize,
    full_bypasses: usize,
) -> Metrics {
    let mut m = Metrics::default();

    for p in &w.pax {
        if p.used_stairs {
            m.stair_users += 1;
            continue;
        }
        if p.t_exit <= 0.0 {
            continue;
        }
        m.wait.add(p.wait());
        m.ride.add(p.ride());
        m.door_to_door.add(p.door_to_door());
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
    m.pax_total = w.pax.len();
    m.pax_served = m.door_to_door.count;
    if m.pax_served > 0 {
        m.pax_any_bypass = m.pax_with_bypass as f64 / m.pax_served as f64;
        m.pax_bypass3_frac = m.pax_bypass3 as f64 / m.pax_served as f64;
    }
    if !w.pax.is_empty() {
        m.stair_share = m.stair_users as f64 / w.pax.len() as f64;
    }

    // 楼层视角
    let floors = w.cfg.building.floors.max(1);
    let mut fs: Vec<FloorStats> = vec![FloorStats::default(); floors as usize];
    for p in &w.pax {
        if p.used_stairs || p.t_exit <= 0.0 {
            continue;
        }
        if p.from < 1 || p.from > floors {
            continue;
        }
        let s = &mut fs[(p.from - 1) as usize];
        s.count += 1;
        s.wait_sum += p.wait();
        s.wait_max = s.wait_max.max(p.wait());
        s.wait_samples.add(p.wait());
        s.bypass_total += p.bypass_count;
        if p.bypass_count > 0 {
            s.bypassed += 1;
        }
        s.declined += p.declined_count;
        s.no_space += p.no_space_count;
        if p.used_stairs {
            s.stair_users += 1;
        }
    }
    for s in fs.iter_mut() {
        if s.count > 0 {
            s.wait_mean = s.wait_sum / s.count as f64;
        }
    }
    m.by_floor = fs;

    // 系统视角
    m.stalls = stalls;
    m.stall_seconds = stall_seconds;
    m.calls_orphaned = orphaned;
    m.ghost_stops = ghost_stops;
    m.full_bypasses = full_bypasses;
    m.n_cars = w.cars.len();
    for c in &w.cars {
        m.rtt_sum += c.rtt_total;
        m.rtt_count += c.rtt_count;
        m.trips += c.trips;
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


/// 处理一次停靠的上下客。抽成独立函数是为了让 `&mut World` 不被长时间独占。
#[allow(clippy::too_many_arguments)]
/// 处理一次停靠的上下客。
///
/// 抽成独立函数有两个原因。一是让 `&mut World` 不被长时间独占，
/// 二是上客这个循环要同时读 `w.pax` 和改 `w.cars`，放在 match 里会撞借用。
#[allow(clippy::too_many_arguments)]
fn do_boarding<B: Boarding>(
    w: &mut World,
    id: CarId,
    boarding: &B,
    crush: usize,
    doors: &crate::motion::DoorTiming,
    overload_on: bool,
    trigger: f64,
    rng: &mut Streams,
    heap: &mut std::collections::BinaryHeap<Ev>,
    seq: &mut u64,
) {
    let lv = w.cars[id].level.round() as i16;
    let now = w.now;
    let capacity_rated = w.cfg.cars[id].capacity_rated;

    // 先下后上
    w.cars[id].occupants.retain(|&p| {
        if w.pax[p].to == lv {
            w.pax[p].t_exit = now;
            w.pax[p].boarded_car = None;
            false
        } else {
            true
        }
    });
    w.cars[id].load_kg = w.cars[id].occupants.iter().map(|&p| w.pax[p].weight).sum();

    // 再上人。反复扫整段队列，直到一整轮没人能上为止。
    // 队首那位挤不上时，后面的人仍有机会上车，这是真实电梯的样子
    let qi = w.qidx(lv);
    let mut boarded = 0usize;
    loop {
        let mut progress = false;
        let mut idx = 0usize;
        let n = w.queues[qi].arrived.min(w.queues[qi].waiting.len());
        while idx < n {
            let pid = w.queues[qi].waiting[idx];
            if w.pax[pid].boarded_car.is_some() {
                idx += 1;
                continue;
            }
            if try_board_one(w, id, lv, qi, idx, pid, boarding, crush, now, rng) {
                boarded += 1;
                progress = true;
                idx += 1;
            } else {
                idx += 1;
            }
        }
        if !progress {
            break;
        }
    }
    w.cars[id].last_boarded = boarded;

    // 关门时刻
    let in_t = doors.transfer_time(boarded, 0);
    let base = (doors.open_s - doors.early_open_credit_s).max(0.0) as f64;
    let total = base + in_t + doors.close_s as f64 + doors.ctrl_resp_s as f64;
    let close_at = w.cars[id].stop_started + total;

    if overload_on && w.cars[id].load_kg > trigger {
        // 红线 6：超载报警。门关不上，没人退出就一直僵着
        w.cars[id].phase = CarPhase::Stalled;
        w.cars[id].stall_since = now;
        w.cars[id].next_event = f64::INFINITY;
        heap.push(Ev {
            at: now + w.cfg.overload.stall_tick_s,
            seq: *seq,
            kind: Kind::StallTick(id),
        });
        *seq += 1;
        return;
    }

    w.cars[id].phase = if boarded > 0 { CarPhase::Moving } else { CarPhase::Idle };
    // 关门时刻若已经过去就让它等于现在。写成 max(0.001)+now 会变成
    // 每毫秒触发一次停靠，关门永远走不完
    w.cars[id].next_event = close_at.max(now);

    if boarded == 0 {
        if w.cars[id].trip_start > 0.0 {
            let rtt = now - w.cars[id].trip_start;
            if rtt > 0.0 && rtt < 3600.0 {
                w.cars[id].rtt_total += rtt;
                w.cars[id].rtt_count += 1;
                w.cars[id].trips += 1;
                w.cars[id].trip_start = now;
            }
        }
    }
    let _ = capacity_rated;
}

/// 考察队列第 `idx` 位能否上车。能就让他上，返回 true。
#[allow(clippy::too_many_arguments)]
fn try_board_one<B: Boarding>(
    w: &mut World,
    id: CarId,
    lv: i16,
    qi: usize,
    idx: usize,
    pid: usize,
    boarding: &B,
    crush: usize,
    now: f64,
    rng: &mut Streams,
) -> bool {
    let capacity_rated = w.cfg.cars[id].capacity_rated;
    let wants = if w.pax[pid].to > lv {
        Dir::Up
    } else if w.pax[pid].to < lv {
        Dir::Down
    } else {
        Dir::None
    };
    // 轿厢方向要匹配。停着不动时任何方向都收
    let matching = match w.cars[id].dir {
        Dir::None => true,
        d => d == wants,
    };
    if std::env::var("TIHU_BOARD").is_ok() {
        eprintln!(
            "[board] t={:.2} lv={} pid={} wants={:?} dir={:?} matching={} free={} load={}/{}",
            now, lv, pid, wants, w.cars[id].dir, matching,
            capacity_rated.saturating_sub(w.cars[id].occupants.len()),
            w.cars[id].occupants.len(), capacity_rated
        );
    }
    if !matching {
        return false;
    }
    let _ = qi;
    let _ = idx;
    let free = capacity_rated.saturating_sub(w.cars[id].occupants.len());
    let load_ratio = w.cars[id].occupants.len() as f64 / capacity_rated.max(1) as f64;
    let waited = (now - w.pax[pid].t_arrive).max(0.0);
    let sample = AdmitSample {
        load_ratio,
        free_slots: free,
        waited_s: waited,
        weight_ratio: w.cars[id].load_kg / (w.cfg.cars[id].rated_load_kg as f64).max(1.0),
    };
    let tol = w.pax[pid].tol;

    let reject = |w: &mut World| {
        w.pax[pid].bypass_count += 1;
    };

    match boarding.admit_one(&sample, tol) {
        Verdict::NoSpace => {
            w.pax[pid].no_space_count += 1;
            reject(w);
            false
        }
        Verdict::Declined => {
            w.pax[pid].declined_count += 1;
            reject(w);
            false
        }
        Verdict::Admit => {
            // 中间地带要掷骰子。策略对象本身不掷，随机性归内核。
            // 这一步是红线 3 的核心：越接近满载，越可能有人放弃
            let prob = default_accept_prob(&sample, tol);
            if prob < 1.0 && !rng.chance(Stream::Squeeze, prob) {
                w.pax[pid].declined_count += 1;
                reject(w);
                return false;
            }
            // 同行团不可拆分。位子不够就整车不上，
            // 这是「没坐满就开走」的第二个成因
            if w.cars[id].occupants.len() >= crush {
                w.pax[pid].no_space_count += 1;
                reject(w);
                return false;
            }
            w.cars[id].occupants.push(pid);
            w.cars[id].load_kg += w.pax[pid].weight;
            w.pax[pid].boarded_car = Some(id);
            w.pax[pid].t_board = now;
            // 反向叫梯的人登记转向层，一路坐到折返点再下来
            if w.pax[pid].strategy == Strategy::PressUpOnly
                || w.pax[pid].strategy == Strategy::PressBoth
            {
                w.pax[pid].reversal_floor = Some(lv);
            }
            true
        }
    }
}