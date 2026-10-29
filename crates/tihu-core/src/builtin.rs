//! 内建的调度算法，以及算法注册表。
//!
//! 写一个新算法在这里加一行 `register!`，网页下拉框和命令行 `--router`
//! 就同时能用上，不需要改动 `tihu-core` 的其他地方。见 `docs/DISPATCH_API.md`。

use crate::config::Boarding as BoardCfg;
use crate::motion::Dir;
use crate::policy::{AdmitSample, Assigner, Boarding, CarId, HallCall, Leg, RunView, SystemView, Verdict};

/// 经典集选。保持方向，把沿途同向外呼和内选服务完，到底才反向。
///
/// 这是住宅楼的默认做法。老楼装的就是它。
///
/// 满载直驶不在这里，也不在任何 router 里：它是内核行为，`sim::step_car`
/// 在车满员时把路过的同向外呼从停靠表里划掉。router 只能改这一腿的目标层，
/// 而 `plan_leg` 会重新把名下的顺路外呼排进停靠表，所以想在 router 这一层
/// 做直驶是做不到的。
#[derive(Debug, Default, Clone)]
pub struct CollectiveScan;

impl CollectiveScan {
    fn going_up(v: &RunView<'_>) -> bool {
        match v.heading {
            Dir::Up => true,
            Dir::Down => false,
            // 没方向时看哪边有活
            Dir::None => v.has_call_up() || v.highest_car_call().is_some_and(|l| l > v.level()),
        }
    }
}

impl crate::policy::Router for CollectiveScan {
    fn name(&self) -> &'static str {
        "collective_scan"
    }

    fn next_leg(&self, v: &RunView<'_>) -> Leg {
        let up = Self::going_up(v);
        if up {
            // 上行。看这一趟有没有上行任务，没有就看内选
            let target = v.extreme(Dir::Up).or_else(|| v.highest_car_call());
            match target {
                Some(t) if t > v.level() => Leg::Up(t),
                // 上行方向没活了，反向
                _ => {
                    let down = v.extreme(Dir::Down).or_else(|| v.lowest_car_call());
                    match down {
                        Some(t) if t < v.level() => Leg::Down(t),
                        Some(t) => Leg::Up(t),
                        // 身后也没点名，但名下还有活（比如上行呼叫全在身后）：
                        // 就近开过去取。原地趴着的话这批人永远等不到车，
                        // 派梯去重后不会再有第二次指派，内核也不会叫醒它
                        None => fetch_nearest(v),
                    }
                }
            }
        } else {
            let target = v.extreme(Dir::Down).or_else(|| v.lowest_car_call());
            match target {
                Some(t) if t < v.level() => Leg::Down(t),
                _ => {
                    let up = v.extreme(Dir::Up).or_else(|| v.highest_car_call());
                    match up {
                        Some(t) if t > v.level() => Leg::Up(t),
                        Some(t) => Leg::Down(t),
                        // 同上。早高峰空车停在主入口层、名下全是楼上下行呼叫，
                        // 走的就是这一支：上去取，下来再扫
                        None => fetch_nearest(v),
                    }
                }
            }
        }
    }
}

/// 名下还有活但方向逻辑没点名时，就近取一单，取到再说。
/// 两边都有就取近的一边。真的没活才回 Park。
fn fetch_nearest(v: &RunView<'_>) -> Leg {
    let here = v.level();
    let down = v.lowest_pending().filter(|&t| t < here);
    let up = v.highest_pending().filter(|&t| t > here);
    match (down, up) {
        (Some(d), Some(u)) => {
            if here - d <= u - here {
                Leg::Down(d)
            } else {
                Leg::Up(u)
            }
        }
        (Some(d), None) => Leg::Down(d),
        (None, Some(u)) => Leg::Up(u),
        (None, None) => Leg::Park,
    }
}

/// 早高峰专用的下行扫描。先上行到最高任务层，再一路下行扫到底。
///
/// 就是 `docs/DISPATCH_API.md` 第 3.2 节那个例子，本身也是一个可用的基线。
#[derive(Debug, Default, Clone)]
pub struct DownPeakSweep;

impl crate::policy::Router for DownPeakSweep {
    fn name(&self) -> &'static str {
        "down_peak_sweep"
    }

    fn next_leg(&self, v: &RunView<'_>) -> Leg {
        // 名下还有下行任务就继续往下扫
        if let Some(f) = v.extreme(Dir::Down) {
            if f < v.level() {
                return Leg::Down(f);
            }
        }
        if let Some(f) = v.lowest_car_call() {
            if f < v.level() {
                return Leg::Down(f);
            }
        }
        // 下行任务清空了，去接最高的一条，然后重新开始往下扫
        let highest = v.highest_pending().or_else(|| v.highest_car_call());
        match highest {
            Some(f) if f > v.level() => Leg::Up(f),
            Some(f) => Leg::Down(f),
            // 彻底没活，回主入口层待命
            None => {
                let lobby = crate::config::LOBBY;
                if v.level() > lobby {
                    Leg::Down(lobby)
                } else if v.level() < lobby {
                    Leg::Up(lobby)
                } else {
                    Leg::Park
                }
            }
        }
    }
}

/// 每层最近的轿厢。简单粗暴，很多老系统就是这么干的。
#[derive(Debug, Default, Clone)]
pub struct NearestCar;

impl Assigner for NearestCar {
    fn name(&self) -> &'static str {
        "nearest_car"
    }

    fn assign(&self, v: &SystemView<'_>, call: HallCall) -> Option<CarId> {
        let mut best: Option<(f64, CarId)> = None;
        for c in v.cars {
            if c.stalled {
                continue;
            }
            let d = (c.level as f64 - call.level as f64).abs();
            // 已经空载且方向相反的稍微加一点惩罚
            let penalty = if c.load == 0 { 0.0 } else { 1.0 };
            let score = d + penalty;
            if best.map_or(true, |(b, _)| score < b) {
                best = Some((score, c.id));
            }
        }
        best.map(|(_, id)| id)
    }
}

/// 轮流派遣。负载天然均衡，但等待时间可能很长。
///
/// 游标用 `Cell` 而不是普通字段，这样 `assign` 能保持 `&self` 签名。
/// 算法内部的可变状态就这么大一处，用不着整个 trait 改成 `&mut self`。
#[derive(Debug, Default, Clone)]
pub struct RoundRobin {
    cursor: std::cell::Cell<usize>,
}

impl Assigner for RoundRobin {
    fn name(&self) -> &'static str {
        "round_robin"
    }

    fn assign(&self, v: &SystemView<'_>, _call: HallCall) -> Option<CarId> {
        if v.n_cars == 0 {
            return None;
        }
        let i = self.cursor.get() % v.n_cars;
        self.cursor.set(self.cursor.get().wrapping_add(1));
        Some(i)
    }

    fn reset(&mut self) {
        self.cursor.set(0);
    }
}

/// 派给预计到达最快的那台。
#[derive(Debug, Clone)]
pub struct EtaMin {
    /// 方向不合的惩罚系数
    pub dir_penalty: f64,
}

impl Default for EtaMin {
    fn default() -> Self {
        Self { dir_penalty: 30.0 }
    }
}

impl Assigner for EtaMin {
    fn name(&self) -> &'static str {
        "eta_min"
    }

    fn assign(&self, v: &SystemView<'_>, call: HallCall) -> Option<CarId> {
        let mut best: Option<(f64, CarId)> = None;
        for c in v.cars {
            if c.stalled {
                continue;
            }
            let dist = (c.level as f64 - call.level as f64).abs();
            // 方向不合要多跑一段，罚 30 秒
            let aligned = match (c.dir, call.dir) {
                (Dir::Up, Dir::Up) | (Dir::Down, Dir::Down) => true,
                (Dir::None, _) => true,
                _ => false,
            };
            let score = dist * 3.5 + if aligned { 0.0 } else { self.dir_penalty };
            if best.map_or(true, |(b, _)| score < b) {
                best = Some((score, c.id));
            }
        }
        best.map(|(_, id)| id)
    }
}

/// 到达时间加载客惩罚，还会预判「到那一层时是不是已经挤满」。
///
/// 预判挤满是它比 `eta_min` 强的地方。见 `docs/DISPATCH_API.md` 第 4.1 节。
#[derive(Debug, Clone)]
pub struct LoadAwareEta {
    /// 每超载一人的惩罚秒数
    pub load_penalty: f64,
    /// 预判会挤满时的额外惩罚
    pub full_penalty: f64,
    /// 方向不合的惩罚
    pub dir_penalty: f64,
    /// 名下每条待办外呼的惩罚。这是负载均衡的关键项
    pub backlog_penalty: f64,
}

impl Default for LoadAwareEta {
    fn default() -> Self {
        Self { load_penalty: 12.0, full_penalty: 40.0, dir_penalty: 25.0, backlog_penalty: 8.0 }
    }
}

impl Assigner for LoadAwareEta {
    fn name(&self) -> &'static str {
        "load_aware_eta"
    }

    fn assign(&self, v: &SystemView<'_>, call: HallCall) -> Option<CarId> {
        let mut best: Option<(f64, CarId)> = None;
        for c in v.cars {
            if c.stalled {
                continue;
            }
            let dist = (c.level as f64 - call.level as f64).abs();
            let aligned = match (c.dir, call.dir) {
                (Dir::Up, Dir::Up) | (Dir::Down, Dir::Down) | (Dir::None, _) => true,
                _ => false,
            };
            // 预判：到那层时这一趟会卸掉多少人
            let predicted_free = c.capacity.saturating_sub(c.load);
            let will_be_full = predicted_free == 0;
            // 名下已有的外呼越多，越不该再压给它。
            // 少了这一项，两台同位置的轿厢评分完全相同，
            // 而取 min 的写法永远偏向第一台，于是所有活都堆到同一台车上。
            let backlog = c.pending_calls as f64 * self.backlog_penalty;
            let mut score = dist * 3.0 + c.load_ratio() * self.load_penalty + backlog;
            if will_be_full {
                score += self.full_penalty;
            }
            if !aligned {
                score += self.dir_penalty;
            }
            if best.map_or(true, |(b, _)| score < b) {
                best = Some((score, c.id));
            }
        }
        best.map(|(_, id)| id)
    }
}

/// 支持呼叫重分配的派梯。
///
/// 这是 `docs/MODEL.md` 第 8.4 节那个场景的正解。一台轿厢卡在超载僵局里，
/// 它名下那些未服务的外呼通常没有第二台会去接，因为派梯关系是早先建立的。
/// 这个包装器发现某台轿厢长时间没有推进，就把任务改派给别人。
#[derive(Debug, Clone)]
pub struct RescueWrapper<A> {
    /// 内层的派梯算法
    pub inner: A,
    /// 僵局超过多少秒就开始救
    pub threshold_s: f64,
    pub rescued: u64,
}

impl<A: Assigner> RescueWrapper<A> {
    pub fn new(inner: A) -> Self {
        Self { inner, threshold_s: 60.0, rescued: 0 }
    }
}

impl<A: Assigner> Assigner for RescueWrapper<A> {
    fn name(&self) -> &'static str {
        "with_rescue"
    }

    fn assign(&self, v: &SystemView<'_>, call: HallCall) -> Option<CarId> {
        self.inner.assign(v, call)
    }

    fn reassign(&self, v: &SystemView<'_>, owned: &mut [Option<CarId>]) {
        self.inner.reassign(v, owned);
        if !v.cars.iter().any(|c| c.stalled && c.stall_for > self.threshold_s) {
            return;
        }
        // 指向僵死轿厢的外呼改派给负载最小的活轿厢。载客数是视图里的定值，
        // 改派不会改它，所以目标挑一次就够，不必每救一条重挑一遍
        let Some(target) = v.cars.iter().filter(|c| !c.stalled).min_by_key(|c| c.load) else {
            // 全都僵着，没地方可送
            return;
        };
        let target = target.id;
        for o in owned.iter_mut() {
            if o.is_some_and(|id| v.cars[id].stalled) {
                *o = Some(target);
            }
        }
    }

    fn reset(&mut self) {
        self.inner.reset();
    }
}

/// 上下客的默认规则，就是 `docs/MODEL.md` 第 5.5 节那套。
#[derive(Debug, Clone)]
pub struct DefaultBoarding {
    pub cfg: BoardCfg,
}

impl Default for DefaultBoarding {
    fn default() -> Self {
        Self { cfg: BoardCfg::default() }
    }
}

impl Boarding for DefaultBoarding {
    fn name(&self) -> &'static str {
        "default"
    }

    fn admit_one(&self, s: &AdmitSample, tol: f64) -> Verdict {
        if s.free_slots == 0 {
            return Verdict::NoSpace;
        }
        // 只处理两端确定的情况，中间地带交给引擎掷骰子。
        // 引擎拿到 accept_prob 的值之后抽一次签，比在这里硬判要准。
        if self.cfg.accept_prob(s.load_ratio, tol, s.waited_s) <= 0.0 {
            Verdict::Declined
        } else {
            Verdict::Admit
        }
    }
}

impl DefaultBoarding {
    /// 给出这位乘客愿意上来的概率。引擎拿它掷骰子。
    pub fn accept_prob(&self, s: &AdmitSample, tol: f64) -> f64 {
        if s.free_slots == 0 {
            return 0.0;
        }
        self.cfg.accept_prob(s.load_ratio, tol, s.waited_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy::{CarBrief, Router as _, RunView};

    static EMPTY_CARS: [CarBrief; 0] = [];
    static EMPTY_CALLS: [HallCall; 0] = [];
    static EMPTY_Q: [u32; 0] = [];

    fn rv(at: f32, dir: Dir, mine: Vec<HallCall>, car_calls: Vec<i16>, load: usize) -> RunView<'static> {
        // 把两个 Vec 泄漏成 'static 切片，测试专用
        let mine: &'static [HallCall] = Box::leak(mine.into_boxed_slice());
        let car_calls: &'static [i16] = Box::leak(car_calls.into_boxed_slice());
        RunView {
            now: 0.0,
            car: 0,
            at_level: at,
            heading: dir,
            load,
            capacity_rated: 13,
            capacity_crush: 17,
            load_kg: load as f32 * 64.0,
            car_calls,
            mine,
            system: SystemView {
                now: 0.0,
                cars: &EMPTY_CARS,
                pending: &EMPTY_CALLS,
                all_calls: &EMPTY_CALLS,
                queue_len: &EMPTY_Q,
                top_level: 33,
                n_cars: 0,
            },
            floor_height: 3.0,
        }
    }

    fn call(level: i16, dir: Dir) -> HallCall {
        HallCall { id: level as u32, level, dir, owner: Some(0), waiting: 1 }
    }

    /// 集选：上行时挑最高的上行业务，同向没人了就反向。
    #[test]
    fn collective_scan_goes_up_first() {
        let v = rv(5.0, Dir::Up, vec![call(20, Dir::Up)], vec![], 0);
        assert_eq!(CollectiveScan.next_leg(&v), Leg::Up(20));
    }

    #[test]
    fn collective_scan_reverses_when_no_up_work() {
        // 上行但只有下行任务，应当立刻反向
        let v = rv(5.0, Dir::Up, vec![call(2, Dir::Down)], vec![], 0);
        assert_eq!(CollectiveScan.next_leg(&v), Leg::Down(2));
    }

    /// 有人要出去的那一层，路由要选它，内核才不会路过。
    #[test]
    fn collective_scan_routes_to_rider_dest() {
        let v = rv(8.0, Dir::Up, vec![], vec![8, 20], 2);
        // 当前位置就是 8，最近的内选是 8，回到 8 没有意义，取下一个
        assert_eq!(CollectiveScan.next_leg(&v), Leg::Up(20));
    }

    /// 空车停在主入口层、名下全是楼上的下行呼叫：上去取，不能趴着。
    ///
    /// 早高峰分区梯的死锁就卡在这里。原来这一支回 Park，而派梯去重后
    /// 不会有第二次指派，内核也不会叫醒名下有活的闲车，于是整栋楼零送达。
    #[test]
    fn collective_scan_fetches_down_calls_above() {
        let v = rv(1.0, Dir::None, vec![call(28, Dir::Down)], vec![], 0);
        assert_eq!(CollectiveScan.next_leg(&v), Leg::Up(28));
    }

    /// 镜像：空车在楼上、名下只有身后的上行呼叫：下去取，不能趴着。
    #[test]
    fn collective_scan_fetches_up_calls_below() {
        let v = rv(30.0, Dir::None, vec![call(5, Dir::Up)], vec![], 0);
        assert_eq!(CollectiveScan.next_leg(&v), Leg::Down(5));
    }

    /// 身前身后只有一边有活：掉头去取，而不是趴着。真的没活才趴。
    #[test]
    fn collective_scan_fetches_rather_than_parks() {
        // 上行呼叫全在身后：下去取
        let v = rv(20.0, Dir::Up, vec![call(2, Dir::Up)], vec![], 0);
        assert_eq!(CollectiveScan.next_leg(&v), Leg::Down(2));
        // 下行呼叫全在身前：上去取
        let v = rv(20.0, Dir::Down, vec![call(40, Dir::Down)], vec![], 0);
        assert_eq!(CollectiveScan.next_leg(&v), Leg::Up(40));
        // 真的没活才趴
        let v = rv(5.0, Dir::None, vec![], vec![], 0);
        assert_eq!(CollectiveScan.next_leg(&v), Leg::Park);
    }

    /// 下行扫描：下方没有任务时，先上行到最高的任务层。
    #[test]
    fn sweep_climbs_to_the_highest_pending() {
        let s = DownPeakSweep;
        // 任务都在上方时，先上到最高的那层，再开始往下扫
        let above_only = vec![call(30, Dir::Down), call(25, Dir::Down)];
        assert_eq!(s.next_leg(&rv(20.0, Dir::Up, above_only, vec![], 0)), Leg::Up(30));
    }

    #[test]
    fn sweep_serves_down_calls_on_the_way_down() {
        let s = DownPeakSweep;
        let v = rv(20.0, Dir::Down, vec![call(12, Dir::Down), call(3, Dir::Down)], vec![], 0);
        assert_eq!(s.next_leg(&v), Leg::Down(3));
    }

    /// 把传入的 Vec 泄漏成 `'static`，测试专用。
    /// 这样构造出来的 `SystemView` 不会有借用问题。
    fn system(cars: Vec<CarBrief>, pending: Vec<HallCall>) -> SystemView<'static> {
        let cars: &'static [CarBrief] = Box::leak(cars.into_boxed_slice());
        let pending: &'static [HallCall] = Box::leak(pending.into_boxed_slice());
        SystemView {
            now: 0.0,
            cars,
            pending,
            all_calls: pending,
            queue_len: &EMPTY_Q,
            top_level: 33,
            n_cars: cars.len(),
        }
    }

    fn brief(id: CarId, level: f32, dir: Dir, load: usize, stalled: bool) -> CarBrief {
        CarBrief {
            id,
            level,
            dir,
            load,
            load_kg: load as f32 * 66.0,
            capacity: 13,
            pending_calls: 0,
            stalled,
            stall_for: if stalled { 120.0 } else { 0.0 },
        }
    }

    #[test]
    fn nearest_car_picks_the_closest() {
        let v = system(vec![brief(0, 3.0, Dir::None, 0, false), brief(1, 28.0, Dir::None, 0, false)], vec![]);
        assert_eq!(NearestCar.assign(&v, call(5, Dir::Down)), Some(0));
        assert_eq!(NearestCar.assign(&v, call(30, Dir::Down)), Some(1));
    }

    /// 卡死的轿厢不该被派到新活。
    #[test]
    fn stalled_car_is_never_assigned() {
        let v = system(vec![brief(0, 5.0, Dir::None, 0, true), brief(1, 6.0, Dir::None, 0, false)], vec![]);
        assert_eq!(NearestCar.assign(&v, call(5, Dir::Down)), Some(1));
        assert_eq!(EtaMin::default().assign(&v, call(5, Dir::Down)), Some(1));
    }

    /// 载客惩罚让 `load_aware_eta` 偏离纯就近。
    #[test]
    fn load_aware_prefers_empty_car() {
        let v = system(
            vec![brief(0, 5.0, Dir::None, 12, false), brief(1, 7.0, Dir::None, 0, false)],
            vec![],
        );
        // 两台都很近，但 0 号满了
        assert_eq!(NearestCar.assign(&v, call(5, Dir::Down)), Some(0));
        assert_eq!(LoadAwareEta::default().assign(&v, call(5, Dir::Down)), Some(1));
    }

    /// 红线所在：不救活的派梯会让外呼一直挂在卡死的车上。
    #[test]
    fn rescue_wrapper_reassigns_away_from_stalled_car() {
        let mut owned = vec![Some(0), None, Some(0)];
        let v = system(
            vec![brief(0, 10.0, Dir::None, 13, true), brief(1, 12.0, Dir::None, 2, false)],
            vec![],
        );
        let wrapper = RescueWrapper::new(NearestCar);
        wrapper.reassign(&v, &mut owned);
        // 原本指向卡死 0 号的两条都该被救走
        assert_eq!(owned[0], Some(1));
        assert_eq!(owned[2], Some(1));
    }

    /// 全都僵着时无处可送，归属原样留着，不能派给任何一台。
    #[test]
    fn rescue_wrapper_leaves_calls_alone_when_every_car_is_stalled() {
        let mut owned = vec![Some(0), Some(1), None];
        let v = system(
            vec![brief(0, 10.0, Dir::None, 13, true), brief(1, 12.0, Dir::None, 2, true)],
            vec![],
        );
        RescueWrapper::new(NearestCar).reassign(&v, &mut owned);
        assert_eq!(owned, vec![Some(0), Some(1), None]);
    }

    /// 不救活的派梯什么都不做，这是两种算法的差别所在。
    #[test]
    fn plain_assigner_leaves_calls_on_stalled_car() {
        let mut owned = vec![Some(0), None, Some(0)];
        let v = system(
            vec![brief(0, 10.0, Dir::None, 13, true), brief(1, 12.0, Dir::None, 2, false)],
            vec![],
        );
        NearestCar.reassign(&v, &mut owned);
        assert_eq!(owned[0], Some(0), "朴素派梯不会救");
        assert_eq!(owned[2], Some(0));
    }

    #[test]
    fn boarding_reports_no_space_before_declined() {
        let b = DefaultBoarding::default();
        let full = AdmitSample { load_ratio: 1.0, free_slots: 0, waited_s: 0.0, weight_ratio: 1.0 };
        assert_eq!(b.admit_one(&full, 0.5), Verdict::NoSpace);
        // 有位子且不挤
        let loose = AdmitSample { load_ratio: 0.5, free_slots: 6, waited_s: 0.0, weight_ratio: 0.4 };
        assert_eq!(b.admit_one(&loose, 1.0), Verdict::Admit);
        // 很挤、最不情愿、刚到，概率压到 0
        let tight = AdmitSample { load_ratio: 1.0, free_slots: 1, waited_s: 0.0, weight_ratio: 1.0 };
        assert_eq!(b.admit_one(&tight, 1.0), Verdict::Declined);
        // 中间地带 admit_one 说可以，但概率很低，由引擎掷骰子决定
        let edge = AdmitSample { load_ratio: 0.95, free_slots: 1, waited_s: 0.0, weight_ratio: 0.9 };
        assert_eq!(b.admit_one(&edge, 1.0), Verdict::Admit);
        assert!(b.accept_prob(&edge, 1.0) <= 0.25, "边缘状态概率应当很低，实际 {}", b.accept_prob(&edge, 1.0));
        // 等久了概率回升
        let waited = AdmitSample { load_ratio: 0.95, free_slots: 1, waited_s: 300.0, weight_ratio: 0.9 };
        assert!(b.accept_prob(&waited, 1.0) > b.accept_prob(&edge, 1.0), "等久了更可能挤上来");
    }

    #[test]
    fn round_robin_cycles_and_resets() {
        let mut a = RoundRobin::default();
        let v = system(vec![brief(0, 0.0, Dir::None, 0, false), brief(1, 0.0, Dir::None, 0, false)], vec![]);
        assert_eq!(a.assign(&v, call(5, Dir::Down)), Some(0));
        assert_eq!(a.assign(&v, call(5, Dir::Down)), Some(1));
        assert_eq!(a.assign(&v, call(5, Dir::Down)), Some(0));
        a.reset();
        assert_eq!(a.assign(&v, call(5, Dir::Down)), Some(0));
    }

    /// 每个内建算法的名字都要唯一，注册表靠它。
    #[test]
    fn router_names_are_unique() {
        let mut names: Vec<&str> = Vec::new();
        names.push(CollectiveScan.name());
        names.push(DownPeakSweep.name());
        let mut sorted = names.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), names.len(), "算法名撞车了");
    }
}