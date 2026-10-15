//! 事件驱动仿真引擎。
//!
//! 主循环见 `docs/ARCHITECTURE.md` 第 3 节。堆里只有「轿厢数加楼层数」个元素，
//! 全部容器在进循环之前按配置推算的上界预分配，之后不再申请内存。

use crate::config::{Config, Pattern, Strategy};
use crate::motion::{traverse_time, Dir};
use crate::policy::{CarBrief, CarId, HallCall, RunView, SystemView};
use crate::rng::Stream;

/// 一个乘客。
#[derive(Debug, Clone)]
pub struct Pax {
    /// 出发层
    pub from: i16,
    /// 目的层
    pub to: i16,
    /// 到电梯厅的时刻
    pub t_arrive: f64,
    /// 按键策略
    pub strategy: Strategy,
    /// 个人拒挤度，0 最愿意挤，1 最不愿意
    pub tol: f64,
    /// 体重，公斤
    pub weight: f64,
    /// 是否已经登记了外呼
    pub called: bool,
    /// 是否已经上了某台轿厢
    pub boarded_car: Option<CarId>,
    /// 成功上梯的时刻
    pub t_board: f64,
    /// 出梯时刻
    pub t_exit: f64,
    /// 到站但没上去的次数
    pub bypass_count: u32,
    /// 因为挤不上而没上去的次数
    pub declined_count: u32,
    /// 因为没位子而没上去的次数
    pub no_space_count: u32,
    /// 是否走楼梯
    pub used_stairs: bool,
    /// 反向叫梯者的转向层
    pub reversal_floor: Option<i16>,
    /// 是否因为超载僵局退出过轿厢
    pub requeued_after_stall: bool,
}

impl Pax {
    /// 等梯时间。上楼成功时才有意义。
    pub fn wait(&self) -> f64 {
        if self.t_board > 0.0 {
            (self.t_board - self.t_arrive).max(0.0)
        } else {
            0.0
        }
    }

    /// 上梯之后的乘坐时间。
    pub fn ride(&self) -> f64 {
        if self.t_exit > 0.0 && self.t_board > 0.0 {
            (self.t_exit - self.t_board).max(0.0)
        } else {
            0.0
        }
    }

    /// 门到门时间。反向叫梯者的这一项包含绕行。
    pub fn door_to_door(&self) -> f64 {
        if self.t_exit > 0.0 {
            (self.t_exit - self.t_arrive).max(0.0)
        } else {
            0.0
        }
    }
}

/// 轿厢当前处在哪个阶段。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarPhase {
    /// 在楼层上停着，门关着
    Idle,
    /// 在两层之间
    Moving,
    /// 门开着，正在上下客
    Boarding,
    /// 超载报警，没人退出
    Stalled,
}

/// 一台轿厢的运行时状态。
#[derive(Debug, Clone)]
pub struct Car {
    pub id: CarId,
    /// 当前所在层，小数
    pub level: f32,
    /// 当前运动方向
    pub dir: Dir,
    pub phase: CarPhase,
    /// 下一次推进的时刻
    pub next_event: f64,
    /// 轿厢里的乘客
    pub occupants: Vec<usize>,
    /// 名下未服务的外呼编号
    pub assigned: Vec<u32>,
    /// 载重，公斤
    pub load_kg: f64,
    /// 本次停靠的上车人数
    pub last_boarded: usize,
    /// 本次停靠的下车人数
    pub last_alighted: usize,
    /// 本次停靠开始时刻
    pub stop_started: f64,
    /// 僵持开始的时刻
    pub stall_since: f64,
    /// 上一趟完成的时刻，用来算往返时间
    pub trip_start: f64,
    /// 累计往返时间样本
    pub rtt_total: f64,
    pub rtt_count: u64,
    /// 完成了几趟
    pub trips: u64,
    /// 当前这条腿的目标层。腿是「从起点一次走到这里，中途只在需要时停」的一段
    pub leg_target: i16,
    /// 这条腿的起点层
    pub leg_from: i16,
    /// 这条腿的起点时刻
    pub leg_start: f64,
    /// 这条腿要走的距离，米
    pub leg_dist: f64,
    /// 这条腿上已经预定要停的楼层。由外呼指派与制动可行性共同决定
    pub stops_planned: Vec<(i16, Dir)>,
    /// 幽灵停站次数：开门了却没人上、也没人下
    pub ghost_stops: u32,
    /// 未满载就发车、而层上还有人等的停靠次数
    pub partial_departures: u32,
    /// 因为制动来不及而跳过的外呼层次数
    pub overshoots: u32,
}

/// 一条登记在案的外呼。
#[derive(Debug, Clone)]
pub struct Call {
    pub id: u32,
    pub level: i16,
    pub dir: Dir,
    pub owner: Option<CarId>,
    /// 同层同方向有多少人在等，用于派梯算法参考
    pub waiting: usize,
    /// 是否由乘客独立面板呼叫，若是则该呼叫只属于这一台车
    pub exclusive: bool,
}

/// 每层的等待队列。
///
/// `waiting[head..head+next_arrival]` 是还没到电梯厅的乘客，按到达时刻排序。
/// `waiting[head..]` 里已经到场的排在前面。
/// 这样「到点进场」就是一次指针推进，不需要每次重建队列。
#[derive(Debug, Clone, Default)]
pub struct FloorQueue {
    /// 已登记的乘客，按到达时刻升序
    pub waiting: Vec<usize>,
    /// `waiting` 里前多少个已经到场。前 `arrived` 个站在电梯厅里，
    /// 其后还在路上。上客循环只扫前一段
    pub arrived: usize,
}

/// 造视图用的预分配缓冲。反复复用，运行期不再分配。
#[derive(Debug, Clone, Default)]
pub struct Scratch {
    pub(crate) briefs: Vec<CarBrief>,
    pub(crate) all: Vec<HallCall>,
    pub(crate) pending: Vec<HallCall>,
    pub(crate) q: Vec<u32>,
    /// 当前轿厢的目的楼层
    pub(crate) car_calls: Vec<i16>,
    /// 当前轿厢名下的外呼
    pub(crate) mine: Vec<HallCall>,
    pub(crate) top_level: i16,
}

impl Scratch {
    pub fn new(n_cars: usize, n_levels: usize) -> Self {
        Self {
            briefs: Vec::with_capacity(n_cars),
            all: Vec::with_capacity(n_levels * 2),
            pending: Vec::with_capacity(n_levels * 2),
            q: Vec::with_capacity(n_levels),
            car_calls: Vec::with_capacity(32),
            mine: Vec::with_capacity(16),
            top_level: 0,
        }
    }
}

/// 一层楼上的外呼登记表。按 `(level, dir)` 去重，红线 1 在这里落地。
///
/// 外呼的 id 就是它在 `calls` 里的下标，永不删除，所以 `get` 是 O(1)。
#[derive(Debug, Clone, Default)]
pub struct CallBoard {
    /// 每层每方向一条外呼。下标由 `Call::slot_index` 算出
    pub slots: Vec<Option<u32>>,
    pub calls: Vec<Call>,
    /// 地下层数。槽位下标要把它算进去
    pub basement: i16,
}

impl Call {
    /// 外呼槽位的下标。
    ///
    /// 楼层 `level` 的队列下标是 `level + basement`。主入口是 1 层，
    /// 地下层是负数，两者都落在合法区间内。
    #[inline]
    pub fn slot_index(level: i16, dir: Dir, basement: i16) -> usize {
        let d = match dir {
            Dir::Up => 1,
            _ => 0,
        };
        (level + basement).max(0) as usize * 2 + d
    }
}

impl CallBoard {
    fn new(n_slots: usize, basement: i16) -> Self {
        Self { slots: vec![None; n_slots], calls: Vec::new(), basement }
    }

    #[inline]
    fn slot(&self, level: i16, dir: Dir) -> usize {
        Call::slot_index(level, dir, self.basement)
    }

    /// 按下按钮。返回这条外呼的编号，以及它是不是新建的。
    ///
    /// 这是红线 1 的核心：同一个 `(楼层, 方向)` 上第二次按，不会产生第二条外呼，
    /// 也不会召来第二台轿厢，只是等待人数加一。
    pub fn register(&mut self, level: i16, dir: Dir, waiting: usize, exclusive: bool) -> (u32, bool) {
        let s = self.slot(level, dir);
        if let Some(id) = self.slots[s] {
            self.calls[id as usize].waiting = waiting;
            return (id, false);
        }
        let id = self.calls.len() as u32;
        self.calls.push(Call { id, level, dir, owner: None, waiting, exclusive });
        self.slots[s] = Some(id);
        (id, true)
    }

    pub fn clear_slot(&mut self, level: i16, dir: Dir) {
        let s = self.slot(level, dir);
        self.slots[s] = None;
    }

    pub fn get(&self, id: u32) -> Option<&Call> {
        self.calls.get(id as usize)
    }

    pub fn get_mut(&mut self, id: u32) -> Option<&mut Call> {
        self.calls.get_mut(id as usize)
    }
}

/// 仿真世界的全部状态。
pub struct World {
    pub cfg: Config,
    pub pax: Vec<Pax>,
    pub cars: Vec<Car>,
    pub calls: CallBoard,
    pub queues: Vec<FloorQueue>,
    /// 地下层数
    pub basement: i16,
    /// 最高住宅层
    pub top: i16,
    /// 楼层槽位总数，从 `-basement` 到 `top`
    pub n_levels: usize,
    /// 每台轿厢服务哪些楼层。下标 `[car][level + basement]`
    pub car_stops: Vec<Vec<bool>>,
    /// 当前仿真时刻，秒
    pub(crate) now: f64,
}

impl World {
    /// 按配置把世界造出来，含全部乘客。
    ///
    /// 乘客的到达时刻、目的层、拒挤度、体重、策略全部在这里抽完。
    /// 之后不管换什么调度算法，楼里都是同一批人。这是公平比较的前提。
    pub fn new(cfg: Config) -> Self {
        let basement = cfg.building.basement_levels.max(0);
        let top = cfg.building.floors;
        let n_levels = (top + basement + 1) as usize;
        let cfg = cfg.sanitized();

        let mut w = World {
            pax: Vec::with_capacity(cfg.expected_trips() + 16),
            cars: Vec::with_capacity(cfg.cars.len()),
            calls: CallBoard::new(n_levels * 2, basement),
            queues: (0..n_levels)
                .map(|i| FloorQueue {
                    // 队列下标 i 对应楼层 `i - basement`。住宅层从 1 起，所以这里要减掉地下层数
                    waiting: Vec::with_capacity(cfg.building.residents_at(i as i16 - basement).max(4) + 4),
                    arrived: 0,
                })
                .collect(),
            basement,
            top,
            n_levels,
            car_stops: Vec::new(),
            now: 0.0,
            cfg,
        };
        w.build_stop_sets();
        w.build_cars();
        w.build_pax();
        w
    }

    /// 把每台轿厢的停靠描述展开成位图，运行期查起来是一次数组访问。
    fn build_stop_sets(&mut self) {
        let basement = self.basement;
        self.car_stops = self
            .cfg
            .cars
            .iter()
            .map(|spec| {
                (0..self.n_levels)
                    .map(|i| spec.stops.serves(i as i16 - basement))
                    .collect()
            })
            .collect();
    }

    /// 这台轿厢停不停这一层。
    #[inline]
    pub fn car_serves(&self, id: CarId, level: i16) -> bool {
        if level < -self.basement || level > self.top {
            return false;
        }
        self.car_stops[id][(level + self.basement) as usize]
    }

    /// 这一层有哪些轿厢愿意停，按 `assigned` 少的排前面。
    pub fn serving_cars_least_busy(&self, level: i16) -> Vec<CarId> {
        let mut ids: Vec<CarId> = (0..self.cars.len())
            .filter(|&i| self.car_serves(i, level) && self.cars[i].phase != CarPhase::Stalled)
            .collect();
        ids.sort_by_key(|&i| (self.cars[i].assigned.len(), i));
        ids
    }

    fn build_cars(&mut self) {
        let n_levels = self.n_levels;
        for (i, spec) in self.cfg.cars.iter().enumerate() {
            self.cars.push(Car {
                id: i,
                level: spec.initial_level as f32,
                dir: Dir::None,
                phase: CarPhase::Idle,
                next_event: f64::INFINITY,
                occupants: Vec::with_capacity(self.cfg.boarding.capacity_crush(spec.capacity_rated) + 2),
                assigned: Vec::with_capacity(n_slots_for(&self.cfg)),
                load_kg: 0.0,
                last_boarded: 0,
                last_alighted: 0,
                stop_started: 0.0,
                stall_since: 0.0,
                trip_start: 0.0,
                rtt_total: 0.0,
                rtt_count: 0,
                trips: 0,
                leg_target: 0,
                leg_from: 0,
                leg_start: 0.0,
                leg_dist: 0.0,
                stops_planned: Vec::with_capacity(n_levels * 2),
                ghost_stops: 0,
                partial_departures: 0,
                overshoots: 0,
            });
        }
    }

    /// 生成全部乘客。这是「同一批人」的来源。
    fn build_pax(&mut self) {
        let mut rng = crate::rng::Streams::new(self.cfg.sim.seed);
        let pattern = self.cfg.traffic.pattern;
        let window = self.cfg.traffic.window_s;
        let horizon = self.cfg.sim.horizon_s.max(window);
        let overload = &self.cfg.overload;
        let stairs = &self.cfg.stairs;
        // 每层待进场的乘客，按到达时刻排好。造完立刻搬进队列，不进 World
        let mut arrivals: Vec<Vec<usize>> = vec![Vec::new(); self.n_levels];

        // 逐层生成。每层内部再按同行团聚合。
        for level in 1..=self.cfg.building.floors {
            let residents = self.cfg.building.residents_at(level);
            if residents == 0 {
                continue;
            }
            let down_target: i16 = if self.cfg.building.basement_levels > 0 && rng.chance(Stream::Demand, 0.07) {
                -(1 + rng.below(Stream::Demand, self.cfg.building.basement_levels.min(2) as u32) as i16)
            } else {
                0
            };
            // 主方向。早高峰往下，晚高峰往上
            let primary_up = matches!(pattern, Pattern::EveningUp);
            let depart_dir = if primary_up { Dir::Up } else { Dir::Down };
            // 地下车库只能往上开
            let dest_dir = if down_target < 0 { Dir::Up } else { depart_dir };

            let mut remaining = residents;
            while remaining > 0 {
                // 户均人数 2 到 4 之间取一个整数，剩下的住户留给下一户
                let base = self.cfg.building.persons_per_unit.round() as usize;
                let size = (base.clamp(1, 4)).min(remaining);
                remaining -= size;

                let go_primary = rng.chance(Stream::Destination, self.cfg.traffic.departure_ratio as f64);
                if !go_primary {
                    continue;
                }
                // 强度曲线上抽一个出发时刻。用钟形的逆变换近似：先均匀再压向中段
                let u = rng.f64(Stream::Arrival);
                let shaped = self.shape_intensity(u);
                let t0 = shaped * window;
                // 同一户的人出门时间略有差异
                let is_child_party = size >= 2 && rng.chance(Stream::Strategy, self.cfg.traffic.child_party_ratio as f64);

                for k in 0..size {
                    let jitter = if k == 0 { 0.0 } else { rng.range(Stream::Arrival, 0.0, 90.0) };
                    let t_arrive = (t0 + jitter).min(horizon);
                    let tol = rng.tolerance();
                    let weight = if is_child_party && k + 1 < size {
                        rng.gens_for_weight(overload.child_weight_kg, overload.weight_cv)
                    } else {
                        rng.gens_for_weight(overload.adult_weight_kg, overload.weight_cv)
                    };
                    let strategy = self.cfg.strategy.pick(rng.f64(Stream::Strategy));
                    // 地下车库的话目的层就是那个负数，否则在大堂
                    let dest_level: i16 = if down_target < 0 { down_target } else { 0 };

                    // 红线 4：低楼层有人走楼梯。概率见 Stairs::probability
                    let est_total = estimate_wait(level, &self.cfg) + estimate_ride(level, &self.cfg);
                    let stair_cost = if dest_dir == Dir::Down { stairs.cost_down(level) } else { stairs.cost_up(level) };
                    let use_stairs = stairs.enabled
                        && rng.chance(Stream::Demand, stairs.probability(level, est_total));

                    let pax = Pax {
                        from: level,
                        to: dest_level,
                        t_arrive,
                        strategy,
                        tol,
                        weight,
                        called: false,
                        boarded_car: None,
                        t_board: 0.0,
                        t_exit: 0.0,
                        bypass_count: 0,
                        declined_count: 0,
                        no_space_count: 0,
                        used_stairs: use_stairs,
                        reversal_floor: None,
                        requeued_after_stall: false,
                    };
                    // 走楼梯的人不进等待队列，但也留在 pax 里用于统计
                    let idx = self.pax.len();
                    self.pax.push(pax);
                    if use_stairs {
                        self.pax[idx].t_board = t_arrive;
                        self.pax[idx].t_exit = t_arrive + stair_cost;
                    } else {
                        arrivals[level as usize].push(idx);
                    }
                }
            }
        }

        // 同一层里到达时刻相近的乘客并成一个同行团，见 MODEL.md 第 4.4 节
        for lvl in 0..self.n_levels {
            let mut v = std::mem::take(&mut arrivals[lvl]);
            v.sort_by(|&a, &b| {
                self.pax[a].t_arrive.total_cmp(&self.pax[b].t_arrive).then(a.cmp(&b))
            });
            // 整张表搬进队列。arrived=0 表示还没人进场
            self.queues[lvl].waiting = v;
            self.queues[lvl].arrived = 0;
        }
    }

    /// 钟形强度。把均匀分布压向中段，模拟高峰的集中。
    ///
    /// 均匀变量 `u` 经 Beta(α, β) 的逆 CDF 映射成出行时刻占比。默认
    /// `α = β = 2.5`，单峰、峰位在正中。用二分求逆，够快也够稳。
    fn shape_intensity(&self, u: f64) -> f64 {
        let (a, b) = (self.cfg.traffic.peak_alpha as f64, self.cfg.traffic.peak_beta as f64);
        let (mut lo, mut hi) = (0.0f64, 1.0f64);
        for _ in 0..24 {
            let mid = 0.5 * (lo + hi);
            if beta_cdf(mid, a, b) < u {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// 某层在队列里的下标。
    #[inline]
    pub fn qidx(&self, level: i16) -> usize {
        (level + self.basement) as usize
    }

    fn car_brief(&self, id: CarId) -> CarBrief {
        let c = &self.cars[id];
        CarBrief {
            id,
            level: c.level,
            dir: c.dir,
            load: c.occupants.len(),
            capacity: self.cfg.cars[id].capacity_rated,
            pending_calls: c.assigned.len(),
            stalled: c.phase == CarPhase::Stalled,
            stall_for: if c.phase == CarPhase::Stalled { self.now - c.stall_since } else { 0.0 },
        }
    }

    /// 填充全机视图的缓冲。这些 Vec 预先分配好，反复复用，运行期不再分配。
    pub fn fill_views(&self, sv: &mut Scratch) {
        sv.briefs.clear();
        for i in 0..self.cars.len() {
            sv.briefs.push(self.car_brief(i));
        }
        sv.all.clear();
        for c in &self.calls.calls {
            sv.all.push(HallCall { id: c.id, level: c.level, dir: c.dir, owner: c.owner, waiting: c.waiting });
        }
        sv.pending.clear();
        sv.pending.extend(sv.all.iter().copied().filter(|c| c.owner.is_none()));
        sv.q.clear();
        for q in &self.queues {
            sv.q.push(q.waiting.len() as u32);
        }
        sv.top_level = self.top;
    }

    /// 填一台轿厢的详细视图。填充缓冲区后借用切片，运行期不分配。
    pub(crate) fn fill_run_view<'a>(&'a self, sv: &'a mut Scratch, id: CarId) -> RunView<'a> {
        let spec = &self.cfg.cars[id];
        let car = &self.cars[id];
        // 内选楼层由在梯乘客的目的层现算。早先有一个 Car::car_calls 字段，
        // 但从来没人往里写，于是 will_alight 对算法永远回答 false
        sv.car_calls.clear();
        sv.car_calls.extend(car.occupants.iter().map(|&p| self.pax[p].to));
        sv.car_calls.sort_unstable();
        sv.mine.clear();
        for cid in &car.assigned {
            if let Some(c) = self.calls.get(*cid) {
                sv.mine.push(HallCall { id: c.id, level: c.level, dir: c.dir, owner: c.owner, waiting: c.waiting });
            }
        }
        RunView {
            now: self.now,
            car: id,
            at_level: car.level,
            heading: car.dir,
            load: car.occupants.len(),
            capacity_rated: spec.capacity_rated,
            capacity_crush: self.cfg.boarding.capacity_crush(spec.capacity_rated),
            load_kg: car.load_kg as f32,
            car_calls: &sv.car_calls,
            mine: &sv.mine,
            system: SystemView {
                now: self.now,
                cars: &sv.briefs,
                pending: &sv.pending,
                all_calls: &sv.all,
                queue_len: &sv.q,
                top_level: sv.top_level,
                n_cars: self.cars.len(),
            },
            floor_height: self.cfg.building.floor_height,
        }
    }
}

fn n_slots_for(cfg: &Config) -> usize {
    (cfg.building.floors + cfg.building.basement_levels.max(0) + 1) as usize * 2
}

/// Beta 分布的累积分布，用连分数算。
fn beta_cdf(x: f64, a: f64, b: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    if x >= 1.0 {
        return 1.0;
    }
    let ln_beta = ln_gamma(a + b) - ln_gamma(a) - ln_gamma(b);
    let front = (ln_beta + a * x.ln() + b * (1.0 - x).ln()).exp();
    if x < (a + 1.0) / (a + b + 2.0) {
        front * betacf(a, b, x) / a
    } else {
        1.0 - front * betacf(b, a, 1.0 - x) / b
    }
}

fn betacf(a: f64, b: f64, x: f64) -> f64 {
    let (qab, qap, qam) = (a + b, a + 1.0, a - 1.0);
    let (mut c, mut d) = (1.0f64, 1.0 - qab * x / qap);
    if d.abs() < 1e-30 {
        d = 1e-30;
    }
    d = 1.0 / d;
    let mut h = d;
    for m in 1..=64 {
        let m = m as f64;
        let m2 = 2.0 * m;
        let aa = m * (b - m) * x / ((qam + m2) * (a + m2));
        d = 1.0 + aa * d;
        if d.abs() < 1e-30 {
            d = 1e-30;
        }
        c = 1.0 + aa / c;
        if c.abs() < 1e-30 {
            c = 1e-30;
        }
        d = 1.0 / d;
        h *= d * c;
        let aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2));
        d = 1.0 + aa * d;
        if d.abs() < 1e-30 {
            d = 1e-30;
        }
        c = 1.0 + aa / c;
        if c.abs() < 1e-30 {
            c = 1e-30;
        }
        d = 1.0 / d;
        let del = d * c;
        h *= del;
        if (del - 1.0).abs() < 1e-12 {
            break;
        }
    }
    h
}

/// Lanczos 近似的 ln Γ。
fn ln_gamma(z: f64) -> f64 {
    const COF: [f64; 9] = [
        0.999_999_999_999_809_9,
        676.520_368_121_885_1,
        -1259.139_216_722_402_8,
        771.323_428_777_653_1,
        -176.615_029_162_140_6,
        12.507_343_278_686_905,
        -0.138_571_095_265_720_12,
        9.984_369_578_019_572e-6,
        1.505_632_735_149_311_6e-7,
    ];
    if z < 0.5 {
        (std::f64::consts::PI / (std::f64::consts::PI * z).sin()).ln() - ln_gamma(1.0 - z)
    } else {
        let z = z - 1.0;
        let mut x = COF[0];
        for (i, c) in COF.iter().enumerate().skip(1) {
            x += c / (z + i as f64);
        }
        let t = z + 7.5;
        0.5 * (2.0 * std::f64::consts::PI).ln() + (z + 0.5) * t.ln() - t + x.ln()
    }
}

/// 高峰期一台轿厢一趟要服务多少层停靠。
///
/// 早高峰下行时轿厢满载往下扫，途中几乎每层都有人要下。但不是每层都有人，
/// 也有部分楼层被满载直驶掠过，所以取个系数并封顶。
fn stops_per_trip(target_level: i16, cfg: &Config) -> f64 {
    let _ = cfg;
    (target_level.max(0) as f64 * 0.65).min(22.0)
}

/// 粗略估一下某层等梯要多久。
///
/// 这只是乘客脑子里的先验信念，不是仿真算出来的真实值。它的作用有两个：
/// 一是决定走不走楼梯，二是给派梯算法一个兜底代价。
///
/// 关键是别把它估得太乐观。33 层 6 户 2 梯的楼，早高峰等两分钟是常事，
/// 而一轮往返本身就要三分钟以上。如果按「半个往返」去估，高层住户会算出
/// 走楼梯很划算，那是错的。
fn estimate_wait(level: i16, cfg: &Config) -> f64 {
    let spec = &cfg.cars[0];
    let height = level.abs() as f64 * cfg.building.floor_height as f64;
    let one_way = traverse_time(height, spec.down).time;
    let per_stop = spec.doors.stop_overhead();
    let stops = stops_per_trip(level, cfg);
    // 一轮往返：上行接人加下行送人，各停一次
    let rtt = 2.0 * one_way + 2.0 * stops * per_stop;
    // 层越高越久
    let height_factor = 1.0 + level.abs() as f64 / cfg.building.floors.max(1) as f64 * 0.4;
    // 多几台梯能降低等待，但不是线性降低。用平方根，再封一个下限。
    let n_cars = cfg.cars.len().max(1) as f64;
    let relief = (n_cars.sqrt()).max(1.0);
    (0.5 * rtt * height_factor / relief).max(per_stop)
}

/// 粗略估一下从某层坐下去要多久。途中要停很多次，每次都是白花的时间。
fn estimate_ride(level: i16, cfg: &Config) -> f64 {
    let spec = &cfg.cars[0];
    let height = level.abs() as f64 * cfg.building.floor_height as f64;
    let one_way = traverse_time(height, spec.down).time;
    let per_stop = spec.doors.stop_overhead();
    // 下行途中自己还要被顺路捎带的楼层买单
    let stops_above = (level.max(0) as f64 * 0.4).min(14.0);
    one_way + stops_above * per_stop + per_stop
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beta_cdf_endpoints() {
        assert!(beta_cdf(0.0, 2.5, 2.5).abs() < 1e-9);
        assert!((beta_cdf(1.0, 2.5, 2.5) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn beta_cdf_is_monotonic() {
        let mut prev = 0.0;
        for i in 1..=50 {
            let v = beta_cdf(i as f64 / 50.0, 2.5, 2.5);
            assert!(v >= prev, "CDF 不该下降");
            prev = v;
        }
    }

    /// Beta(2.5,2.5) 的中位数应是 0.5。
    #[test]
    fn beta_cdf_median_is_half() {
        assert!((beta_cdf(0.5, 2.5, 2.5) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn ln_gamma_matches_factorials() {
        assert!((ln_gamma(1.0) - 0.0).abs() < 1e-9);
        assert!((ln_gamma(5.0) - 24.0f64.ln()).abs() < 1e-9);
        assert!((ln_gamma(0.5) - std::f64::consts::PI.sqrt().ln()).abs() < 1e-9);
    }

    /// 红线 4：层越高越不愿意走楼梯。
    #[test]
    fn stairs_probability_decreases_with_height() {
        let cfg = Config::default();
        let s = &cfg.stairs;
        let mut prev = 1.0;
        for level in 1..=cfg.building.floors {
            let est = estimate_wait(level, &cfg) + estimate_ride(level, &cfg);
            let p = s.probability(level, est);
            assert!(p <= prev + 1e-12, "第 {level} 层的概率不该比上一层更高");
            prev = p;
        }
    }

    /// 红线 4：全楼走楼梯的比例落在 2% 到 5%，十层以上接近零。
    ///
    /// 这条测试是 MODEL.md 第 7.2 节的验收标准。参数一改，它就会报警。
    #[test]
    fn stair_share_lands_in_target_band() {
        // 单次运行只有两百人，抽样噪声大。取二十个种子求平均。
        let mut total = 0usize;
        let mut movers = 0usize;
        for seed in 0..20 {
            let mut cfg = preset_cfg();
            cfg.traffic.departure_ratio = 1.0;
            cfg.sim.seed = seed;
            let w = World::new(cfg);
            total += w.pax.len();
            movers += w.pax.iter().filter(|p| p.used_stairs).count();
        }
        let share = movers as f64 / total as f64;
        assert!(
            (0.02..=0.05).contains(&share),
            "全楼走楼梯比例 {share:.4} 超出 2% 到 5% 的目标区间"
        );
    }

    /// 红线 4：走楼梯的人集中在低层。
    #[test]
    fn stair_users_concentrate_on_low_floors() {
        let mut cfg = preset_cfg();
        cfg.traffic.departure_ratio = 1.0;
        let w = World::new(cfg);
        let low: usize = w.pax.iter().filter(|p| p.used_stairs && p.from <= 6).count();
        let high: usize = w.pax.iter().filter(|p| p.used_stairs && p.from > 9).count();
        assert!(low > 0, "低层应当有人走楼梯");
        assert_eq!(high, 0, "十层以上不该有人走楼梯，实际 {high} 人");
    }

    /// 关掉走楼梯之后一个人都不该走。
    #[test]
    fn disabling_stairs_removes_all_walkers() {
        let mut cfg = preset_cfg();
        cfg.traffic.departure_ratio = 1.0;
        cfg.stairs.enabled = false;
        let w = World::new(cfg);
        assert_eq!(w.pax.iter().filter(|p| p.used_stairs).count(), 0);
    }

    /// 红线 1：同一个楼层同一个方向按两次，只产生一条外呼。
    #[test]
    fn hall_call_dedups_by_level_and_direction() {
        let mut b = CallBoard::new(64, 0);
        let (id1, fresh1) = b.register(10, Dir::Down, 1, false);
        let (id2, fresh2) = b.register(10, Dir::Down, 2, false);
        assert!(fresh1, "第一次按是新呼叫");
        assert!(!fresh2, "第二次按不应新建");
        assert_eq!(id1, id2, "两次应当是同一条外呼");
        assert_eq!(b.calls.len(), 1, "外呼表里只该有一条");
        assert_eq!(b.get(id1).unwrap().waiting, 2, "等待人数要累加");

        // 方向不同则是另一条
        let (id3, fresh3) = b.register(10, Dir::Up, 1, false);
        assert!(fresh3);
        assert_ne!(id3, id1);
        assert_eq!(b.calls.len(), 2);

        // 楼层不同也是另一条
        let (id4, _) = b.register(11, Dir::Down, 1, false);
        assert_ne!(id4, id1);
        assert_eq!(b.calls.len(), 3);
    }

    /// 上下都按时，会产生两条外呼。
    #[test]
    fn press_both_registers_two_calls() {
        let mut b = CallBoard::new(64, 0);
        let (up, fresh_up) = b.register(7, Dir::Up, 1, false);
        let (down, fresh_down) = b.register(7, Dir::Down, 1, false);
        assert!(fresh_up && fresh_down, "两条都应当是新建");
        assert_ne!(up, down, "上下两个方向是两条外呼");
    }

    #[test]
    fn clear_slot_allows_new_call_after_service() {
        let mut b = CallBoard::new(64, 0);
        let (id1, _) = b.register(5, Dir::Down, 1, false);
        b.clear_slot(5, Dir::Down);
        let (id2, fresh) = b.register(5, Dir::Down, 1, false);
        assert!(fresh, "清掉槽位后应当能新建");
        assert_ne!(id1, id2);
    }

    /// 清槽之后旧编号不复活。register 只认槽位，不回扫历史记录，
    /// 否则服务过的呼叫会被下一次按键重新点亮，幽灵停站统计失效。
    #[test]
    fn cleared_call_id_does_not_resurrect() {
        let mut b = CallBoard::new(64, 0);
        let (id1, _) = b.register(5, Dir::Down, 1, false);
        b.clear_slot(5, Dir::Down);
        let (id2, _) = b.register(5, Dir::Down, 1, false);
        assert_ne!(id1, id2, "旧编号不该被复用");
        assert!(b.get(id1).is_some_and(|c| c.level == 5), "记录本身仍在，供指标统计");
    }

    /// 同一个种子两次造世界，乘客完全一致。
    #[test]
    fn same_seed_same_passengers() {
        let cfg = preset_cfg();
        let w1 = World::new(cfg.clone());
        let w2 = World::new(cfg);
        assert_eq!(w1.pax.len(), w2.pax.len());
        for (a, b) in w1.pax.iter().zip(w2.pax.iter()) {
            assert_eq!(a.from, b.from);
            assert_eq!(a.to, b.to);
            assert_eq!(a.t_arrive.to_bits(), b.t_arrive.to_bits());
            assert_eq!(a.tol.to_bits(), b.tol.to_bits());
            assert_eq!(a.weight.to_bits(), b.weight.to_bits());
        }
    }

    #[test]
    fn different_seed_gives_different_passengers() {
        let mut c1 = preset_cfg();
        let mut c2 = c1.clone();
        c1.sim.seed = 1;
        c2.sim.seed = 2;
        let w1 = World::new(c1);
        let w2 = World::new(c2);
        let a: Vec<u64> = w1.pax.iter().map(|p| p.t_arrive.to_bits()).collect();
        let b: Vec<u64> = w2.pax.iter().map(|p| p.t_arrive.to_bits()).collect();
        assert_ne!(a, b);
    }

    #[test]
    fn stair_users_are_marked_and_others_queued() {
        let w = World::new(preset_cfg());
        let stairs = w.pax.iter().filter(|p| p.used_stairs).count();
        assert!(stairs > 0, "低楼层应当有人走楼梯");
        for p in &w.pax {
            if p.used_stairs {
                assert!(p.t_exit > 0.0, "走楼梯的人要有出梯时刻");
            }
        }
    }

    #[test]
    fn arrivals_are_sorted_per_floor() {
        let w = World::new(preset_cfg());
        for lvl in 0..w.n_levels {
            let v = &w.queues[lvl].waiting;
            for i in 1..v.len() {
                assert!(
                    w.pax[v[i - 1]].t_arrive.total_cmp(&w.pax[v[i]].t_arrive) != std::cmp::Ordering::Greater,
                    "第 {lvl} 层的到达事件没有排好"
                );
            }
        }
    }

    #[test]
    fn world_fits_configured_bounds() {
        let cfg = preset_cfg();
        let w = World::new(cfg.clone());
        assert_eq!(w.cars.len(), cfg.cars.len());
        assert_eq!(w.queues.len(), w.n_levels);
        assert_eq!(w.calls.calls.capacity(), 0, "预分配后不该立刻扩容");
    }

    fn preset_cfg() -> Config {
        crate::config::preset("std_residential").unwrap()
    }
}