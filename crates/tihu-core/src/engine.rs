//! 事件驱动仿真引擎。
//!
//! 主循环见 `docs/ARCHITECTURE.md` 第 3 节。堆里只有「轿厢数加楼层数」个元素，
//! 全部容器在进循环之前按配置推算的上界预分配，之后不再申请内存。

use crate::config::{Config, Pattern, Strategy, LOBBY};
use crate::motion::{traverse_time, Dir};
use crate::policy::{CarBrief, CarId, HallCall, RunView, SystemView};
use crate::rng::Stream;

/// 一条行程腿：从 `from` 层去 `to` 层。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hop {
    pub from: i16,
    pub to: i16,
}

/// 一个乘客。
#[derive(Debug, Clone)]
pub struct Pax {
    /// 预生成的行程腿。最多两条：出门，以及过一阵子回来。
    /// 两条腿的乘客就是条件性需求：到达目的地后停留一段时间，再从那里返回
    pub hops: [Hop; 2],
    /// 实际有几条腿
    pub n_hops: u8,
    /// 当前走到第几条腿
    pub hop_idx: u8,
    /// 当前腿的出发层
    pub from: i16,
    /// 当前腿的目的层
    pub to: i16,
    /// 当前腿到电梯厅的时刻
    pub t_arrive: f64,
    /// 到站后在原地停留多久再走下一腿，秒
    pub dwell_s: f64,
    /// 愿意在电梯厅等多久。超过就改走楼梯
    pub patience_s: f64,
    /// 当前这条腿的耐心截止时刻。换腿和僵局重排都会往后推，
    /// 旧事件到点就忽略，只认这一个，见 `abandon_to_stairs`
    pub abandon_at: f64,
    /// 按键策略
    pub strategy: Strategy,
    /// 个人拒挤度，0 最愿意挤，1 最不愿意
    pub tol: f64,
    /// 体重，公斤
    pub weight: f64,
    /// 是否已经上了某台轿厢
    pub boarded_car: Option<CarId>,
    /// 当前腿成功上梯的时刻
    pub t_board: f64,
    /// 当前腿的出梯时刻
    pub t_exit: f64,
    /// 当前这条腿因为不肯挤而没上去的次数
    pub declined_count: u32,
    /// 当前这条腿因为没位子而没上去的次数
    pub no_space_count: u32,
    /// 已经走完的腿累计的次数。出梯时从单腿计数搬过来再清零，人物级的
    /// 统计读两者之和。换腿要清零单腿计数，累计值不能跟着清
    pub declined_total: u32,
    pub no_space_total: u32,
    /// 出发前就决定走楼梯，见红线 4
    pub used_stairs: bool,
    /// 等得不耐烦改走楼梯。与 `used_stairs` 分开统计
    pub abandoned: bool,
    /// 反向叫梯者的转向层
    pub reversal_floor: Option<i16>,
    /// 是否因为超载僵局退出过轿厢
    pub requeued_after_stall: bool,
    /// 走完了几条腿
    pub hops_done: u32,
    /// 整个行程结束的时刻。还有下一腿时为 0
    pub final_exit: f64,
}

impl Pax {
    /// 这一条腿的等梯时间。
    pub fn wait(&self) -> f64 {
        if self.t_board > 0.0 {
            (self.t_board - self.t_arrive).max(0.0)
        } else {
            0.0
        }
    }

    /// 这一条腿上梯之后的乘坐时间。
    pub fn ride(&self) -> f64 {
        if self.t_exit > 0.0 && self.t_board > 0.0 {
            (self.t_exit - self.t_board).max(0.0)
        } else {
            0.0
        }
    }

    /// 这一条腿的门到门时间。反向叫梯者的这一项包含绕行。
    pub fn door_to_door(&self) -> f64 {
        if self.t_exit > 0.0 {
            (self.t_exit - self.t_arrive).max(0.0)
        } else {
            0.0
        }
    }

    /// 这位乘客此刻还在电梯厅等吗。
    pub fn waiting(&self) -> bool {
        self.t_exit <= 0.0 && !self.used_stairs && !self.abandoned && self.boarded_car.is_none()
    }

    /// 这位乘客在 `level` 层会不会按 `dir` 这个方向的按钮。
    ///
    /// 与真实目的方向不同。只按上行的下行乘客会按上行，双按的人两边都按。
    /// 判定「这一层这个方向还有没有人等」必须用这个，不能再用 `to > level`。
    pub fn presses(&self, level: i16, dir: Dir) -> bool {
        match self.strategy {
            Strategy::Honest | Strategy::MultiCall => match dir {
                Dir::Up => self.to > level,
                Dir::Down => self.to < level,
                Dir::None => true,
            },
            Strategy::PressBoth => match dir {
                Dir::Up | Dir::Down => self.to != level,
                Dir::None => true,
            },
            Strategy::PressUpOnly => match dir {
                Dir::Up => self.to != level,
                Dir::Down => false,
                Dir::None => true,
            },
        }
    }

    /// 当前这条腿被跳过几次。不肯挤与没位子各算一次。
    pub fn leg_bypass(&self) -> u32 {
        self.declined_count + self.no_space_count
    }

    /// 整个行程被跳过几次，含已经走完的腿。
    pub fn bypass(&self) -> u32 {
        self.declined_total + self.no_space_total + self.leg_bypass()
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
    /// 策略要求延长开门。新一轮到站的人可以在这段时间里直接上车
    pub holding: bool,
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
    /// 刚在哪一层收过尾。停靠完还停在这一层时，不许原地再开一次门，
    /// 否则车满不了又走不掉，会在同一层反复空停靠卡死（分区梯的死锁就出在这）
    pub settled_level: Option<i16>,
    /// 幽灵停站次数：开门了却没人上、也没人下
    pub ghost_stops: u32,
    /// 未满载就发车、而层上还有人等的停靠次数
    pub partial_departures: u32,
    /// 因为制动来不及而跳过的外呼层次数
    pub overshoots: u32,
    /// 满载直驶次数：满员时路过一条还亮着的同向外呼却没停
    pub full_bypasses: u32,
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
    /// 归到一台卡死的轿厢名下期间，层上的等待者全部放弃过。
    /// 只给 `calls_orphaned` 用，保证一条外呼最多记一次
    pub orphaned: bool,
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
    /// 事件之间复用的外呼归属表，供重分配用
    pub(crate) owned: Vec<Option<CarId>>,
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
            owned: Vec::with_capacity(n_levels * 2),
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
    /// 算法看到的形状。新增字段时只改这里，两个视图与派梯入口一起跟上
    pub fn view(&self, waiting: usize) -> HallCall {
        HallCall {
            id: self.id,
            level: self.level,
            dir: self.dir,
            owner: self.owner,
            waiting,
        }
    }

    /// 外呼槽位的下标。
    ///
    /// 楼层 `level` 的队列下标是 `level + basement`。主入口是 1 层，
    /// 地下层是负数，两者都落在合法区间内。
    #[inline]
    pub fn slot_index(level: i16, dir: Dir, basement: i16) -> usize {
        let d = match dir {
            Dir::Up => 1,
            Dir::Down | Dir::None => 0,
        };
        (level + basement) as usize * 2 + d
    }
}

impl CallBoard {
    fn new(n_slots: usize, basement: i16) -> Self {
        Self {
            slots: vec![None; n_slots],
            calls: Vec::new(),
            basement,
        }
    }

    #[inline]
    fn slot(&self, level: i16, dir: Dir) -> usize {
        Call::slot_index(level, dir, self.basement)
    }

    /// 按下按钮。返回这条外呼的编号，以及它是不是新建的。
    ///
    /// 这是红线 1 的核心：同一个 `(楼层, 方向)` 上第二次按，不会产生第二条外呼，
    /// 也不会召来第二台轿厢，只是等待人数加一。
    ///
    /// 独立面板的多呼叫走另一条：同一个 `(楼层, 方向)` 可以并存两条外呼，
    /// 各属于一台车，不走槽位去重。红线 2 在这里与红线 1 分道。
    pub fn register(
        &mut self,
        level: i16,
        dir: Dir,
        waiting: usize,
        exclusive: bool,
    ) -> (u32, bool) {
        if exclusive {
            let id = self.calls.len() as u32;
            self.calls.push(Call {
                id,
                level,
                dir,
                owner: None,
                waiting,
                exclusive: true,
                orphaned: false,
            });
            return (id, true);
        }
        let s = self.slot(level, dir);
        if let Some(id) = self.slots[s] {
            self.calls[id as usize].waiting = waiting;
            return (id, false);
        }
        let id = self.calls.len() as u32;
        self.calls.push(Call {
            id,
            level,
            dir,
            owner: None,
            waiting,
            exclusive,
            orphaned: false,
        });
        self.slots[s] = Some(id);
        (id, true)
    }

    /// 到站消号：属于 `owner` 在 `level` 层、`dir` 方向的独立呼叫消号。
    ///
    /// 只在「这层这个方向确实没人再等」时由调用方触发。共享槽位不动，
    /// 别人的独立呼叫也不动。消号只清归属，记录本身保留，
    /// 外呼编号永不复用的约定不变。
    pub fn clear_exclusive_owned_by(&mut self, owner: CarId, level: i16, dir: Dir) {
        for c in &mut self.calls {
            if c.exclusive && c.level == level && c.dir == dir && c.owner == Some(owner) {
                c.owner = None;
            }
        }
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
    /// 每层电梯厅里此刻在等的人。无序，进出一个乘客都是常数级
    pub hall: Vec<Vec<usize>>,
    /// 地下层数
    pub basement: i16,
    /// 最高住宅层
    pub top: i16,
    /// 楼层槽位总数，从 `-basement` 到 `top`
    pub n_levels: usize,
    /// 每台轿厢服务哪些楼层。下标 `[car][level + basement]`
    pub car_stops: Vec<Vec<bool>>,
    /// 下客名单的复用缓冲。停靠时借出去，用完放回，运行期不申请内存
    pub(crate) alight_buf: Vec<usize>,
    /// 当前仿真时刻，秒
    pub(crate) now: f64,
}

impl World {
    /// 按配置把世界造出来，含全部乘客。
    ///
    /// 乘客的到达时刻、目的层、拒挤度、体重、策略全部在这里抽完。
    /// 之后不管换什么调度算法，楼里都是同一批人。这是公平比较的前提。
    pub fn new(cfg: Config) -> Self {
        // 空梯直接拦在这里，不然后面抽样深处索引越界，报错看不出原因。
        // 网页链路先过 validate() 到不了这里，直调内核的研究者看这条。
        assert!(
            !cfg.cars.is_empty(),
            "至少要有一台轿厢，先跑 Config::validate"
        );
        // 先夹取再量几何。反过来会让楼层格位与 `cfg.building.top_level`
        // 对不上：sanitized() 会把小于 2 的 top_level 抬到 2，而 n_levels
        // 是按旧值算的，生成出来的乘客落在数组外面
        let cfg = cfg.sanitized();
        let basement = cfg.building.basement_levels;
        let top = cfg.building.top_level;
        let n_levels = (top + basement + 1) as usize;

        let mut w = World {
            pax: Vec::with_capacity(cfg.expected_trips() + 16),
            cars: Vec::with_capacity(cfg.cars.len()),
            calls: CallBoard::new(n_levels * 2, basement),
            hall: vec![Vec::new(); n_levels],
            basement,
            top,
            n_levels,
            car_stops: Vec::new(),
            alight_buf: Vec::with_capacity(16),
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

    /// 这一层愿意停、又没卡死的最闲的一台。派梯兜底用，不分配内存
    pub fn least_busy_serving(&self, level: i16) -> Option<CarId> {
        (0..self.cars.len())
            .filter(|&i| self.car_serves(i, level) && self.cars[i].phase != CarPhase::Stalled)
            .min_by_key(|&i| (self.cars[i].assigned.len(), i))
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
                occupants: Vec::with_capacity(
                    self.cfg.boarding.capacity_crush(spec.capacity_rated) + 2,
                ),
                assigned: Vec::with_capacity(n_slots_for(&self.cfg)),
                load_kg: 0.0,
                last_boarded: 0,
                last_alighted: 0,
                stop_started: 0.0,
                holding: false,
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
                settled_level: None,
                ghost_stops: 0,
                partial_departures: 0,
                overshoots: 0,
                full_bypasses: 0,
            });
        }
    }

    /// 生成全部乘客。这是「同一批人」的来源。
    fn build_pax(&mut self) {
        let mut rng = crate::rng::Streams::new(self.cfg.sim.seed);
        let pattern = self.cfg.traffic.pattern;
        let window = self.cfg.traffic.window_s;
        let horizon = self.cfg.sim.horizon_s.max(window);
        // 只有这四条用到，先取出来免得反复借 self
        let dep = self.cfg.traffic.departure_ratio as f64;
        let rev = self.cfg.traffic.reverse_ratio as f64;
        let ret = self.cfg.traffic.return_ratio as f64;
        let evening = matches!(pattern, Pattern::EveningUp);
        let mixed = matches!(pattern, Pattern::NoonMixed);
        if matches!(pattern, Pattern::InterFloor) {
            self.build_inter_floor(&mut rng, dep, window, horizon);
            return;
        }
        // 逐户生成。一户出一个同行团，团人数从户均人数涌现出来。
        //
        // 早高峰的主流是「住宅层到出口层」，晚高峰反过来。中午两向都有。
        // 出口层默认是主入口层，有一部分人下到地下车库。
        // 有一小撮人走反向，见 reverse_ratio。
        for level in 1..=self.cfg.building.top_level {
            let units = self.cfg.building.units_at(level);
            if units == 0 {
                continue;
            }
            for _u in 0..units {
                if !rng.chance(Stream::Demand, dep) {
                    continue;
                }
                let size = self.cfg.building.persons_per_unit.round().clamp(1.0, 4.0) as usize;
                // 强度曲线上抽一个出发时刻。用钟形的逆变换近似：先均匀再压向中段
                let t0 = self.shape_intensity(rng.f64(Stream::Arrival)) * window;
                let is_child = size >= 2
                    && rng.chance(Stream::Strategy, self.cfg.traffic.child_party_ratio as f64);
                let outbound = if mixed {
                    rng.chance(Stream::Destination, 0.5)
                } else {
                    !evening
                };
                if outbound {
                    // 住宅层到出口层
                    let exit = pick_entrance_level(&mut rng, &self.cfg.building);
                    let is_return = rng.chance(Stream::Demand, ret);
                    self.emit_party(
                        level, exit, size, t0, is_child, is_return, horizon, &mut rng,
                    );
                } else {
                    // 出口层到住宅层。有一部分人从地下车库上楼回家
                    let ent = pick_entrance_level(&mut rng, &self.cfg.building);
                    let is_return = rng.chance(Stream::Demand, ret);
                    self.emit_party(ent, level, size, t0, is_child, is_return, horizon, &mut rng);
                }
                // 反向细流。按户掷，不用「户数乘比例取整」，
                // 否则小楼里那一撮人会被四舍五入抹成零
                if rng.chance(Stream::Destination, rev) {
                    if evening {
                        let exit = pick_entrance_level(&mut rng, &self.cfg.building);
                        self.emit_party(level, exit, size, t0, false, false, horizon, &mut rng);
                    } else {
                        let ent = pick_entrance_level(&mut rng, &self.cfg.building);
                        self.emit_party(ent, level, size, t0, false, false, horizon, &mut rng);
                    }
                }
            }
        }
    }

    /// 层间交通的生成器。医院、写字楼这类场景用它。
    ///
    /// 客流表里每一股给出起终点与占比。出行人数是「全楼常驻人数 × 出行率」，
    /// 再按占比摊到各股上，每股再按团人数分成若干个同行团。所以
    /// `units_per_floor` 在这个模式下读作「每层几个科室或病房」，
    /// `persons_per_unit` 读作「每间常驻几人」。
    ///
    /// 团人数只决定这些人怎么分组，不能再乘一次：它已经含在常驻人数里了。
    fn build_inter_floor(
        &mut self,
        rng: &mut crate::rng::Streams,
        dep: f64,
        window: f64,
        horizon: f64,
    ) {
        let flows = self.cfg.traffic.flows.clone();
        if flows.is_empty() {
            return;
        }
        let total: f64 = flows.iter().map(|f| f.share.max(0.0)).sum();
        if total <= 0.0 {
            return;
        }
        let size = self.cfg.building.persons_per_unit.round().clamp(1.0, 4.0) as usize;
        let travelers = (self.cfg.building.total_residents() as f64 * dep)
            .round()
            .max(1.0);
        let parties = (travelers / size as f64).round().max(1.0) as usize;
        for f in &flows {
            let n = ((parties as f64) * f.share.max(0.0) / total).round() as usize;
            for _ in 0..n {
                let t0 = self.shape_intensity(rng.f64(Stream::Arrival)) * window;
                self.emit_party(f.from, f.to, size, t0, false, false, horizon, rng);
            }
        }
    }

    /// 造一个同行团。同一个团的到达时刻只差几十秒，见 MODEL.md 第 4.4 节。
    ///
    /// `with_return` 为真时补上第二条腿：到了目的地停一阵，再从那里回来。
    /// 这就是条件性需求。停留时长在生成时用同一份随机流抽定，所以可复现。
    #[allow(clippy::too_many_arguments)]
    fn emit_party(
        &mut self,
        from: i16,
        to: i16,
        size: usize,
        t0: f64,
        is_child: bool,
        with_return: bool,
        horizon: f64,
        rng: &mut crate::rng::Streams,
    ) {
        // 出发层就是目的层的人不用电梯。主入口层住人的时候，他们早高峰出门
        // 抬脚就到，晚高峰回家也不用上楼。生成零长度的行程只会往等待分布里
        // 掺进一批 0 秒的假样本，把均值压低。
        if from == to {
            return;
        }
        let stairs = &self.cfg.stairs;
        let dwell = rng
            .lognormal(self.cfg.traffic.return_dwell_s.max(1.0), 0.5)
            .max(30.0);
        let patience = if stairs.enabled && stairs.abandon_after_s > 0.0 {
            rng.lognormal(stairs.abandon_after_s, stairs.abandon_cv.max(0.01))
        } else {
            // 关掉弃梯，或者把阈值设成 0 时，谁都不中途弃梯
            f64::INFINITY
        };
        for k in 0..size {
            let jitter = if k == 0 {
                0.0
            } else {
                rng.range(Stream::Arrival, 0.0, 90.0)
            };
            let t_arrive = (t0 + jitter).min(horizon);
            let tol = rng.tolerance();
            // 带小孩的同行团，尾部那几位走小孩体重那一峰，保证相关性
            let weight = rng.body_weight(&self.cfg.weight, is_child && k + 1 < size);
            let strategy = self.cfg.strategy.pick(rng.f64(Stream::Strategy));
            // 红线 4：低楼层有人走楼梯。这是出发前就定下的，与半路弃梯分开。
            // 跨度按实际行程算，方向也要分：晚高峰从大堂往上爬的人，
            // 走的是 cost_up 那条路。用 from.abs() 会把 1 层到 9 层算成
            // 「下 1 层 12 秒」，于是晚高峰冒出一成多爬楼的人
            let est = estimate_wait(from, &self.cfg) + estimate_ride(from, &self.cfg);
            let span = (to - from).abs();
            let walk_p = if to < from {
                stairs.probability(span, est)
            } else {
                stairs.probability_up(span, est)
            };
            let use_stairs = stairs.enabled && rng.chance(Stream::Demand, walk_p);
            let mut hops = [Hop { from, to }, Hop { from: to, to: from }];
            let n_hops = if with_return { 2 } else { 1 };
            if !with_return {
                hops[1] = hops[0];
            }
            let pax = Pax {
                hops,
                n_hops,
                hop_idx: 0,
                from,
                to,
                t_arrive,
                dwell_s: dwell,
                patience_s: patience,
                strategy,
                tol,
                weight,
                boarded_car: None,
                t_board: 0.0,
                t_exit: 0.0,
                declined_count: 0,
                no_space_count: 0,
                declined_total: 0,
                no_space_total: 0,
                used_stairs: use_stairs,
                abandoned: false,
                reversal_floor: None,
                requeued_after_stall: false,
                abandon_at: f64::INFINITY,
                hops_done: 0,
                final_exit: 0.0,
            };
            let idx = self.pax.len();
            self.pax.push(pax);
            if use_stairs {
                // 走楼梯的人不进电梯厅，但也留在 pax 里用于统计
                let cost = if to < from {
                    stairs.cost_down(from - to)
                } else {
                    stairs.cost_up(to - from)
                };
                self.pax[idx].t_board = t_arrive;
                self.pax[idx].t_exit = t_arrive + cost;
                self.pax[idx].hops_done = 1;
                self.pax[idx].final_exit = self.pax[idx].t_exit;
            }
        }
    }

    /// 钟形强度。把均匀分布压向中段，模拟高峰的集中。
    fn shape_intensity(&self, u: f64) -> f64 {
        let (a, b) = (
            self.cfg.traffic.peak_alpha as f64,
            self.cfg.traffic.peak_beta as f64,
        );
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

    /// 这一层在楼里吗。层号 0 不存在，有地下层时也照样排除。
    ///
    /// `qidx` 只做加法，调用方拿到的层号得先过这一关再当下标用。
    #[inline]
    pub fn in_bounds(&self, level: i16) -> bool {
        level != 0 && level >= -self.basement && level <= self.top
    }

    fn car_brief(&self, id: CarId) -> CarBrief {
        let c = &self.cars[id];
        CarBrief {
            id,
            level: c.level,
            dir: c.dir,
            load: c.occupants.len(),
            load_kg: c.load_kg as f32,
            capacity: self.cfg.cars[id].capacity_rated,
            pending_calls: c.assigned.len(),
            stalled: c.phase == CarPhase::Stalled,
            stall_for: if c.phase == CarPhase::Stalled {
                self.now - c.stall_since
            } else {
                0.0
            },
        }
    }

    /// 这条外呼还算不算数。共享呼叫看槽位还指不指它，独立呼叫看归属还在不在。
    ///
    /// 编号永不复用，灭灯后的旧记录留在 `calls` 里备查，但它们不进算法视图。
    /// 一条已经灭灯的呼叫挂在 `pending` 里，会把派梯算法引到没人的楼层去。
    pub fn call_is_live(&self, c: &Call) -> bool {
        if c.exclusive {
            return c.owner.is_some();
        }
        if c.level < -self.basement || c.level > self.top {
            return false;
        }
        let slot = Call::slot_index(c.level, c.dir, self.basement);
        self.calls.slots.get(slot).copied().flatten() == Some(c.id)
    }

    /// 填充全机视图的缓冲。这些 Vec 预先分配好，反复复用，运行期不再分配。
    pub fn fill_views(&self, sv: &mut Scratch) {
        sv.briefs.clear();
        for i in 0..self.cars.len() {
            sv.briefs.push(self.car_brief(i));
        }
        sv.all.clear();
        for c in &self.calls.calls {
            if !self.call_is_live(c) {
                continue;
            }
            sv.all.push(c.view(c.waiting));
        }
        sv.pending.clear();
        sv.pending
            .extend(sv.all.iter().copied().filter(|c| c.owner.is_none()));
        sv.q.clear();
        for h in &self.hall {
            sv.q.push(h.len() as u32);
        }
    }

    /// 全机视图。字段借预先填好的缓冲，构造本身不分配。
    ///
    /// `top_level` 读世界自己的值，不读缓冲里的快照。第一次 `CarStep` 发生在
    /// 任何事件之前，缓冲还没填过，读快照会拿到 0。
    pub(crate) fn system_view<'a>(&'a self, sv: &'a Scratch) -> SystemView<'a> {
        SystemView {
            now: self.now,
            cars: &sv.briefs,
            pending: &sv.pending,
            all_calls: &sv.all,
            queue_len: &sv.q,
            top_level: self.top,
            n_cars: self.cars.len(),
        }
    }

    /// 填一台轿厢的详细视图。填充缓冲区后借用切片，运行期不分配。
    pub(crate) fn fill_run_view<'a>(&'a self, sv: &'a mut Scratch, id: CarId) -> RunView<'a> {
        let spec = &self.cfg.cars[id];
        let car = &self.cars[id];
        // 内选楼层由在梯乘客的目的层现算。早先有一个 Car::car_calls 字段，
        // 但从来没人往里写，于是 will_alight 对算法永远回答 false
        sv.car_calls.clear();
        sv.car_calls
            .extend(car.occupants.iter().map(|&p| self.pax[p].to));
        sv.car_calls.sort_unstable();
        sv.mine.clear();
        for cid in &car.assigned {
            if let Some(c) = self.calls.get(*cid) {
                sv.mine.push(c.view(c.waiting));
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
            system: self.system_view(&*sv),
            floor_height: self.cfg.building.floor_height,
        }
    }
}

fn n_slots_for(cfg: &Config) -> usize {
    (cfg.building.top_level + cfg.building.basement_levels + 1) as usize * 2
}

/// 主入口层还是地下车库。出门与回家两股客流都用它。
///
/// 绝大多数人去主入口层，少数直接去地下车库。车库占比 7% 是
/// `docs/MODEL.md` 第 4.3 节的取值，待标定。
fn pick_entrance_level(rng: &mut crate::rng::Streams, b: &crate::config::Building) -> i16 {
    let basements = b.basement_levels;
    if basements > 0 && rng.chance(Stream::Demand, crate::config::BASEMENT_ENTRANCE_PROB) {
        -1 - rng.below(Stream::Demand, basements.min(2) as u32) as i16
    } else {
        LOBBY
    }
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
fn stops_per_trip(target_level: i16) -> f64 {
    // TODO(calibration) 0.65 与 22 两个常数还没有出处，是粗估
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
    let stops = stops_per_trip(level);
    // 一轮往返：上行接人加下行送人，各停一次
    let rtt = 2.0 * one_way + 2.0 * stops * per_stop;
    // 层越高越久
    let height_factor = 1.0 + level.abs() as f64 / cfg.building.top_level.max(1) as f64 * 0.4;
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
        for level in 1..=cfg.building.top_level {
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

    /// 晚高峰几乎没人爬高楼。从大堂往上爬，费力是下楼的数倍，
    /// 跨 6 层以上的人不该出现在楼梯间。之前按「下 1 层」决策时，
    /// 晚高峰有 11% 的人走楼梯，平均爬 9 层。
    #[test]
    fn evening_stairs_stay_near_zero() {
        let mut total = 0usize;
        let mut long = 0usize;
        for seed in 0..20 {
            let mut cfg = preset_cfg();
            cfg.traffic.pattern = crate::config::Pattern::EveningUp;
            cfg.traffic.departure_ratio = 1.0;
            cfg.sim.seed = seed;
            let w = World::new(cfg);
            total += w.pax.len();
            long += w
                .pax
                .iter()
                .filter(|p| p.used_stairs && (p.to - p.from).abs() > 6)
                .count();
        }
        assert_eq!(
            long, 0,
            "晚高峰爬 6 层以上的人应当是零，实际 {long} / {total}"
        );
    }

    /// 同样的跨度，往上爬的意愿远低于往下走。
    #[test]
    fn climbing_up_is_less_likely_than_going_down() {
        let s = crate::config::Stairs::default();
        // 下 1 层 12 秒 vs 爬 8 层 101 秒，前者两成走，后者几乎没人走
        assert!(s.probability(1, 60.0) > 0.1);
        assert!(s.probability_up(8, 60.0) < 0.05);
        assert!(s.probability_up(8, 60.0) < s.probability(8, 60.0));
    }

    /// 红线 4：走楼梯的人集中在低层。
    #[test]
    fn stair_users_concentrate_on_low_floors() {
        let mut cfg = preset_cfg();
        cfg.traffic.departure_ratio = 1.0;
        let w = World::new(cfg);
        let low: usize = w
            .pax
            .iter()
            .filter(|p| p.used_stairs && p.from <= 6)
            .count();
        let high: usize = w.pax.iter().filter(|p| p.used_stairs && p.from > 9).count();
        assert!(low > 0, "低层应当有人走楼梯");
        let total = w.pax.len().max(1) as f64;
        assert!(
            (high as f64) / total < 0.01,
            "十层以上走楼梯的人该是零星的，实际 {high} 人"
        );
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
        assert!(
            b.get(id1).is_some_and(|c| c.level == 5),
            "记录本身仍在，供指标统计"
        );
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
        // 乘客不再按楼层预排，而是各排各的进场事件。这里只确认每层的
        // 电梯厅初始是空的，进场顺序由事件堆保证
        let w = World::new(preset_cfg());
        assert_eq!(w.hall.len(), w.n_levels);
        assert!(w.hall.iter().all(|h| h.is_empty()));
    }

    #[test]
    fn world_fits_configured_bounds() {
        let cfg = preset_cfg();
        let w = World::new(cfg.clone());
        assert_eq!(w.cars.len(), cfg.cars.len());
        assert_eq!(w.hall.len(), w.n_levels);
        assert_eq!(w.calls.calls.capacity(), 0, "预分配后不该立刻扩容");
    }

    /// 几何要从夹取之后的配置来量。`sanitized()` 会把顶层号抬到 2，
    /// 若先按旧值算格位数，生成出来的乘客就落在数组外面。
    #[test]
    fn world_geometry_follows_sanitized_config() {
        let mut c = Config::default();
        c.building.top_level = 1;
        c.building.units_per_floor = vec![4];
        c.traffic.window_s = 300.0;
        c.sim.horizon_s = 300.0;
        let w = World::new(c);
        assert_eq!(w.cfg.building.top_level, 2, "前提：夹取把顶层号抬到 2");
        assert_eq!(w.top, 2, "世界的顶层要跟夹取后的配置一致");
        assert_eq!(w.n_levels, 3);
        assert!(w.calls.slots.len() >= w.n_levels * 2, "外呼槽位要覆盖全楼");
        assert!(
            w.pax
                .iter()
                .all(|p| w.in_bounds(p.from) && w.in_bounds(p.to)),
            "乘客的起终点都要在楼里"
        );
    }

    /// 层间交通的总人数是「常驻人数 × 出行率」。团人数只决定这些人怎么
    /// 分组，不能再乘一次，否则户均三人时客流会变成设计值的三倍。
    #[test]
    fn inter_floor_passengers_match_expected_trips() {
        let c = crate::config::preset("hospital").unwrap();
        let travelers = c.building.total_residents() as f64 * c.traffic.departure_ratio as f64;
        let exp = c.expected_trips() as f64;
        assert!(
            (exp - travelers).abs() < 1.0,
            "估算应当就是常驻人数乘出行率 {travelers:.1}，实际 {exp}"
        );
        let got = World::new(c).pax.len() as f64;
        // 团数要取整，再按客流占比摊到各股上，允许一成的分配误差
        assert!(
            (got - exp).abs() <= exp * 0.1,
            "实际生成 {got} 人，与估算的 {exp} 相差太远"
        );
    }

    /// 空梯配置在造世界时就明确报错，而不是在抽样深处索引越界。
    #[test]
    #[should_panic(expected = "至少要有一台轿厢")]
    fn empty_cars_fails_fast_with_message() {
        let mut c = preset_cfg();
        c.cars.clear();
        c.traffic.departure_ratio = 1.0;
        let _w = World::new(c);
    }

    fn preset_cfg() -> Config {
        crate::config::preset("std_residential").unwrap()
    }
}
