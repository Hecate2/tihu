//! 仿真配置。
//!
//! 所有默认值与校验规则见 `docs/MODEL.md`。标「待标定」的常数在这里挂着
//! `TODO(calibration)`，标定完成后删掉这条注释并把出处写进 MODEL。

use crate::motion::{DoorTiming, MotionProfile};

/// 楼层编号约定。
///
/// **层号 1 是主入口层**，也就是住户进出大楼的那一层。住宅层从 2 开始往上排，
/// 地下层用负数，B1 是 -1，B2 是 -2。层号 0 不存在。
pub const LOBBY: i16 = 1;

/// 住户从车库层出门或回家的比例。有地下层时，入口层落回主入口层的概率
/// 就是 `1 - BASEMENT_ENTRANCE_PROB`。
///
/// TODO(calibration) 车库进出占比。现在是行业通行量级，等实测标定。
/// 生成阶段（`engine::pick_entrance_level`）与估算阶段（`expected_trips`）
/// 共用这一个常数，改一处两处一起动。
pub const BASEMENT_ENTRANCE_PROB: f64 = 0.07;

/// 一次批量运行最多几个种子。
///
/// 实测一千个种子在十八层两梯四户约 2.3 秒，三十三层两梯六户约 6.5 秒，见
/// `docs/PERFORMANCE.md` 第 6.1 节。这是网页主线程同步等待的上限，再多就把
/// 界面冻住太久。超出的范围在校验阶段挡掉。
pub const MAX_SEEDS: u64 = 1000;

/// 一栋楼。
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct Building {
    /// 最高层号。层号 1 是主入口层，所以 19 表示上面还有 18 个住宅层
    pub top_level: i16,
    /// 层高，米
    pub floor_height: f32,
    /// 逐层户数表，下标 = 层号 - 1，长度必须等于 `top_level`。
    /// 下标 0 是主入口层那一层
    pub units_per_floor: Vec<u8>,
    /// 户均人数
    pub persons_per_unit: f32,
    /// 地下层数，0 表示没有。有几个就有 B1 到 B几
    pub basement_levels: i16,
    /// 每层套内面积，米²。给了才会做密度校验
    pub floor_area_m2: Option<f32>,
    /// 主入口层是否也住人。默认住人，关掉就变成纯大堂、架空层或者商铺
    pub ground_floor_residential: bool,
    /// 顶层是否为跃层，户数减半
    pub top_floor_duplex: bool,
    /// 最小套型面积，米²
    pub min_unit_area_m2: f32,
    /// 人均最低面积，米²
    pub min_area_per_person_m2: f32,
}

impl Default for Building {
    fn default() -> Self {
        Self {
            // 18 层住宅，层号 1 到 18，主入口层就是 1 层，也住人
            top_level: 18,
            floor_height: 3.0,
            units_per_floor: vec![4u8; 18],
            persons_per_unit: 2.8,
            basement_levels: 0,
            floor_area_m2: None,
            ground_floor_residential: true,
            top_floor_duplex: false,
            min_unit_area_m2: 45.0, // TODO(calibration) 对照住宅设计规范条文
            min_area_per_person_m2: 20.0, // TODO(calibration)
        }
    }
}

impl Building {
    /// 全楼最低层号。有地下层时是 `-basement_levels`
    pub fn lowest_level(&self) -> i16 {
        -self.basement_levels.max(0)
    }

    /// 全楼最高层号
    pub fn highest_level(&self) -> i16 {
        self.top_level.max(LOBBY)
    }

    /// 全楼一共几个楼层格位
    pub fn n_levels(&self) -> usize {
        (self.highest_level() - self.lowest_level() + 1) as usize
    }

    /// 某层实际有几户。
    ///
    /// 层号 1 是主入口层。地下层不住人。户数表可能还没补齐（校验阶段就会
    /// 走到这里），越界一律按 0 户算。
    pub fn units_at(&self, level: i16) -> u8 {
        if level < LOBBY || level > self.highest_level() {
            return 0;
        }
        let mut n = self
            .units_per_floor
            .get((level - 1) as usize)
            .copied()
            .unwrap_or(0);
        if n > 0 && self.top_floor_duplex && level == self.highest_level() {
            n = (n / 2).max(1);
        }
        if n > 0 && !self.ground_floor_residential && level == LOBBY {
            n = 0;
        }
        n
    }

    /// 某层住多少人。
    pub fn residents_at(&self, level: i16) -> usize {
        (self.units_at(level) as f32 * self.persons_per_unit).round() as usize
    }

    /// 全楼住户总数。
    pub fn total_residents(&self) -> usize {
        (LOBBY..=self.highest_level())
            .map(|f| self.residents_at(f))
            .sum()
    }

    /// 最高住宅层的绝对高度，米。
    pub fn roof_height(&self) -> f64 {
        self.top_level as f64 * self.floor_height as f64
    }

    /// 某个楼层的绝对高度，米。地下层往下算。
    pub fn level_height(&self, level: i16) -> f64 {
        level as f64 * self.floor_height as f64
    }
}

/// 体重分布里一个分量的身份。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum WeightKind {
    /// 成年男性
    AdultMale,
    /// 成年女性
    AdultFemale,
    /// 小孩
    Child,
    /// 很重的人。分布要专门留一撮给他
    Heavy,
}

/// 体重分布的一个峰。
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WeightPeak {
    pub kind: WeightKind,
    /// 这个峰占多少比例，不必归一化
    pub share: f64,
    /// 中位数体重，公斤。对数正态用中位数而不是均值
    pub median_kg: f64,
    /// 变异系数
    pub cv: f64,
}

/// 体重分布：若干对数正态分量的混合。
///
/// 用混合而不是单一分布，是因为真人不是一个峰。成年男性、成年女性、小孩、
/// 以及少数很重的人，四拨人体重差得远，混在一起会得到一个谁也不像的中间值。
///
/// 默认值取自中国居民营养与慢性病状况报告：成年男性平均体重约 69.6 公斤，
/// 女性约 58.1 公斤。对数正态的中位数略低于均值，所以这里取 68 与 56。
/// 再各自加一份衣物与随身物品的固定余量。**具体年份与分布形态待核。**
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct WeightMix {
    /// 各分量
    pub peaks: Vec<WeightPeak>,
    /// 衣物与随身物品的固定增量，公斤
    pub carried_kg: f64,
}

impl Default for WeightMix {
    fn default() -> Self {
        Self {
            peaks: vec![
                WeightPeak {
                    kind: WeightKind::AdultMale,
                    share: 0.38,
                    median_kg: 68.0,
                    cv: 0.15,
                },
                WeightPeak {
                    kind: WeightKind::AdultFemale,
                    share: 0.36,
                    median_kg: 56.0,
                    cv: 0.15,
                },
                // 小孩占比较小，早高峰跟着大人出门的那一撮
                WeightPeak {
                    kind: WeightKind::Child,
                    share: 0.18,
                    median_kg: 26.0,
                    cv: 0.22,
                },
                // 很重的人。比例小但绝对数不小，是超载报警的主要来源
                WeightPeak {
                    kind: WeightKind::Heavy,
                    share: 0.06,
                    median_kg: 105.0,
                    cv: 0.18,
                },
            ],
            carried_kg: 2.5, // TODO(calibration) 衣物加随身物品
        }
    }
}

impl WeightMix {
    /// 抽一个分量，返回它的中位数与变异系数。
    ///
    /// `force_child` 为真时直接走小孩那一峰。同行团里带小孩的那几位走这条路径，
    /// 保证相关性，不让「带小孩的团」变成大人小孩随机混。
    pub fn pick(&self, u: f64, force_child: bool) -> (f64, f64) {
        if force_child {
            if let Some(p) = self.peaks.iter().find(|p| p.kind == WeightKind::Child) {
                return (p.median_kg, p.cv);
            }
        }
        let total: f64 = self.peaks.iter().map(|p| p.share.max(0.0)).sum();
        if total <= 0.0 {
            return (65.0, 0.18);
        }
        let target = u * total;
        let mut acc = 0.0;
        for p in &self.peaks {
            acc += p.share.max(0.0);
            if target < acc {
                return (p.median_kg, p.cv);
            }
        }
        self.peaks
            .last()
            .map(|p| (p.median_kg, p.cv))
            .unwrap_or((65.0, 0.18))
    }
}

/// 一台轿厢停靠哪些楼层。
///
/// 高层住宅的梯群常常有分区：一部只停单层，一部只停双层，或者高低区分开。
/// 这里给出几种写起来方便的写法，可以互相组合。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum StopSpec {
    /// 全部楼层
    All,
    /// 显式枚举要停的楼层
    List(Vec<i16>),
    /// 只停奇数层。主入口层与地下层一律停靠
    Odd,
    /// 只停偶数层。主入口层与地下层一律停靠
    Even,
    /// 只停 `[lo, hi]` 这个闭区间
    Zone { lo: i16, hi: i16 },
    /// 上面若干条的并集
    Union(Vec<StopSpec>),
}

impl Default for StopSpec {
    fn default() -> Self {
        StopSpec::All
    }
}

impl StopSpec {
    /// 枚举若干层。
    pub fn list(levels: impl IntoIterator<Item = i16>) -> Self {
        StopSpec::List(levels.into_iter().collect())
    }

    /// 一个闭区间。
    pub fn zone(lo: i16, hi: i16) -> Self {
        StopSpec::Zone { lo, hi }
    }

    /// 单双层。`odd = true` 是单层梯。
    pub fn parity(odd: bool) -> Self {
        if odd {
            StopSpec::Odd
        } else {
            StopSpec::Even
        }
    }

    /// 若干条的并集。
    pub fn union(parts: Vec<StopSpec>) -> Self {
        StopSpec::Union(parts)
    }

    /// 这台车停不停 `level` 层。
    pub fn serves(&self, level: i16) -> bool {
        match self {
            StopSpec::All => true,
            StopSpec::List(v) => v.contains(&level),
            // 单双层说的是住宅楼层。主入口层与地下层人人都停，
            // 否则地下车库那几层会没有一台车愿意去
            StopSpec::Odd => level <= LOBBY || level % 2 != 0,
            StopSpec::Even => level <= LOBBY || level % 2 == 0,
            StopSpec::Zone { lo, hi } => level >= *lo && level <= *hi,
            StopSpec::Union(parts) => parts.iter().any(|p| p.serves(level)),
        }
    }
}

/// 一台轿厢的规格。
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct CarSpec {
    /// 额定载重，公斤
    pub rated_load_kg: f32,
    /// 额定载客人数，等于额定载重除以 75
    pub capacity_rated: usize,
    /// 上行曲线
    pub up: MotionProfile,
    /// 下行曲线
    pub down: MotionProfile,
    /// 开关门时序
    pub doors: DoorTiming,
    /// 停靠楼层。默认全部
    pub stops: StopSpec,
    /// 初始所在层。默认停在主入口层
    pub initial_level: i16,
}

impl Default for CarSpec {
    fn default() -> Self {
        Self {
            rated_load_kg: 1000.0,
            capacity_rated: 13,
            up: MotionProfile::default(),
            down: MotionProfile::default(),
            doors: DoorTiming::default(),
            stops: StopSpec::All,
            initial_level: LOBBY,
        }
    }
}

impl CarSpec {
    /// 由额定载重推出额定载客人数。行业常规按每人 75 公斤折算。
    pub fn from_load(rated_load_kg: f32) -> Self {
        let cap = (rated_load_kg / 75.0).round().max(1.0) as usize;
        Self {
            rated_load_kg,
            capacity_rated: cap,
            ..Self::default()
        }
    }
}

/// 客流模式。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Pattern {
    /// 早高峰下行
    MorningDown,
    /// 晚高峰上行
    EveningUp,
    /// 中午双向
    NoonMixed,
    /// 层间交通。起终点由 `Traffic::flows` 指定，用来模拟医院、写字楼这类
    /// 大量发生在楼层之间的客流
    InterFloor,
}

impl Pattern {
    pub fn as_str(self) -> &'static str {
        match self {
            Pattern::MorningDown => "morning_down",
            Pattern::EveningUp => "evening_up",
            Pattern::NoonMixed => "noon_mixed",
            Pattern::InterFloor => "inter_floor",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "morning_down" => Some(Pattern::MorningDown),
            "evening_up" => Some(Pattern::EveningUp),
            "noon_mixed" => Some(Pattern::NoonMixed),
            "inter_floor" => Some(Pattern::InterFloor),
            _ => None,
        }
    }
}

/// 一股指定的层间客流：从 `from` 层去 `to` 层，占全部层间客流的多大比例。
///
/// 医院里就是「三层门诊去十二层手术室」这种。`share` 不必归一化。
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Flow {
    pub from: i16,
    pub to: i16,
    pub share: f64,
}

impl Flow {
    pub fn new(from: i16, to: i16, share: f64) -> Self {
        Self { from, to, share }
    }
}

/// 客流配置。
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct Traffic {
    pub pattern: Pattern,
    /// 高峰窗口长度，秒
    pub window_s: f64,
    /// 窗口内多大比例的住户离家
    pub departure_ratio: f32,
    /// 反向客流的占比，取下行总量的这个倍数
    pub reverse_ratio: f32,
    /// 强度曲线左参数
    pub peak_alpha: f32,
    /// 强度曲线右参数
    pub peak_beta: f32,
    /// 同户乘客在这个时间窗内到达就并成一个同行团，秒
    pub party_window_s: f64,
    /// 同行团里小孩的人数分布参数。带小孩的团才有
    pub child_party_ratio: f32,
    /// 出门的人里有多大比例会在目的地待一阵再回来。
    /// 大于 0 就会生成第二条行程腿，也就是条件性需求
    pub return_ratio: f32,
    /// 返程前在目的地停留多久的中位数，秒
    pub return_dwell_s: f64,
    /// 层间客流表。只在 `Pattern::InterFloor` 下生效
    pub flows: Vec<Flow>,
}

impl Default for Traffic {
    fn default() -> Self {
        Self {
            pattern: Pattern::MorningDown,
            window_s: 3600.0,
            departure_ratio: 0.66,
            reverse_ratio: 0.08, // TODO(calibration)
            peak_alpha: 2.5,
            peak_beta: 2.5,
            party_window_s: 20.0,
            child_party_ratio: 0.35, // TODO(calibration) 早高峰要送小孩上学
            return_ratio: 0.005,     // TODO(calibration) 折返拿东西、接送
            return_dwell_s: 900.0,   // TODO(calibration)
            flows: Vec::new(),
        }
    }
}

/// 乘客的按键策略。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum Strategy {
    /// 只按真实方向
    Honest,
    /// 上下都按，哪趟先来上哪趟
    PressBoth,
    /// 只按上行，跟着轿厢折返再下来
    PressUpOnly,
    /// 同时呼叫多台
    MultiCall,
}

impl Strategy {
    pub fn as_str(self) -> &'static str {
        match self {
            Strategy::Honest => "honest",
            Strategy::PressBoth => "press_both",
            Strategy::PressUpOnly => "press_up_only",
            Strategy::MultiCall => "multi_call",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "honest" => Some(Strategy::Honest),
            "press_both" => Some(Strategy::PressBoth),
            "press_up_only" => Some(Strategy::PressUpOnly),
            "multi_call" => Some(Strategy::MultiCall),
            _ => None,
        }
    }
}

/// 各策略占多大比例，会被归一化。
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct StrategyMix {
    pub honest: f64,
    pub press_both: f64,
    pub press_up_only: f64,
    pub multi_call: f64,
}

impl Default for StrategyMix {
    /// 默认全员老实。
    fn default() -> Self {
        Self {
            honest: 1.0,
            press_both: 0.0,
            press_up_only: 0.0,
            multi_call: 0.0,
        }
    }
}

impl StrategyMix {
    /// 按比例抽一个策略。用独立的策略流，所以改算法不会挪位。
    pub fn pick(&self, u: f64) -> Strategy {
        let total = self.honest + self.press_both + self.press_up_only + self.multi_call;
        if total <= 0.0 {
            return Strategy::Honest;
        }
        let mut acc = self.honest / total;
        if u < acc {
            return Strategy::Honest;
        }
        acc += self.press_both / total;
        if u < acc {
            return Strategy::PressBoth;
        }
        acc += self.press_up_only / total;
        if u < acc {
            return Strategy::PressUpOnly;
        }
        Strategy::MultiCall
    }

    /// 归一化后的四元组，供演化更新使用。
    pub fn normalized(&self) -> [f64; 4] {
        let total = self.honest + self.press_both + self.press_up_only + self.multi_call;
        if total <= 0.0 {
            return [1.0, 0.0, 0.0, 0.0];
        }
        [
            self.honest / total,
            self.press_both / total,
            self.press_up_only / total,
            self.multi_call / total,
        ]
    }
}

/// 走楼梯的配置。
///
/// 走楼梯是一个**行为**模型，不是一个物理模型。住户不会做「电梯全程耗时与
/// 爬楼全程耗时」这种全局比较，他们凭日常经验：两三楼的邻居天天爬，
/// 八楼以上的人想都不会想。所以这里用两个参数刻画意愿：
/// 一层楼的人有多大可能走，以及这个意愿随层数衰减多快。
///
/// 耗时与费力仍然要算，因为它们决定住户在「打算走」之后会不会反悔。
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct Stairs {
    /// 是否启用
    pub enabled: bool,
    /// 每层下行秒数
    pub per_floor_s: f64,
    /// 每层费力惩罚，折算成秒
    pub per_floor_effort_s: f64,
    /// 走到楼梯间的固定开销
    pub fixed_s: f64,
    /// 比较优势上的判定噪声，秒。人对几十秒的差异才敏感
    pub tau_s: f64,
    /// 上行比下行难受多少倍
    pub up_effort_mult: f64,
    /// 一层楼的住户有多大可能走楼梯
    pub base_willingness: f64,
    /// 意愿随层数的相对衰减。每层乘上 `1 - decay`
    pub decay_per_floor: f64,
    /// 超过这个层数几乎没人走楼梯，0 表示不设上限
    ///
    /// 这一条不是概率参数，是可行性约束。十八层的楼里没有人会走十八层楼梯
    /// 去上班，哪怕电梯慢到离谱。物业也不让这么干。
    pub practical_floor_cap: i16,
    /// 等多久之后会改走楼梯的中位数，秒。0 或关闭时不再有人中途弃梯
    pub abandon_after_s: f64,
    /// 耐心阈值的变异系数。越大越参差
    pub abandon_cv: f64,
}

impl Default for Stairs {
    fn default() -> Self {
        Self {
            enabled: true,
            per_floor_s: 4.5,        // TODO(calibration)
            per_floor_effort_s: 2.5, // TODO(calibration)
            fixed_s: 5.0,            // TODO(calibration)
            tau_s: 45.0,             // TODO(calibration)
            up_effort_mult: 3.0,     // TODO(calibration)
            base_willingness: 0.24,  // TODO(calibration)
            decay_per_floor: 0.055,  // TODO(calibration) 每层的相对衰减
            practical_floor_cap: 6,  // TODO(calibration) 六层以上基本没人走
            abandon_after_s: 1200.0, // TODO(calibration) 二十分钟等不到人就会走
            abandon_cv: 0.45,        // TODO(calibration)
        }
    }
}

impl Stairs {
    /// 从某层走到大堂要多久，含费力惩罚。
    pub fn cost_down(&self, floors: i16) -> f64 {
        let n = floors.max(0) as f64;
        n * self.per_floor_s + n * self.per_floor_effort_s + self.fixed_s
    }

    /// 从大堂爬到某层要多久。
    pub fn cost_up(&self, floors: i16) -> f64 {
        let n = floors.max(0) as f64;
        n * self.per_floor_s + n * self.per_floor_effort_s * self.up_effort_mult + self.fixed_s
    }

    /// 这位住户主观上有多想走楼梯。层数越高越不想。
    ///
    /// 用平方衰减而不是线性。线性衰减在十八层的楼上会让八层以上全部归零，
    /// 那太陡了：现实里八楼十楼的人只是懒得走，不是完全不会走。
    pub fn willingness(&self, floors: i16) -> f64 {
        let n = floors.max(0) as f64;
        let soft = self.base_willingness * (1.0 - n * self.decay_per_floor);
        if self.practical_floor_cap > 0 && floors > self.practical_floor_cap {
            // 超过可行层数，只剩一点点可能性，留给电梯慢得离谱的那种情形
            return (soft * 0.02).clamp(0.0, 1.0);
        }
        soft.clamp(0.0, 1.0)
    }

    /// 完整的走楼梯概率。意愿与「电梯比楼梯快不了多少」这个事实相乘。
    /// 下楼方向，见 MODEL 7.1。
    pub fn probability(&self, floors: i16, elevator_estimate: f64) -> f64 {
        self.probability_with(floors, elevator_estimate, self.cost_down(floors))
    }

    /// 上楼方向的走楼梯概率。往上爬的费力乘数高得多，见 MODEL 7.3。
    /// 晚高峰从大堂回家必须走这一条，用 cost_down 会把爬 9 层算成下 1 层。
    pub fn probability_up(&self, floors: i16, elevator_estimate: f64) -> f64 {
        self.probability_with(floors, elevator_estimate, self.cost_up(floors))
    }

    fn probability_with(&self, floors: i16, elevator_estimate: f64, stair: f64) -> f64 {
        let willing = self.willingness(floors);
        if willing <= 0.0 {
            return 0.0;
        }
        let z = (elevator_estimate - stair) / self.tau_s.max(1e-6);
        let worth_it = 1.0 / (1.0 + (-z).exp());
        (willing * worth_it).clamp(0.0, 1.0)
    }
}

/// 挤梯判据的配置。
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct Boarding {
    /// 载客率低于这条线人人都上，不掷骰子
    pub soft_start_ratio: f64,
    /// 拒绝概率曲线的陡度
    pub gamma: f64,
    /// 等多久算焦躁满格，秒
    pub wait_ref_s: f64,
    /// 焦躁补偿的上限
    pub impatience_gain: f64,
    /// 物理极限相对额定人数的倍数
    pub crush_factor: f64,
    /// 物理载重上限相对额定载重的倍数。超过这个重量就再也进不来人。
    ///
    /// 全身重量是硬约束，人数只是乘客自己的心理尺度。两者各管一头：
    /// 乘客看人数决定愿不愿意挤，轿厢看千克决定还装不装得下。
    ///
    /// 默认 1.10 比报警线 1.05 高，这样「先挤到报警线、再僵住」的因果链才成立
    pub crush_load_ratio: f64,
    /// 载客率达到这条就开始满载直驶
    pub bypass_load_ratio: f64,
}

impl Default for Boarding {
    fn default() -> Self {
        Self {
            soft_start_ratio: 0.75,
            gamma: 1.0,
            wait_ref_s: 120.0,
            impatience_gain: 0.4,
            crush_factor: 1.25,
            crush_load_ratio: 1.10, // TODO(calibration) 对照 GB 7588 超载保护动作值
            bypass_load_ratio: 0.80,
        }
    }
}

impl Boarding {
    /// 物理极限人数。
    pub fn capacity_crush(&self, capacity_rated: usize) -> usize {
        ((capacity_rated as f64 * self.crush_factor).ceil() as usize).max(capacity_rated)
    }

    /// 物理极限载重，千克。超过就再也进不来人。
    pub fn load_crush_kg(&self, rated_load_kg: f64) -> f64 {
        rated_load_kg * self.crush_load_ratio.max(1.0)
    }

    /// 某位乘客这次愿不愿意挤上来。见 `docs/MODEL.md` 第 5.5.2 节。
    pub fn accept_prob(&self, load_ratio: f64, tol: f64, waited_s: f64) -> f64 {
        if load_ratio <= self.soft_start_ratio {
            return 1.0;
        }
        let span = 1.0 - self.soft_start_ratio;
        let rho = ((load_ratio - self.soft_start_ratio) / span)
            .clamp(0.0, 1.0)
            .powf(self.gamma);
        let impatience = (waited_s / self.wait_ref_s).min(1.0) * self.impatience_gain;
        (1.0 - tol * rho + impatience).clamp(0.0, 1.0)
    }
}

/// 呼叫面板形式。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum CallSurface {
    /// 并联群控，共用一套上下按钮。一次按键一个外呼
    GroupSingle,
    /// 每台轿厢独立按钮，可以指定轿厢，也可以同时呼叫多台
    PerCarPanel,
}

impl CallSurface {
    pub fn as_str(self) -> &'static str {
        match self {
            CallSurface::GroupSingle => "group_single",
            CallSurface::PerCarPanel => "per_car_panel",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "group_single" => Some(CallSurface::GroupSingle),
            "per_car_panel" => Some(CallSurface::PerCarPanel),
            _ => None,
        }
    }
}

/// 超载死锁的配置。默认关闭。
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct Overload {
    /// 是否启用。默认关闭，因为它极其罕见
    pub enabled: bool,
    /// 报警线相对额定载重的倍数
    pub trigger_ratio: f64,
    /// 每隔多久问一次有没有人愿意退出，秒
    pub stall_tick_s: f64,
    /// 最后上车的人退出概率
    pub q_last: f64,
    /// 其他人每人的退出概率
    pub q_other: f64,
    /// 每累积 30 秒，退出概率乘以这个
    pub stress_growth: f64,
    /// 兜底超时，超时后强制最近门口的人退出
    pub stall_timeout_s: f64,
}

impl Default for Overload {
    fn default() -> Self {
        Self {
            enabled: false,
            trigger_ratio: 1.05, // TODO(calibration) 对照安全规范条文
            stall_tick_s: 10.0,
            q_last: 0.35,       // TODO(calibration)
            q_other: 0.05,      // TODO(calibration)
            stress_growth: 1.6, // TODO(calibration)
            stall_timeout_s: 300.0,
        }
    }
}

/// 仿真控制。
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct Sim {
    /// 根种子
    pub seed: u64,
    /// 种子范围的结束值。比根种子大时依次跑完这中间每个种子再汇总，
    /// 缺席或者不大于根种子时只跑根种子那一个。见 `batch::run_seeds`
    pub seed_to: Option<u64>,
    /// 仿真总时长，秒
    pub horizon_s: f64,
    /// 跑多少天。1 表示单日
    pub days: u32,
    /// 演化时策略比例下限，防止某个策略彻底灭绝后再也无法复活
    pub evolution_epsilon: f64,
}

impl Default for Sim {
    /// 仿真窗口默认比出行窗口长 50%。多出来的那半小时是恢复期：
    /// 高峰结束后还有人在等，仿真必须跑完把他们送下去。
    /// 两者相等的话，最后一批人刚到电梯厅仿真就结束了，结果是「完成数」偏低，
    /// 而那不是拥堵造成的，是仿真截断造成的
    fn default() -> Self {
        Self {
            seed: 42,
            seed_to: None,
            horizon_s: 5400.0,
            days: 1,
            evolution_epsilon: 0.01,
        }
    }
}

/// 一份完整的仿真配置。
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct Config {
    pub building: Building,
    /// 每台轿厢一份规格
    pub cars: Vec<CarSpec>,
    pub traffic: Traffic,
    pub stairs: Stairs,
    pub boarding: Boarding,
    /// 乘客体重分布
    pub weight: WeightMix,
    pub strategy: StrategyMix,
    pub call_surface: CallSurface,
    pub overload: Overload,
    pub sim: Sim,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            building: Building::default(),
            cars: vec![CarSpec::default(); 2],
            traffic: Traffic::default(),
            stairs: Stairs::default(),
            boarding: Boarding::default(),
            weight: WeightMix::default(),
            strategy: StrategyMix::default(),
            call_surface: CallSurface::GroupSingle,
            overload: Overload::default(),
            sim: Sim::default(),
        }
    }
}

/// 校验的严重程度。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// 硬约束违反，不让跑
    Error,
    /// 接近硬约束，跑但在网页上要显示黄条
    Warn,
}

/// 一条校验结果。
#[derive(Debug, Clone)]
pub struct Issue {
    pub severity: Severity,
    pub code: &'static str,
    pub message: String,
    /// 涉及的具体楼层，给了就让网页高亮那一层
    pub level: Option<i16>,
}

impl Issue {
    fn error(code: &'static str, message: String) -> Self {
        Self {
            severity: Severity::Error,
            code,
            message,
            level: None,
        }
    }

    fn warn(code: &'static str, message: String) -> Self {
        Self {
            severity: Severity::Warn,
            code,
            message,
            level: None,
        }
    }
}

/// 校验结果。
#[derive(Debug, Clone, Default)]
pub struct Validation {
    pub issues: Vec<Issue>,
}

impl Validation {
    pub fn is_ok(&self) -> bool {
        !self.issues.iter().any(|i| i.severity == Severity::Error)
    }

    pub fn errors(&self) -> impl Iterator<Item = &Issue> {
        self.issues.iter().filter(|i| i.severity == Severity::Error)
    }

    pub fn warnings(&self) -> impl Iterator<Item = &Issue> {
        self.issues.iter().filter(|i| i.severity == Severity::Warn)
    }
}

impl Config {
    /// 校验配置。红线 5 的密度约束在这里落地。
    pub fn validate(&self) -> Validation {
        let mut v = Validation::default();
        let b = &self.building;

        // 层号 1 是主入口层，所以顶层号至少要是 2
        if b.top_level < 2 || b.top_level > 81 {
            v.issues.push(Issue::error(
                "floors_range",
                format!(
                    "最高层号应在 2 到 81 之间（层号 1 是主入口层），当前 {}",
                    b.top_level
                ),
            ));
        }
        if b.floor_height < 2.4 || b.floor_height > 6.0 {
            v.issues.push(Issue::error(
                "floor_height_range",
                format!("层高应在 2.4 到 6.0 米之间，当前 {:.2}", b.floor_height),
            ));
        }
        if b.persons_per_unit < 1.0 || b.persons_per_unit > 6.0 {
            v.issues.push(Issue::error(
                "persons_range",
                format!("户均人数应在 1 到 6 之间，当前 {:.1}", b.persons_per_unit),
            ));
        }
        if self.cars.is_empty() {
            v.issues
                .push(Issue::error("no_cars", "至少要有一台轿厢".into()));
        }
        if self.cars.len() > 12 {
            v.issues.push(Issue::warn(
                "too_many_cars",
                format!("{} 台轿厢超出了常规住宅的范围", self.cars.len()),
            ));
        }

        // 户数表长度必须与最高层号一致。层号 1 是主入口层，也占一格
        if b.units_per_floor.len() != b.top_level as usize {
            v.issues.push(Issue::error(
                "units_length",
                format!(
                    "逐层户数表的长度是 {}，最高层号是 {}，两者必须一致",
                    b.units_per_floor.len(),
                    b.top_level
                ),
            ));
        }

        // 红线 5：层数、户数、户均人数不是三个自由变量
        if let Some(area) = b.floor_area_m2 {
            let max_units = (area / b.min_unit_area_m2).floor() as i32;
            let max_persons = (area / b.min_area_per_person_m2).floor() as i32;
            for level in 1..=b.top_level {
                let idx = (level - 1) as usize;
                let Some(&units) = b.units_per_floor.get(idx) else {
                    continue;
                };
                let units = units as i32;
                let persons = units as f32 * b.persons_per_unit;
                if units > max_units {
                    v.issues.push(Issue {
                        severity: Severity::Error,
                        code: "density_units",
                        message: format!(
                            "第 {} 层设了 {} 户。按最小套型 {:.0} m² 反推，这层至少要有 {:.0} m² 的套内面积才塞得下",
                            level,
                            units,
                            b.min_unit_area_m2,
                            units as f32 * b.min_unit_area_m2
                        ),
                        level: Some(level),
                    });
                }
                // 人均面积越接近下限越可疑。落在下限的一点五倍以内就提醒。
                let per_person = area / persons.max(1.0);
                if persons as i32 > max_persons {
                    v.issues.push(Issue {
                        severity: Severity::Error,
                        code: "density_persons",
                        message: format!(
                            "第 {} 层按每户 {:.1} 人算是 {:.0} 人。按人均 {:.0} m² 反推，这层至少要有 {:.0} m² 才合规",
                            level,
                            b.persons_per_unit,
                            persons,
                            b.min_area_per_person_m2,
                            persons * b.min_area_per_person_m2
                        ),
                        level: Some(level),
                    });
                } else if per_person < b.min_area_per_person_m2 * 1.5 {
                    v.issues.push(Issue {
                        severity: Severity::Warn,
                        code: "density_tight",
                        message: format!(
                            "第 {} 层人均 {:.1} m²，已经逼近 {:.0} m² 的下限，实际楼盘不太可能这样",
                            level, per_person, b.min_area_per_person_m2
                        ),
                        level: Some(level),
                    });
                }
            }
        }

        if self.traffic.departure_ratio <= 0.0 || self.traffic.departure_ratio > 1.0 {
            v.issues.push(Issue::error(
                "departure_ratio",
                format!(
                    "出行率应在 0 到 1 之间，当前 {:.2}",
                    self.traffic.departure_ratio
                ),
            ));
        }
        if self.traffic.peak_alpha <= 0.0 || self.traffic.peak_beta <= 0.0 {
            v.issues.push(Issue::error(
                "peak_shape",
                "强度曲线的两个参数都要大于 0".into(),
            ));
        }
        if self.sim.horizon_s <= 0.0 {
            v.issues
                .push(Issue::error("horizon", "仿真时长要大于 0".into()));
        }
        // 种子范围的宽度。结束值不大于根种子时按单次跑，不报错
        if let Some(to) = self.sim.seed_to {
            if to > self.sim.seed && to - self.sim.seed + 1 > MAX_SEEDS {
                v.issues.push(Issue::error(
                    "seed_range",
                    format!(
                        "种子范围最多跑 {MAX_SEEDS} 个，当前 {} 到 {} 一共 {} 个",
                        self.sim.seed,
                        to,
                        to - self.sim.seed + 1
                    ),
                ));
            }
        }
        if self.boarding.crush_factor < 1.0 {
            v.issues.push(Issue::error(
                "crush_factor",
                "物理极限人数不可能小于额定人数".into(),
            ));
        }
        // 每一层都要有至少一台轿厢愿意停，否则那一层的人永远等不到车。
        // 层号 0 不存在，跳过
        for level in b.lowest_level()..=b.highest_level() {
            if level == 0 {
                continue;
            }
            if !self.cars.iter().any(|c| c.stops.serves(level)) {
                v.issues.push(Issue {
                    severity: Severity::Error,
                    code: "unserved_floor",
                    message: format!("第 {level} 层没有任何一台轿厢停靠，那一层的人上不了梯"),
                    level: Some(level),
                });
            }
        }
        // 有住户的楼层，还得有一台既停这一层又停主入口层的轿厢。
        // 早晚高峰的主体客流是「住宅层与主入口层之间」，分区梯漏掉主入口，
        // 那一层的住户就上不了车也下不去
        for level in LOBBY..=b.highest_level() {
            if b.residents_at(level) == 0 {
                continue;
            }
            let through = self
                .cars
                .iter()
                .any(|c| c.stops.serves(level) && c.stops.serves(LOBBY));
            if !through {
                v.issues.push(Issue {
                    severity: Severity::Error,
                    code: "no_through_car",
                    message: format!(
                        "第 {level} 层没有一台轿厢同时停靠主入口层，这一层的人去了主入口层就回不来",
                    ),
                    level: Some(level),
                });
            }
        }
        // 层间客流表里的每一股，起终点都要在楼里，而且要有一台车同时停靠
        if self.traffic.pattern == Pattern::InterFloor {
            if self.traffic.flows.is_empty() {
                v.issues.push(Issue::error(
                    "no_flows",
                    "层间交通模式下必须给出至少一股客流".into(),
                ));
            }
            for f in &self.traffic.flows {
                if f.share < 0.0 {
                    v.issues
                        .push(Issue::error("flow_share", "客流占比不能是负数".into()));
                }
                for level in [f.from, f.to] {
                    if level < b.lowest_level() || level > b.highest_level() {
                        v.issues.push(Issue::error(
                            "flow_level",
                            format!("层间客流的 {level} 层超出楼的范围"),
                        ));
                    } else if !self
                        .cars
                        .iter()
                        .any(|c| c.stops.serves(f.from) && c.stops.serves(f.to))
                    {
                        v.issues.push(Issue::error(
                            "flow_unserved",
                            format!("没有一台轿厢同时停靠 {} 层与 {} 层", f.from, f.to),
                        ));
                    }
                }
            }
        }
        v
    }

    /// 把配置里明显不合理的值夹到合理范围。校验通过之后调用。
    pub fn sanitized(mut self) -> Self {
        self.building.top_level = self.building.top_level.max(2);
        self.building.floor_height = self.building.floor_height.clamp(2.4, 6.0);
        self.building.persons_per_unit = self.building.persons_per_unit.clamp(1.0, 6.0);
        if self.building.units_per_floor.len() != self.building.top_level as usize {
            let n = self.building.top_level.max(1) as usize;
            self.building.units_per_floor.resize(n, 4);
        }
        for u in &mut self.building.units_per_floor {
            // 防呆上限，不是建模约束。18 户这种大板楼合法，真正的人口密度约束
            // 在给了楼板面积时由 validate() 按 MODEL 2.2 算。这个数只拦
            // 手误多打一个 0 那种输入，免得网页一次造出几十万乘客卡死
            *u = (*u).clamp(0, 32);
        }
        // 主入口层不住人时，补表或夹取之后都要把这个约定恢复回来
        if !self.building.ground_floor_residential {
            if let Some(first) = self.building.units_per_floor.first_mut() {
                *first = 0;
            }
        }
        for c in &mut self.cars {
            c.up = c.up.sanitized();
            c.down = c.down.sanitized();
            c.capacity_rated = c.capacity_rated.clamp(1, 40);
            c.rated_load_kg = c.rated_load_kg.clamp(100.0, 5000.0);
            c.initial_level = c.initial_level.clamp(
                -self.building.basement_levels.max(0),
                self.building.top_level.max(1),
            );
        }
        self
    }

    /// 全楼住户乘出行率，估出的上梯人数。用于给乘客向量预留容量和展示。
    ///
    /// 口径要与 `engine::build_pax` 实际生成的东西对齐：每户出一团，团人数
    /// 是户均人数四舍五入，另有一小撮反向细流（`reverse_ratio`）。所以先按
    /// 户数算，不能按住户总数算，也不能漏掉反向那一撮。
    ///
    /// 主入口层那几户是个例外。他们的出发层和入口层抽到一起时是零长度行程，
    /// 不上电梯，`emit_party` 会把这种团丢掉。入口抽在主入口层，还是抽到车库，
    /// 由 `BASEMENT_ENTRANCE_PROB` 定，所以主入口层的户只有一小截剩得下。
    pub fn expected_trips(&self) -> usize {
        let b = &self.building;
        let dep = self.traffic.departure_ratio as f64;
        let rev = self.traffic.reverse_ratio as f64;
        let per_party = b.persons_per_unit.round().clamp(1.0, 4.0) as f64;
        let units: f64 = (1..=b.highest_level()).map(|l| b.units_at(l) as f64).sum();
        // 入口层落回主入口层的概率。落回时主入口层住户的团走零长度行程，
        // 生成阶段被丢掉。有地下层才有抽到车库的可能，没有就全落回主入口层
        let p_lobby_entrance = if b.basement_levels > 0 {
            1.0 - BASEMENT_ENTRANCE_PROB
        } else {
            1.0
        };
        let lobby_units = b.units_at(LOBBY) as f64;
        let riders = (units - lobby_units * p_lobby_entrance).max(0.0);
        // 反向细流是在「这一户已经有人出门」之后再掷的，不是每户独立掷一次，
        // 所以是 dep*(1+rev)，不是 dep+rev。写成后者在低出行率下会高估
        (riders * per_party * dep * (1.0 + rev)) as usize
    }
}

/// 网页与命令行共用的预设。
///
/// 「N 层住宅」的 `top_level` 就是 N，层号 1 到 N，主入口层就是 1 层，也住人。
/// 编号约定见 `docs/MODEL.md` 第 2 节。
pub fn preset(name: &str) -> Option<Config> {
    let mut c = Config::default();
    let set = |c: &mut Config, levels: i16, units: u8, cars: usize| {
        c.building.top_level = levels;
        c.building.units_per_floor = vec![units; levels as usize];
        c.building.ground_floor_residential = true;
        c.cars = vec![CarSpec::default(); cars];
    };
    match name {
        "std_residential" => set(&mut c, 18, 4, 2),
        "std_tall" => set(&mut c, 33, 6, 2),
        "std_underserved" => set(&mut c, 33, 4, 1),
        "stress_gaming" => {
            set(&mut c, 33, 6, 2);
            c.strategy = StrategyMix {
                honest: 0.7,
                press_both: 0.25,
                press_up_only: 0.05,
                multi_call: 0.0,
            };
        }
        "stress_stalled_car" => {
            set(&mut c, 33, 8, 1);
            c.overload.enabled = true;
            c.boarding.crush_factor = 1.4;
        }
        "stress_mixed_traffic" => {
            set(&mut c, 24, 5, 3);
            c.traffic.pattern = Pattern::NoonMixed;
            c.traffic.reverse_ratio = 0.5;
        }
        "hospital" => {
            // 一栋二十四层的院楼。层号 1 到 24，1 层是主入口层
            set(&mut c, 24, 4, 3);
            c.building.persons_per_unit = 3.0;
            c.traffic.pattern = Pattern::InterFloor;
            c.traffic.departure_ratio = 0.35;
            c.traffic.flows = vec![
                Flow::new(3, 12, 0.30), // 门诊去手术层
                Flow::new(12, 2, 0.25), // 手术层送回门诊
                Flow::new(6, 20, 0.25), // 检验去住院病区
                Flow::new(20, 6, 0.20), // 病区回检验
            ];
        }
        _ => return None,
    }
    Some(c)
}

/// 全部预设的名字。
pub fn preset_names() -> &'static [&'static str] {
    &[
        "std_residential",
        "std_tall",
        "std_underserved",
        "stress_gaming",
        "stress_stalled_car",
        "stress_mixed_traffic",
        "hospital",
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_valid() {
        let v = Config::default().validate();
        assert!(
            v.is_ok(),
            "默认配置应当合法，实际 {:?}",
            v.errors().collect::<Vec<_>>()
        );
    }

    /// 红线 5：层数、户数、户均人数不是三个自由变量。
    #[test]
    fn density_constraint_rejects_impossible_building() {
        let mut c = Config::default();
        c.building.top_level = 10;
        c.building.units_per_floor = vec![20; 10];
        c.building.persons_per_unit = 3.0;
        c.building.floor_area_m2 = Some(400.0);
        let v = c.validate();
        assert!(!v.is_ok(), "400 m² 塞 20 户 3 人的楼应当被拦下");
        let msgs: Vec<&str> = v.errors().map(|i| i.message.as_str()).collect();
        assert!(
            msgs.iter().any(|m| m.contains("套内面积")),
            "报错要给出至少需要多大面积，实际 {msgs:?}"
        );
    }

    #[test]
    fn density_warning_when_tight() {
        let mut c = Config::default();
        c.building.top_level = 4;
        c.building.units_per_floor = vec![0, 6, 6, 6];
        c.building.persons_per_unit = 2.0;
        // 6 户 2 人 = 12 人。人均 1200/12 = 100 m²，很宽松，不该告警
        c.building.floor_area_m2 = Some(1200.0);
        let v = c.validate();
        assert!(v.is_ok());
        assert_eq!(v.warnings().count(), 0);

        // 换成 300 m²。6 户按最小套型要 270 m²，12 人按人均 20 要 240 m²，都过关。
        // 但人均只有 25 m²，逼近 20 m² 的下限，应当只警告不拦
        c.building.floor_area_m2 = Some(300.0);
        let v = c.validate();
        assert!(v.is_ok(), "逼近下限不该是硬错误");
        assert!(v.warnings().count() > 0, "应当给出黄条");
    }

    #[test]
    fn units_table_length_must_match_floors() {
        let mut c = Config::default();
        c.building.top_level = 10;
        c.building.units_per_floor = vec![4; 5];
        let v = c.validate();
        assert!(!v.is_ok());
    }

    #[test]
    fn no_cars_is_an_error() {
        let mut c = Config::default();
        c.cars.clear();
        assert!(!c.validate().is_ok());
    }

    #[test]
    fn sanitize_fills_units_table() {
        let mut c = Config::default();
        c.building.top_level = 12;
        c.building.units_per_floor = vec![3; 4];
        let c = c.sanitized();
        assert_eq!(c.building.units_per_floor.len(), 12);
        assert!(c.validate().is_ok());
    }

    /// 每层 18 户的大板楼合法，sanitized 不许打回 12。
    #[test]
    fn sanitize_keeps_eighteen_units_per_floor() {
        let mut c = Config::default();
        c.building.top_level = 10;
        c.building.units_per_floor = vec![18; 10];
        c.building.persons_per_unit = 4.0;
        let c = c.sanitized();
        assert!(c.building.units_per_floor.iter().all(|&u| u == 18));
        assert_eq!(c.building.residents_at(2), 72);
        assert!(c.validate().is_ok());
    }

    #[test]
    fn residents_follow_units_and_density() {
        let mut b = Building::default();
        b.top_level = 3;
        b.ground_floor_residential = true;
        b.units_per_floor = vec![4, 4, 4];
        b.persons_per_unit = 2.5;
        assert_eq!(b.residents_at(1), 10, "层号 1 是主入口层，设了住户就算住户");
        assert_eq!(b.residents_at(0), 0, "层号 0 不存在");
        assert_eq!(b.residents_at(4), 0, "超过最高层号就没有住户");
        assert_eq!(b.total_residents(), 30);
    }

    /// 主入口层默认住人。
    #[test]
    fn ground_floor_is_residential_by_default() {
        let c = Config::default();
        assert!(c.building.ground_floor_residential, "主入口层默认住人");
        assert!(c.building.residents_at(1) > 0, "默认配置里主入口层有住户");
        assert!(c.building.residents_at(2) > 0);
    }

    /// 关掉开关之后，主入口层的住户要被清掉。
    #[test]
    fn toggling_ground_floor_off_clears_residents() {
        let mut c = Config::default();
        c.building.top_level = 10;
        c.building.units_per_floor = vec![4; 10];
        c.building.ground_floor_residential = false;
        let c = c.sanitized();
        assert_eq!(c.building.residents_at(1), 0, "关掉之后主入口层不该有住户");
        assert!(c.building.residents_at(2) > 0, "二层起才是住宅");
    }

    #[test]
    fn duplex_top_floor_halves_units() {
        let mut b = Building::default();
        b.top_level = 3;
        b.ground_floor_residential = true;
        b.units_per_floor = vec![4, 4, 4];
        b.top_floor_duplex = true;
        assert_eq!(b.units_at(3), 2);
        assert_eq!(b.units_at(2), 4);
    }

    /// `docs/MODEL.md` 第 7.2 节给的数字。三层住户走楼梯成本 19 秒。
    #[test]
    fn stair_cost_matches_doc() {
        let s = Stairs::default();
        assert!(
            (s.cost_down(2) - 19.0).abs() < 1e-9,
            "两层应为 19 秒，实际 {}",
            s.cost_down(2)
        );
        // 十层以上基本没人走，因为下行加费力已超过 70 秒
        assert!(s.cost_down(10) > 70.0);
        // 上行明显更难受
        assert!(s.cost_up(10) > s.cost_down(10));
    }

    /// `docs/MODEL.md` 第 5.5 节的三条容量线。
    #[test]
    fn capacity_lines() {
        let b = Boarding::default();
        assert_eq!(b.capacity_crush(13), 17, "13 人的 1.25 倍向上取整");
        assert!(b.capacity_crush(13) > 13);
        // 低载客率人人都上
        assert_eq!(b.accept_prob(0.5, 1.0, 0.0), 1.0);
        // 满载且最有耐心的人也上不了
        assert_eq!(b.accept_prob(1.0, 1.0, 0.0), 0.0);
        // 满载且已经等了很久，焦躁补偿起作用
        assert!(b.accept_prob(1.0, 1.0, 600.0) > 0.3);
    }

    /// 同样拥挤，等了三分钟的人和刚到 20 秒的人决定不同。
    #[test]
    fn impatience_changes_the_decision() {
        let b = Boarding::default();
        let fresh = b.accept_prob(0.95, 0.8, 20.0);
        let tired = b.accept_prob(0.95, 0.8, 300.0);
        assert!(
            tired > fresh,
            "等久了更可能挤上去，fresh={fresh} tired={tired}"
        );
    }

    #[test]
    fn strategy_mix_picks_by_ratio() {
        let all_honest = StrategyMix::default();
        for _ in 0..100 {
            assert_eq!(all_honest.pick(0.99), Strategy::Honest);
        }
        let half = StrategyMix {
            honest: 0.5,
            press_both: 0.5,
            ..Default::default()
        };
        assert_eq!(half.pick(0.1), Strategy::Honest);
        assert_eq!(half.pick(0.9), Strategy::PressBoth);
        // 比例未归一化也能用。honest 占 0.7，所以 u=0.5 落在诚实区间
        let unnormalized = StrategyMix {
            honest: 7.0,
            press_both: 3.0,
            ..Default::default()
        };
        assert_eq!(unnormalized.pick(0.5), Strategy::Honest);
        assert_eq!(unnormalized.pick(0.9), Strategy::PressBoth);
    }

    #[test]
    fn strategy_mix_normalizes() {
        let m = StrategyMix {
            honest: 2.0,
            press_both: 2.0,
            ..Default::default()
        };
        let n = m.normalized();
        assert!((n.iter().sum::<f64>() - 1.0).abs() < 1e-12);
        assert!((n[0] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn rated_load_gives_capacity() {
        let c = CarSpec::from_load(1000.0);
        assert_eq!(c.capacity_rated, 13);
        assert_eq!(CarSpec::from_load(800.0).capacity_rated, 11);
        assert_eq!(CarSpec::from_load(1350.0).capacity_rated, 18);
    }

    #[test]
    fn overload_is_off_by_default() {
        assert!(!Config::default().overload.enabled, "超载死锁默认必须关闭");
    }

    #[test]
    fn all_presets_validate() {
        for name in preset_names() {
            let c = preset(name).unwrap();
            let v = c.validate();
            assert!(
                v.is_ok(),
                "预设 {name} 不合法：{:?}",
                v.errors().collect::<Vec<_>>()
            );
        }
    }

    #[test]
    fn expected_trips_scales_with_population() {
        let c = preset("std_tall").unwrap();
        // 层号 1 到 33 都住人，每层 round(6 × 2.8) = 17 人，主入口层也是住宅层。
        // 团人数是 round(2.8) = 3。没有地下层时入口全落回主入口层，主入口层
        // 那 6 户的零长度团会被生成阶段丢掉，其余 33×6-6 户都算数。反向细流
        // 与主客流同权重，一起进公式。
        assert_eq!(c.building.residents_at(1), 17, "主入口层住人，也是住宅层");
        assert_eq!(c.building.residents_at(2), 17);
        assert_eq!(c.building.total_residents(), 33 * 17);
        assert_eq!(c.building.basement_levels, 0);
        let riders = 33.0 * 6.0 - 6.0;
        assert_eq!(c.expected_trips(), (riders * 3.0 * 0.66 * 1.08) as usize);
    }

    /// 配置往返 JSON 必须一模一样。网页传的就是这份 JSON，
    /// 一旦某个字段漏了 serde 属性，这里立刻会红。
    #[cfg(feature = "serde")]
    #[test]
    fn config_roundtrips_through_json() {
        let mut c = preset("hospital").unwrap();
        c.building.basement_levels = 2;
        c.cars[0].stops = StopSpec::parity(true);
        c.cars[1].stops = StopSpec::zone(2, 12);
        c.cars[2].stops = StopSpec::union(vec![StopSpec::Odd, StopSpec::list([1, 2, 20])]);
        c.cars[0].initial_level = -1;
        c.overload.enabled = true;
        c.call_surface = CallSurface::PerCarPanel;
        c.strategy = StrategyMix {
            honest: 0.5,
            press_both: 0.3,
            press_up_only: 0.1,
            multi_call: 0.1,
        };

        let a = serde_json::to_string(&c).unwrap();
        let back: Config = serde_json::from_str(&a).unwrap();
        let b = serde_json::to_string(&back).unwrap();
        assert_eq!(a, b, "配置往返 JSON 不一致");
    }

    /// 只给一小部分字段也要能建出完整配置，缺的都走默认值。
    #[cfg(feature = "serde")]
    #[test]
    fn partial_json_falls_back_to_defaults() {
        let j =
            r#"{"building":{"top_level":7,"units_per_floor":[0,4,4,4,4,4,4]},"sim":{"seed":5}}"#;
        let c: Config = serde_json::from_str(j).unwrap();
        assert_eq!(c.building.top_level, 7);
        assert_eq!(c.sim.seed, 5);
        assert!(
            (c.building.floor_height - 3.0).abs() < 1e-6,
            "没给的字段应当走默认值"
        );
        assert_eq!(c.cars.len(), 2, "没给轿厢表时应当有两台默认轿厢");
    }
}
