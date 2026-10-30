//! 调度策略的三个替换点。
//!
//! 这里是本仓库最重要的接口。承诺是：写一个新算法在 100 行以内完成，
//! 且不改动 `tihu-core` 的任何一行。见 `docs/DISPATCH_API.md`。
//!
//! 所有算法拿到的都是只读视图。不许改动世界，只能回答决策问题。
//! 这条约束让内核可以随时重算，也让算法之间可以公平比较。

use crate::motion::Dir;

/// 轿厢编号。
pub type CarId = usize;

/// 一台轿厢的只读摘要。
#[derive(Debug, Clone, Copy)]
pub struct CarBrief {
    pub id: CarId,
    /// 当前所在层，可以是小数
    pub level: f32,
    /// 当前方向
    pub dir: Dir,
    /// 轿厢里的乘客数
    pub load: usize,
    /// 当前载重，公斤。比人数更能说明白「还能不能再上人」
    pub load_kg: f32,
    /// 额定人数
    pub capacity: usize,
    /// 名下还没服务完的外呼数
    pub pending_calls: usize,
    /// 是否卡在超载僵局里
    pub stalled: bool,
    /// 已经僵了多久，秒。非僵局时为 0
    pub stall_for: f64,
}

impl CarBrief {
    /// 载客率，0 到 1 以上都有可能。额定人数由校验与 sanitized 保证至少 1。
    pub fn load_ratio(&self) -> f64 {
        self.load as f64 / self.capacity as f64
    }
}

/// 一条外呼。
#[derive(Debug, Clone, Copy)]
pub struct HallCall {
    pub id: u32,
    /// 按键所在楼层
    pub level: i16,
    /// 希望去的方向
    pub dir: Dir,
    /// 已经派给谁。`None` 表示还没派
    pub owner: Option<CarId>,
    /// 这个楼层同方向此刻有多少人在等。到达、上车、弃梯都会刷新它
    pub waiting: usize,
}

/// 全机视图。派梯算法看到的全部内容。
#[derive(Debug, Clone)]
pub struct SystemView<'a> {
    pub now: f64,
    pub cars: &'a [CarBrief],
    /// 还没分配给任何轿厢、也还亮着的外呼
    pub pending: &'a [HallCall],
    /// 全部还算数的外呼，含已分配。灭灯的历史记录不进视图
    pub all_calls: &'a [HallCall],
    /// 每层的等待人数，下标是 `(level + 1) * 2 + dir`
    pub queue_len: &'a [u32],
    /// 最高住宅层
    pub top_level: i16,
    pub n_cars: usize,
}

/// 某台轿厢的详细只读视图。
#[derive(Debug, Clone)]
pub struct RunView<'a> {
    pub now: f64,
    pub car: CarId,
    /// 当前所在层，可以是小数
    pub at_level: f32,
    /// 当前方向
    pub heading: Dir,
    /// 轿厢里的乘客数
    pub load: usize,
    /// 额定人数
    pub capacity_rated: usize,
    /// 物理极限人数
    pub capacity_crush: usize,
    /// 当前载重，公斤
    pub load_kg: f32,
    /// 轿厢内乘客的目的楼层，升序。借用内核的预分配缓冲，不分配
    pub car_calls: &'a [i16],
    /// 名下未服务的外呼。同上，借用不分配
    pub mine: &'a [HallCall],
    /// 全机视图。持有值而不是引用，免得内联构造时出现临时借用
    pub system: SystemView<'a>,
    /// 层高，米
    pub floor_height: f32,
}

impl RunView<'_> {
    /// 当前所在层的整数部分。
    pub fn level(&self) -> i16 {
        self.at_level.round() as i16
    }

    /// 轿厢里是否有人要在 `level` 层出去。
    pub fn will_alight(&self, level: i16) -> bool {
        self.car_calls.contains(&level)
    }

    /// 名下是否有一条 `level` 层 `dir` 方向的外呼。
    pub fn has_call(&self, level: i16, dir: Dir) -> bool {
        self.mine.iter().any(|c| c.level == level && c.dir == dir)
    }

    /// 名下有没有任何上行外呼。
    pub fn has_call_up(&self) -> bool {
        self.mine.iter().any(|c| c.dir == Dir::Up)
    }

    /// 名下有没有任何下行外呼。
    pub fn has_call_down(&self) -> bool {
        self.mine.iter().any(|c| c.dir == Dir::Down)
    }

    /// 名下 `dir` 方向外呼里最高的一层。
    pub fn extreme(&self, dir: Dir) -> Option<i16> {
        let mut it = self.mine.iter().filter(|c| c.dir == dir).map(|c| c.level);
        let first = it.next()?;
        Some(it.fold(first, |acc, l| match dir {
            Dir::Up => acc.max(l),
            Dir::Down | Dir::None => acc.min(l),
        }))
    }

    /// 轿厢内目的楼层里最高的一层。
    pub fn highest_car_call(&self) -> Option<i16> {
        self.car_calls.iter().copied().max()
    }

    /// 轿厢内目的楼层里最低的一层。
    pub fn lowest_car_call(&self) -> Option<i16> {
        self.car_calls.iter().copied().min()
    }

    /// 载客率。额定人数由校验与 sanitized 保证至少 1。
    pub fn load_ratio(&self) -> f64 {
        self.load as f64 / self.capacity_rated as f64
    }

    /// 名下有没有任何未完成的任务。
    pub fn has_work(&self) -> bool {
        !self.mine.is_empty() || !self.car_calls.is_empty()
    }

    /// 全部未完成任务里最高的一层，方向不分。
    ///
    /// 上行接人时用这个。早高峰名下几乎全是下行外呼，若按方向分别取极值，
    /// 一个方向的任务会被漏掉，所以这里取所有任务里字面最高的一层。
    pub fn highest_pending(&self) -> Option<i16> {
        self.mine
            .iter()
            .map(|c| c.level)
            .max()
            .or_else(|| self.highest_car_call())
    }

    /// 全部未完成任务里最低的一层，方向不分。
    pub fn lowest_pending(&self) -> Option<i16> {
        self.mine
            .iter()
            .map(|c| c.level)
            .min()
            .or_else(|| self.lowest_car_call())
    }
}

/// 轿厢的下一程。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Leg {
    /// 上行去 `target` 层
    Up(i16),
    /// 下行去 `target` 层
    Down(i16),
    /// 原地不动
    Park,
}

/// 决定轿厢往哪走。
///
/// 这是最常被替换的一个点。只想改走向的研究者只写这里，不碰另外两个。
///
/// **停靠决策归内核，不归算法。** 算法只回答「下一程去哪」，内核负责沿这条腿
/// 逐层推进，并在有乘客要下、或名下有同向外呼、且制动来得及的楼层停靠。
/// 这样算法无论怎么写都不会「路过该停的层」或「停在没活可干的层」。
pub trait Router {
    /// 算法名。绑定层（`tihu-wasm` 的 `ROUTERS` 表）用它把下拉框里的名字
    /// 对到这个类型上，报告里的 `router` 字段取的也是那个名字。两边必须一致，
    /// 由 `table_names_match_the_types` 那条测试钉住。
    fn name(&self) -> &'static str;

    /// 决定轿厢下一程去哪。返回 `Leg::Park` 表示原地待命。
    ///
    /// 车上有乘客时内核会先送客，不经过这里。算法只处理「接了活去哪」以及
    /// 「彻底空闲时停哪」两种情况。
    fn next_leg(&self, v: &RunView<'_>) -> Leg;

    /// 场景开始时调用一次，用来清掉上一轮留下的私有状态
    fn reset(&mut self) {}
}

/// 派梯。决定一条外呼归哪台轿厢。
pub trait Assigner {
    fn name(&self) -> &'static str {
        "unnamed"
    }

    /// 一条新外呼产生时被调用。返回 `None` 表示暂时不派
    fn assign(&self, v: &SystemView<'_>, call: HallCall) -> Option<CarId>;

    /// 每个事件之后调用一次，允许推翻先前的分配。
    ///
    /// `owned` 是当前所有外呼的归属，可以就地改写。
    /// 这是区分派梯算法好坏的关口，见 `docs/MODEL.md` 第 8.4 节。
    fn reassign(&self, _v: &SystemView<'_>, _owned: &mut [Option<CarId>]) {}

    fn reset(&mut self) {}
}

/// 上下客规则的判据。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// 可以上
    Admit,
    /// 没位子了
    NoSpace,
    /// 有位子但这位乘客不愿意挤
    Declined,
}

/// 考察一位乘客时能看到的全部信息。
#[derive(Debug, Clone, Copy)]
pub struct AdmitSample {
    /// 轿厢当前载客率
    pub load_ratio: f64,
    /// 空着的位数
    pub free_slots: usize,
    /// 这位乘客已经等了多久
    pub waited_s: f64,
    /// 轿厢载重相对额定载重的倍数
    pub weight_ratio: f64,
}

/// 决定谁有权上车、什么时候关门。
///
/// 绝大多数评测不需要动这个。默认实现就是 `docs/MODEL.md` 第 5.5 节那套规则。
pub trait Boarding {
    fn name(&self) -> &'static str {
        "default"
    }

    /// 考察一位乘客是否上车。
    ///
    /// 注意返回 `Declined` 与返回 `NoSpace` 的区别。前者说明运力够但人不愿意，
    /// 后者说明运力真的不够。混成一个数会让「该不该加电梯」这个判断失真。
    fn admit_one(&self, s: &AdmitSample, tol: f64) -> Verdict;

    /// 这位乘客实际能上车的概率。默认 1.0，也就是 `Admit` 一定上得去。
    ///
    /// 引擎拿这个概率在自己的随机流上掷一次，策略本身不碰随机数。这样同一个
    /// 种子换算法不会把客流序列挪位，A/B 比较才成立。
    fn accept_prob(&self, _s: &AdmitSample, _tol: f64) -> f64 {
        1.0
    }

    /// 车门是否该关。默认是「到了关门时刻就关」。
    ///
    /// 内核按开关门时序算出关门时刻，到点问策略一次。返回 `false` 就再开一个
    /// 保持开启时长，这段时间里新到的人还能上车。一直返回 `false` 会让车停在
    /// 原地，仿真由时间上限兜底。
    fn close_now(&self, _v: &RunView<'_>) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn view(high: Option<i16>, low: Option<i16>) -> RunView<'static> {
        // 这几个测试只关心纯函数式的查询helper，system 传一个空的
        static EMPTY_CARS: [CarBrief; 0] = [];
        static EMPTY_CALLS: [HallCall; 0] = [];
        static EMPTY_Q: [u32; 0] = [];
        let mine: &'static [HallCall] = Box::leak(
            vec![
                HallCall {
                    id: 0,
                    level: high.unwrap_or(20),
                    dir: Dir::Up,
                    owner: Some(0),
                    waiting: 2,
                },
                HallCall {
                    id: 1,
                    level: low.unwrap_or(3),
                    dir: Dir::Down,
                    owner: Some(0),
                    waiting: 5,
                },
            ]
            .into_boxed_slice(),
        );
        RunView {
            now: 0.0,
            car: 0,
            at_level: 10.0,
            heading: Dir::Up,
            load: 8,
            capacity_rated: 13,
            capacity_crush: 17,
            load_kg: 512.0,
            car_calls: &[],
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

    #[test]
    fn extreme_follows_direction() {
        let v = view(Some(28), Some(4));
        assert_eq!(v.extreme(Dir::Up), Some(28));
        assert_eq!(v.extreme(Dir::Down), Some(4));
        assert_eq!(v.highest_pending(), Some(28));
        assert_eq!(v.lowest_pending(), Some(4));
    }

    #[test]
    fn empty_view_reports_no_work() {
        static EMPTY_CARS: [CarBrief; 0] = [];
        static EMPTY_CALLS: [HallCall; 0] = [];
        static EMPTY_Q: [u32; 0] = [];
        let v = RunView {
            now: 0.0,
            car: 0,
            at_level: 10.0,
            heading: Dir::Up,
            load: 0,
            capacity_rated: 13,
            capacity_crush: 17,
            load_kg: 0.0,
            car_calls: &[],
            mine: &[],
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
        };
        assert!(!v.has_work());
        assert_eq!(v.highest_pending(), None);
        assert_eq!(v.lowest_pending(), None);
        assert_eq!(v.extreme(Dir::Up), None);
    }

    #[test]
    fn has_call_checks_level_and_direction() {
        let v = view(Some(20), Some(3));
        assert!(v.has_call(20, Dir::Up));
        assert!(v.has_call(3, Dir::Down));
        assert!(!v.has_call(20, Dir::Down), "方向不对不该算命中");
        assert!(!v.has_call(9, Dir::Up));
    }

    #[test]
    fn load_ratio_uses_rated_capacity() {
        let v = view(None, None);
        assert!((v.load_ratio() - 8.0 / 13.0).abs() < 1e-12);
    }

    #[test]
    fn level_rounds_float_position() {
        let mut v = view(None, None);
        v.at_level = 7.4;
        assert_eq!(v.level(), 7);
        v.at_level = 7.6;
        assert_eq!(v.level(), 8);
    }
}
