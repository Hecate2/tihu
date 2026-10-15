# 调度算法接口 DISPATCH_API

这份文档只有一个目的：**让你在 100 行以内写出一个新的调度算法，并且不改动 `tihu-core` 的任何一行。**

如果这个承诺没有兑现，那是本仓库的 bug，请直接提。

---

## 1. 三个替换点

| 替换点 | 管什么 | 什么时候被调用 |
| --- | --- | --- |
| `Assigner` | 一条新外呼派给哪台轿厢 | 每次产生新外呼，以及每次重分配检查 |
| `Router` | 一台轿厢往哪走、在哪停 | 每到达一层一次 |
| `Boarding` | 谁有权上车、什么时候关门 | 每次停靠若干次 |

三者互不耦合。绝大多数研究者只需要写 `Router`。`Boarding` 留给想专门研究上下车规则的人：排队礼让的先后顺序、接近满载时的拒挤行为、以及超载之后由谁退出。

---

## 2. 算法能看到什么

所有替换点拿到的都是**只读视图**。算法不许改动世界，只能回答决策问题。这条约束让内核可以安全地随时重算。

```rust
pub enum Dir { Up, Down, None }

pub struct SystemView<'a> {
    pub now: f64,
    pub cars: &'a [CarBrief],        // 每台轿厢的位置、方向、载客率
    pub pending: &'a [HallCall],     // 尚未分配给任何轿厢的外呼
    pub assigned: &'a Assignment,    // 已分配的外呼与归属
    pub queue_len: &'a [(usize, usize)], // 每层等待人数，(上行, 下行)
}

pub struct RunView<'a> {
    pub now: f64,
    pub car: CarId,
    pub at_floor: u16,
    pub heading: Dir,
    pub load: usize,
    pub capacity_rated: usize,
    pub load_ratio: f64,
    pub onboard: &'a [PaxIdx],       // 在梯名单
    pub will_alight: &'a dyn Fn(u16) -> bool, // 这一层是否有人要出
    pub car_calls: &'a FloorSet,     // 内选登记的楼层
    pub mine: &'a CallSet,           // 派给本轿厢且未服务的外呼
    pub system: &'a SystemView<'a>,  // 整机视图
}
```

载客率直接给到 `load_ratio`，省得算法自己算，也保证所有算法看到同一个定义。

**地下室楼层的编号是负層**,用 `i16` 表示层数，零点是一层大堂。写算法时不要假设层号楼从 1 开始。

---

## 3. `Router`

```rust
pub trait Router {
    fn name(&self) -> &'static str;

    /// 轿厢即将经过一层时被调用。返回 true 就减速停靠
    fn should_stop(&self, v: &RunView<'_>) -> bool;

    /// 停完之后决定下一程。返回目标楼层与方向
    fn next_leg(&self, v: &RunView<'_>) -> Leg;

    /// 完全空闲时干什么
    fn idle(&self, v: &RunView<'_>) -> Idle;
}

pub enum Leg { Up(u16), Down(u16), Park }
pub enum Idle { Hold, GoTo(u16) }
```

`should_stop` 必须只读且幂等。接近某一层的过程中，它可能被调用多次，答案必须前后一致。内核不缓存这个答案，你可以放心依赖当前的真实状态。

### 3.1 内建实现

| 名字 | 行为 |
| --- | --- |
| `collective_scan` | 经典集选。保持方向，把沿途同向外呼和内选服务完，到底才反向。**住宅默认值** |
| `selective_collective` | 集选加满载直驶。载客率到阈值后掠过同向外呼 |
| `down_peak_sweep` | 早高峰专用。先上行到最高的任务楼层，再一路下行扫到底 |
| `eta_optimal` | 每次决策做一次轻量前向推算，取使所有等待者的预计等待之和最小的那个方向 |
| `destination_group` | 目的层群控。按 `dest_floor` 分组装填 |

### 3.2 完整例子

下面是一个真能跑的实现，早高峰下行扫的策略。行数不到三十。

```rust
use tihu_core::policy::*;

pub struct DownPeakSweep;

impl Router for DownPeakSweep {
    fn name(&self) -> &'static str { "down_peak_sweep" }

    fn should_stop(&self, v: &RunView<'_>) -> bool {
        // 下行时，本层有下行外呼，或者有人在电梯里要到这一层，就停
        let has_job = v.mine.has(v.at_floor, Dir::Down) || (v.will_alight)(v.at_floor);
        // 上行途中只跑去最高那层接人，中途不停
        match v.heading {
            Dir::Down => has_job,
            Dir::Up   => v.at_floor == v.mine.highest(),
            Dir::None => has_job,
        }
    }

    fn next_leg(&self, v: &RunView<'_>) -> Leg {
        // 名下还有下行任务就继续往下扫
        if let Some(f) = v.mine.lowest(Dir::Down) { return Leg::Down(f); }
        if let Some(f) = v.car_calls.lowest() { return Leg::Down(f); }
        // 下行任务清空了，去接最高的一条任务，然后重新开始往下扫
        let highest = v.mine.highest().max(v.car_calls.highest());
        match highest {
            Some(f) => Leg::Up(f),
            None    => Leg::Park,
        }
    }

    fn idle(&self, _v: &RunView<'_>) -> Idle { Idle::GoTo(1) }
}
```

写完之后只需要在注册表里加一行：

```rust
register_router!("down_peak_sweep", || Box::new(DownPeakSweep));
```

然后它同时出现在网页的下拉框和命令行的 `--router` 参数里。**整个过程不需要改 `tihu-core` 的任何一行。**

---

## 4. `Assigner`

```rust
pub trait Assigner {
    fn name(&self) -> &'static str;

    /// 每个 scenario 开始时调用一次，用来预分配自己的私有状态
    fn reset(&mut self, cfg: &SimConfig);

    /// 一条新外呼产生时被调用，返回派给哪台轿厢。返回 None 表示暂时不派
    fn assign(&mut self, v: &SystemView<'_>, call: HallCall) -> Option<CarId>;

    /// 每个事件之后调用一次，允许你推翻先前的分配结果
    fn reassign(&mut self, v: &SystemView<'_>, plan: &mut Assignment) {}
}
```

### 4.1 内建实现

| 名字 | 行为 |
| --- | --- |
| `nearest_car` | 派给当前位置上最近的那台。**简单粗暴的老系统默认值** |
| `round_robin` | 轮流派遣 |
| `eta_min` | 派给预计到达最快的那台 |
| `load_aware_eta` | 到达时间加一个载客惩罚，还会预判「到那一层时是不是已经挤满」。推薦作为基线 |
| `dest_dispatch` | 目的层派梯。乘客先登记目的楼层，系统指定轿厢 |

### 4.2 `reassign` 为什么会拉开差距

这是最值得写算法的地方。轿厢可能因为超载报警长时间僵在原地，它名下那些未服务的外呼就悬在半空。**层上的人再按一次按钮是没有用的**，灯已经亮着。

不支持重分配的 `Assigner` 会让这些人一直等到僵局自己解除。支持重分配的 `Assigner` 一旦发现某台轿厢长时间没有推进,就把任务改派给别人。

同一个场景下，两种做法的平均等待时间常常差出好几倍。详见 `docs/MODEL.md` 第 8.4 节。

---

## 5. `Boarding`

```rust
pub trait Boarding {
    /// 逐个考察队首的同行团，决定它能不能上
    fn try_board(&self, ctx: &mut BoardCtx<'_>) -> Verdict;

    /// 车门当前是否该关
    fn close_now(&self, v: &RunView<'_>) -> bool;
}
```

默认实现 `DefaultBoarding` 就是 `MODEL.md` 第 5 节写好的那套规则：方向匹配、空间够、逐人掷拒挤判定。

想在研究方向上做文章的，通常换的是这两件事：

- 排队礼让规则。默认按到达时刻先来后到。可以改成离门近者优先，或者随机哄抢
- 超载后的处理。默认就是我们 MODEL.md 写好的僵持循环，可以改成「最后上车的人必然退出」

绝大多数评测不需要动这里。**不确定要不要动，就不要动。**

---

## 6. 怎么公正地比两个算法

### 6.1 必须用同一批人

两个算法要比，必须让它们服务同一批乘客。`ARCHITECTURE.md` 第 5 节解释了随机流的分流做法。

### 6.2 用配对差值，不要比均值

朴素做法是分别算两个算法的均值再相减。噪声大得让人绝望。

正确做法是**同一个种子跑两个算法，逐种子取差值**，再对差值序列做均值与置信区间。公共随机数把噪声消掉了一大截，同样的工作量能把结论的可信度提高一个量级。

```
diff_i = 指标(算法A, 种子i) − 指标(算法B, 种子i)
报告 mean(diff) 与 bootstrap 95% 置信区间
```

### 6.3 一条命令

```
tihu-cli bench --routers collective_scan,eta_optimal --assigners nearest_car,load_aware_eta \
               --suite std --seeds 200 --out table.md
```

输出表结构如下，实测数字待跑。

| 派梯 | 走向 | 平均等待 | 相对基线 | 95% 置信区间 | P90 等待 | 被挤率 |
| --- | --- | --- | --- | --- | --- | --- |
| 待跑 | 待跑 | 待跑 | 待跑 | 待跑 | 待跑 | 待跑 |

<!-- TODO: 这张表由 `make bench-table` 实跑生成，不许手填 -->

---

## 7. 标准测试套件

`tihu-cli bench --suite std` 会跑下面这组固定场景。每个新算法都应该能在这里交出成绩。

| 场景 | 配置 | 考什么 |
| --- | --- | --- |
| `std_residential` | 18 层二梯四户 | 常规住宅 |
| `std_tall` | 33 层二梯六户 | 高层高密度 |
| `std_underserved` | 33 层一梯四户 | 运力明显不够时的表现 |
| `std_evening` | 晚高峰上行 | 反向客流的处理 |
| `stress_gaming` | 三成的人反向叫梯 | 人人抢座的环境下的稳健性 |
| `stress_stalled_car` | 开启超载死锁 | **轿厢长时间停滞时的恢复能力** |
| `stress_mixed_traffic` | 双向均衡客流 | 两向需求接近时的调度能力 |

---

## 8. 常见坑

1. **别在 `Router` 里跨帧缓存状态却不处理 `reset`。** 忘了的话，第二次运行会带着上一次的脏状态。
2. **`should_stop` 不许有副作用。** 它是被反复调用的，改了世界会让内核的状态和它的记录对不上。
3. **别忘了地下室。** 层数用 `i16`，1 以下还有 B1、B2。
4. **别假设只有下行客。** 早高峰仍然有百分之几的上行客流，写 `DownPeakSweep` 这种专用算法时要显式处理它。
5. **别用 `f64::NAN` 当「没有估价」。** 返回一个枚举，让类型系统替你兜着。
6. **不要为了让自己的算法好看去改 `Boarding`。** 评测时统一用默认规则，换规则请单独说明，别混在一张表里。
