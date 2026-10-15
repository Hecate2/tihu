# 调度算法接口 DISPATCH_API

这份文档只有一个目的：**让你在 100 行以内写出一个新的调度算法，并且不改动
`tihu-core` 的任何一行。**

如果这个承诺没有兑现，那是本仓库的 bug。请直接提。

---

## 1. 三个替换点

| 替换点 | 管什么 | 什么时候被调用 |
| --- | --- | --- |
| `Router` | 一台轿厢下一程去哪一层 | 每次轿厢停稳、以及每次它重新决策时 |
| `Assigner` | 一条外呼派给哪台轿厢 | 每次产生新外呼，以及每次重分配检查 |
| `Boarding` | 谁有权上车 | 每次停靠，每位候选乘客一次 |

三者互不耦合。绝大多数研究者只需要写 `Router`。

---

## 2. 最重要的一条：停靠决策归内核

**算法只回答「下一程去哪」，不回答「这一层停不停」。**

内核沿这条腿逐层推进，每到一层自己判断要不要停，判据只有两条：

1. 车上有乘客要在这一层下
2. 这一层在这条腿的预定停靠表里

预定停靠表由内核在开腿时填，来源是这台轿厢名下的同向外呼与车上乘客的目的层。
外呼在途中被指派进来时，内核先做一次制动可行性检查，来得及才插进去。

这条分工解决了两类老问题。算法再怎么写都不会「路过该停的楼层」，也不会「停在
没活可干的楼层」。你不需要在算法里处理这些。

因此 `Router` 只有一个方法。

```rust
pub trait Router {
    fn name(&self) -> &'static str;

    /// 决定轿厢下一程去哪。返回 `Leg::Park` 表示原地待命
    fn next_leg(&self, v: &RunView<'_>) -> Leg;

    /// 每次仿真开始时调用一次，清掉上一轮留下的私有状态
    fn reset(&mut self) {}
}

pub enum Leg {
    Up(i16),
    Down(i16),
    Park,
}
```

`Leg::Up(20)` 的意思是「接下来开到 20 层去」，方向由目标层与当前层的大小关系决定。
返回 `Leg::Down(20)` 且车在 10 层，效果与 `Leg::Up(20)` 相同。写清楚方向只是
为了可读。

还有一个硬约束：**车上有乘客时内核会先送客，不经过 `next_leg`。** 送人是电梯的
义务，压过一切优化。算法只处理「接了活去哪」与「彻底空闲时停哪」。

---

## 3. 算法能看到什么

所有替换点拿到的都是**只读视图**。算法不许改动世界，只能回答决策问题。

```rust
pub struct RunView<'a> {
    pub now: f64,
    pub car: CarId,
    pub at_level: f32,          // 当前所在层，可以是小数
    pub heading: Dir,           // Up / Down / None
    pub load: usize,            // 在梯人数
    pub capacity_rated: usize,  // 额定人数
    pub capacity_crush: usize,  // 物理极限人数
    pub load_kg: f32,           // 当前载重
    pub car_calls: &'a [i16],   // 在梯乘客的目的层，升序
    pub mine: &'a [HallCall],   // 名下未服务的外呼
    pub system: SystemView<'a>, // 整机视图
    pub floor_height: f32,
}

pub struct SystemView<'a> {
    pub now: f64,
    pub cars: &'a [CarBrief],   // 每台轿厢的位置、方向、载客率、名下待办数
    pub pending: &'a [HallCall],   // 还没分配给任何轿厢的外呼
    pub all_calls: &'a [HallCall], // 全部外呼，含已分配
    pub queue_len: &'a [u32],      // 每层电梯厅里现在有几个人
    pub top_level: i16,
    pub n_cars: usize,
}
```

视图里的两个切片直接借用内核的复用缓冲，构造视图不分配内存。算法可以放心在
热路径上反复调用。

`RunView` 上有一组现成的查询方法，省得你手写循环。

| 方法 | 返回什么 |
| --- | --- |
| `level()` | 当前层号，`at_level` 取整 |
| `will_alight(level)` | 在梯乘客里有没有人要在这一层下 |
| `has_call(level, dir)` | 名下有没有这一条外呼 |
| `has_call_up()` / `has_call_down()` | 名下有没有任何上行业务 / 下行业务 |
| `extreme(dir)` | 名下该方向外呼里最高或最低的一层 |
| `highest_car_call()` / `lowest_car_call()` | 在梯乘客目的层的最高或最低 |
| `highest_pending()` / `lowest_pending()` | 名片下全部任务里最高或最低的一层 |
| `load_ratio()` | 载客率 |
| `has_work()` | 名片下还有没有没干完的活 |

### 3.1 算法能看到的整车状态

| 位置 | 有什么 |
| --- | --- |
| `RunView.load` | 在梯人数 |
| `RunView.load_kg` | 当前载重，公斤 |
| `RunView.car_calls` | 在梯乘客的目的层，升序 |
| `RunView.mine` | 这台车名下的外呼 |
| `RunView.system.cars` | 每台车的位置、方向、人数、载重、名下待办数、是否僵住 |
| `RunView.system.pending` | 还没派出去的外呼 |
| `RunView.system.all_calls` | 全部外呼，含已分配 |
| `RunView.system.queue_len` | 每层电梯厅里现在有几个人 |
| `RunView.floor_height` | 层高 |

`CarBrief` 里带了 `load_kg`，所以派梯算法可以直接看载重，不必只看人数。
按方向分开的每层等待人数还没有暴露，需要的话再加。见 [`RL.md`](RL.md) 第 4.2 节
关于把视图拍成定长观测向量的讨论。

**楼层编号是 `i16`，层号 1 是主入口层，住宅从 2 起，地下是负数。**
写算法时不要假设层号从 1 开始就有住户，也不要假设只有下行客。

---

## 4. 内建实现

| 名字 | 行为 |
| --- | --- |
| `collective_scan` | 经典集选。保持方向，把沿途同向外呼与内选服务完，到底才反向 |
| `selective_collective` | 集选加满载直驶。载客率过默认 80% 的阈值后只按内选走 |
| `down_peak_sweep` | 早高峰专用。先上行到最高任务层，再一路下行扫到底。**默认算法** |
| `nearest_car` | 派给位置上最近的一台 |
| `round_robin` | 轮流派遣 |
| `eta_min` | 派给预计到达最快的那台 |
| `load_aware_eta` | 到达时间加载客惩罚，还会预判到那层时是不是已经挤满。**默认派梯** |
| `with_rescue` | 包装器。发现某台轿厢长时间没推进，把它的活改派给别人 |

---

## 5. 完整例子

下面这个算法能用，而且不到三十行。它就是内建的 `down_peak_sweep`。

```rust
use tihu_core::policy::*;

pub struct DownPeakSweep;

impl Router for DownPeakSweep {
    fn name(&self) -> &'static str { "down_peak_sweep" }

    fn next_leg(&self, v: &RunView<'_>) -> Leg {
        // 名下还有下行任务就继续往下扫
        if let Some(f) = v.extreme(Dir::Down) {
            if f < v.level() { return Leg::Down(f); }
        }
        if let Some(f) = v.lowest_car_call() {
            if f < v.level() { return Leg::Down(f); }
        }
        // 下行任务清空了，去接最高的一条，然后重新往下扫
        let highest = v.highest_pending().or_else(|| v.highest_car_call());
        match highest {
            Some(f) if f > v.level() => Leg::Up(f),
            Some(f) => Leg::Down(f),
            // 彻底没活，回主入口层待命
            None => {
                let lobby = tihu_core::config::LOBBY;
                if v.level() > lobby { Leg::Down(lobby) }
                else if v.level() < lobby { Leg::Up(lobby) }
                else { Leg::Park }
            }
        }
    }
}
```

写完加一行注册，它同时出现在网页下拉框与命令行的 `--router` 参数里。
**整个过程不需要改 `tihu-core` 的任何一行。**

---

## 6. `Assigner`

```rust
pub trait Assigner {
    fn name(&self) -> &'static str { "unnamed" }

    /// 一条新外呼产生时被调用。返回 None 表示暂时不派
    fn assign(&self, v: &SystemView<'_>, call: HallCall) -> Option<CarId>;

    /// 每个事件之后调用一次，允许推翻先前的分配
    fn reassign(&self, v: &SystemView<'_>, owned: &mut [Option<CarId>]) {}

    fn reset(&mut self) {}
}
```

### 6.1 停靠区域由内核兜底

一个算法如果派了一台不停那一层的轿厢去接这条外呼，内核会换一台真正服务的车
顶上。所以算法不检查停靠区域也不会出错，但会失去对派梯结果的控制。想写得准，
就自己先筛一遍。

### 6.2 `reassign` 为什么会拉开差距

轿厢可能因为超载报警长时间僵在原地，名下那些未服务的外呼就悬在半空。
**层上的人再按一次按钮是没有用的**，灯已经亮着。

不支持重分配的 `Assigner` 会让这些人一直等到僵局自己解除。支持重分配的
`Assigner` 一旦发现某台轿厢长时间没推进，就把任务改派给别人。同一个场景下
两种做法的平均等待常常差出好几倍。见 [`MODEL.md`](MODEL.md) 第 8 节。

---

## 7. `Boarding`

```rust
pub trait Boarding {
    fn name(&self) -> &'static str { "default" }

    /// 考察一位乘客是否上车
    fn admit_one(&self, s: &AdmitSample, tol: f64) -> Verdict;
}

pub enum Verdict {
    Admit,     // 可以上
    NoSpace,   // 真的没位子了
    Declined,  // 有位子，但这位乘客不愿意挤
}
```

`AdmitSample` 给出载客率、空位、这位乘客已经等了多久、以及载重相对额定的倍数。

**`NoSpace` 与 `Declined` 必须分开。** 前者说明运力真不够，后者说明运力够但人
不愿意。混成一个数会让「这个楼盘要不要加电梯」的判断失真。

车门什么时候关不归算法管，内核按开关门时序自己算。

---

## 8. 怎么公正地比两个算法

### 8.1 必须用同一批人

所有乘客在仿真开始之前一次性生成完。同一个种子配任何算法，楼里都是同一批人。
这是公平比较的前提，也是 [`ARCHITECTURE.md`](ARCHITECTURE.md) 第 5 节分流做法的
目的。

### 8.2 用配对差值，不要比均值

朴素做法是分别算两个算法的均值再相减。噪声大得让人绝望。

正确做法是**同一个种子跑两个算法，逐种子取差值**，再对差值序列做均值与置信
区间。公共随机数把噪声消掉了一大截。

```
diff_i = 指标(算法A, 种子i) − 指标(算法B, 种子i)
报告 mean(diff) 与 bootstrap 95% 置信区间
```

### 8.3 一条命令

```
tihu-cli bench --routers collective_scan,down_peak_sweep --assigners nearest_car,load_aware_eta \
               --suite std --seeds 200 --out table.md
```

输出表结构如下，实测数字待跑。

| 派梯 | 走向 | 平均等待 | 相对基线 | 95% 置信区间 | P90 等待 | 被挤率 |
| --- | --- | --- | --- | --- | --- | --- |
| 待跑 | 待跑 | 待跑 | 待跑 | 待跑 | 待跑 | 待跑 |

<!-- TODO: 这张表由 tihu-cli bench 实跑生成，不许手填 -->

`tihu-cli` 目前还是空壳，这条命令还没实现。见 [`PLAN.md`](PLAN.md)。

---

## 9. 标准测试套件

`tihu-cli bench --suite std` 将来会跑下面这组固定场景。每个新算法都应该能在这里
交出成绩。

| 场景 | 配置 | 考什么 |
| --- | --- | --- |
| `std_residential` | 十八层住宅，二梯四户 | 常规住宅 |
| `std_tall` | 三十三层住宅，二梯六户 | 高层高密度 |
| `std_underserved` | 三十三层住宅，一梯四户 | 运力明显不够时的表现 |
| `stress_gaming` | 三成的人反向叫梯 | 人人抢座的环境下的稳健性 |
| `stress_stalled_car` | 开启超载死锁 | 轿厢长时间停滞时的恢复能力 |
| `stress_mixed_traffic` | 双向均衡客流 | 两向需求接近时的调度能力 |

---

## 10. 常见坑

1. **别在 `Router` 里跨帧缓存状态却不处理 `reset`。** 忘了的话第二次运行会带着
   上一次的脏状态。
2. **别假设自己能决定停不停。** 你只选目标层，停靠由内核定。想在途中插一次停靠，
   只能通过改目标层实现。
3. **别忘了地下室与主入口层。** 层号是 `i16`，1 是主入口，负数在地下。
4. **别假设只有下行客。** 早高峰仍有百分之几的上行客流。
5. **别拿 `NaN` 当「没有估价」。** 返回一个枚举，让类型系统替你兜着。
6. **不要为了让自己的算法好看去改 `Boarding`。** 评测时统一用默认规则，换规则
   请单独说明，别混在一张表里。
