# 梯户模拟器

`tihu`

销售告诉你的是「两梯四户」。他不会告诉你，33 层那栋楼在早上七点五十到八点二十之间，有多少人同时在按同一个下行键。他也不会告诉你，第 9 层的人平均要被跳过两趟才挤得进去。

梯户模拟器把三件事算清楚：

- 等多久来
- 来了挤不挤得上
- 进去之后还要晃多久到

它是一个不需要服务器的静态网页，也是一个能吃满 CPU 的命令行扫描器。两种形态共用同一套 Rust 仿真内核，所以结果不会打架。

---

## 马上试

<!-- TODO: 演示站点的地址填到这里 -->

或者本地跑起来：

```bash
git clone https://github.com/<your>/tihu.git
cd tihu
make web-open
```

`make web-open` 会编译内核并打开浏览器。先装好两样东西：

```bash
rustup target add wasm32-unknown-unknown
cargo install wasm-pack
```

想打包成一个文件带走：

```bash
make single      # 产出 dist/tihu.html
```

这个文件把内核内嵌成 base64，断网可以打开，双击也可以打开。

---

## 网页上能调什么

左边填参数，右边出结果。参数分四组：

| 组 | 内容 |
| --- | --- |
| 楼栋 | 层数、每层户数、户均人数、层高、有没有地下室 |
| 轿厢 | 台数、额定载重、上行与下行各自的极限速度、加速度、减速度、加加速度、开关门时序 |
| 客流 | 早高峰还是晚高峰、窗口多长、多大比例的人出行 |
| 行为 | 多少人反向叫梯、多少人同时呼叫多台、多少人愿意走楼梯、要不要开超载死锁 |

填错了会被拦下来。把每层户数填成 20 户，工具会反过来告诉你这一层至少需要多大的楼板面积。

右上角有几个预设可以直接点：18 层二梯四户、33 层二梯六户、33 层一梯四户。

你的配置会自动写进网址，复制链接就把这份楼栋发给别人了。

---

## 结果里有什么

三个大字，加四张图。

**大字**

1. 平均等待，以及 P90 等待。后者才是日常体感，因为平均总是被少数幸运儿拉低
2. 至少被跳过一次的人占多少。这一项才是「挤不挤得上」的答案
3. 进去之后平均还要坐多久才到

**图**

1. 等待时间直方图
2. 逐楼层柱状图。同一栋楼里 9 层和 25 层的差距，一眼看得出来
3. 楼层乘时间的排队人数热力图
4. 反向叫梯的比例扫描线，顺带标出临界点

---

## 内卷那一块

有一个现象很值得算。低楼层的人等下行等不来，干脆先按上行，跟着轿厢一路顶到最上层，再跟着折返下来。

对个人，这笔账经常是划算的。他有座位，肯定能下去。对全楼，每台被占用去做无用功的轿厢都在挤压别人的运力。

网页上的滑块调整这个比例。命令行还支持多日演化：

```bash
tihu-cli evolve --config 33f.toml --days 30 --out evolve.csv
```

第一天只有 5% 的人这么干，第五天变成 40%，第三周全楼都学会了，而所有人的出门时间都比第一天更长。

```bash
tihu-cli nash --config 33f.toml --strategy press_both
```

这条二分找出临界比例。跨过它之后，对个人来说理性的选择就是跟风。

---

## 命令行

单次运行：

```bash
tihu-cli run --floors 33 --per-floor 6 --per-unit 2.8 --cars 2 --seed 42
```

十万级扫描：

```bash
tihu-cli sweep --grid grid.toml --reps 20 --threads auto --out sweep.csv
```

扫出来的 CSV 一行一个组合，列全部摊开，丢进 pandas 或者 Excel 就能画图。

---

## 写一个新的调度算法

这个仓库同时是电梯调度算法的试金石。写一个新算法通常是一百行以内的事，而且不需要改动内核的任何一行。

```rust
impl Router for MyRouter {
    fn should_stop(&self, v: &RunView<'_>) -> bool { /* ... */ }
    fn next_leg(&self, v: &RunView<'_>) -> Leg { /* ... */ }
    fn idle(&self, v: &RunView<'_>) -> Idle { Idle::GoTo(1) }
}

register_router!("my_router", || Box::new(MyRouter));
```

写完之后跟标准算法对打：

```bash
tihu-cli bench --routers collective_scan,my_router --suite std --seeds 200
```

所有算法跑的是同一批客流和同一份种子，报告的是配对差值和置信区间。细节在 [`docs/DISPATCH_API.md`](docs/DISPATCH_API.md)。

---

## 文档

| 想问什么 | 看哪份 |
| --- | --- |
| 仓库红线与工作约定 | [`AGENTS.md`](AGENTS.md) |
| 某个参数代表什么 | [`docs/MODEL.md`](docs/MODEL.md) |
| 代码怎么分层 | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| 怎么写调度算法 | [`docs/DISPATCH_API.md`](docs/DISPATCH_API.md) |
| 有多快 | [`docs/PERFORMANCE.md`](docs/PERFORMANCE.md) |
| 下一步做什么 | [`docs/PLAN.md`](docs/PLAN.md) |

---

## 关于准确度

这是一个量级工具，不是电梯招投标依据。

真实结果受厂家群控算法、开关门延迟、有没有并联、有没有地下层、住户错峰习惯的影响。模型里每个默认值都标了出处，查不到出处的都写了「待标定」，详见 [`docs/MODEL.md`](docs/MODEL.md)。

想让它贴近某一栋具体的楼，用 `tihu-cli calibrate` 接你在现场掐表的数据反推参数。这个功能排在 P6。

---

## License

MIT
