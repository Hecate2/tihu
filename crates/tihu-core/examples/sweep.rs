//! 参数扫描示例。这一份同时是 `docs/SIMULATE.md` 里所有数字的出处。
//!
//! 用法：
//!
//! ```bash
//! cargo run --release --example sweep -p tihu-core
//! ```
//!
//! 每个参数点跑五个种子再取平均。单跑一次的结果会被客流抽签的噪声盖住，
//! 看趋势必须先把噪声压下去。这不是可选项，是读这类表的前提。

use tihu_core::config::{preset, Config, Pattern, StopSpec};
use tihu_core::motion::MotionProfile;
use tihu_core::sim::quick;

/// 每个参数点跑几个种子。
const SEEDS: [u64; 5] = [1, 2, 3, 4, 5];

/// 一次扫描读出来的六个量。全部是五个种子上的平均。
#[derive(Debug, Clone, Copy, Default)]
struct Stats {
    /// 平均等梯时间，秒
    wait_mean: f64,
    /// P90 等梯时间，秒
    wait_p90: f64,
    /// 至少被跳过一次的乘客占送达人数的比例
    bypass: f64,
    /// 出门前就走楼梯的人数占比
    stairs: f64,
    /// 送达人数占总人数的比例
    served: f64,
    /// 往返时间均值，秒
    rtt: f64,
}

impl Stats {
    /// 同一份配置跑多个种子，取平均。
    fn of(cfg: &Config) -> Self {
        let n = SEEDS.len() as f64;
        let mut s = Stats::default();
        for seed in SEEDS {
            let r = quick(cfg.clone(), seed);
            let m = &r.metrics;
            s.wait_mean += m.wait.mean();
            s.wait_p90 += m.wait.p90();
            s.bypass += m.pax_any_bypass;
            s.stairs += m.stair_share;
            s.rtt += m.rtt_mean;
            if m.pax_total > 0 {
                s.served += m.pax_served as f64 / m.pax_total as f64;
            }
        }
        s.wait_mean /= n;
        s.wait_p90 /= n;
        s.bypass /= n;
        s.stairs /= n;
        s.served /= n;
        s.rtt /= n;
        s
    }
}

/// 一张固定宽度的表。直接输出成 markdown，方便贴进文档。
struct Table {
    cols: Vec<(&'static str, usize)>,
}

impl Table {
    fn new(cols: Vec<(&'static str, usize)>) -> Self {
        Self { cols }
    }

    fn head(&self) {
        let mut line = String::from("|");
        let mut bar = String::from("|");
        for (name, w) in &self.cols {
            line.push_str(&format!(" {name:<w$} |", w = *w));
            bar.push_str(&format!(" {} |", "-".repeat(*w)));
        }
        println!("{line}");
        println!("{bar}");
    }

    fn row(&self, cells: &[String]) {
        let mut line = String::from("|");
        for (i, (_, w)) in self.cols.iter().enumerate() {
            let c = cells.get(i).map(|s| s.as_str()).unwrap_or("");
            // 中文字符占了两个显示宽度，按字符数补空格会歪。这里用显示的宽度估算
            let shown = c.chars().map(|ch| if ch as u32 > 0x2000 { 2 } else { 1 }).sum::<usize>();
            let pad = w.saturating_sub(shown);
            line.push_str(&format!(" {c}{} |", " ".repeat(pad)));
        }
        println!("{line}");
    }
}

fn f1(x: f64) -> String {
    format!("{x:.1}")
}

fn pct(x: f64) -> String {
    format!("{:.1}%", x * 100.0)
}

/// 一栋住宅楼，指定台数。
fn house(tall: bool, cars: usize) -> Config {
    let mut c = preset(if tall { "std_tall" } else { "std_residential" }).unwrap();
    c.cars.resize(cars, tihu_core::config::CarSpec::default());
    c
}

fn section(title: &str, note: &str) {
    println!();
    println!("### {title}");
    println!();
    println!("{note}");
    println!();
}

fn main() {
    println!("全部数字由本文件实跑得出，每个点五个种子取平均。");
    println!("层号 1 是主入口层，所以「十八层住宅」的顶层号是 19。");

    // ---------------- 梯数 ----------------
    section(
        "梯数从一台加到四台",
        "十八层四户与三十三层六户，早高峰下行，出行率 55%。",
    );
    let t = Table::new(vec![
        ("楼盘", 12),
        ("台数", 4),
        ("平均等梯 s", 12),
        ("P90 等梯 s", 12),
        ("被跳过", 8),
        ("送达率", 8),
        ("往返 s", 8),
    ]);
    t.head();
    for (name, tall) in [("十八层四户", false), ("三十三层六户", true)] {
        for cars in 1..=4 {
            let s = Stats::of(&house(tall, cars));
            t.row(&[
                name.to_string(),
                cars.to_string(),
                f1(s.wait_mean),
                f1(s.wait_p90),
                pct(s.bypass),
                pct(s.served),
                f1(s.rtt),
            ]);
        }
    }

    // ---------------- 出行率 ----------------
    section(
        "出行率",
        "三十三层六户两梯，早高峰下行。出行率是窗口内出门住户的比例。",
    );
    let t = Table::new(vec![
        ("出行率", 8),
        ("生成人数", 10),
        ("平均等梯 s", 12),
        ("P90 等梯 s", 12),
        ("被跳过", 8),
        ("送达率", 8),
    ]);
    t.head();
    for dep in [0.20, 0.35, 0.55, 0.75, 0.95] {
        let mut c = house(true, 2);
        c.traffic.departure_ratio = dep;
        c.traffic.window_s = 3600.0;
        c.sim.horizon_s = 5400.0;
        let s = Stats::of(&c);
        t.row(&[
            format!("{:.0}%", dep * 100.0),
            c.expected_trips().to_string(),
            f1(s.wait_mean),
            f1(s.wait_p90),
            pct(s.bypass),
            pct(s.served),
        ]);
    }

    // ---------------- 停靠分区 ----------------
    section(
        "停靠分区",
        "三十三层六户两梯。分区梯改变的是每台车扫楼的节奏，不是一个可以乘常数抹平的差别。",
    );
    let t = Table::new(vec![
        ("停靠方式", 14),
        ("平均等梯 s", 12),
        ("P90 等梯 s", 12),
        ("被跳过", 8),
        ("往返 s", 8),
    ]);
    t.head();
    let top = 34;
    let variants: [(&str, Vec<StopSpec>); 4] = [
        ("两台都全停", vec![StopSpec::All, StopSpec::All]),
        ("单双层", vec![StopSpec::parity(true), StopSpec::parity(false)]),
        (
            "高低区",
            vec![
                StopSpec::zone(1, 17),
                StopSpec::union(vec![StopSpec::zone(18, top), StopSpec::zone(1, 1)]),
            ],
        ),
        ("一台全停一台单层", vec![StopSpec::All, StopSpec::parity(true)]),
    ];
    for (name, specs) in variants {
        let mut c = house(true, 2);
        for (car, spec) in c.cars.iter_mut().zip(specs) {
            car.stops = spec;
        }
        let v = c.validate();
        if !v.is_ok() {
            println!("| {name} 配置不合法，跳过 |", name = name);
            continue;
        }
        let s = Stats::of(&c);
        t.row(&[name.to_string(), f1(s.wait_mean), f1(s.wait_p90), pct(s.bypass), f1(s.rtt)]);
    }

    // ---------------- 速度档 ----------------
    section(
        "额定速度档",
        "三十三层六户两梯，全停。速度档连加速度与加加速度一起换，见 docs/MODEL.md 第 3.5 节。",
    );
    let t = Table::new(vec![
        ("速度档", 12),
        ("额定 m/s", 10),
        ("平均等梯 s", 12),
        ("P90 等梯 s", 12),
        ("往返 s", 8),
    ]);
    t.head();
    for name in ["low_rise_1_0", "std_1_6", "std_1_75", "high_2_5", "high_3_0"] {
        let p = MotionProfile::preset(name).unwrap();
        let mut c = house(true, 2);
        for car in &mut c.cars {
            car.up = p;
            car.down = p;
        }
        let s = Stats::of(&c);
        t.row(&[name.to_string(), format!("{:.2}", p.v_max), f1(s.wait_mean), f1(s.wait_p90), f1(s.rtt)]);
    }

    // ---------------- 客流模式 ----------------
    section(
        "客流时段",
        "三十三层六户两梯，全停。同一栋楼换时段，堵的位置不一样。",
    );
    let t = Table::new(vec![
        ("时段", 14),
        ("平均等梯 s", 12),
        ("P90 等梯 s", 12),
        ("被跳过", 8),
        ("走楼梯", 8),
    ]);
    t.head();
    for (name, p) in [
        ("早高峰下行", Pattern::MorningDown),
        ("晚高峰上行", Pattern::EveningUp),
        ("中午双向", Pattern::NoonMixed),
    ] {
        let mut c = house(true, 2);
        c.traffic.pattern = p;
        if p == Pattern::NoonMixed {
            c.traffic.reverse_ratio = 0.5;
        }
        let s = Stats::of(&c);
        t.row(&[name.to_string(), f1(s.wait_mean), f1(s.wait_p90), pct(s.bypass), pct(s.stairs)]);
    }

    // ---------------- 走楼梯 ----------------
    section(
        "走楼梯开关",
        "十八层四户两梯。走楼梯的人下车流，等梯时间跟着变，但只变一点点。",
    );
    let t = Table::new(vec![
        ("走楼梯", 10),
        ("平均等梯 s", 12),
        ("P90 等梯 s", 12),
        ("走楼梯比例", 12),
        ("送达率", 8),
    ]);
    t.head();
    for (name, on) in [("开启", true), ("关闭", false)] {
        let mut c = house(false, 2);
        c.stairs.enabled = on;
        let s = Stats::of(&c);
        t.row(&[name.to_string(), f1(s.wait_mean), f1(s.wait_p90), pct(s.stairs), pct(s.served)]);
    }

    // ---------------- 种子噪声 ----------------
    section(
        "种子的噪声有多大",
        "三十三层六户两梯，同一份配置换种子。看趋势时至少要跑三个种子，否则会把噪声当趋势。",
    );
    let c = house(true, 2);
    let t = Table::new(vec![("种子", 8), ("平均等梯 s", 12), ("P90 等梯 s", 12), ("送达率", 8)]);
    t.head();
    let mut means = Vec::new();
    for seed in 1..=5u64 {
        let r = quick(c.clone(), seed);
        let m = &r.metrics;
        means.push(m.wait.mean());
        t.row(&[
            seed.to_string(),
            f1(m.wait.mean()),
            f1(m.wait.p90()),
            pct(if m.pax_total > 0 { m.pax_served as f64 / m.pax_total as f64 } else { 0.0 }),
        ]);
    }
    let lo = means.iter().cloned().fold(f64::INFINITY, f64::min);
    let hi = means.iter().cloned().fold(0.0_f64, f64::max);
    println!();
    println!(
        "五个种子的平均等梯从 {:.1} 秒到 {:.1} 秒，跨度 {:.1} 秒，是均值的 {:.0}%。",
        lo,
        hi,
        hi - lo,
        (hi - lo) / (means.iter().sum::<f64>() / 5.0) * 100.0
    );
}
