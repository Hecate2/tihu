//! 一段随机种子的批量运行。
//!
//! 每个种子跑完立刻取数、立刻把那份 `Metrics` 丢掉，全程只留累加器。单份
//! `Metrics` 的逐层直方图有 1200 格，三十四层就是一兆多字节，攒一千份会把
//! WASM 线性内存顶穿。这里的内存只与楼层数有关，与种子个数无关。
//!
//! 汇总口径：所有秒数与比例取各次运行的平均，逐层的较好与较差取各次平均里
//! 的最小与最大，等待时间的分布把各次的格数相加。某个种子某层一个乘客都没
//! 有出发时，那一次不进这一层的平均，否则零样本会把均值拉下去。
//!
//! 与单次运行的关系见 [`crate::sim::run`]。这里逐位可复现：种子序列是显式
//! 的，同一段范围跑两次结果一模一样。

use crate::config::Config;
use crate::metrics::{FloorStats, Metrics, Summary};
use crate::policy::{Assigner, Boarding, Router};
use crate::sim;

/// 一个统计量的累加器。非有限值直接丢掉，免得 NaN 顺着平均传下去。
#[derive(Debug, Clone, Copy, Default)]
struct Acc {
    n: u64,
    sum: f64,
}

impl Acc {
    fn add(&mut self, x: f64) {
        if x.is_finite() {
            self.n += 1;
            self.sum += x;
        }
    }

    fn mean(&self) -> f64 {
        if self.n == 0 {
            0.0
        } else {
            self.sum / self.n as f64
        }
    }
}

/// 一组时间样本在各次运行上的平均。字段与网页报告里的 `Num` 一一对应。
#[derive(Debug, Clone, Copy, Default)]
pub struct NumAvg {
    pub count: f64,
    pub mean: f64,
    pub p50: f64,
    pub p75: f64,
    pub p90: f64,
    pub p95: f64,
    pub p99: f64,
    pub max: f64,
    pub min: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct NumAcc {
    count: Acc,
    mean: Acc,
    p50: Acc,
    p75: Acc,
    p90: Acc,
    p95: Acc,
    p99: Acc,
    max: Acc,
    min: Acc,
}

impl NumAcc {
    fn add(&mut self, s: &Summary) {
        self.count.add(s.count as f64);
        self.mean.add(s.mean());
        self.p50.add(s.p50());
        self.p75.add(s.p75());
        self.p90.add(s.p90());
        self.p95.add(s.p95());
        self.p99.add(s.p99());
        self.max.add(s.max);
        // 空样本的 min 是无穷大，那种次数不算一次
        self.min.add(if s.count == 0 { 0.0 } else { s.min });
    }

    fn finish(&self) -> NumAvg {
        NumAvg {
            count: self.count.mean(),
            mean: self.mean.mean(),
            p50: self.p50.mean(),
            p75: self.p75.mean(),
            p90: self.p90.mean(),
            p95: self.p95.mean(),
            p99: self.p99.mean(),
            max: self.max.mean(),
            min: self.min.mean(),
        }
    }
}

/// 某一层在各次运行上的汇总。
#[derive(Debug, Clone, Copy, Default)]
pub struct FloorAvg {
    /// 平均每次运行从这一层出发几个人
    pub count: f64,
    /// 各次运行该层平均等待时间的平均
    pub wait_mean: f64,
    /// 各次运行该层平均等待时间里最短的那次
    pub wait_best: f64,
    /// 各次运行该层平均等待时间里最长的那次
    pub wait_worst: f64,
    /// 有几个种子这一层真有人出发。零样本的种子不进上面三个数
    pub live: usize,
    pub wait_p50: f64,
    pub wait_p90: f64,
    pub wait_max: f64,
    /// 各次运行该层门到门均值的平均
    pub d2d_mean: f64,
    /// 各次运行该层门到门均值里最短的那次
    pub d2d_best: f64,
    /// 各次运行该层门到门均值里最长的那次
    pub d2d_worst: f64,
    pub d2d_p50: f64,
    pub d2d_p90: f64,
    pub d2d_max: f64,
    pub bypass_total: f64,
    pub declined: f64,
    pub no_space: f64,
    pub stair_users: f64,
}

#[derive(Debug, Clone, Copy)]
struct FloorAcc {
    count: Acc,
    wait: Acc,
    wait_p50: Acc,
    p90: Acc,
    wait_max: Acc,
    d2d: Acc,
    d2d_p50: Acc,
    d2d_p90: Acc,
    d2d_max: Acc,
    bypass_total: Acc,
    declined: Acc,
    no_space: Acc,
    stair_users: Acc,
    best: f64,
    worst: f64,
    d2d_best: f64,
    d2d_worst: f64,
    live: usize,
}

impl Default for FloorAcc {
    fn default() -> Self {
        Self {
            count: Acc::default(),
            wait: Acc::default(),
            wait_p50: Acc::default(),
            p90: Acc::default(),
            wait_max: Acc::default(),
            d2d: Acc::default(),
            d2d_p50: Acc::default(),
            d2d_p90: Acc::default(),
            d2d_max: Acc::default(),
            bypass_total: Acc::default(),
            declined: Acc::default(),
            no_space: Acc::default(),
            stair_users: Acc::default(),
            best: f64::INFINITY,
            worst: 0.0,
            d2d_best: f64::INFINITY,
            d2d_worst: 0.0,
            live: 0,
        }
    }
}

impl FloorAcc {
    fn add(&mut self, f: &FloorStats) {
        self.count.add(f.count as f64);
        self.bypass_total.add(f.bypass_total as f64);
        self.declined.add(f.declined as f64);
        self.no_space.add(f.no_space as f64);
        self.stair_users.add(f.stair_users as f64);
        if f.count > 0 {
            self.live += 1;
            self.wait.add(f.wait_mean);
            self.wait_p50.add(f.wait_samples.p50());
            self.p90.add(f.wait_samples.p90());
            self.wait_max.add(f.wait_max);
            self.best = self.best.min(f.wait_mean);
            self.worst = self.worst.max(f.wait_mean);
            // 门到门与等待同一次进出，live 口径一致
            self.d2d.add(f.d2d_mean);
            self.d2d_p50.add(f.d2d_samples.p50());
            self.d2d_p90.add(f.d2d_samples.p90());
            self.d2d_max.add(f.d2d_max);
            self.d2d_best = self.d2d_best.min(f.d2d_mean);
            self.d2d_worst = self.d2d_worst.max(f.d2d_mean);
        }
    }

    fn finish(&self) -> FloorAvg {
        let live = self.live > 0;
        FloorAvg {
            count: self.count.mean(),
            wait_mean: self.wait.mean(),
            wait_best: if live { self.best } else { 0.0 },
            wait_worst: if live { self.worst } else { 0.0 },
            live: self.live,
            wait_p50: self.wait_p50.mean(),
            wait_p90: self.p90.mean(),
            wait_max: self.wait_max.mean(),
            d2d_mean: self.d2d.mean(),
            d2d_best: if live { self.d2d_best } else { 0.0 },
            d2d_worst: if live { self.d2d_worst } else { 0.0 },
            d2d_p50: self.d2d_p50.mean(),
            d2d_p90: self.d2d_p90.mean(),
            d2d_max: self.d2d_max.mean(),
            bypass_total: self.bypass_total.mean(),
            declined: self.declined.mean(),
            no_space: self.no_space.mean(),
            stair_users: self.stair_users.mean(),
        }
    }
}

/// 标量的各次平均。
#[derive(Debug, Clone, Copy, Default)]
pub struct Scalars {
    pub pax_total: f64,
    pub pax_served: f64,
    pub pax_unfinished: f64,
    pub serve_ratio: f64,
    pub stair_users: f64,
    pub stair_share: f64,
    pub abandoned: f64,
    pub pax_press_and_run: f64,
    pub trips_expected: f64,
    pub trips: f64,
    pub rtt_mean: f64,
    pub rtt_dropped: f64,
    pub throughput_5min: f64,
    pub pax_with_bypass: f64,
    pub pax_any_bypass: f64,
    pub pax_after_stall: f64,
    pub stalls: f64,
    pub stall_seconds: f64,
    pub calls_orphaned: f64,
    pub ghost_stops: f64,
    pub partial_departures: f64,
    pub full_bypasses: f64,
    pub overshoots: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct ScalarAcc {
    pax_total: Acc,
    pax_served: Acc,
    pax_unfinished: Acc,
    serve_ratio: Acc,
    stair_users: Acc,
    stair_share: Acc,
    abandoned: Acc,
    pax_press_and_run: Acc,
    trips_expected: Acc,
    trips: Acc,
    rtt_mean: Acc,
    rtt_dropped: Acc,
    throughput_5min: Acc,
    pax_with_bypass: Acc,
    pax_any_bypass: Acc,
    pax_after_stall: Acc,
    stalls: Acc,
    stall_seconds: Acc,
    calls_orphaned: Acc,
    ghost_stops: Acc,
    partial_departures: Acc,
    full_bypasses: Acc,
    overshoots: Acc,
}

impl ScalarAcc {
    fn add(&mut self, m: &Metrics, expected_trips: usize, horizon_s: f64) {
        self.pax_total.add(m.pax_total as f64);
        self.pax_served.add(m.pax_served as f64);
        self.pax_unfinished.add(m.pax_unfinished as f64);
        self.serve_ratio.add(if m.pax_total == 0 {
            0.0
        } else {
            m.pax_served as f64 / m.pax_total as f64
        });
        self.stair_users.add(m.stair_users as f64);
        self.stair_share.add(m.stair_share);
        self.abandoned.add(m.abandoned as f64);
        self.pax_press_and_run.add(m.pax_press_and_run as f64);
        self.trips_expected.add(expected_trips as f64);
        self.trips.add(m.trips as f64);
        self.rtt_mean.add(m.rtt_mean);
        self.rtt_dropped.add(m.rtt_dropped as f64);
        self.throughput_5min.add(m.throughput_5min(horizon_s));
        self.pax_with_bypass.add(m.pax_with_bypass as f64);
        self.pax_any_bypass.add(m.pax_any_bypass);
        self.pax_after_stall.add(m.pax_after_stall as f64);
        self.stalls.add(m.stalls as f64);
        self.stall_seconds.add(m.stall_seconds);
        self.calls_orphaned.add(m.calls_orphaned as f64);
        self.ghost_stops.add(m.ghost_stops as f64);
        self.partial_departures.add(m.partial_departures as f64);
        self.full_bypasses.add(m.full_bypasses as f64);
        self.overshoots.add(m.overshoots as f64);
    }

    fn finish(&self) -> Scalars {
        Scalars {
            pax_total: self.pax_total.mean(),
            pax_served: self.pax_served.mean(),
            pax_unfinished: self.pax_unfinished.mean(),
            serve_ratio: self.serve_ratio.mean(),
            stair_users: self.stair_users.mean(),
            stair_share: self.stair_share.mean(),
            abandoned: self.abandoned.mean(),
            pax_press_and_run: self.pax_press_and_run.mean(),
            trips_expected: self.trips_expected.mean(),
            trips: self.trips.mean(),
            rtt_mean: self.rtt_mean.mean(),
            rtt_dropped: self.rtt_dropped.mean(),
            throughput_5min: self.throughput_5min.mean(),
            pax_with_bypass: self.pax_with_bypass.mean(),
            pax_any_bypass: self.pax_any_bypass.mean(),
            pax_after_stall: self.pax_after_stall.mean(),
            stalls: self.stalls.mean(),
            stall_seconds: self.stall_seconds.mean(),
            calls_orphaned: self.calls_orphaned.mean(),
            ghost_stops: self.ghost_stops.mean(),
            partial_departures: self.partial_departures.mean(),
            full_bypasses: self.full_bypasses.mean(),
            overshoots: self.overshoots.mean(),
        }
    }
}

/// 一段种子跑完之后的结果。
#[derive(Debug, Clone)]
pub struct SeedBatch {
    /// 一共跑了几个种子
    pub runs: usize,
    pub seed_from: u64,
    pub seed_to: u64,
    pub wait: NumAvg,
    pub ride: NumAvg,
    pub door_to_door: NumAvg,
    /// 逐层汇总，下标与 [`Metrics::by_floor`] 同一套编号，长度是 `n_levels`
    pub floors: Vec<FloorAvg>,
    /// 各次运行的等待时间直方图逐格相加
    pub wait_hist: Vec<u32>,
    /// 直方图的格宽，秒
    pub wait_hist_bin_s: f64,
    pub scalars: Scalars,
}

#[derive(Debug, Default)]
struct Accum {
    wait: NumAcc,
    ride: NumAcc,
    door_to_door: NumAcc,
    floors: Vec<FloorAcc>,
    wait_hist: Vec<u32>,
    wait_hist_bin_s: f64,
    scalars: ScalarAcc,
}

impl Accum {
    fn new(n_levels: usize) -> Self {
        Self {
            floors: vec![FloorAcc::default(); n_levels],
            ..Self::default()
        }
    }

    fn add(&mut self, m: &Metrics, expected_trips: usize, horizon_s: f64) {
        self.wait.add(&m.wait);
        self.ride.add(&m.ride);
        self.door_to_door.add(&m.door_to_door);
        for (i, f) in m.by_floor.iter().enumerate() {
            if let Some(a) = self.floors.get_mut(i) {
                a.add(f);
            }
        }
        let (bins, bin_s) = m.wait.histogram();
        if self.wait_hist.is_empty() {
            self.wait_hist = vec![0; bins.len()];
            self.wait_hist_bin_s = bin_s;
        }
        for (i, &c) in bins.iter().enumerate() {
            self.wait_hist[i] += c;
        }
        self.scalars.add(m, expected_trips, horizon_s);
    }

    fn finish(self, runs: usize, seed_from: u64, seed_to: u64) -> SeedBatch {
        SeedBatch {
            runs,
            seed_from,
            seed_to,
            wait: self.wait.finish(),
            ride: self.ride.finish(),
            door_to_door: self.door_to_door.finish(),
            floors: self.floors.iter().map(FloorAcc::finish).collect(),
            wait_hist: self.wait_hist,
            wait_hist_bin_s: self.wait_hist_bin_s,
            scalars: self.scalars.finish(),
        }
    }
}

/// 跑一段种子。范围取自 `cfg.sim.seed` 到 `cfg.sim.seed_to`，`seed_to` 缺席
/// 或者不大于根种子时只跑那一个种子。
///
/// 三个替换点照旧由调用方给。研究者换一条走向算法，只要传另一个 `Router`。
pub fn run_seeds<R, A, B>(cfg: &Config, router: &mut R, assigner: &mut A, boarding: &B) -> SeedBatch
where
    R: Router,
    A: Assigner,
    B: Boarding,
{
    let from: u64 = cfg.sim.seed;
    // 结束值不大于根种子时就当单次跑，与网页上填一样或更小是一回事
    let to: u64 = match cfg.sim.seed_to {
        Some(t) if t > from => t,
        _ => from,
    };
    let horizon_s = cfg.sim.horizon_s;
    let expected_trips = cfg.expected_trips();

    let mut acc = Accum::new(cfg.building.n_levels());
    for seed in from..=to {
        let mut c = cfg.clone();
        c.sim.seed = seed;
        // 别让批量标记漏进内核里的这一次单跑
        c.sim.seed_to = None;
        let run = sim::run(c, router, assigner, boarding);
        acc.add(&run.metrics, expected_trips, horizon_s);
    }
    acc.finish((to - from + 1) as usize, from, to)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin::{DefaultBoarding, DownPeakSweep, LoadAwareEta};
    use crate::config::preset;
    use crate::sim::Run;

    fn one(cfg: &Config, seed: u64) -> Run {
        let mut c = cfg.clone();
        c.sim.seed = seed;
        c.sim.seed_to = None;
        sim::run(
            c,
            &mut DownPeakSweep,
            &mut LoadAwareEta::default(),
            &DefaultBoarding::default(),
        )
    }

    /// 批量结果必须等于逐次单跑再平均。这是唯一的口径来源。
    #[test]
    fn batch_matches_averaging_single_runs() {
        let mut cfg = preset("std_residential").unwrap();
        cfg.sim.seed = 42;
        cfg.sim.seed_to = Some(44);
        let b = run_seeds(
            &cfg,
            &mut DownPeakSweep,
            &mut LoadAwareEta::default(),
            &DefaultBoarding::default(),
        );
        assert_eq!(b.runs, 3);
        assert_eq!(b.seed_from, 42);
        assert_eq!(b.seed_to, 44);

        let singles: Vec<_> = (42..=44).map(|s| one(&cfg, s).metrics).collect();
        let want_mean: f64 =
            singles.iter().map(|m| m.wait.mean()).sum::<f64>() / singles.len() as f64;
        assert!(
            (b.wait.mean - want_mean).abs() < 1e-9,
            "全局均值 {} 与逐次平均 {want_mean} 不符",
            b.wait.mean
        );

        // 逐层：均值取各次平均的平均，较好与较差取其中的最小与最大
        for (i, fa) in b.floors.iter().enumerate() {
            let live: Vec<f64> = singles
                .iter()
                .filter(|m| m.by_floor[i].count > 0)
                .map(|m| m.by_floor[i].wait_mean)
                .collect();
            assert_eq!(fa.live, live.len(), "第 {i} 格的有样本次数对不上");
            if live.is_empty() {
                continue;
            }
            let mean = live.iter().sum::<f64>() / live.len() as f64;
            assert!((fa.wait_mean - mean).abs() < 1e-9, "第 {i} 格均值不符");
            let best = live.iter().copied().fold(f64::INFINITY, f64::min);
            let worst = live.iter().copied().fold(0.0, f64::max);
            assert!((fa.wait_best - best).abs() < 1e-9, "第 {i} 格较好值不符");
            assert!((fa.wait_worst - worst).abs() < 1e-9, "第 {i} 格较差值不符");
            assert!(fa.wait_best <= fa.wait_mean + 1e-9 && fa.wait_mean <= fa.wait_worst + 1e-9);
            // 门到门走同一套口径
            let dlive: Vec<f64> = singles
                .iter()
                .filter(|m| m.by_floor[i].count > 0)
                .map(|m| m.by_floor[i].d2d_mean)
                .collect();
            assert_eq!(dlive.len(), live.len(), "第 {i} 格门到门的样本次数对不上");
            if !dlive.is_empty() {
                let dmean = dlive.iter().sum::<f64>() / dlive.len() as f64;
                assert!(
                    (fa.d2d_mean - dmean).abs() < 1e-9,
                    "第 {i} 格门到门均值不符"
                );
                let dbest = dlive.iter().copied().fold(f64::INFINITY, f64::min);
                let dworst = dlive.iter().copied().fold(0.0, f64::max);
                assert!(
                    (fa.d2d_best - dbest).abs() < 1e-9,
                    "第 {i} 格门到门较好值不符"
                );
                assert!(
                    (fa.d2d_worst - dworst).abs() < 1e-9,
                    "第 {i} 格门到门较差值不符"
                );
                assert!(fa.d2d_best <= fa.d2d_mean + 1e-9 && fa.d2d_mean <= fa.d2d_worst + 1e-9);
            }
        }

        // 分布图是各次相加
        let sum: u64 = b.wait_hist.iter().map(|&c| c as u64).sum();
        let want: u64 = singles.iter().map(|m| m.wait.count).sum();
        assert_eq!(sum, want, "分布图的样本总数应当等于各次样本数之和");
    }

    /// 一段种子跑两次，结果逐位一致。
    #[test]
    fn batch_is_reproducible() {
        let mut cfg = preset("std_residential").unwrap();
        cfg.sim.seed = 7;
        cfg.sim.seed_to = Some(10);
        let mk = || {
            run_seeds(
                &cfg,
                &mut DownPeakSweep,
                &mut LoadAwareEta::default(),
                &DefaultBoarding::default(),
            )
        };
        let a = mk();
        let b = mk();
        assert_eq!(a.runs, b.runs);
        assert_eq!(a.wait.mean.to_bits(), b.wait.mean.to_bits());
        assert_eq!(a.wait_hist, b.wait_hist);
        assert_eq!(a.floors.len(), b.floors.len());
        for (x, y) in a.floors.iter().zip(b.floors.iter()) {
            assert_eq!(x.wait_mean.to_bits(), y.wait_mean.to_bits());
            assert_eq!(x.wait_best.to_bits(), y.wait_best.to_bits());
        }
    }

    /// 没有范围时只跑一个种子，结果与单跑一致。
    #[test]
    fn single_seed_matches_one_run() {
        let cfg = preset("std_residential").unwrap();
        let b = run_seeds(
            &cfg,
            &mut DownPeakSweep,
            &mut LoadAwareEta::default(),
            &DefaultBoarding::default(),
        );
        assert_eq!(b.runs, 1);
        let m = one(&cfg, cfg.sim.seed).metrics;
        assert!((b.wait.mean - m.wait.mean()).abs() < 1e-9);
    }
}
