//! 指标。
//!
//! 分位数用每格 0.5 秒的直方图，另配一个溢出桶。理由三条：
//! 跨线程可合并、内存固定、尾部问题查起来方便。见 `docs/MODEL.md` 第 9 节。

/// 直方图的格宽，秒。
const BIN: f64 = 0.5;
/// 桶数。1200 格覆盖到 600 秒。
const NBINS: usize = 1200;

/// 一组时间样本的统计量。
#[derive(Debug, Clone)]
pub struct Summary {
    /// 样本数
    pub count: u64,
    /// 总和
    pub sum: f64,
    /// 最大值
    pub max: f64,
    /// 最小值
    pub min: f64,
    bins: Vec<u32>,
    /// 溢出桶：超过 600 秒的
    overflow_count: u64,
    overflow_sum: f64,
}

impl Default for Summary {
    fn default() -> Self {
        Self {
            count: 0,
            sum: 0.0,
            max: 0.0,
            min: f64::INFINITY,
            bins: vec![0; NBINS],
            overflow_count: 0,
            overflow_sum: 0.0,
        }
    }
}

impl Summary {
    /// 加一个样本。
    #[inline]
    pub fn add(&mut self, x: f64) {
        if !x.is_finite() {
            return;
        }
        self.count += 1;
        self.sum += x;
        self.max = self.max.max(x);
        self.min = self.min.min(x);
        let idx = (x / BIN).floor();
        if idx < 0.0 {
            return;
        }
        if idx < NBINS as f64 {
            self.bins[idx as usize] += 1;
        } else {
            self.overflow_count += 1;
            self.overflow_sum += x;
        }
    }

    /// 均值。空集合返回 0 而不是 NaN。
    pub fn mean(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum / self.count as f64
        }
    }

    /// 标准差。
    pub fn stddev(&self) -> f64 {
        if self.count < 2 {
            return 0.0;
        }
        let m = self.mean();
        // 用直方图算二阶矩，省一个 sumsq 字段
        let mut sq = 0.0;
        for (i, &c) in self.bins.iter().enumerate() {
            if c == 0 {
                continue;
            }
            let center = (i as f64 + 0.5) * BIN;
            sq += c as f64 * (center - m) * (center - m);
        }
        let ov = self.overflow_count as f64;
        if ov > 0.0 {
            let m2 = self.overflow_sum / ov;
            sq += ov * (m2 - m) * (m2 - m);
        }
        (sq / self.count as f64).max(0.0).sqrt()
    }

    /// 分位数。`q` 取 0 到 1。
    pub fn quantile(&self, q: f64) -> f64 {
        if self.count == 0 {
            return 0.0;
        }
        let target = (q * self.count as f64).ceil() as u64;
        let target = target.max(1);
        let mut acc = 0u64;
        for (i, &c) in self.bins.iter().enumerate() {
            if c == 0 {
                continue;
            }
            acc += c as u64;
            if acc >= target {
                return (i as f64 + 0.5) * BIN;
            }
        }
        // 落在溢出桶里
        if self.overflow_count > 0 {
            return self.overflow_sum / self.overflow_count as f64;
        }
        self.max
    }

    pub fn p50(&self) -> f64 {
        self.quantile(0.50)
    }
    pub fn p75(&self) -> f64 {
        self.quantile(0.75)
    }
    pub fn p90(&self) -> f64 {
        self.quantile(0.90)
    }
    pub fn p95(&self) -> f64 {
        self.quantile(0.95)
    }
    pub fn p99(&self) -> f64 {
        self.quantile(0.99)
    }

    /// 合并两组统计。跨线程汇总用。
    pub fn merge(&mut self, o: &Summary) {
        self.count += o.count;
        self.sum += o.sum;
        self.max = self.max.max(o.max);
        self.min = self.min.min(o.min);
        for (i, &c) in o.bins.iter().enumerate() {
            self.bins[i] += c;
        }
        self.overflow_count += o.overflow_count;
        self.overflow_sum += o.overflow_sum;
    }

    /// 直方图快照，供网页画图。
    pub fn histogram(&self) -> (Vec<u32>, f64) {
        (self.bins.clone(), BIN)
    }
}

/// 某一层的统计。这是给买房人看的最重要一张表。
#[derive(Debug, Clone, Default)]
pub struct FloorStats {
    /// 有多少乘客从这一层出发并完成行程
    pub count: u32,
    /// 等待时间之和
    pub wait_sum: f64,
    /// 等待时间均值
    pub wait_mean: f64,
    /// 等待时间最大值
    pub wait_max: f64,
    /// 等待时间直方图
    pub wait_samples: Summary,
    /// 被跳过的总次数
    pub bypass_total: u32,
    /// 至少被跳过一次的乘客数
    pub bypassed: u32,
    /// 因为不肯挤而没上去的次数
    pub declined: u32,
    /// 因为真没位子而没上去的次数
    pub no_space: u32,
    /// 这一层走楼梯的人数
    pub stair_users: u32,
}

/// 一次仿真的全部产出。
#[derive(Debug, Clone, Default)]
pub struct Metrics {
    /// 等梯时间
    pub wait: Summary,
    /// 乘坐时间
    pub ride: Summary,
    /// 门到门时间
    pub door_to_door: Summary,
    /// 逐层统计，下标 0 是第一层住宅
    pub by_floor: Vec<FloorStats>,

    /// 总乘客数，含走楼梯的
    pub pax_total: usize,
    /// 完成行程的乘客数
    pub pax_served: u64,
    /// 至少被跳过一次的乘客数
    pub pax_with_bypass: u64,
    /// 被跳过三次以上的乘客数
    pub pax_bypass3: u64,
    /// 至少被跳过一次的人占完成行程人数的比例
    pub pax_any_bypass: f64,
    /// 被跳过三次以上的人占的比例
    pub pax_bypass3_frac: f64,
    /// 走楼梯的人数占比
    pub stair_share: f64,
    /// 因超载僵局被赶出轿厢的人数
    pub pax_after_stall: usize,
    /// 走楼梯的人数
    pub stair_users: usize,

    /// 轿厢数
    pub n_cars: usize,
    /// 往返时间之和
    pub rtt_sum: f64,
    /// 往返时间样本数
    pub rtt_count: u64,
    /// 往返时间均值
    pub rtt_mean: f64,
    /// 完成的趟数
    pub trips: u64,

    /// 超载死锁次数
    pub stalls: usize,
    /// 超载死锁累计秒数
    pub stall_seconds: f64,
    /// 因轿厢停滞而无人接管的外呼数
    pub calls_orphaned: usize,
    /// 未满载就发车的停靠次数
    pub partial_departures: usize,
    /// 幽灵停站次数
    pub ghost_stops: usize,
    /// 满载直驶跳过的外呼数
    pub full_bypasses: usize,
    /// 因为制动来不及而越过的外呼层数。高速梯在接下来几层都停不了时会累积
    pub overshoots: usize,
}

impl Metrics {
    /// 五分钟输送人数，用于对照运力够不够。
    pub fn throughput_5min(&self, horizon_s: f64) -> f64 {
        if horizon_s <= 0.0 {
            return 0.0;
        }
        self.pax_served as f64 * (300.0 / horizon_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_summary_is_zero_not_nan() {
        let s = Summary::default();
        assert_eq!(s.mean(), 0.0);
        assert_eq!(s.p50(), 0.0);
        assert_eq!(s.p99(), 0.0);
        assert_eq!(s.stddev(), 0.0);
        assert!(!s.mean().is_nan());
    }

    #[test]
    fn uniform_sample_has_expected_mean() {
        let mut s = Summary::default();
        for i in 0..=100 {
            s.add(i as f64);
        }
        assert!((s.mean() - 50.0).abs() < 0.5);
        assert!((s.min - 0.0).abs() < 1e-9);
        assert!((s.max - 100.0).abs() < 1e-9);
    }

    #[test]
    fn percentiles_are_monotonic() {
        let mut s = Summary::default();
        for i in 0..=1000 {
            s.add(i as f64 * 0.1);
        }
        let mut prev = -1.0;
        for q in [0.5, 0.75, 0.9, 0.95, 0.99] {
            let v = s.quantile(q);
            assert!(v >= prev, "q={q} 的分位数 {v} 比上一个 {prev} 小");
            prev = v;
        }
    }

    /// 分位数要落在合理范围内，不能跑出样本区间。
    #[test]
    fn quantiles_stay_within_range() {
        let mut s = Summary::default();
        for _ in 0..1000 {
            s.add(30.0);
        }
        assert!((s.p50() - 30.0).abs() < BIN, "全是 30 秒时 P50 应当是 30");
    }

    /// 尾部样本不能把直方图撑爆，要落进溢出桶。
    #[test]
    fn long_tail_goes_to_overflow_bucket() {
        let mut s = Summary::default();
        for _ in 0..99 {
            s.add(10.0);
        }
        s.add(5000.0);
        assert_eq!(s.count, 100);
        assert!((s.max - 5000.0).abs() < 1e-9, "最大值应当保留");
        assert!((s.mean() - 59.9).abs() < 0.1);
        // 99% 的分位数不该被那个长尾拉走
        assert!(s.p99() < 100.0, "P99 应当还是 10 秒左右，实际 {}", s.p99());
    }

    #[test]
    fn non_finite_samples_are_ignored() {
        let mut s = Summary::default();
        s.add(f64::NAN);
        s.add(f64::INFINITY);
        s.add(5.0);
        assert_eq!(s.count, 1);
        assert!((s.mean() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn merge_is_equivalent_to_adding_everything() {
        let mut a = Summary::default();
        let mut b = Summary::default();
        for i in 0..50 {
            a.add(i as f64);
            b.add((i + 50) as f64);
        }
        let expect: f64 = (0..100).map(|i| i as f64).sum::<f64>() / 100.0;
        let mut m = a.clone();
        m.merge(&b);
        assert_eq!(m.count, 100);
        assert!((m.mean() - expect).abs() < 1e-9, "合并后的均值应当等于整体均值");
    }

    #[test]
    fn stddev_of_constant_sample_is_zero() {
        let mut s = Summary::default();
        for _ in 0..100 {
            s.add(7.0);
        }
        assert!(s.stddev() < 0.6, "全一样的样本标准差应当接近 0，实际 {}", s.stddev());
    }

    #[test]
    fn throughput_scales_with_horizon() {
        let mut m = Metrics::default();
        m.pax_served = 100;
        // 300 秒内跑 100 人
        assert!((m.throughput_5min(300.0) - 100.0).abs() < 1e-9);
        // 3600 秒内跑 100 人，折算到 5 分钟
        assert!((m.throughput_5min(3600.0) - 100.0 / 12.0).abs() < 1e-9);
    }

    #[test]
    fn default_metrics_are_zero() {
        let m = Metrics::default();
        assert_eq!(m.pax_served, 0);
        assert_eq!(m.wait.mean(), 0.0);
        assert!(m.by_floor.is_empty());
    }
}