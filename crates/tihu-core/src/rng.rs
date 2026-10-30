//! 确定性随机数。
//!
//! 手写 PCG32，纯整数运算，跨平台逐位一致。不依赖任何外部库，
//! 因为外部库的默认发生器可能随版本变化，那样基准和回归测试就失去意义。
//!
//! 分流规则见 `docs/ARCHITECTURE.md` 第 5 节。凡是描述乘客自身属性的随机量，
//! 全部在策略介入之前抽完，这样同一个种子下无论配哪个算法，楼里都是同一批人。

/// PCG32 发生器。`XSH RR 64/32` 变体。
#[derive(Debug, Clone)]
pub struct Pcg32 {
    state: u64,
    inc: u64,
}

const MULT: u64 = 6364136223846793005;

impl Pcg32 {
    /// 用一个 64 位种子初始化。
    pub fn new(seed: u64) -> Self {
        let mut g = Self {
            state: 0,
            inc: (seed << 1) | 1,
        };
        g.step();
        g.state = g.state.wrapping_add(seed);
        g.step();
        g
    }

    #[inline]
    fn step(&mut self) {
        self.state = self.state.wrapping_mul(MULT).wrapping_add(self.inc);
    }

    /// 取一个 32 位随机数。
    #[inline]
    pub fn next_u32(&mut self) -> u32 {
        let old = self.state;
        self.step();
        let xorshifted = (((old >> 18) ^ old) >> 27) as u32;
        let rot = (old >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    /// 取一个 64 位随机数。
    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        ((self.next_u32() as u64) << 32) | self.next_u32() as u64
    }

    /// 取一个 0 到 1 之间的 `f64`，取值落在 24 位尾数能精确表示的格点上。
    #[inline]
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u32() >> 8) as f64 * (1.0 / 16777216.0)
    }

    /// 取一个 `[lo, hi)` 区间内的 `f64`。
    #[inline]
    pub fn range_f64(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.next_f64() * (hi - lo)
    }

    /// 取一个 `[0, n)` 区间内的整数。`n` 为 0 时返回 0。
    #[inline]
    pub fn below(&mut self, n: u32) -> u32 {
        if n == 0 {
            0
        } else {
            // 无模偏，做拒绝采样。n 小的时候最多重试几次。
            let threshold = n.wrapping_neg() % n;
            loop {
                let r = self.next_u32();
                if r >= threshold {
                    return r % n;
                }
            }
        }
    }

    /// 以概率 `p` 返回 true。
    #[inline]
    pub fn chance(&mut self, p: f64) -> bool {
        self.next_f64() < p
    }

    /// 标准正态分布，Box-Muller 变换。
    pub fn normal(&mut self) -> f64 {
        loop {
            let u1 = self.next_f64();
            if u1 > f64::MIN_POSITIVE {
                let u2 = self.next_f64();
                return (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos();
            }
        }
    }

    /// 对数正态分布，中位数 `median`，变异系数 `cv`。
    pub fn lognormal(&mut self, median: f64, cv: f64) -> f64 {
        let sigma = (1.0 + cv * cv).ln().sqrt();
        (median.ln() + sigma * self.normal()).exp()
    }

    /// Beta(a, b) 分布，用两个 Gamma 相除。
    pub fn beta(&mut self, a: f64, b: f64) -> f64 {
        let x = self.gamma(a);
        let y = self.gamma(b);
        if x + y <= 0.0 {
            return 0.5;
        }
        (x / (x + y)).clamp(0.0, 1.0)
    }

    /// Marsaglia-Tsang 的 Gamma 采样，形状参数大于 1。
    fn gamma(&mut self, shape: f64) -> f64 {
        if shape < 1.0 {
            let u = self.next_f64().max(f64::MIN_POSITIVE);
            return self.gamma(shape + 1.0) * u.powf(1.0 / shape);
        }
        let d = shape - 1.0 / 3.0;
        let c = (9.0 * d).sqrt();
        loop {
            let x = self.normal();
            let v = 1.0 + c * x;
            if v <= 0.0 {
                continue;
            }
            let v3 = v * v * v;
            let u = self.next_f64();
            if u < 1.0 - 0.0331 * x.powi(4) {
                return d * v3;
            }
            if u.ln() < 0.5 * x * x + d * (1.0 - v3 + v3.ln()) {
                return d * v3;
            }
        }
    }
}

/// `splitmix64`，用来从一个根种子派生出互不干扰的多条流。
#[inline]
pub fn splitmix64(seed: u64, stream: u64) -> u64 {
    let mut z = seed
        .wrapping_add(0x9E3779B97F4A7C15)
        .wrapping_add(stream.wrapping_mul(0xBF58476D1CE4E5B9));
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}

/// 随机流的编号。凡是新增一条流，在这里加一个。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stream {
    /// 乘客到达电梯厅的时刻
    Arrival = 0,
    /// 每位乘客的目的层
    Destination = 1,
    /// 每位乘客的拒挤度
    Tolerance = 2,
    /// 每位乘客的体重
    Weight = 3,
    /// 每位乘客采用哪种按键策略
    Strategy = 4,
    /// 每次停靠时谁愿意挤
    Squeeze = 5,
    /// 开关门与逗留时间的抖动
    Timing = 6,
    /// 客流强度的非齐次泊松抽签
    Demand = 7,
    /// 超载僵持时谁愿意退出
    Stall = 8,
    /// 每位乘客愿意等多久才改走楼梯
    Patience = 9,
    /// 谁乱按内选、谁按了就跑，以及按哪几层
    Malice = 10,
}

impl Stream {
    /// 全部流，用于自检。
    pub const ALL: [Stream; 11] = [
        Stream::Arrival,
        Stream::Destination,
        Stream::Tolerance,
        Stream::Weight,
        Stream::Strategy,
        Stream::Squeeze,
        Stream::Timing,
        Stream::Demand,
        Stream::Stall,
        Stream::Patience,
        Stream::Malice,
    ];
}

/// 一组按用途分开的随机流。
///
/// 分流的意义：改动策略模块时不能把客流序列挪位，否则同一个种子的
/// 算法对比就失去意义。详见 `docs/ARCHITECTURE.md` 第 5 节。
#[derive(Debug, Clone)]
pub struct Streams {
    gens: Vec<Pcg32>,
}

impl Streams {
    /// 从根种子派生出全部流。
    pub fn new(root: u64) -> Self {
        Self {
            gens: Stream::ALL
                .iter()
                .map(|&s| Pcg32::new(splitmix64(root, s as u64)))
                .collect(),
        }
    }

    #[inline]
    fn idx(s: Stream) -> usize {
        s as usize
    }

    /// 拿到某一条流。调用方拿到的就是可变引用，可以随便抽。
    #[inline]
    pub fn get(&mut self, s: Stream) -> &mut Pcg32 {
        &mut self.gens[Self::idx(s)]
    }

    /// 拿一条流并立刻抽一个 `f64`。
    #[inline]
    pub fn f64(&mut self, s: Stream) -> f64 {
        self.gens[Self::idx(s)].next_f64()
    }

    /// 拿一条流并立刻抽一个整数。
    #[inline]
    pub fn below(&mut self, s: Stream, n: u32) -> u32 {
        self.gens[Self::idx(s)].below(n)
    }

    /// 拿一条流并立刻做一次概率判定。
    #[inline]
    pub fn chance(&mut self, s: Stream, p: f64) -> bool {
        self.gens[Self::idx(s)].chance(p)
    }

    /// 抽一个 `[lo, hi)` 区间内的 `f64`。
    #[inline]
    pub fn range(&mut self, s: Stream, lo: f64, hi: f64) -> f64 {
        self.gens[Self::idx(s)].range_f64(lo, hi)
    }

    /// 抽一个拒挤度。
    #[inline]
    pub fn tolerance(&mut self) -> f64 {
        self.gens[Self::idx(Stream::Tolerance)].beta(2.0, 2.0)
    }

    /// 抽一个对数正态分布的时长。
    #[inline]
    pub fn lognormal(&mut self, median: f64, cv: f64) -> f64 {
        self.gens[Self::idx(Stream::Patience)].lognormal(median, cv)
    }

    /// 从多分量体重分布里抽一个体重，公斤。
    ///
    /// 选分量与抽具体数值都走体重流，不新开流。`force_child` 为真时锁定小孩那一峰。
    #[inline]
    pub fn body_weight(&mut self, mix: &crate::config::WeightMix, force_child: bool) -> f64 {
        let g = &mut self.gens[Self::idx(Stream::Weight)];
        let (median, cv) = mix.pick(g.next_f64(), force_child);
        g.lognormal(median, cv.max(0.01)) + mix.carried_kg.max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_sequence() {
        let mut a = Pcg32::new(42);
        let mut b = Pcg32::new(42);
        for _ in 0..1000 {
            assert_eq!(a.next_u32(), b.next_u32());
        }
    }

    #[test]
    fn different_seed_diverges() {
        let mut a = Pcg32::new(1);
        let mut b = Pcg32::new(2);
        let va: Vec<u32> = (0..100).map(|_| a.next_u32()).collect();
        let vb: Vec<u32> = (0..100).map(|_| b.next_u32()).collect();
        assert_ne!(va, vb);
    }

    #[test]
    fn f64_in_unit_interval() {
        let mut g = Pcg32::new(7);
        for _ in 0..10_000 {
            let x = g.next_f64();
            assert!((0.0..1.0).contains(&x), "越界 {x}");
        }
    }

    #[test]
    fn below_respects_bound() {
        let mut g = Pcg32::new(9);
        for n in 1..12u32 {
            for _ in 0..500 {
                assert!(g.below(n) < n);
            }
        }
        assert_eq!(g.below(0), 0);
    }

    #[test]
    fn below_covers_whole_range() {
        let mut g = Pcg32::new(11);
        let mut seen = [false; 6];
        for _ in 0..2000 {
            seen[g.below(6) as usize] = true;
        }
        assert!(seen.iter().all(|&b| b), "6 个取值应当都出现过");
    }

    /// 分流之间不能互相干扰，这是算法对比的前提。
    #[test]
    fn streams_are_independent() {
        let mut s1 = Streams::new(2024);
        let mut s2 = Streams::new(2024);
        let a: Vec<f64> = (0..50).map(|_| s1.f64(Stream::Arrival)).collect();
        let b: Vec<f64> = (0..50).map(|_| s2.f64(Stream::Destination)).collect();
        assert_ne!(a, b, "两条流抽出了同样的序列");
    }

    /// 多抽一条流不应影响其他流。这是最容易被后续改动破坏的性质。
    #[test]
    fn extra_draws_do_not_shift_other_streams() {
        let mut base = Streams::new(555);
        let expect: Vec<f64> = (0..20).map(|_| base.f64(Stream::Arrival)).collect();

        let mut perturbed = Streams::new(555);
        for _ in 0..1000 {
            perturbed.f64(Stream::Squeeze);
            perturbed.f64(Stream::Stall);
        }
        let got: Vec<f64> = (0..20).map(|_| perturbed.f64(Stream::Arrival)).collect();
        assert_eq!(expect, got, "抽别的流把到达流挪位了");
    }

    #[test]
    fn splitmix_is_stable() {
        // 这个值一旦变了，说明基准与黄金向量全部作废
        assert_eq!(splitmix64(0, 0), 0xE220A8397B1DCDAF);
        assert_ne!(splitmix64(1, 0), splitmix64(0, 1));
    }

    #[test]
    fn normal_has_right_moments() {
        let mut g = Pcg32::new(31);
        let n = 200_000;
        let mut sum = 0.0;
        let mut sq = 0.0;
        for _ in 0..n {
            let x = g.normal();
            sum += x;
            sq += x * x;
        }
        let mean = sum / n as f64;
        let var = sq / n as f64 - mean * mean;
        assert!(mean.abs() < 0.02, "均值 {mean} 偏离太多");
        assert!((var - 1.0).abs() < 0.03, "方差 {var} 偏离太多");
    }

    #[test]
    fn beta_stays_in_unit_interval() {
        let mut g = Pcg32::new(77);
        for _ in 0..10_000 {
            let x = g.beta(2.0, 2.0);
            assert!((0.0..=1.0).contains(&x));
        }
    }

    #[test]
    fn beta_mean_is_as_specified() {
        let mut g = Pcg32::new(78);
        let n = 200_000;
        let mut sum = 0.0;
        for _ in 0..n {
            sum += g.beta(2.0, 2.0);
        }
        let mean = sum / n as f64;
        assert!(
            (mean - 0.5).abs() < 0.01,
            "Beta(2,2) 均值应为 0.5，实际 {mean}"
        );
    }

    #[test]
    fn lognormal_median_and_spread() {
        let mut g = Pcg32::new(88);
        let n = 100_000;
        let mut sum = 0.0;
        let mut sq = 0.0;
        for _ in 0..n {
            let w = g.lognormal(64.0, 0.18);
            sum += w;
            sq += w * w;
        }
        let mean = sum / n as f64;
        let var = sq / n as f64 - mean * mean;
        assert!(
            (mean - 65.1).abs() < 0.5,
            "体重的均值应在 65 kg 附近，实际 {mean}"
        );
        assert!(
            (var.sqrt() - 11.7).abs() < 1.5,
            "标准差应在 12 kg 附近，实际 {}",
            var.sqrt()
        );
    }

    /// 卡方检验：把 `next_u32` 切成 16 个桶，样本要均匀。
    ///
    /// 16 个桶、卡方统计量在自由度 15 下的 99.9% 临界值是 37.7。
    #[test]
    fn next_u32_passes_chi_square() {
        let buckets = 16u32;
        let per_bucket = 20_000u32;
        let mut counts = vec![0u32; buckets as usize];
        let mut g = Pcg32::new(0xBADC0FFE);
        for _ in 0..buckets * per_bucket {
            let v = g.next_u32();
            counts[(v >> 28) as usize] += 1;
        }
        let expect = per_bucket as f64;
        let chi: f64 = counts
            .iter()
            .map(|&c| {
                let d = c as f64 - expect;
                d * d / expect
            })
            .sum();
        assert!(chi < 37.7, "卡方统计量 {chi} 偏大，低位分布可疑");
    }

    /// 取模采样 `below` 也要均匀，且不因拒绝采样偏掉。
    #[test]
    fn below_is_uniform_for_awkward_moduli() {
        for n in [3u32, 7, 10, 100] {
            let buckets = n;
            let rounds = 20_000u32;
            let mut counts = vec![0u32; buckets as usize];
            let mut g = Pcg32::new(0x1234_5678 ^ n as u64);
            for _ in 0..buckets * rounds {
                counts[g.below(n) as usize] += 1;
            }
            let expect = rounds as f64;
            let chi: f64 = counts
                .iter()
                .map(|&c| {
                    let d = c as f64 - expect;
                    d * d / expect
                })
                .sum();
            // 自由度 n-1。n=100 时 99.9% 临界值约 148
            let limit = (n as f64) * 1.5 + 12.0;
            assert!(chi < limit, "n={n} 的卡方统计量 {chi} 偏大");
        }
    }

    /// 相邻两次抽签不能有明显相关。相关系数应接近 0。
    #[test]
    fn consecutive_draws_are_uncorrelated() {
        let mut g = Pcg32::new(0xC0FFEE);
        let n = 200_000;
        let mut xs = Vec::with_capacity(n);
        let mut ys = Vec::with_capacity(n);
        let mut prev = g.next_f64();
        for _ in 1..n {
            let cur = g.next_f64();
            xs.push(prev);
            ys.push(cur);
            prev = cur;
        }
        let mean = 0.5;
        let mut cov = 0.0;
        let mut vx = 0.0;
        let mut vy = 0.0;
        for (x, y) in xs.iter().zip(ys.iter()) {
            cov += (x - mean) * (y - mean);
            vx += (x - mean) * (x - mean);
            vy += (y - mean) * (y - mean);
        }
        let r = cov / (vx.sqrt() * vy.sqrt());
        assert!(r.abs() < 0.01, "相邻序列相关系数 {r} 偏大");
    }

    /// 0 与 1 的位数要均衡。偏离一半太多说明发生器有结构性问题。
    #[test]
    fn bits_are_balanced() {
        let mut g = Pcg32::new(0xDEADBEEF);
        let n = 100_000u64;
        let mut ones = 0u64;
        for _ in 0..n {
            ones += g.next_u32().count_ones() as u64;
        }
        let ratio = ones as f64 / (n * 32) as f64;
        assert!((ratio - 0.5).abs() < 0.005, "1 的占比 {ratio} 偏离一半太多");
    }

    /// 根种子派生出的各条流不能撞在同一串数字上。
    #[test]
    fn derived_streams_do_not_collide() {
        let mut seen = std::collections::HashSet::new();
        for root in 0..200u64 {
            for s in Stream::ALL {
                let seed = splitmix64(root, s as u64);
                assert!(
                    seen.insert(seed),
                    "root={root} stream={s:?} 的派生种子撞车了"
                );
            }
        }
    }

    /// 派生种子本身也要过均匀性检查，不能因为加了个常数就退化。
    #[test]
    fn splitmix_output_is_well_spread() {
        let mut counts = [0u32; 16];
        let n = 16u32 * 4000;
        for i in 0..n {
            let v = splitmix64(i as u64, 3);
            counts[(v >> 60) as usize] += 1;
        }
        let expect = 4000.0;
        let chi: f64 = counts
            .iter()
            .map(|&c| {
                let d = c as f64 - expect;
                d * d / expect
            })
            .sum();
        assert!(chi < 37.7, "splitmix64 高位分布可疑，卡方 {chi}");
    }
}
