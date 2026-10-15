//! 轿厢运动学。
//!
//! 上下行各带一套参数，见 `docs/MODEL.md` 第 3 节。核心是一条对称 S 曲线
//! 速度剖面，加减速段各自由三个量决定。
//!
//! 全部公式在 `docs/MODEL.md` 第 3.2 节展开，本文件的注释与之一一对应。

/// 单个方向的运动曲线参数。
///
/// `v_max` 是该方向的极限速度。`a_acc` 与 `a_dec` 可以不同，`j_acc` 与
/// `j_dec` 也可以不同，这十个量都能单独设置。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionProfile {
    /// 该方向的极限速度，单位 m/s
    pub v_max: f32,
    /// 起步加速度，单位 m/s^2
    pub a_acc: f32,
    /// 停车减速度，单位 m/s^2
    pub a_dec: f32,
    /// 起步加加速度，单位 m/s^3
    pub j_acc: f32,
    /// 停车加加速度，单位 m/s^3
    pub j_dec: f32,
}

impl Default for MotionProfile {
    /// 住宅主流配置，待标定。见 `docs/MODEL.md` 第 3.1 节的表。
    fn default() -> Self {
        Self { v_max: 1.75, a_acc: 0.80, a_dec: 0.80, j_acc: 1.00, j_dec: 1.00 }
    }
}

impl MotionProfile {
    /// 住宅常见的速度档位预设。加速度与加加速度一律待标定。
    ///
    /// 表中的时间数字由 [`traverse_time`] 算出，可重跑 `cargo bench --bench kinematics` 复现。
    pub fn preset(name: &str) -> Option<Self> {
        let p = match name {
            "low_rise_1_0" => (1.00, 0.50, 0.60),
            "std_1_6" => (1.60, 0.65, 0.80),
            "std_1_75" => (1.75, 0.80, 1.00),
            "high_2_5" => (2.50, 1.00, 1.20),
            "high_3_0" => (3.00, 1.00, 1.30),
            _ => return None,
        };
        Some(Self {
            v_max: p.0,
            a_acc: p.1,
            a_dec: p.1,
            j_acc: p.2,
            j_dec: p.2,
        })
    }

    /// 全部预设的名字，供网页下拉框与命令行共用。
    pub fn preset_names() -> &'static [&'static str] {
        &["low_rise_1_0", "std_1_6", "std_1_75", "high_2_5", "high_3_0"]
    }

    /// 把参数夹到物理上说得过去的范围，防止除零与负数。
    pub fn sanitized(mut self) -> Self {
        self.v_max = self.v_max.clamp(0.1, 20.0);
        self.a_acc = self.a_acc.clamp(0.05, 5.0);
        self.a_dec = self.a_dec.clamp(0.05, 5.0);
        self.j_acc = self.j_acc.clamp(0.05, 20.0);
        self.j_dec = self.j_dec.clamp(0.05, 20.0);
        self
    }
}

/// 走一段距离的结果。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Traverse {
    /// 耗时，秒
    pub time: f64,
    /// 实际达到的峰值速度，m/s
    pub peak_speed: f32,
    /// 是否真的跑满了 `v_max`
    pub reached_rated: bool,
}

/// 加速到速度 `v` 所需的时间与位移。
#[inline]
fn accel_to(v: f32, a: f32, j: f32) -> (f64, f64) {
    let t = v as f64 / a as f64 + a as f64 / j as f64;
    let d = v as f64 * v as f64 / (2.0 * a as f64) + v as f64 * a as f64 / (2.0 * j as f64);
    (t, d)
}

/// 从速度 `v` 减速停车的耗时与位移。
#[inline]
fn decel_from(v: f32, a: f32, j: f32) -> (f64, f64) {
    (v as f64 / a as f64 + a as f64 / j as f64, v as f64 * v as f64 / (2.0 * a as f64) + v as f64 * a as f64 / (2.0 * j as f64))
}

/// 走 `dist` 米要多久。
///
/// 对应 `docs/MODEL.md` 第 3.2 节。第一问是行程够不够长到跑满 `v_max`，
/// 够就三段走，不够就解一个二次方程求峰值速度。
pub fn traverse_time(dist: f64, p: MotionProfile) -> Traverse {
    let p = p.sanitized();
    if dist <= 0.0 {
        return Traverse { time: 0.0, peak_speed: 0.0, reached_rated: false };
    }
    let (t_acc, d_acc) = accel_to(p.v_max, p.a_acc, p.j_acc);
    let (t_dec, d_dec) = decel_from(p.v_max, p.a_dec, p.j_dec);

    if dist >= d_acc + d_dec {
        let cruise = (dist - d_acc - d_dec) / p.v_max as f64;
        return Traverse { time: t_acc + cruise + t_dec, peak_speed: p.v_max, reached_rated: true };
    }

    // 短行程。从 d_acc(v_p) + d_dec(v_p) = dist 解出峰值速度。
    let aa = 1.0 / (2.0 * p.a_acc as f64) + 1.0 / (2.0 * p.a_dec as f64);
    let bb = p.a_acc as f64 / (2.0 * p.j_acc as f64) + p.a_dec as f64 / (2.0 * p.j_dec as f64);
    let disc = bb * bb + 4.0 * aa * dist;
    let v_p = ((-bb + disc.sqrt()) / (2.0 * aa)) as f32;
    let (t1, _) = accel_to(v_p, p.a_acc, p.j_acc);
    let (t2, _) = decel_from(v_p, p.a_dec, p.j_dec);
    Traverse { time: t1 + t2, peak_speed: v_p, reached_rated: false }
}

/// 一次停靠里「运行时间之外」的额外开销。
///
/// 启停的加减速损耗不在这里算，它已经包含在 S 曲线里了。见 `docs/MODEL.md` 第 3.6 节。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DoorTiming {
    /// 开门耗时
    pub open_s: f32,
    /// 平层前预开门带来的补偿，从开门时间里扣掉
    pub early_open_credit_s: f32,
    /// 没人上下时的保持开启时间
    pub dwell_s: f32,
    /// 关门耗时，含光幕确认
    pub close_s: f32,
    /// 控制器响应
    pub ctrl_resp_s: f32,
    /// 每人进梯耗时
    pub pax_in_s: f32,
    /// 每人出梯耗时
    pub pax_out_s: f32,
    /// 同时上下客时的重叠折扣，取值 0 到 1
    pub overlap: f32,
}

impl Default for DoorTiming {
    /// 全部待标定。默认值合计 6.0 秒的一次停靠开销。
    fn default() -> Self {
        Self {
            open_s: 2.0,
            early_open_credit_s: 0.5,
            dwell_s: 1.5,
            close_s: 2.8,
            ctrl_resp_s: 0.2,
            pax_in_s: 1.0,
            pax_out_s: 0.8,
            overlap: 0.35,
        }
    }
}

impl DoorTiming {
    /// 空停一次站的开销。
    pub fn stop_overhead(&self) -> f64 {
        (self.open_s - self.early_open_credit_s).max(0.0) as f64
            + self.dwell_s as f64
            + self.close_s as f64
            + self.ctrl_resp_s as f64
    }

    /// 一次停靠里上下客所花的时间。
    ///
    /// 进出可以并行，所以取较大者，再加上重叠折扣乘较小者。
    pub fn transfer_time(&self, n_in: usize, n_out: usize) -> f64 {
        let t_in = n_in as f64 * self.pax_in_s as f64;
        let t_out = n_out as f64 * self.pax_out_s as f64;
        if n_in == 0 || n_out == 0 {
            t_in + t_out
        } else {
            t_in.max(t_out) + self.overlap as f64 * t_in.min(t_out)
        }
    }
}

/// 按行驶方向取对应的运动曲线。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    /// 上行
    Up,
    /// 下行
    Down,
    /// 停在原地，没有方向
    None,
}

impl Dir {
    /// 另一个方向。
    pub fn opposite(self) -> Dir {
        match self {
            Dir::Up => Dir::Down,
            Dir::Down => Dir::Up,
            Dir::None => Dir::None,
        }
    }

    /// 是不是真正的移动方向。
    pub fn is_moving(self) -> bool {
        !matches!(self, Dir::None)
    }

    /// 按方向取上行或下行的曲线。
    pub fn profile<'a>(&self, up: &'a MotionProfile, down: &'a MotionProfile) -> &'a MotionProfile {
        match self {
            Dir::Up => up,
            _ => down,
        }
    }

    /// 供网页下拉框使用的名字。
    pub fn as_str(self) -> &'static str {
        match self {
            Dir::Up => "up",
            Dir::Down => "down",
            Dir::None => "none",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `docs/MODEL.md` 第 3.3 节的展开算式，逐项对照。
    #[test]
    fn single_floor_matches_doc_arithmetic() {
        let p = MotionProfile::default();
        let t = traverse_time(3.0, p).time;
        assert!((t - 4.75).abs() < 0.01, "单层应为 4.75 秒，实际 {t}");

        // 峰值速度解二次方程，doc 里给的是 1.262
        let tr = traverse_time(3.0, p);
        assert!((tr.peak_speed - 1.262).abs() < 0.002, "峰值应为 1.262，实际 {}", tr.peak_speed);
        assert!(!tr.reached_rated, "单层跑不满额定");
    }

    /// 上下对称时退化简单形式 t = L/v + v/a + a/j。
    #[test]
    fn symmetric_reduces_to_closed_form() {
        let p = MotionProfile::default();
        for &d in &[10.0f64, 30.0, 90.0, 120.0] {
            let tr = traverse_time(d, p);
            assert!(tr.reached_rated, "{d} 米应当跑满额定");
            let closed = d / p.v_max as f64
                + p.v_max as f64 / p.a_acc as f64
                + p.a_acc as f64 / p.j_acc as f64;
            assert!((tr.time - closed).abs() < 1e-9, "{d} 米处退化关系不成立");
        }
    }

    /// `docs/MODEL.md` 第 3.5 节预设表里的五个数字。
    #[test]
    fn preset_table_matches_doc() {
        let cases: [(&str, f64, f64); 5] = [
            ("low_rise_1_0", 5.83, 92.83),
            ("std_1_6", 5.19, 59.52),
            ("std_1_75", 4.75, 54.42),
            ("high_2_5", 4.40, 39.33),
            ("high_3_0", 4.32, 33.77),
        ];
        for (name, one_floor, thirty) in cases {
            let p = MotionProfile::preset(name).unwrap();
            let a = traverse_time(3.0, p).time;
            let b = traverse_time(90.0, p).time;
            assert!((a - one_floor).abs() < 0.01, "{name} 单层应为 {one_floor}，实际 {a}");
            assert!((b - thirty).abs() < 0.01, "{name} 跑 30 层应为 {thirty}，实际 {b}");
        }
    }

    /// 第 3.4 节说层高 3.0 m 时额定超过 1.262 m/s 后单层时间不再下降。
    #[test]
    fn short_run_saturates_above_threshold() {
        let slow = traverse_time(3.0, MotionProfile::preset("std_1_75").unwrap()).time;
        for name in ["high_2_5", "high_3_0"] {
            // 换用更高额定但保持同一套 a 与 j，峰值速度只由行程决定
            let p = MotionProfile { v_max: MotionProfile::preset(name).unwrap().v_max, ..MotionProfile::default() };
            assert!((traverse_time(3.0, p).time - slow).abs() < 1e-9, "{name} 单层应当与 std_1_75 相同");
        }
    }

    /// 上下不对称时下行会慢下来。`docs/MODEL.md` 第 3.5 节给的是 39.93 对 48.30。
    #[test]
    fn asymmetric_up_down_differs() {
        let up = MotionProfile { v_max: 2.5, ..MotionProfile::default() };
        let down = MotionProfile { v_max: 2.0, ..MotionProfile::default() };
        let a = traverse_time(90.0, up).time;
        let b = traverse_time(90.0, down).time;
        assert!((a - 39.93).abs() < 0.01, "上行应为 39.93，实际 {a}");
        assert!((b - 48.30).abs() < 0.01, "下行应为 48.30，实际 {b}");
    }

    /// 层高每多 0.1 m，单层多约 0.07 秒。见第 3.7 节。
    #[test]
    fn floor_height_sensitivity() {
        let p = MotionProfile::default();
        let cases = [(2.8, 4.63f64), (2.9, 4.69), (3.0, 4.75), (3.1, 4.82), (3.3, 4.94)];
        for (h, want) in cases {
            let got = traverse_time(h, p).time;
            assert!((got - want).abs() < 0.01, "层高 {h} 应为 {want}，实际 {got}");
        }
    }

    /// 容差取 1e-6 而不是 1e-9。`2.8` 与 `0.2` 在 f32 下不可精确表示，
    /// 四项相加的误差就在这个量级。文档里的 6.0 秒指的是这个 f32 近似值。
    #[test]
    fn stop_overhead_matches_doc() {
        assert!((DoorTiming::default().stop_overhead() - 6.0).abs() < 1e-6);
    }

    #[test]
    fn transfer_time_overlaps_in_and_out() {
        let d = DoorTiming::default();
        // 只进不出
        assert!((d.transfer_time(3, 0) - 3.0).abs() < 1e-9);
        // 同时进出，取较大者加折扣
        let t = d.transfer_time(3, 3);
        assert!(t > 3.0 && t < 4.8, "同时进出应有折扣，实际 {t}");
    }

    #[test]
    fn zero_and_negative_distance_is_free() {
        assert_eq!(traverse_time(0.0, MotionProfile::default()).time, 0.0);
        assert_eq!(traverse_time(-5.0, MotionProfile::default()).time, 0.0);
    }

    /// 耗时必须随距离单调不减。
    #[test]
    fn time_is_monotonic_in_distance() {
        let p = MotionProfile::default();
        let mut prev = 0.0;
        for k in 0..40 {
            let t = traverse_time(k as f64 * 3.0, p).time;
            assert!(t >= prev, "距离 {k} 层时耗时反而变短");
            prev = t;
        }
    }
}