//! 拿解析公式校验往返时间。
//!
//! 对应 `docs/MODEL.md` 第 10 节。四个条件全部满足时，仿真退化成一个
//! 教科书问题，实测的 `rtt_mean` 应当落在解析值附近。对不上就说明事件
//! 循环或者运动学写错了。
//!
//! 这一组配置把四个条件都钉死：
//!
//! - 只有一股上行客流，所有人从主入口层去 20 层，没有反向、没有返程
//! - 全部老实按键，不按双向外呼
//! - 超载死锁关掉，楼梯关掉，软起步线提到 1.0，不存在拒绝上车
//! - 一圈平均装十几个人，额定载客 26 人，一趟装得下
//!
//! 四条里最容易被忽略的是「一趟装得下所有人」：装不下就会有人留在厅里，
//! 停靠次数不再是 1，解析式的前提也就没了。
//!
//! 解析式按额定速度算行程，没有计每个停站的加减速损耗，所以仿真结果
//! 稳定地高出约 7%。10% 的容差正是为这一条留的，见 MODEL 10.1。

use tihu_core::batch::run_seeds;
use tihu_core::builtin::{DefaultBoarding, DownPeakSweep, LoadAwareEta};
use tihu_core::config::{preset, CarSpec, Config, Flow, Pattern, StrategyMix};

/// 全部乘客都从主入口层去这一层。
const TARGET: i16 = 20;

fn config() -> Config {
    let mut c = preset("std_tall").unwrap();
    c.building.top_level = TARGET;
    c.building.units_per_floor = vec![20; TARGET as usize];
    c.building.persons_per_unit = 1.0;
    c.cars = vec![CarSpec {
        capacity_rated: 26,
        rated_load_kg: 2000.0,
        ..CarSpec::default()
    }];
    // 只有一股客流，所有人去同一层。停靠次数因此恒等于 1
    c.traffic.pattern = Pattern::InterFloor;
    c.traffic.flows = vec![Flow::new(1, TARGET, 1.0)];
    c.traffic.departure_ratio = 1.0;
    c.traffic.window_s = 3600.0;
    c.traffic.reverse_ratio = 0.0;
    c.traffic.return_ratio = 0.0;
    c.stairs.enabled = false;
    c.overload.enabled = false;
    c.boarding.soft_start_ratio = 1.0;
    c.strategy = StrategyMix::default();
    c.sim.horizon_s = 7200.0;
    c
}

/// `RTT = 2·H·t_v + (S+1)·t_s + 2·P·t_p`
///
/// H 是最高折返层到主入口层的行程，这里只有一个目的层，H 是常数。
/// S 恒为 1：一层接人，一层送人。
fn analytic(c: &Config, riders_per_trip: f64) -> f64 {
    let car = &c.cars[0];
    let floors = (TARGET - 1) as f64;
    let t_v = c.building.floor_height as f64 / car.up.v_max as f64;
    let t_s = car.doors.stop_overhead();
    let t_p = (car.doors.pax_in_s + car.doors.pax_out_s) as f64 / 2.0;
    2.0 * floors * t_v + 2.0 * t_s + 2.0 * riders_per_trip * t_p
}

#[test]
fn up_peak_rtt_matches_the_analytic_formula() {
    let mut cfg = config();
    cfg.sim.seed = 1;
    cfg.sim.seed_to = Some(20);
    let b = run_seeds(
        &cfg,
        &mut DownPeakSweep,
        &mut LoadAwareEta::default(),
        &DefaultBoarding {
            cfg: cfg.boarding.clone(),
        },
    );

    // 前提：所有人都送到了，没有走楼梯的，也没有超长的圈被丢掉
    assert_eq!(b.scalars.pax_unfinished, 0.0, "还有人没送到，前提不成立");
    assert_eq!(b.scalars.stair_users, 0.0, "不该有人走楼梯");
    assert_eq!(b.scalars.pax_served, b.scalars.pax_total);
    assert_eq!(b.scalars.rtt_dropped, 0.0, "不该有圈被丢掉");
    // `scalars` 里都是每次运行的平均，圈数也是
    assert!(b.scalars.trips > 20.0, "每次的圈数太少，均值不稳");

    let p = b.scalars.pax_served / b.scalars.trips;
    assert!(
        p < 26.0 * 0.75,
        "一圈平均 {p} 人，离额定容量太近，不再是一趟装得下的前提"
    );

    let want = analytic(&cfg, p);
    let got = b.scalars.rtt_mean;
    let diff = got / want - 1.0;
    assert!(
        diff.abs() < 0.10,
        "实测往返 {got:.1} 秒，解析式 {want:.1} 秒，差 {:+.1}%。\
         差太多说明事件循环或运动学有问题，见 MODEL 10.2",
        diff * 100.0
    );
}
