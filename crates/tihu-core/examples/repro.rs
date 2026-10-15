fn main(){
    for (name, floors, units, ppu, cars) in [
        ("5层1梯4户", 5usize, 4u8, 2.0f32, 1usize),
        ("18层2梯4户", 18, 4, 2.8, 2),
        ("33层2梯6户", 33, 6, 2.8, 2),
        ("33层1梯4户", 33, 4, 2.8, 1),
    ] {
        let mut cfg = tihu_core::config::Config::default();
        // 「N 层住宅」的顶层号就是 N，主入口层是 1 层，也住人
        cfg.building.top_level = floors as i16;
        cfg.building.units_per_floor = vec![units; floors];
        cfg.building.persons_per_unit = ppu;
        cfg.cars = vec![tihu_core::config::CarSpec::default(); cars];
        let t0 = std::time::Instant::now();
        let r = tihu_core::sim::quick(cfg.clone(), 42);
        let m = &r.metrics;
        let boarded = r.world.pax.iter().filter(|p| p.t_board > 0.0).count();
        println!("== {} ==", name);
        println!("  耗时{:?} 乘客{} 上梯{} 完成{} 走楼梯{}", t0.elapsed(), m.pax_total, boarded, m.pax_served, m.stair_users);
        println!("  等梯 均值{:.1}s P90={:.1}s | 乘坐{:.1}s", m.wait.mean(), m.wait.p90(), m.ride.mean());
        println!("  趟数{} 每趟{:.1}人 RTT={:.1}s 外呼{} 被跳过{:.1}%",
            m.trips, boarded as f64 / m.trips.max(1) as f64, m.rtt_mean,
            r.world.calls.calls.len(), m.pax_any_bypass*100.0);
    }
}
