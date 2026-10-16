//! 网页绑定。
//!
//! 跨边界只传两样东西：一份配置 JSON 进去，一份报告 JSON 出来。仿真过程本身
//! 不跨界，一次调用在 WASM 内部跑完全部行程，只把聚合结果交出去。理由见
//! `docs/ARCHITECTURE.md` 第 7 节。
//!
//! 报告是**紧凑 DTO**，不是 `Metrics` 本身。`Metrics` 里每层的等待直方图有
//! 1200 格，四十层楼就是近五万个数字，直接序列化会把 JSON 撑到没法用。这里
//! 每层只留几个分位数。
//!
//! 对外的手写 ABI 只有四个函数，见本文件末尾。不引 wasm-bindgen，因为那会
//! 给内核带来一条与仿真无关的依赖，也会让 `file://` 下的加载多一层胶水。

use serde::Serialize;
use tihu_core::batch::{self, NumAvg, SeedBatch};
use tihu_core::builtin::{CollectiveScan, DownPeakSweep, LoadAwareEta, SelectiveCollective};
use tihu_core::config::{Config, Severity, LOBBY};
use tihu_core::metrics::{Metrics, Summary};
use tihu_core::sim::{self, Run};

/// 可选的走向算法。序号与网页下拉框一一对应。
const ROUTERS: [&str; 3] = ["down_peak_sweep", "collective_scan", "selective_collective"];

/// 走向算法的中文说明。网页直接显示，不需要再维护第二份表。
const ROUTER_LABELS: [&str; 3] = [
    "定向扫楼：先上到顶再一路下行",
    "集选：沿一个方向走，沿途有人按就停",
    "集选加满载直驶：载客率过 80% 就不再绕路",
];

/// 走向算法的一句话解释。网页在下拉框下面单独开一块显示。
///
/// 这里说的是**走向**，也就是车开出去之后往哪走。新按下的一条外呼分给哪台
/// 轿厢由内核的派梯规则统一负责，三种算法在这一层完全一样，见
/// `docs/DISPATCH_API.md`。名字不能混用，见 `AGENTS.md` 第 6 节的名词表。
const ROUTER_DESCS: [&str; 3] = [
    "先开到当前最高的那个呼叫层，再一路往下扫到最低。早高峰大家同时下楼时最省事，晚高峰要上楼的客流会被它绕远。",
    "轿厢认准一个方向走，路上有同向的呼叫就停，走到头再掉头。任何时段都能用，是住宅楼的通用选择。",
    "先用集选的走法，等车里载客率超过 80% 就只送车上的人，不再拐去接新外呼。它能挡住一部分没人上车的白停靠。",
];

/// 取三位小数。报告只用于显示，多余的位数只是把 JSON 撑大。
fn r3(x: f64) -> f64 {
    if x.is_finite() {
        (x * 1000.0).round() / 1000.0
    } else {
        0.0
    }
}

/// 一组时间样本的摘要。直方图不进报告。
#[derive(Serialize)]
struct Num {
    count: u64,
    mean: f64,
    p50: f64,
    p75: f64,
    p90: f64,
    p95: f64,
    p99: f64,
    max: f64,
    min: f64,
}

impl Num {
    fn of(s: &Summary) -> Self {
        Self {
            count: s.count,
            mean: r3(s.mean()),
            p50: r3(s.p50()),
            p75: r3(s.p75()),
            p90: r3(s.p90()),
            p95: r3(s.p95()),
            p99: r3(s.p99()),
            max: r3(s.max),
            min: if s.count == 0 { 0.0 } else { r3(s.min) },
        }
    }

    /// 多次运行时，每个数字取各次运行的平均。样本数也是平均到每一次。
    fn of_avg(a: &NumAvg) -> Self {
        Self {
            count: a.count.round() as u64,
            mean: r3(a.mean),
            p50: r3(a.p50),
            p75: r3(a.p75),
            p90: r3(a.p90),
            p95: r3(a.p95),
            p99: r3(a.p99),
            max: r3(a.max),
            min: r3(a.min),
        }
    }
}

/// 某一层的汇总。`label` 由内核侧算好，网页不用自己重建层号约定。
#[derive(Serialize)]
struct FloorRow {
    level: i16,
    label: String,
    count: u32,
    wait_mean: f64,
    wait_p90: f64,
    wait_max: f64,
    bypass_total: u32,
    declined: u32,
    no_space: u32,
    stair_users: u32,
    /// 各次运行该层平均等待时间里最短的那次。单次运行时不出现
    #[serde(skip_serializing_if = "Option::is_none")]
    wait_best: Option<f64>,
    /// 各次运行该层平均等待时间里最长的那次。单次运行时不出现
    #[serde(skip_serializing_if = "Option::is_none")]
    wait_worst: Option<f64>,
    /// 有几个种子的这一层真有人出发。单次运行时不出现
    #[serde(skip_serializing_if = "Option::is_none")]
    live: Option<usize>,
}

/// 层号的显示写法。主入口层是 1，地下层写成 B1、B2。
///
/// 编号约定见 `AGENTS.md` 第 2.8 条：B1 是层号 -1，B2 是 -2，层号 0 不存在。
pub fn level_label(level: i16) -> String {
    if level < LOBBY {
        format!("B{}", -level)
    } else {
        format!("{level}F")
    }
}

/// 一趟仿真的结果。
#[derive(Serialize)]
struct SimReport {
    /// 等梯时间，秒
    wait: Num,
    /// 乘坐时间，秒
    ride: Num,
    /// 门到门时间，秒
    door_to_door: Num,
    /// 逐层明细，按层号从小到大排
    floors: Vec<FloorRow>,
    /// 等待时间的分布，每格 5 秒，最长画到 300 秒
    wait_hist: Vec<u32>,
    /// 等待时间直方图的格宽，秒
    wait_hist_bin_s: f64,

    pax_total: usize,
    pax_served: u64,
    serve_ratio: f64,
    stair_users: usize,
    stair_share: f64,
    abandoned: usize,
    /// 全楼住户乘出行率估出来的行程段数，用来判断客流有没有跑完
    trips_expected: usize,

    n_cars: usize,
    trips: u64,
    rtt_mean: f64,
    throughput_5min: f64,
    horizon_s: f64,

    pax_with_bypass: u64,
    pax_any_bypass: f64,
    pax_after_stall: usize,
    stalls: usize,
    stall_seconds: f64,
    ghost_stops: usize,
    partial_departures: usize,
    full_bypasses: usize,
    overshoots: usize,

    /// 全楼最低层号，地下层为负
    lowest_level: i16,
    top_level: i16,
    /// 本次用的走向算法名
    router: String,

    /// 一共跑过几个种子。单次运行时不出现
    #[serde(skip_serializing_if = "Option::is_none")]
    runs: Option<usize>,
    /// 种子范围的起点。单次运行时不出现
    #[serde(skip_serializing_if = "Option::is_none")]
    seed_from: Option<u64>,
    /// 种子范围的终点。单次运行时不出现
    #[serde(skip_serializing_if = "Option::is_none")]
    seed_to: Option<u64>,
}

impl SimReport {
    fn of(run: &Run, router: &str) -> Self {
        let m: &Metrics = &run.metrics;
        let b = &run.cfg().building;
        let basement = b.basement_levels.max(0);
        // 层号 0 不存在，逐层明细里也不许出现这一格
        let floors = m
            .by_floor
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != basement as usize)
            .map(|(i, f)| {
                let level = i as i16 - basement;
                FloorRow {
                    level,
                    label: level_label(level),
                    count: f.count,
                    wait_mean: r3(f.wait_mean),
                    wait_p90: r3(f.wait_samples.p90()),
                    wait_max: r3(f.wait_max),
                    bypass_total: f.bypass_total,
                    declined: f.declined,
                    no_space: f.no_space,
                    stair_users: f.stair_users,
                    wait_best: None,
                    wait_worst: None,
                    live: None,
                }
            })
            .collect();
        let serve_ratio = if m.pax_total == 0 {
            0.0
        } else {
            r3(m.pax_served as f64 / m.pax_total as f64)
        };
        Self {
            wait: Num::of(&m.wait),
            ride: Num::of(&m.ride),
            door_to_door: Num::of(&m.door_to_door),
            floors,
            wait_hist: downsample(&m.wait, 5.0, 60),
            wait_hist_bin_s: 5.0,

            pax_total: m.pax_total,
            pax_served: m.pax_served,
            serve_ratio,
            stair_users: m.stair_users,
            stair_share: r3(m.stair_share),
            abandoned: m.abandoned,
            trips_expected: run.cfg().expected_trips(),

            n_cars: m.n_cars,
            trips: m.trips,
            rtt_mean: r3(m.rtt_mean),
            throughput_5min: r3(m.throughput_5min(run.cfg().sim.horizon_s)),
            horizon_s: run.cfg().sim.horizon_s,

            pax_with_bypass: m.pax_with_bypass,
            pax_any_bypass: r3(m.pax_any_bypass),
            pax_after_stall: m.pax_after_stall,
            stalls: m.stalls,
            stall_seconds: r3(m.stall_seconds),
            ghost_stops: m.ghost_stops,
            partial_departures: m.partial_departures,
            full_bypasses: m.full_bypasses,
            overshoots: m.overshoots,

            lowest_level: b.lowest_level(),
            top_level: b.highest_level(),
            router: router.to_string(),

            runs: None,
            seed_from: None,
            seed_to: None,
        }
    }

    /// 一段种子跑完之后的结果。数字取各次运行的平均，逐层多带较好与较差两个值。
    ///
    /// 报告结构与单次运行完全相同，只是标量换成平均，逐层多三个键。网页照原样
    /// 画图，不必区分两条路径。编号约定与单次一致，层号 0 那一格照样过滤掉。
    fn of_batch(s: &SeedBatch, cfg: &Config, router: &str) -> Self {
        let b = &cfg.building;
        let basement = b.basement_levels.max(0);
        // 层号 0 不存在，逐层明细里也不许出现这一格
        let floors = s
            .floors
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != basement as usize)
            .map(|(i, f)| {
                let level = i as i16 - basement;
                FloorRow {
                    level,
                    label: level_label(level),
                    count: f.count.round() as u32,
                    wait_mean: r3(f.wait_mean),
                    wait_p90: r3(f.wait_p90),
                    wait_max: r3(f.wait_max),
                    bypass_total: f.bypass_total.round() as u32,
                    declined: f.declined.round() as u32,
                    no_space: f.no_space.round() as u32,
                    stair_users: f.stair_users.round() as u32,
                    wait_best: Some(r3(f.wait_best)),
                    wait_worst: Some(r3(f.wait_worst)),
                    live: Some(f.live),
                }
            })
            .collect();
        let g = &s.scalars;
        Self {
            wait: Num::of_avg(&s.wait),
            ride: Num::of_avg(&s.ride),
            door_to_door: Num::of_avg(&s.door_to_door),
            floors,
            wait_hist: downsample_bins(&s.wait_hist, s.wait_hist_bin_s, 5.0, 60),
            wait_hist_bin_s: 5.0,

            pax_total: g.pax_total.round() as usize,
            pax_served: g.pax_served.round() as u64,
            serve_ratio: r3(g.serve_ratio),
            stair_users: g.stair_users.round() as usize,
            stair_share: r3(g.stair_share),
            abandoned: g.abandoned.round() as usize,
            trips_expected: g.trips_expected.round() as usize,

            n_cars: cfg.cars.len(),
            trips: g.trips.round() as u64,
            rtt_mean: r3(g.rtt_mean),
            throughput_5min: r3(g.throughput_5min),
            horizon_s: cfg.sim.horizon_s,

            pax_with_bypass: g.pax_with_bypass.round() as u64,
            pax_any_bypass: r3(g.pax_any_bypass),
            pax_after_stall: g.pax_after_stall.round() as usize,
            stalls: g.stalls.round() as usize,
            stall_seconds: r3(g.stall_seconds),
            ghost_stops: g.ghost_stops.round() as usize,
            partial_departures: g.partial_departures.round() as usize,
            full_bypasses: g.full_bypasses.round() as usize,
            overshoots: g.overshoots.round() as usize,

            lowest_level: b.lowest_level(),
            top_level: b.highest_level(),
            router: router.to_string(),

            runs: Some(s.runs),
            seed_from: Some(s.seed_from),
            seed_to: Some(s.seed_to),
        }
    }
}

/// 把 0.5 秒一格的直方图压成 `bin_s` 秒一格，只留前 `n` 格。
///
/// 溢出桶（超过 600 秒的那一撮）不进这张图，它在报告里以 `wait.max` 与
/// `wait.p99` 的形式出现。
fn downsample(s: &Summary, bin_s: f64, n: usize) -> Vec<u32> {
    let (bins, w) = s.histogram();
    downsample_bins(&bins, w, bin_s, n)
}

/// 把已经合好的一张 0.5 秒级直方图压成 `bin_s` 秒一格，只留前 `n` 格。
///
/// 批量运行交出来的就是各次相加后的格数，这里不再走 `Summary`。
fn downsample_bins(bins: &[u32], w: f64, bin_s: f64, n: usize) -> Vec<u32> {
    let mut out = vec![0u32; n];
    for (i, &c) in bins.iter().enumerate() {
        let k = (i as f64 * w / bin_s) as usize;
        if k < n {
            out[k] += c;
        }
    }
    out
}

/// 一条校验结果。
#[derive(Serialize)]
struct IssueDto {
    /// "error" 或 "warn"
    severity: &'static str,
    code: &'static str,
    message: String,
    level: Option<i16>,
}

impl IssueDto {
    fn of(i: &tihu_core::config::Issue) -> Self {
        Self {
            severity: match i.severity {
                Severity::Error => "error",
                Severity::Warn => "warn",
            },
            code: i.code,
            message: i.message.clone(),
            level: i.level,
        }
    }
}

/// 顶层响应。网页只看 `ok`、`issues`、`sim` 三个键。
#[derive(Serialize)]
struct Response {
    ok: bool,
    issues: Vec<IssueDto>,
    #[serde(skip_serializing_if = "Option::is_none")]
    sim: Option<SimReport>,
}

fn fail(code: &'static str, message: String) -> String {
    serde_json::to_string(&Response {
        ok: false,
        issues: vec![IssueDto { severity: "error", code, message, level: None }],
        sim: None,
    })
    .unwrap_or_else(|_| r#"{"ok":false,"issues":[]}"#.to_string())
}

/// 默认配置的 JSON。网页开局就拉这一份，之后只改用户动过的字段。
pub fn default_config_json() -> String {
    serde_json::to_string(&Config::default()).unwrap_or_else(|_| "{}".to_string())
}

/// 全部预设的名字与中文说明。
pub fn presets_json() -> String {
    let list: Vec<serde_json::Value> = tihu_core::config::preset_names()
        .iter()
        .map(|n| {
            serde_json::json!({ "name": n, "label": preset_label(n) })
        })
        .collect();
    serde_json::to_string(&list).unwrap_or_else(|_| "[]".to_string())
}

/// 预设的中文说明。只在网页的下拉框里出现。
fn preset_label(name: &str) -> &'static str {
    match name {
        "std_residential" => "18 层住宅，2 梯 4 户",
        "std_tall" => "33 层住宅，2 梯 6 户",
        "std_underserved" => "33 层住宅只有 1 台梯",
        "stress_gaming" => "有人乱按键，2 梯 6 户",
        "stress_stalled_car" => "1 台梯，超载死锁开着",
        "stress_mixed_traffic" => "中午双向，3 台梯",
        "hospital" => "院楼，层间交通为主",
        _ => "预设",
    }
}

/// 某个预设的配置 JSON。
pub fn preset_config_json(name: &str) -> String {
    match tihu_core::config::preset(name) {
        Some(c) => serde_json::to_string(&c).unwrap_or_else(|_| "{}".to_string()),
        None => fail("unknown_preset", format!("没有叫 {name} 的预设")),
    }
}

/// 走向算法的名字、短标签与一句话解释，供网页下拉框与说明区使用。
pub fn routers_json() -> String {
    let list: Vec<serde_json::Value> = ROUTERS
        .iter()
        .zip(ROUTER_LABELS.iter())
        .zip(ROUTER_DESCS.iter())
        .map(|((n, l), d)| serde_json::json!({ "name": n, "label": l, "desc": d }))
        .collect();
    serde_json::to_string(&list).unwrap_or_else(|_| "[]".to_string())
}

/// 常用速度档位，供网页在速度输入框下面给建议。
pub fn speed_tiers_json() -> String {
    let list: Vec<serde_json::Value> = tihu_core::motion::SPEED_TIERS
        .iter()
        .map(|t| serde_json::json!({ "v": t.0, "a": t.1, "j": t.2 }))
        .collect();
    serde_json::to_string(&list).unwrap_or_else(|_| "[]".to_string())
}

/// 按名字挑一套算法跑一次。
fn dispatch(cfg: Config, router: &str) -> Run {
    let b = tihu_core::builtin::DefaultBoarding::default();
    match router {
        "collective_scan" => {
            sim::run(cfg, &mut CollectiveScan, &mut LoadAwareEta::default(), &b)
        }
        "selective_collective" => {
            sim::run(cfg, &mut SelectiveCollective::default(), &mut LoadAwareEta::default(), &b)
        }
        _ => sim::run(cfg, &mut DownPeakSweep, &mut LoadAwareEta::default(), &b),
    }
}

/// 按名字挑一套算法，把整段种子跑完再汇总。范围取自配置里的 `seed_to`。
///
/// 与 `dispatch` 用同一组算法对象，区别只在循环在哪儿。聚合口径见
/// [`tihu_core::batch`]。
fn dispatch_batch(cfg: Config, router: &str) -> SeedBatch {
    let b = tihu_core::builtin::DefaultBoarding::default();
    match router {
        "collective_scan" => {
            batch::run_seeds(&cfg, &mut CollectiveScan, &mut LoadAwareEta::default(), &b)
        }
        "selective_collective" => {
            batch::run_seeds(&cfg, &mut SelectiveCollective::default(), &mut LoadAwareEta::default(), &b)
        }
        _ => batch::run_seeds(&cfg, &mut DownPeakSweep, &mut LoadAwareEta::default(), &b),
    }
}

/// 跑一次仿真。这是本 crate 的核心入口，网页与测试都走它。
///
/// 配置不合法时返回 `{"ok":false,"issues":[...]}`，不让任何一条硬约束被绕过。
pub fn report_json(cfg_json: &str, router: &str) -> String {
    let cfg: Config = match serde_json::from_str(cfg_json) {
        Ok(c) => c,
        Err(e) => {
            return fail("json_parse", format!("配置不是合法的 JSON：{e}"));
        }
    };
    let v = cfg.validate();
    if !v.is_ok() {
        let issues: Vec<IssueDto> = v.issues.iter().map(IssueDto::of).collect();
        return serde_json::to_string(&Response { ok: false, issues, sim: None })
            .unwrap_or_else(|_| r#"{"ok":false,"issues":[]}"#.to_string());
    }
    let router = if ROUTERS.contains(&router) { router } else { ROUTERS[0] };
    let cfg = cfg.sanitized();
    // 结束值大于根种子才走批量，否则照旧单跑，报告里一个批量字段都不出现
    let sim = if cfg.sim.seed_to.is_some_and(|t| t > cfg.sim.seed) {
        let batch = dispatch_batch(cfg.clone(), router);
        SimReport::of_batch(&batch, &cfg, router)
    } else {
        let run = dispatch(cfg, router);
        SimReport::of(&run, router)
    };
    serde_json::to_string(&Response { ok: true, issues: vec![], sim: Some(sim) })
        .unwrap_or_else(|_| r#"{"ok":false,"issues":[]}"#.to_string())
}

// ---------------------------------------------------------------------------
// 手写 ABI。四个函数，没有胶水，没有 wasm-bindgen。
//
// 返回值的约定统一是「4 字节小端长度 + UTF-8 负载」，一共 `len + 4` 字节。
// JS 读完必须调 `tihu_free(ptr, len + 4)`，否则连点几十次就会漏掉几十块内存。
//
// 输入方向相反：JS 先 `tihu_alloc(n)` 拿一块缓冲区，写进配置 JSON，
// 再把指针和字节数交给 `tihu_run`。
// ---------------------------------------------------------------------------

/// 申请一块 `len` 字节的缓冲区，用来放配置 JSON。
#[no_mangle]
pub extern "C" fn tihu_alloc(len: usize) -> *mut u8 {
    if len == 0 {
        return std::ptr::null_mut();
    }
    let mut buf = Vec::<u8>::with_capacity(len);
    let p = buf.as_mut_ptr();
    std::mem::forget(buf);
    p
}

/// 释放本模块交出去的缓冲区。
///
/// # Safety
/// `ptr` 必须来自 `tihu_alloc` 或本模块任意返回字符串的函数，`cap` 必须是当初
/// 申请的字节数。重复释放、或者把 `cap` 写错，都会破坏 WASM 线性内存。
#[no_mangle]
pub unsafe extern "C" fn tihu_free(ptr: *mut u8, cap: usize) {
    if ptr.is_null() || cap == 0 {
        return;
    }
    // SAFETY: 调用方保证这块内存是本模块用 Vec<u8> 以 `cap` 容量申请并 forget 的，
    // 分配器一致，长度按 0 传只回收容量，不读取任何未初始化字节。
    drop(Vec::from_raw_parts(ptr, 0, cap));
}

/// 把一段字符串按「长度前缀 + 负载」的格式交出去。
fn to_buf(s: String) -> *mut u8 {
    let bytes = s.into_bytes();
    let n = bytes.len();
    let mut out = Vec::<u8>::with_capacity(n + 4);
    out.extend_from_slice(&(n as u32).to_le_bytes());
    out.extend_from_slice(&bytes);
    let p = out.as_mut_ptr();
    std::mem::forget(out);
    p
}

/// 跑一次仿真。`router_id` 是走向算法序号，越界时用 0。
///
/// # Safety
/// `cfg_ptr` 必须指向 `cfg_len` 个有效字节，且那些字节拼起来是合法 UTF-8。
#[no_mangle]
pub unsafe extern "C" fn tihu_run(cfg_ptr: *const u8, cfg_len: usize, router_id: u32) -> *mut u8 {
    let router = ROUTERS.get(router_id as usize).copied().unwrap_or(ROUTERS[0]);
    let json = if cfg_ptr.is_null() || cfg_len == 0 {
        String::new()
    } else {
        // SAFETY: 调用方保证指针与长度有效，这里只读不写。
        let raw = std::slice::from_raw_parts(cfg_ptr, cfg_len);
        match std::str::from_utf8(raw) {
            Ok(s) => s.to_string(),
            Err(_) => {
                return to_buf(fail("bad_utf8", "配置字节不是合法的 UTF-8".into()));
            }
        }
    };
    to_buf(report_json(&json, router))
}

/// 交出默认配置、某个预设、预设清单、算法清单，或者速度档位表。
///
/// `kind` 取值：0 默认配置，1 预设清单，2 算法清单，100 速度档位表，
/// 3 到 99 是第 `kind - 3` 个预设。100 留得靠后，以后加预设也不会撞上。
#[no_mangle]
pub extern "C" fn tihu_config(kind: u32) -> *mut u8 {
    let names = tihu_core::config::preset_names();
    let body = match kind {
        0 => default_config_json(),
        1 => presets_json(),
        2 => routers_json(),
        100 => speed_tiers_json(),
        _ => {
            let i = (kind - 3) as usize;
            match names.get(i) {
                Some(n) => preset_config_json(n),
                None => fail("unknown_preset", format!("预设序号 {i} 越界")),
            }
        }
    };
    to_buf(body)
}

/// 报告格式的版本号。网页据此判断是否要提示刷新缓存。
#[no_mangle]
pub extern "C" fn tihu_report_version() -> u32 {
    1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_labels_cover_basement_and_lobby() {
        assert_eq!(level_label(1), "1F");
        assert_eq!(level_label(19), "19F");
        assert_eq!(level_label(-1), "B1");
        assert_eq!(level_label(-2), "B2");
    }

    #[test]
    fn default_config_runs_and_reports_ok() {
        let r = report_json(&default_config_json(), "down_peak_sweep");
        let v: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert_eq!(v["ok"], true, "默认配置应当跑得起来：{r}");
        assert!(v["sim"]["pax_served"].as_u64().unwrap() > 0, "应当有人被送达");
        assert!(v["sim"]["wait"]["mean"].as_f64().unwrap() > 0.0);
        assert_eq!(v["sim"]["router"], "down_peak_sweep");
    }

    /// 逐层明细的层号必须和内核的编号约定对得上，B 层不能错位。
    #[test]
    fn floor_rows_carry_real_level_numbers() {
        let mut c = tihu_core::config::preset("std_residential").unwrap();
        c.building.basement_levels = 2;
        let cfg = serde_json::to_string(&c).unwrap();
        let r = report_json(&cfg, "down_peak_sweep");
        let v: serde_json::Value = serde_json::from_str(&r).unwrap();
        let floors = v["sim"]["floors"].as_array().unwrap();
        assert_eq!(floors.len(), c.building.n_levels() - 1, "层号 0 不存在，明细要比格位数少一格");
        assert!(floors.iter().all(|f| f["level"] != 0), "层号 0 不该出现在明细里");
        assert_eq!(floors[0]["level"], -2);
        assert_eq!(floors[0]["label"], "B2");
        assert_eq!(floors[2]["label"], "1F", "第 1 层是主入口层，前面正好是 B2、B1");
    }

    /// 全部预设都要能跑，一个都不许 panic。
    #[test]
    fn every_preset_runs() {
        for name in tihu_core::config::preset_names() {
            let r = report_json(&preset_config_json(name), "down_peak_sweep");
            let v: serde_json::Value = serde_json::from_str(&r).unwrap();
            assert_eq!(v["ok"], true, "预设 {name} 跑不起来：{r}");
        }
    }

    /// 坏配置要走 JSON 报错，不许 panic，也不许返回 ok。
    #[test]
    fn invalid_config_reports_issues() {
        let mut c = tihu_core::config::preset("std_residential").unwrap();
        c.cars.clear();
        let r = report_json(&serde_json::to_string(&c).unwrap(), "down_peak_sweep");
        let v: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert_eq!(v["ok"], false);
        assert_eq!(v["issues"][0]["code"], "no_cars");
    }

    #[test]
    fn broken_json_reports_an_issue() {
        let r = report_json("{ 这不是 json", "down_peak_sweep");
        let v: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert_eq!(v["ok"], false);
        assert_eq!(v["issues"][0]["code"], "json_parse");
    }

    /// 未知算法名要退回默认，不能报错。
    #[test]
    fn unknown_router_falls_back() {
        let r = report_json(&default_config_json(), "definitely-not-a-router");
        let v: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert_eq!(v["ok"], true);
        assert_eq!(v["sim"]["router"], "down_peak_sweep");
    }

    /// 种子范围大于根种子时走批量，报告里带 runs 与逐层的较好较差。
    #[test]
    fn batch_report_averages_and_carries_range() {
        let mut c = tihu_core::config::preset("std_residential").unwrap();
        c.sim.seed = 42;
        c.sim.seed_to = Some(44);
        let cfg = serde_json::to_string(&c).unwrap();
        let r = report_json(&cfg, "down_peak_sweep");
        let v: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert_eq!(v["ok"], true, "批量跑不起来：{r}");
        assert_eq!(v["sim"]["runs"], 3);
        assert_eq!(v["sim"]["seed_from"], 42);
        assert_eq!(v["sim"]["seed_to"], 44);
        let floors = v["sim"]["floors"].as_array().unwrap();
        for f in floors {
            let (b, m, w) = (
                f["wait_best"].as_f64().unwrap(),
                f["wait_mean"].as_f64().unwrap(),
                f["wait_worst"].as_f64().unwrap(),
            );
            if f["live"].as_u64().unwrap() == 0 {
                assert_eq!((b, m, w), (0.0, 0.0, 0.0), "没人出发的楼层三个值都该是 0");
                continue;
            }
            assert!(b <= m + 1e-9 && m <= w + 1e-9, "第 {f} 层较好 ≤ 平均 ≤ 较差不成立");
        }
    }

    /// 同一段种子跑两次，批量报告逐字节相同。
    #[test]
    fn batch_report_is_reproducible() {
        let mut c = tihu_core::config::preset("std_tall").unwrap();
        c.sim.seed = 7;
        c.sim.seed_to = Some(11);
        let cfg = serde_json::to_string(&c).unwrap();
        assert_eq!(
            report_json(&cfg, "collective_scan"),
            report_json(&cfg, "collective_scan"),
            "同一段种子的两次批量报告必须一模一样"
        );
    }

    /// 种子范围超过上限要拦在校验阶段，不许硬跑。
    #[test]
    fn seed_range_over_limit_reports_issue() {
        let mut c = tihu_core::config::preset("std_residential").unwrap();
        c.sim.seed = 1;
        c.sim.seed_to = Some(1001);
        let r = report_json(&serde_json::to_string(&c).unwrap(), "down_peak_sweep");
        let v: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert_eq!(v["ok"], false);
        assert_eq!(v["issues"][0]["code"], "seed_range");
    }

    /// 单次运行的报告里不许出现批量的键，免得网页误判。
    #[test]
    fn single_run_omits_batch_keys() {
        let r = report_json(&default_config_json(), "down_peak_sweep");
        assert!(!r.contains("\"runs\""), "单次报告不该带 runs：{r}");
        assert!(!r.contains("wait_best"), "单次报告不该带 wait_best：{r}");
        assert!(!r.contains("seed_to"), "单次报告不该带 seed_to：{r}");
    }

    /// 批量配置要能走完整的 ABI 入口，交出来的还是长度前缀加 JSON。
    #[test]
    fn abi_carries_a_batch_config() {
        let mut c = tihu_core::config::preset("std_residential").unwrap();
        c.sim.seed = 5;
        c.sim.seed_to = Some(9);
        let cfg = serde_json::to_string(&c).unwrap();
        let bytes = cfg.as_bytes();
        // SAFETY: 手工铺一块缓冲区当调用方，模拟 JS 侧先 alloc 再写。
        unsafe {
            let p = tihu_alloc(bytes.len());
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), p, bytes.len());
            let out = tihu_run(p, bytes.len(), 0);
            tihu_free(p, bytes.len());
            let head = std::slice::from_raw_parts(out, 4);
            let len = u32::from_le_bytes([head[0], head[1], head[2], head[3]]) as usize;
            let body = std::slice::from_raw_parts(out.add(4), len);
            let v: serde_json::Value = serde_json::from_slice(body).unwrap();
            assert_eq!(v["sim"]["runs"], 5);
            tihu_free(out, len + 4);
        }
    }

    /// 同一份配置同一个种子，两次报告必须逐字节相同。可复现是红线。
    #[test]
    fn same_seed_gives_identical_report() {
        let cfg = serde_json::to_string(&tihu_core::config::preset("std_tall").unwrap()).unwrap();
        let a = report_json(&cfg, "down_peak_sweep");
        let b = report_json(&cfg, "down_peak_sweep");
        assert_eq!(a, b, "同一个种子的两次报告必须一模一样");
    }

    /// 报告里不许出现 NaN。JSON 装不下 NaN，网页画图会直接炸。
    #[test]
    fn report_has_no_nan() {
        let r = report_json(&default_config_json(), "down_peak_sweep");
        assert!(!r.contains("NaN"), "报告里出现了 NaN：{r}");
    }

    /// 缓冲区的前四字节就是负载长度，JS 按这个读。
    #[test]
    fn buffer_prefix_matches_payload() {
        let s = "你好，梯户".to_string();
        let n = s.len();
        let p = to_buf(s);
        // SAFETY: 刚由 to_buf 交出的缓冲区，长度是 n + 4，测试里读回来自用。
        unsafe {
            let head = std::slice::from_raw_parts(p, 4);
            let len = u32::from_le_bytes([head[0], head[1], head[2], head[3]]) as usize;
            assert_eq!(len, n, "长度前缀要等于负载字节数");
            let body = std::slice::from_raw_parts(p.add(4), len);
            assert_eq!(std::str::from_utf8(body).unwrap(), "你好，梯户");
            tihu_free(p, n + 4);
        }
    }

    /// 申请与释放要能配对，不能崩。
    #[test]
    fn alloc_and_free_roundtrip() {
        let p = tihu_alloc(16);
        assert!(!p.is_null());
        // SAFETY: 刚申请 16 字节，写满再还回去。
        unsafe {
            for i in 0..16 {
                *p.add(i) = i as u8;
            }
            assert_eq!(*p.add(15), 15);
            tihu_free(p, 16);
        }
        // SAFETY: 空指针与 0 容量，函数内部会直接返回。
        unsafe { tihu_free(std::ptr::null_mut(), 0) };
    }

    #[test]
    fn config_kinds_return_something() {
        for kind in [0u32, 1, 2] {
            let p = tihu_config(kind);
            // SAFETY: 取长度前缀后整块还回去。
            unsafe {
                let head = std::slice::from_raw_parts(p, 4);
                let len = u32::from_le_bytes([head[0], head[1], head[2], head[3]]) as usize;
                assert!(len > 2, "kind {kind} 交出的内容太短");
                tihu_free(p, len + 4);
            }
        }
    }
}
