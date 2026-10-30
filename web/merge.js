// 分片报告的合并。Worker 池把一段种子切成几块，每块各调一次内核，
// 交回来的都是 SimReport 形状的 JSON，这里按 runs 加权合成一份。
//
// 口径对照 crates/tihu-core/src/batch.rs，逐条如下：
// - Num（wait、ride、door_to_door）与全部标量：各次运行的平均，
//   分片之间按 runs 加权平均。min 与 max 也是各次的平均，不是极值。
// - wait_hist：各次相加。
// - 逐层 count、bypass_total、declined、no_space、stair_users：
//   每次运行都进累加，分片之间按 runs 加权平均。
// - 逐层 mean、p50、p90、max：只进有样本的种子，分片之间按 live 加权。
//   某个分片整段都没人，就不进上面几个数。
// - 逐层 best、worst：有样本分片里的最小与最大。
// - live：相加。
//
// 唯一的近似：分片报告里的数字已经按三位小数取整，合出来与一次跑完
// 相比，小数部分差 0.001 以内，界面只显示一位小数，看不出来。
// 个数类字段（人数、次数）分片侧先取整再平均，半数边界上可能差 1，
// node 以真实产物实测三组配置三组算法，小数最大差 0.0008，整数最大差 1，
// 直方图逐格相等。改了这里的公式就重跑一遍 tools 旁的验证脚本。
// 这个文件没有 DOM 依赖，node 里直接 eval 就能测。

"use strict";

function weightedMean(pairs) {
  let sum = 0, w = 0;
  for (const [v, k] of pairs) {
    if (!isFinite(v) || !isFinite(k) || k <= 0) continue;
    sum += v * k;
    w += k;
  }
  return w > 0 ? sum / w : 0;
}

// 一组时间样本的摘要。各字段都是各次的平均，加权平均即整体平均。
function mergeNum(parts) {
  const runs = parts.map((p) => p.runs);
  const out = {};
  for (const k of ["count", "mean", "p50", "p75", "p90", "p95", "p99", "max", "min"]) {
    out[k] = weightedMean(parts.map((p, i) => [p[k], runs[i]]));
  }
  // 样本数显示成整数，与内核 of_avg 的 round 口径一致
  out.count = Math.round(out.count);
  return out;
}

// 顶层标量。n_cars 这类配置项各分片相同，取第一块的。
function mergeScalars(parts) {
  const runs = parts.map((p) => p.runs);
  const first = parts[0];
  const out = {
    n_cars: first.n_cars,
    horizon_s: first.horizon_s,
    lowest_level: first.lowest_level,
    top_level: first.top_level,
    router: first.router,
    wait_hist_bin_s: first.wait_hist_bin_s,
  };
  for (const k of [
    "pax_total", "pax_served", "pax_unfinished", "serve_ratio", "stair_users", "stair_share",
    "abandoned", "pax_press_and_run", "trips_expected", "trips", "rtt_mean", "rtt_dropped", "throughput_5min",
    "pax_with_bypass", "pax_any_bypass", "pax_after_stall", "stalls", "stall_seconds",
    "calls_orphaned", "ghost_stops", "partial_departures", "full_bypasses", "overshoots",
  ]) {
    out[k] = weightedMean(parts.map((p, i) => [p[k], runs[i]]));
  }
  // 个数显示成整数，与内核 of_batch 的 round 口径一致
  for (const k of [
    "pax_total", "pax_served", "pax_unfinished", "stair_users", "abandoned",
    "pax_press_and_run", "trips_expected",
    "trips", "rtt_dropped", "pax_with_bypass", "pax_after_stall", "stalls", "calls_orphaned",
    "ghost_stops", "partial_departures", "full_bypasses", "overshoots",
  ]) {
    out[k] = Math.round(out[k]);
  }
  return out;
}

// 逐层明细。分片必须同配置，层号对不上就抛错，调用方回退同步。
function mergeFloors(parts) {
  const n = parts[0].floors.length;
  return parts[0].floors.map((f0, i) => {
    for (const p of parts) {
      if (p.floors.length !== n || p.floors[i].level !== f0.level) {
        throw new Error("分片报告的楼层对不上，没法合并");
      }
    }
    const runs = parts.map((p) => p.runs);
    const lives = parts.map((p) => p.floors[i].live || 0);
    const byRuns = (k) => weightedMean(parts.map((p, j) => [p.floors[i][k], runs[j]]));
    const byLive = (k) => weightedMean(parts.map((p, j) => [p.floors[i][k], lives[j]]));
    const liveBest = (k) => {
      let b = Infinity;
      for (const p of parts) if ((p.floors[i].live || 0) > 0) b = Math.min(b, p.floors[i][k]);
      return b === Infinity ? 0 : b;
    };
    const liveWorst = (k) => {
      let w = 0;
      for (const p of parts) if ((p.floors[i].live || 0) > 0) w = Math.max(w, p.floors[i][k]);
      return w;
    };
    return {
      level: f0.level,
      label: f0.label,
      count: Math.round(byRuns("count")),
      wait_mean: byLive("wait_mean"),
      wait_p50: byLive("wait_p50"),
      wait_p90: byLive("wait_p90"),
      wait_max: byLive("wait_max"),
      d2d_mean: byLive("d2d_mean"),
      d2d_p50: byLive("d2d_p50"),
      d2d_p90: byLive("d2d_p90"),
      d2d_max: byLive("d2d_max"),
      bypass_total: Math.round(byRuns("bypass_total")),
      declined: Math.round(byRuns("declined")),
      no_space: Math.round(byRuns("no_space")),
      stair_users: Math.round(byRuns("stair_users")),
      wait_best: liveBest("wait_best"),
      wait_worst: liveWorst("wait_worst"),
      d2d_best: liveBest("d2d_best"),
      d2d_worst: liveWorst("d2d_worst"),
      live: lives.reduce((a, b) => a + b, 0),
    };
  });
}

// 把同一配置、同一算法、不交种子段的几份 SimReport 合成一份。
// 输入是 report_json 交出来的 sim 对象，不是外层 Response。
// 分片必须是批量报告（带 runs 与逐层的 live），单种子报告缺这些键，
// 合出来是错数，这里直接抛错，调用方回退同步重跑。
function mergeReports(partials) {
  if (!partials.length) throw new Error("没有分片可合并");
  for (const p of partials) {
    if (p.runs == null || p.seed_from == null) {
      throw new Error("分片不是批量报告，没法合并");
    }
  }
  const runs = partials.map((p) => p.runs || 1);
  const total = runs.reduce((a, b) => a + b, 0);
  const widest = partials.map((p, i) => ({ p, runs: runs[i] }));
  const numOf = (k) => mergeNum(widest.map(({ p, runs }) => Object.assign({ runs }, p[k])));
  const s = mergeScalars(widest.map(({ p, runs }) => Object.assign({ runs }, p)));
  const floors = mergeFloors(widest.map(({ p, runs }) => ({ runs, floors: p.floors })));
  const histLen = Math.max(...partials.map((p) => p.wait_hist.length));
  const wait_hist = new Array(histLen).fill(0);
  for (const p of partials) {
    p.wait_hist.forEach((c, i) => { wait_hist[i] += c; });
  }
  return Object.assign({}, s, {
    wait: numOf("wait"),
    ride: numOf("ride"),
    door_to_door: numOf("door_to_door"),
    floors,
    wait_hist,
    runs: total,
    seed_from: Math.min(...partials.map((p) => p.seed_from)),
    seed_to: Math.max(...partials.map((p) => p.seed_to)),
  });
}
