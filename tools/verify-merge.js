// 验证 merge.js：整段一次跑 vs 分段跑加合并，差值应在取整漂移内。
const fs = require("fs");
const path = require("path");
const root = path.join(__dirname, "..");
eval(fs.readFileSync(root + "/web/merge.js", "utf8") + "\nglobalThis.mergeReports = mergeReports;");
const mergeReports = globalThis.mergeReports;

(async () => {
  const bytes = fs.readFileSync(root + "/web/tihu.wasm");
  const { instance } = await WebAssembly.instantiate(bytes, {});
  const e = instance.exports;
  const take = (ptr) => {
    const mem = new Uint8Array(e.memory.buffer);
    const len = new DataView(e.memory.buffer).getUint32(ptr, true);
    const s = Buffer.from(mem.slice(ptr + 4, ptr + 4 + len)).toString("utf8");
    e.tihu_free(ptr, len + 4);
    return s;
  };
  const pass = (text, call) => {
    const b = Buffer.from(text, "utf8");
    const p = e.tihu_alloc(b.length);
    new Uint8Array(e.memory.buffer).set(b, p);
    try { return call(p, b.length); } finally { e.tihu_free(p, b.length); }
  };
  const run = (cfg, router) => pass(JSON.stringify(cfg), (p, n) => JSON.parse(take(e.tihu_run(p, n, router))));
  const cfg0 = JSON.parse(take(e.tihu_config(0)));

  for (const [from, to, router] of [[42, 141, 0], [7, 106, 1], [1, 64, 2]]) {
    const full = Object.assign({}, cfg0, { sim: Object.assign({}, cfg0.sim, { seed: from, seed_to: to }) });
    const want = run(full, router).sim;
    // 切 4 段
    const n = 4, ranges = [];
    let cur = from;
    for (let i = 0; i < n; i++) {
      const len = Math.ceil((to - cur + 1) / (n - i));
      ranges.push([cur, cur + len - 1]);
      cur += len;
    }
    const parts = ranges.map(([a, b]) =>
      run(Object.assign({}, cfg0, { sim: Object.assign({}, cfg0.sim, { seed: a, seed_to: b }) }), router).sim);
    const got = mergeReports(parts);
    let worst = 0, where = "", worstInt = 0, whereInt = "";
    const cmp = (a, b, path) => {
      const d = Math.abs(a - b);
      if (d > worst) { worst = d; where = path; }
    };
    const cmpInt = (a, b, path) => {
      const d = Math.abs(a - b);
      if (d > worstInt) { worstInt = d; whereInt = path; }
    };
    for (const k of ["mean", "p50", "p75", "p90", "p95", "p99", "max", "min"]) {
      cmp(got.wait[k], want.wait[k], "wait." + k);
      cmp(got.door_to_door[k], want.door_to_door[k], "d2d." + k);
    }
    cmpInt(got.wait.count, want.wait.count, "wait.count");
    cmpInt(got.door_to_door.count, want.door_to_door.count, "d2d.count");
    for (const k of ["serve_ratio", "stair_share", "rtt_mean", "throughput_5min", "pax_any_bypass", "stall_seconds"]) {
      cmp(got[k], want[k], k);
    }
    for (const k of ["pax_total", "pax_served", "stair_users", "abandoned", "ghost_stops", "partial_departures", "overshoots"]) {
      cmpInt(got[k], want[k], k);
    }
    let histDiff = 0;
    got.wait_hist.forEach((c, i) => { histDiff += Math.abs(c - want.wait_hist[i]); });
    for (let i = 0; i < got.floors.length; i++) {
      const g = got.floors[i], w = want.floors[i];
      for (const k of ["wait_mean", "wait_p50", "wait_p90", "wait_max", "d2d_mean", "d2d_p50", "d2d_p90", "d2d_max", "wait_best", "wait_worst", "d2d_best", "d2d_worst"]) {
        cmp(g[k], w[k], "floor" + g.level + "." + k);
      }
      for (const k of ["count", "bypass_total", "declined", "no_space", "stair_users", "live"]) {
        cmpInt(g[k], w[k], "floor" + g.level + "." + k);
      }
    }
    const ok = worst < 0.01 && worstInt <= 1 && histDiff === 0
      && got.runs === want.runs && got.seed_from === want.seed_from && got.seed_to === want.seed_to;
    console.log(`种子 ${from}..${to} router=${router}：小数最大差 ${worst.toFixed(4)}（${where}），整数最大差 ${worstInt}（${whereInt}），直方图差 ${histDiff}，${ok ? "ok" : "FAIL"}`);
    if (!ok) process.exit(1);
  }
})().catch((err) => { console.error("FAIL", err); process.exit(1); });
