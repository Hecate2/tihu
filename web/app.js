// 梯户模拟器的网页前端。
//
// 与内核只通过两份 JSON 打交道：一份配置进去，一份报告出来。仿真过程全部在
// WASM 里跑完，网页不参与任何数值计算。见 docs/ARCHITECTURE.md 第 7 节。
//
// 这个文件同时管两件事：把界面上的控件翻译成配置 JSON，把报告画成图。

"use strict";

// ---------------------------------------------------------------------------
// WASM 的加载与调用
// ---------------------------------------------------------------------------

let ex = null; // WASM 的导出表

function b64ToBytes(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

async function loadWasm() {
  let bytes;
  if (typeof window.TIHU_WASM_BASE64 === "string") {
    // 单文件版：产物以 base64 内嵌在 HTML 里，file:// 下也能双击打开。
    bytes = b64ToBytes(window.TIHU_WASM_BASE64);
  } else {
    const res = await fetch("tihu.wasm");
    if (!res.ok) throw new Error("拿不到 tihu.wasm，状态码 " + res.status);
    bytes = new Uint8Array(await res.arrayBuffer());
  }
  const { instance } = await WebAssembly.instantiate(bytes, {});
  return instance.exports;
}

// 读回「4 字节长度前缀 + UTF-8 负载」的缓冲区，读完立刻释放。
function take(ptr) {
  if (ptr === 0) throw new Error("内核交回了空指针");
  const view = new DataView(ex.memory.buffer);
  const len = view.getUint32(ptr, true);
  const mem = new Uint8Array(ex.memory.buffer);
  const text = new TextDecoder().decode(mem.slice(ptr + 4, ptr + 4 + len));
  ex.tihu_free(ptr, len + 4);
  return text;
}

// 把字符串写进线性内存后调用 fn，无论成败都释放那块输入缓冲。
function pass(text, fn) {
  const bytes = new TextEncoder().encode(text);
  const ptr = ex.tihu_alloc(bytes.length);
  if (ptr === 0 && bytes.length > 0) throw new Error("内核没能分配缓冲区");
  new Uint8Array(ex.memory.buffer).set(bytes, ptr);
  try {
    return fn(ptr, bytes.length);
  } finally {
    ex.tihu_free(ptr, bytes.length);
  }
}

const kernel = {
  config(kind) {
    return JSON.parse(take(ex.tihu_config(kind)));
  },
  run(cfg, routerId) {
    return JSON.parse(pass(JSON.stringify(cfg), (p, n) => take(ex.tihu_run(p, n, routerId))));
  },
};

// ---------------------------------------------------------------------------
// 界面控件 ↔ 配置 JSON 的翻译
// ---------------------------------------------------------------------------

// 速度档位的加速度与加加速度，与内核 motion::MotionProfile::preset 一致。
const SPEEDS = {
  "1.00": [0.5, 0.6],
  "1.60": [0.65, 0.8],
  "1.75": [0.8, 1.0],
  "2.50": [1.0, 1.2],
  "3.00": [1.0, 1.3],
};

const ui = {};
for (const id of [
  "preset", "res_floors", "units", "ppu", "fh", "basement", "gfr",
  "ncars", "load", "speed", "stops",
  "pattern", "dep", "window", "ret",
  "stairs", "pressb", "overload",
  "router", "seed",
]) {
  ui[id] = document.getElementById(id);
}

function num(el) { return Number(el.value); }
function clone(v) { return JSON.parse(JSON.stringify(v)); }

// 层号组合：地下若干层、主入口层、以及这一段住宅层。
function levelsFor(basement, lo, hi) {
  const out = [];
  for (let b = basement; b >= 1; b--) out.push(-b);
  out.push(1);
  for (let f = lo; f <= hi; f++) out.push(f);
  return out;
}

// 按「停靠区域」这个下拉框，给出第 i 台轿厢的停靠描述。
function stopsFor(i, n, top, basement) {
  const mode = ui.stops.value;
  if (mode === "parity") {
    return i % 2 === 0 ? "odd" : "even";
  }
  if (mode === "zones" && n > 1) {
    // 高低区。真实楼盘里每一台都停主入口层，否则高区的人出门之后回不了家。
    // 高区梯再往下跑到主入口，正是它比低区梯慢一截的原因。
    const span = top; // 从 1 层到 top 层，一共 top 个格位
    const lo = Math.max(2, 1 + Math.round((i * (span - 1)) / n) + 1);
    const hi = i === n - 1 ? top : 1 + Math.round(((i + 1) * (span - 1)) / n);
    return { list: levelsFor(basement, lo, Math.max(lo, hi)) };
  }
  if (mode === "zones") {
    return { list: levelsFor(basement, 2, top) };
  }
  return "all";
}

// 把界面上的值写进配置对象。只碰用户能调的那些字段，预设带进来的其他内容
// （比如层间客流的每股占比）一律原样留着。
function applyUi(cfg) {
  const top = num(ui.res_floors) + 1; // 层号 1 是主入口层
  const units = num(ui.units);
  const gfr = ui.gfr.checked;

  cfg.building.top_level = top;
  cfg.building.floor_height = num(ui.fh);
  cfg.building.persons_per_unit = num(ui.ppu);
  cfg.building.basement_levels = num(ui.basement);
  cfg.building.ground_floor_residential = gfr;
  const floors = new Array(top);
  for (let i = 0; i < top; i++) floors[i] = units;
  floors[0] = gfr ? units : 0;
  cfg.building.units_per_floor = floors;

  const n = num(ui.ncars);
  const load = num(ui.load);
  const [a, j] = SPEEDS[ui.speed.value];
  const car = {
    rated_load_kg: load,
    capacity_rated: Math.max(1, Math.round(load / 75)),
    up: { v_max: Number(ui.speed.value), a_acc: a, a_dec: a, j_acc: j, j_dec: j },
    down: { v_max: Number(ui.speed.value), a_acc: a, a_dec: a, j_acc: j, j_dec: j },
    initial_level: 1,
  };
  cfg.cars = [];
  for (let i = 0; i < n; i++) {
    const c = clone(car);
    c.stops = stopsFor(i, n, top, cfg.building.basement_levels);
    cfg.cars.push(c);
  }

  cfg.traffic.pattern = ui.pattern.value;
  cfg.traffic.departure_ratio = num(ui.dep);
  cfg.traffic.window_s = num(ui.window) * 60;
  cfg.traffic.return_ratio = num(ui.ret);
  if (cfg.traffic.pattern === "inter_floor" && (!cfg.traffic.flows || !cfg.traffic.flows.length)) {
    cfg.traffic.flows = defaultFlows(top);
    cfg.traffic.departure_ratio = Math.min(cfg.traffic.departure_ratio, 0.35);
  }

  cfg.stairs.enabled = ui.stairs.checked;
  cfg.overload.enabled = ui.overload.checked;
  const pb = num(ui.pressb);
  cfg.strategy.honest = 1 - pb;
  cfg.strategy.press_both = pb;
  cfg.strategy.press_up_only = 0;
  cfg.strategy.multi_call = 0;

  cfg.sim.seed = num(ui.seed);
  // 仿真窗口比高峰窗口多半小时。多出来的是恢复期，不然最后一批人刚到电梯厅
  // 仿真就结束了，完成人数会偏低。
  cfg.sim.horizon_s = num(ui.window) * 60 + 1800;
  return cfg;
}

// 层间交通的默认客流。用户切到层间模式又没给客流表时补一份，
// 起终点取楼层中段的几对，避免出现「一层都没有」的校验错误。
function defaultFlows(top) {
  const a = 2 + Math.floor((top - 2) / 4);
  const b = top - Math.max(1, Math.floor((top - 2) / 3));
  return [
    { from: a, to: b, share: 0.35 },
    { from: b, to: a, share: 0.30 },
    { from: 2, to: top, share: 0.20 },
    { from: top, to: 2, share: 0.15 },
  ];
}

// 反过来：把一份配置回填到控件上，预设按钮点下去就走这条。
function loadIntoUi(cfg) {
  ui.res_floors.value = cfg.building.top_level - 1;
  ui.units.value = cfg.building.units_per_floor[1] || 4;
  ui.ppu.value = cfg.building.persons_per_unit;
  ui.fh.value = cfg.building.floor_height;
  ui.basement.value = cfg.building.basement_levels || 0;
  ui.gfr.checked = !!cfg.building.ground_floor_residential;

  ui.ncars.value = cfg.cars.length;
  ui.load.value = cfg.cars[0].rated_load_kg;
  const v = cfg.cars[0].up.v_max;
  const key = Object.keys(SPEEDS).find((k) => Math.abs(Number(k) - v) < 0.01);
  ui.speed.value = key || "1.75";
  ui.stops.value = cfg.cars.every((c) => c.stops === "all")
    ? "all"
    : cfg.cars[0].stops === "odd" || cfg.cars[0].stops === "even"
      ? "parity"
      : "zones";

  ui.pattern.value = cfg.traffic.pattern;
  ui.dep.value = cfg.traffic.departure_ratio;
  ui.window.value = Math.round(cfg.traffic.window_s / 60);
  ui.ret.value = cfg.traffic.return_ratio;

  ui.stairs.checked = !!cfg.stairs.enabled;
  ui.overload.checked = !!cfg.overload.enabled;
  ui.pressb.value = cfg.strategy.press_both || 0;

  ui.seed.value = cfg.sim.seed;
  syncLabels();
}

function syncLabels() {
  document.getElementById("dep_v").textContent = Math.round(num(ui.dep) * 100) + "%";
  document.getElementById("ret_v").textContent = Math.round(num(ui.ret) * 100) + "%";
  document.getElementById("pressb_v").textContent = Math.round(num(ui.pressb) * 100) + "%";
}

// ---------------------------------------------------------------------------
// 配置链接。整份界面状态压进 URL 的 hash 里，发给别人就能复现同一张图。
// ---------------------------------------------------------------------------

function b64encode(str) {
  const bytes = new TextEncoder().encode(str);
  let bin = "";
  for (const b of bytes) bin += String.fromCharCode(b);
  return btoa(bin);
}

function b64decode(b64) {
  const bin = atob(b64);
  return new TextDecoder().decode(Uint8Array.from(bin, (c) => c.charCodeAt(0)));
}

function toHash() {
  const spec = {};
  for (const id of Object.keys(ui)) {
    const el = ui[id];
    spec[id] = el.type === "checkbox" ? el.checked : el.value;
  }
  return b64encode(JSON.stringify(spec))
    .replace(/\+/g, "-")
    .replace(/\//g, "_")
    .replace(/=+$/, "");
}

function fromHash(h) {
  try {
    const spec = JSON.parse(b64decode(h.replace(/-/g, "+").replace(/_/g, "/")));
    for (const id of Object.keys(spec)) {
      if (!ui[id]) continue;
      if (ui[id].type === "checkbox") ui[id].checked = !!spec[id];
      else ui[id].value = spec[id];
    }
    syncLabels();
    return true;
  } catch (e) {
    return false;
  }
}

// ---------------------------------------------------------------------------
// 出结果
// ---------------------------------------------------------------------------

let runToken = 0;
let lastRouterId = 0;

async function run() {
  const token = ++runToken;
  const cfg = applyUi(clone(base));
  lastRouterId = Number(ui.router.value);
  let res;
  try {
    res = kernel.run(cfg, lastRouterId);
  } catch (e) {
    showIssues([{ severity: "error", message: "仿真没能跑起来：" + e.message }]);
    return;
  }
  if (token !== runToken) return; // 又被拖了一次，这一趟作废
  if (!res.ok) {
    showIssues(res.issues);
    return;
  }
  showIssues(res.issues);
  render(res.sim);
  location.replace("#s=" + toHash());
}

function showIssues(issues) {
  const box = document.getElementById("issues");
  const errs = (issues || []).filter((i) => i.severity === "error");
  const warns = (issues || []).filter((i) => i.severity !== "error");
  if (!errs.length && !warns.length) {
    box.hidden = true;
    box.innerHTML = "";
    return;
  }
  box.hidden = false;
  box.innerHTML = errs
    .map((i) => `<li>${esc(i.message)}</li>`)
    .concat(warns.map((i) => `<li class="warn">${esc(i.message)}</li>`))
    .join("");
}

function fmt(x, d) { return (x == null || !isFinite(x) ? "—" : x.toFixed(d == null ? 1 : d)); }

function render(s) {
  document.getElementById("v_wait").innerHTML = fmt(s.wait.mean) + "<small>秒</small>";
  document.getElementById("v_wait_n").textContent =
    "样本 " + s.wait.count + " 人，中位数 " + fmt(s.wait.p50) + " 秒";
  document.getElementById("v_p90").innerHTML = fmt(s.wait.p90) + "<small>秒</small>";
  document.getElementById("v_bypass").innerHTML = fmt(s.pax_any_bypass * 100, 1) + "<small>%</small>";
  document.getElementById("v_d2d").innerHTML = fmt(s.door_to_door.p90) + "<small>秒</small>";

  // 逐层图：最高的层画在最上面，读起来像一栋楼
  const rows = s.floors
    .filter((f) => f.count > 0 || f.stair_users > 0)
    .slice()
    .reverse()
    .map((f) => ({ label: f.label, value: f.wait_mean }));
  document.getElementById("chart_floor").innerHTML = rows.length
    ? barChartH(rows, { digits: 1 })
    : '<p class="dim">这一趟没有任何人在住宅层出发。</p>';

  document.getElementById("chart_hist").innerHTML = histogram(s.wait_hist, s.wait_hist_bin_s);

  const bypass = s.floors.reduce((a, f) => a + f.bypass_total, 0);
  const declined = s.floors.reduce((a, f) => a + f.declined, 0);
  const noSpace = s.floors.reduce((a, f) => a + f.no_space, 0);
  const rows2 = [
    ["生成人数 / 送达人数", s.pax_total + " / " + s.pax_served],
    ["送达比例", fmt(s.serve_ratio * 100, 1) + "%"],
    ["出门前就走楼梯", s.stair_users + "（" + fmt(s.stair_share * 100, 1) + "%）"],
    ["半路改走楼梯", s.abandoned + " 人"],
    ["被跳过：位子不够", noSpace + " 次"],
    ["被跳过：人不愿挤", declined + " 次"],
    ["幽灵停站", s.ghost_stops + " 次"],
    ["满载直驶跳过", s.full_bypasses + " 次"],
    ["制动不及，越站", s.overshoots + " 次"],
    ["往返时间均值", fmt(s.rtt_mean) + " 秒"],
    ["五分钟输送", fmt(s.throughput_5min) + " 人"],
    ["超载死锁", s.stalls + " 次，累计 " + fmt(s.stall_seconds) + " 秒"],
    ["被僵局请下车", s.pax_after_stall + " 人"],
  ];
  document.getElementById("balance").innerHTML = rows2
    .map((r) => `<tr><td>${esc(r[0])}</td><td>${esc(r[1])}</td></tr>`)
    .join("");
  void bypass;
}

// ---------------------------------------------------------------------------
// 启动
// ---------------------------------------------------------------------------

let base = null;

function debounce(fn, ms) {
  let t = null;
  return function () {
    clearTimeout(t);
    t = setTimeout(fn, ms);
  };
}

async function main() {
  ex = await loadWasm();
  base = kernel.config(0);

  const presets = kernel.config(1);
  ui.preset.innerHTML = ['<option value="">自定义</option>']
    .concat(presets.map((p) => `<option value="${esc(p.name)}">${esc(p.label)}</option>`))
    .join("");
  const routers = kernel.config(2);
  ui.router.innerHTML = routers
    .map((r, i) => `<option value="${i}">${esc(r.label)}</option>`)
    .join("");

  ui.preset.addEventListener("change", () => {
    const name = ui.preset.value;
    if (!name) return;
    const idx = presets.findIndex((p) => p.name === name);
    base = kernel.config(3 + idx);
    loadIntoUi(base);
    run();
  });

  const kick = debounce(() => run(), 90);
  for (const id of ["res_floors", "units", "ppu", "fh", "basement", "gfr", "ncars", "load",
    "speed", "stops", "pattern", "dep", "window", "ret", "stairs", "pressb", "overload",
    "router", "seed"]) {
    ui[id].addEventListener("input", () => { syncLabels(); kick(); });
    ui[id].addEventListener("change", () => { syncLabels(); kick(); });
  }

  document.getElementById("run").addEventListener("click", () => run());
  document.getElementById("link").addEventListener("click", async (e) => {
    try {
      await navigator.clipboard.writeText(location.href);
      e.target.textContent = "已复制";
    } catch (err) {
      e.target.textContent = "复制失败";
    }
    setTimeout(() => (e.target.textContent = "复制配置链接"), 1200);
  });

  loadIntoUi(base);

  const hash = location.hash;
  if (hash.startsWith("#s=")) {
    if (fromHash(hash.slice(3))) {
      run();
      return;
    }
  }
  run();
}

main().catch((err) => {
  showIssues([{ severity: "error", message: "网页没能启动：" + (err.message || err) }]);
  console.error(err);
});
