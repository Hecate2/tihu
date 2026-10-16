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

// 常用速度档位，由内核给出：[额定速度, 加速度, 加加速度]。网页允许填任意小数
// 速度，加减速度取最接近的那个档位，规则与内核 motion::MotionProfile::for_speed
// 一致，两边不会有第二份表。
let SPEED_TIERS = [];
function profileFor(v) {
  let best = SPEED_TIERS[0] || { a: 0.8, j: 1.0 };
  for (const t of SPEED_TIERS) {
    if (Math.abs(t.v - v) < Math.abs(best.v - v)) best = t;
  }
  return [best.a, best.j];
}

const ui = {};
for (const id of [
  "preset", "res_floors", "units", "ppu", "fh", "basement", "gfr",
  "ncars", "load", "speed", "stops",
  "pattern", "dep", "window", "ret",
  "stairs", "pressb", "overload",
  "router", "seed", "seed_to",
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
  // 整数框里手输小数、负数或者清空，都先收成合法整数再往下走。
  // 不然 new Array(2.5) 直接抛 RangeError，连内核的校验错误都看不到。
  const top = Math.max(1, Math.round(num(ui.res_floors)) || 1); // 填 N 就是 1 到 N 层，主入口层是 1 层
  const units = Math.max(0, Math.round(num(ui.units)) || 0);
  const basement = Math.max(0, Math.round(num(ui.basement)) || 0);
  const gfr = ui.gfr.checked;

  cfg.building.top_level = top;
  cfg.building.floor_height = num(ui.fh);
  cfg.building.persons_per_unit = num(ui.ppu);
  cfg.building.basement_levels = basement;
  cfg.building.ground_floor_residential = gfr;
  const floors = new Array(top);
  for (let i = 0; i < top; i++) floors[i] = units;
  floors[0] = gfr ? units : 0;
  cfg.building.units_per_floor = floors;

  const n = Math.max(1, Math.round(num(ui.ncars)) || 1);
  const load = num(ui.load);
  // 速度可以填任意小数。清了输入框或者填了怪值就退回 1.75，不让内核拿到 0。
  const raw = Number(ui.speed.value);
  const v = isFinite(raw) && raw > 0 ? raw : 1.75;
  const [a, j] = profileFor(v);
  const car = {
    rated_load_kg: load,
    capacity_rated: Math.max(1, Math.round(load / 75)),
    up: { v_max: v, a_acc: a, a_dec: a, j_acc: j, j_dec: j },
    down: { v_max: v, a_acc: a, a_dec: a, j_acc: j, j_dec: j },
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
    // 滑块是多少就是多少，不再往 0.35 钳。原来钳了也不回写滑块，
    // 界面显示 66% 而实际按 35% 跑，对不上。
    cfg.traffic.flows = defaultFlows(top);
  }

  cfg.stairs.enabled = ui.stairs.checked;
  cfg.overload.enabled = ui.overload.checked;
  // 滑块只管乱按里「上下都按」这一项。预设带进来的只按上行、同时呼叫多台
  // 要原样留着，不然 stress_gaming 那 5% 只按上行会被静默归零。
  const pb = num(ui.pressb);
  const other = Math.max(0, (cfg.strategy.press_up_only || 0) + (cfg.strategy.multi_call || 0));
  const pbKept = Math.min(pb, Math.max(0, 1 - other));
  cfg.strategy.press_both = pbKept;
  cfg.strategy.honest = Math.max(0, 1 - pbKept - other);

  // 随机种子与种子范围的结束值。结束值大于起始值就走批量，内核把中间每个
  // 种子都跑一遍再汇总。填一样、填更小，或者留空，都当单次跑。
  const seedA = Math.max(0, Math.round(num(ui.seed)));
  const seedB = ui.seed_to.value === "" ? seedA : Math.max(0, Math.round(Number(ui.seed_to.value)));
  cfg.sim.seed = seedA;
  cfg.sim.seed_to = seedB > seedA ? seedB : null;
  // 仿真窗口比高峰窗口多半小时。多出来的是恢复期，不然最后一批人刚到电梯厅
  // 仿真就结束了，完成人数会偏低。
  cfg.sim.horizon_s = num(ui.window) * 60 + 1800;
  return cfg;
}

// 层间交通的默认客流。用户切到层间模式又没给客流表时补一份，
// 起终点取楼层中段的几对，避免出现「一层都没有」的校验错误。
// 层太少时凑不出两对不同的层，直接用主入口层与顶层对开，
// 不然会出现起终点相同的自环流，内核会静默丢弃那几股。
function defaultFlows(top) {
  const t = Math.max(2, Math.round(top) || 2);
  let a = 2 + Math.floor((t - 2) / 4);
  let b = t - Math.max(1, Math.floor((t - 2) / 3));
  a = Math.min(t, Math.max(1, a));
  b = Math.min(t, Math.max(1, b));
  const flows = [
    { from: a, to: b, share: 0.35 },
    { from: b, to: a, share: 0.30 },
    { from: Math.min(2, t), to: t, share: 0.20 },
    { from: t, to: Math.min(2, t), share: 0.15 },
  ].filter((f) => f.from !== f.to);
  // 层太少时上面会被滤空，直接用主入口层与顶层对开
  if (!flows.length) {
    return [
      { from: 1, to: t, share: 0.5 },
      { from: t, to: 1, share: 0.5 },
    ];
  }
  return flows;
}

// 反过来：把一份配置回填到控件上，预设按钮点下去就走这条。
function loadIntoUi(cfg) {
  ui.res_floors.value = cfg.building.top_level;
  // 每层户数的代表值取第一格非零的。一层是大堂时第一格是 0，跳过它。
  ui.units.value = cfg.building.units_per_floor.find((x) => x > 0) || 4;
  ui.ppu.value = cfg.building.persons_per_unit;
  ui.fh.value = cfg.building.floor_height;
  ui.basement.value = cfg.building.basement_levels || 0;
  ui.gfr.checked = !!cfg.building.ground_floor_residential;

  ui.ncars.value = cfg.cars.length;
  ui.load.value = cfg.cars[0].rated_load_kg;
  ui.speed.value = cfg.cars[0].up.v_max;
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

// 百分比显示。0.5% 这种小比例不能被四舍五入成 1%，所以先收到一位小数。
function pct(x) {
  const v = Math.round(x * 1000) / 10;
  return (Number.isInteger(v) ? v.toFixed(0) : v.toFixed(1)) + "%";
}

function syncLabels() {
  document.getElementById("dep_v").textContent = pct(num(ui.dep));
  document.getElementById("ret_v").textContent = pct(num(ui.ret));
  document.getElementById("pressb_v").textContent = pct(num(ui.pressb));
  markRouter();
}

// 走向算法的说明区。选中哪条就把哪条加粗，三条并排留着对照。
function markRouter() {
  const i = Number(ui.router.value) || 0;
  document.querySelectorAll("#router_list li").forEach((el, k) => {
    el.classList.toggle("on", k === i);
  });
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

  // 批量与单次都只调一次内核，聚合在内核侧完成。这里只按种子范围估个次数，
  // 用来先画一句提示，真正跑几个种子由内核说了算。
  const runs = cfg.sim.seed_to ? cfg.sim.seed_to - cfg.sim.seed + 1 : 1;
  setNote(runs > 1 ? "正在跑 " + runs + " 个种子…" : "");
  if (runs > 1) {
    // 先把这句话画到页面上，再开始算，不然页面会一直停在上一张图。
    await new Promise((r) => setTimeout(r, 0));
    if (token !== runToken) return;
  }

  let sim;
  try {
    const res = kernel.run(cfg, lastRouterId);
    if (!res.ok) { setNote(""); showIssues(res.issues); return; }
    showIssues(res.issues);
    sim = res.sim;
  } catch (e) {
    setNote("");
    showIssues([{ severity: "error", message: "仿真没能跑起来：" + e.message }]);
    return;
  }
  if (token !== runToken) return;
  render(sim);
  location.replace("#s=" + toHash());
}

// 结果区顶上那行说明。单次运行不留话，多种子运行才写。
function setNote(text) {
  const el = document.getElementById("runs_note");
  el.hidden = !text;
  el.textContent = text || "";
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

// 单次运行时下面每个数字就是那一次的结果。多种子运行时内核已经合成过一份，
// 秒数与比例是各次的平均，样本数也是每次的平均，只有分布图是各次相加。
function render(s) {
  const runs = s.runs || 1;
  document.getElementById("v_wait").innerHTML = fmt(s.wait.mean) + "<small>秒</small>";
  document.getElementById("v_wait_n").textContent =
    (runs > 1 ? "每次样本 " : "样本 ") + s.wait.count + " 人，中位数 " + fmt(s.wait.p50) + " 秒";
  document.getElementById("v_p90").innerHTML = fmt(s.wait.p90) + "<small>秒</small>";
  document.getElementById("v_bypass").innerHTML = fmt(s.pax_any_bypass * 100, 1) + "<small>%</small>";
  document.getElementById("v_d2d").innerHTML = fmt(s.door_to_door.p90) + "<small>秒</small>";

  // 逐层图：最高的层画在最上面，读起来像一栋楼。
  // 批量报告的人数是各次的平均再取整，平均不到半个人会被轮成 0，
  // 这种层看 live，有种子真发过车就留着，不然会被藏掉。
  const floors = s.floors
    .filter((f) => f.count > 0 || f.stair_users > 0 || (f.live || 0) > 0)
    .slice()
    .reverse();
  const box = document.getElementById("chart_floor");
  if (!floors.length) {
    box.innerHTML = '<p class="dim">这一趟没有任何人在住宅层出发。</p>';
  } else if (runs > 1) {
    box.innerHTML = barChartRange(
      floors.map((f) => ({
        label: f.label, mean: f.wait_mean, best: f.wait_best, worst: f.wait_worst, live: f.live,
      })),
      { digits: 1 }
    );
  } else {
    box.innerHTML = barChartH(floors.map((f) => ({ label: f.label, value: f.wait_mean })), { digits: 1 });
  }

  const floorNote = document.getElementById("floor_note");
  floorNote.hidden = runs <= 1;
  floorNote.textContent = runs > 1
    ? "共跑 " + runs + " 次。实心短线是各次的平均，浅色带是 " + runs + " 次里最短与最长的那两次。"
    : "";

  const histNote = document.getElementById("hist_note");
  histNote.hidden = runs <= 1;
  histNote.textContent = runs > 1 ? "柱高是 " + runs + " 次运行合计的人次。" : "";

  document.getElementById("chart_hist").innerHTML = histogram(s.wait_hist, s.wait_hist_bin_s);

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

  setNote(runs > 1
    ? "共跑 " + runs + " 个种子，从 " + s.seed_from + " 到 " + s.seed_to +
      "。秒数与比例取各次的平均，样本数也是每次的平均。"
    : "");
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
  document.getElementById("router_list").innerHTML = routers
    .map((r) => `<li><b>${esc(r.label)}</b><span>${esc(r.desc)}</span></li>`)
    .join("");

  // 常用速度档位由内核给出，网页这边只负责摆进输入框的建议列表。
  SPEED_TIERS = kernel.config(100);
  document.getElementById("speed_tiers").innerHTML = SPEED_TIERS
    .map((t) => `<option value="${t.v}">${t.v} 米每秒</option>`)
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
    "router", "seed", "seed_to"]) {
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
