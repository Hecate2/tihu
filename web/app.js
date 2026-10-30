// 梯户模拟器的网页前端。
//
// 与内核只通过两份 JSON 打交道：一份配置进去，一份报告出来。仿真过程全部在
// WASM 里跑完，网页不参与任何数值计算。见 docs/ARCHITECTURE.md 第 7 节。
//
// 这个文件同时管两件事：把界面上的控件翻译成配置 JSON，把报告画成图。

"use strict";

// HTML 转义。charts.js 里有一份同名同实现的 esc，但两个文件没有模块关系，
// 这里用自己的名字，免得隐式依赖对方的加载顺序。
function escHtml(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

// ---------------------------------------------------------------------------
// WASM 的加载与调用
// ---------------------------------------------------------------------------

let ex = null; // WASM 的导出表
let wasmBytes = null; // WASM 产物字节。主线程自用一份，开工人时再各克隆一份发过去

function b64ToBytes(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

async function loadWasm() {
  if (typeof window.TIHU_WASM_BASE64 === "string") {
    // 单文件版：产物以 base64 内嵌在 HTML 里，file:// 下也能双击打开。
    wasmBytes = b64ToBytes(window.TIHU_WASM_BASE64);
    const { instance } = await WebAssembly.instantiate(wasmBytes, {});
    return instance.exports;
  }
  const res = await fetch("tihu.wasm");
  if (!res.ok) throw new Error("拿不到 tihu.wasm，状态码 " + res.status);
  // 流式编译边下载边编译，两件事重叠着做。产物字节另要一份给工人，
  // 所以先 clone，两条分支各读一路，不用下两遍
  const wantStream = WebAssembly.instantiateStreaming &&
    (res.headers.get("content-type") || "").includes("wasm");
  if (wantStream) {
    const copy = res.clone();
    try {
      const { instance } = await WebAssembly.instantiateStreaming(res, {});
      wasmBytes = new Uint8Array(await copy.arrayBuffer());
      return instance.exports;
    } catch (e) {
      // MIME 不对或者流断在中途，退回整块编译
      wasmBytes = new Uint8Array(await copy.arrayBuffer());
    }
  } else {
    wasmBytes = new Uint8Array(await res.arrayBuffer());
  }
  const { instance } = await WebAssembly.instantiate(wasmBytes, {});
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
  // 取最接近的那一档，与内核 motion::MotionProfile::for_speed 同一条规则。
  // 档位表为空时退回住宅常见值，不让解构拿到 undefined。
  let best = null;
  for (const t of SPEED_TIERS) {
    if (!best || Math.abs(t.v - v) < Math.abs(best.v - v)) best = t;
  }
  return best ? [best.a, best.j] : [0.8, 1.0];
}

// 改了要重算的控件。这份清单同时用来挂监听，界面上的输入框与这里的名字
// 一一对应，加控件只改这一处。
const LIVE_IDS = [
  "res_floors", "units", "ppu", "area", "fh", "basement", "gfr",
  "ncars", "load", "speed", "stops",
  "pattern", "dep", "window", "ret",
  "stairs", "pressb", "overload",
  "router", "seed", "seed_count",
];

const ui = {};
// 预设下拉框只换配置、不影响重算，所以只进 ui 不进 LIVE_IDS
for (const id of ["preset", ...LIVE_IDS]) {
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
    // 下界收到 top。梯数多于住宅层数时，后面几台算出的下界会越过顶层，
    // 生成一排根本不存在的楼层，内核直接报错
    const lo = Math.min(top, Math.max(2, 1 + Math.round((i * (span - 1)) / n) + 1));
    const hi = i === n - 1 ? top : 1 + Math.round(((i + 1) * (span - 1)) / n);
    return { list: levelsFor(basement, lo, Math.max(lo, hi)) };
  }
  if (mode === "zones") {
    return { list: levelsFor(basement, 2, top) };
  }
  return "all";
}

// 按 [lo, hi] 收整数输入，超了就地钳并写回输入框，所见即所算。
// 界限与内核 sanitized() 同一套，其中户数上限 32 只是防呆，
// 真正的人口密度约束在给了楼板面积时由内核另算。
// 没超就不碰输入框，免得打断打字。输入框清空时也不写回，按老规矩走。
// 输入框清空时也不写回，按清空前的老规矩走。
function clampWrite(el, v, lo, hi) {
  const c = Math.min(hi, Math.max(lo, v));
  if (el.value !== "" && Number(el.value) !== c) el.value = String(c);
  return c;
}

// 把界面上的值写进配置对象。只碰用户能调的那些字段，预设带进来的其他内容
// （比如层间客流的每股占比）一律原样留着。
function applyUi(cfg) {
  // 整数框里手输小数、负数或者清空，都先收成合法整数再往下走。
  // 不然 new Array(2.5) 直接抛 RangeError，连内核的校验错误都看不到。
  const top = Math.max(1, Math.round(num(ui.res_floors)) || 1); // 填 N 就是 1 到 N 层，主入口层是 1 层
  const units = clampWrite(ui.units, Math.max(0, Math.round(num(ui.units)) || 0), 0, 32);
  const basement = Math.max(0, Math.round(num(ui.basement)) || 0);
  const gfr = ui.gfr.checked;

  cfg.building.top_level = top;
  cfg.building.floor_height = num(ui.fh);
  cfg.building.persons_per_unit = num(ui.ppu);
  // 每层建筑面积可不填。空着就不做密度校验，填了内核才按最小套型与
  // 人均面积反推这层塞不塞得下
  cfg.building.floor_area_m2 = ui.area.value.trim() === "" ? null : num(ui.area);
  cfg.building.basement_levels = basement;
  cfg.building.ground_floor_residential = gfr;
  const floors = new Array(top);
  for (let i = 0; i < top; i++) floors[i] = units;
  floors[0] = gfr ? units : 0;
  cfg.building.units_per_floor = floors;

  const n = Math.max(1, Math.round(num(ui.ncars)) || 1);
  const load = clampWrite(ui.load, num(ui.load) || 0, 100, 5000);
  // 速度可以填任意小数。清了输入框或者填了怪值就退回 1.75，不让内核拿到 0。
  // 上限 20 与内核同界，超了写回输入框。
  const raw = Number(ui.speed.value);
  const v = clampWrite(ui.speed, isFinite(raw) && raw > 0 ? raw : 1.75, 0.1, 20);
  const [a, j] = profileFor(v);
  const car = {
    rated_load_kg: load,
    capacity_rated: Math.min(40, Math.max(1, Math.round(load / 75))),
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

  // 随机种子跑几个。从随机种子开始连跑 count 个，内核把中间每个
  // 种子都跑一遍再汇总。填 1 或者清空，都当单次跑。
  const seedA = Math.max(0, Math.round(num(ui.seed)));
  const count = Math.max(1, Math.round(num(ui.seed_count)) || 1);
  cfg.sim.seed = seedA;
  cfg.sim.seed_to = count > 1 ? seedA + count - 1 : null;
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
  ui.area.value = cfg.building.floor_area_m2 == null ? "" : cfg.building.floor_area_m2;
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

  // 种子不归预设管，用户填的留着。预设切来切去不该把个数打回 1。
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
// 并行池。种子数多了就把范围切成几段，每个 Worker 各算一段，主线程合并。
// 合并函数在 merge.js，口径对照内核 batch.rs。工人源码在 worker.js。
// ---------------------------------------------------------------------------

// 64 个种子起用工人。node 实测单种子约 2.5 毫秒，64 个约 160 毫秒，
// 工人那次实例化加产物复制刚好与之打平，再少就亏了。
const PARALLEL_MIN_RUNS = 64;
// 8 个工人封顶，每段至少 8 个种子，摊掉一次调用的 JSON 开销。
const PARALLEL_MAX_WORKERS = 8;
const PARALLEL_MIN_CHUNK = 8;

// 池子只建一次，之后每轮只换种子区间。早先是每轮新建一批 Worker，每人重发
// 一份四百多 KB 的产物再重新实例化 WASM，拖一次滑块就付一遍这个钱。
// 工人协议变了就加一。浏览器会按 URL 缓存 worker.js，新前端配旧工人时
// 旧工人不回轮次号，主线程认不出结果只能干等，加个查询串强制换新的
const WORKER_VERSION = 2;
const pool = { workers: [], url: null, ready: false, seq: 0 };
const poolWaiters = new Map(); // 「轮次:片号」→ { worker, resolve, reject }
const poolInits = new Map(); // 工人 → { resolve, reject }

// 被顶掉和真出错是两回事。前者回退同步只会白算一遍旧配置，得分开
function cancelledError() {
  const e = new Error("这一轮被顶掉了");
  e.cancelled = true;
  return e;
}

function disposePool(reason) {
  for (const [key, it] of poolWaiters) {
    poolWaiters.delete(key);
    it.reject(reason || cancelledError());
  }
  poolWaiters.clear();
  for (const [w, it] of poolInits) {
    poolInits.delete(w);
    it.reject(reason || cancelledError());
  }
  for (const w of pool.workers) {
    try { w.terminate(); } catch { /* 已经停掉的不用管 */ }
  }
  if (pool.url) URL.revokeObjectURL(pool.url);
  pool.workers = [];
  pool.url = null;
  pool.ready = false;
}

// 一条消息要么是初始化握手，要么是某一轮某一片的结果。轮次号对不上就是
// 被顶掉的旧任务，结果直接丢，不能混进新一轮的合并里
function onWorkerMessage(worker, e) {
  const m = e.data || {};
  if (m.type === "ready" || m.type === "failed") {
    const it = poolInits.get(worker);
    if (!it) return;
    poolInits.delete(worker);
    if (m.type === "ready") it.resolve();
    else it.reject(new Error(m.message || "工人没启动"));
    return;
  }
  const key = m.seq + ":" + m.job;
  const it = poolWaiters.get(key);
  if (!it || it.worker !== worker) return;
  poolWaiters.delete(key);
  if (m.type === "result") it.resolve(m.res);
  else it.reject(new Error(m.message || "工人算错了"));
}

function failWorker(worker) {
  const err = new Error("工人报错，多半是当前环境建不起 Worker");
  const init = poolInits.get(worker);
  if (init) {
    poolInits.delete(worker);
    init.reject(err);
  }
  // 半死的池子留着只会反复失败，整只扔掉，下一轮重建
  disposePool(err);
}

// 建池中。两次 run() 先后脚进来时共用同一次建池，不然会造出两组工人。
let poolBuilding = null;

function ensurePool() {
  if (pool.ready) return Promise.resolve(pool);
  if (poolBuilding) return poolBuilding;
  poolBuilding = buildPool().finally(() => { poolBuilding = null; });
  return poolBuilding;
}

// 工人源码。单文件版由 embed.js 内嵌在 #tihu-worker-src 里，用 Blob 启动。
// 静态服务器下直接指 worker.js 的地址，省掉一次取源码和一次新建 Blob。
async function buildPool() {
  if (pool.ready) return pool;
  let url = null;
  const el = document.getElementById("tihu-worker-src");
  if (el && el.textContent.trim()) {
    url = URL.createObjectURL(new Blob([el.textContent], { type: "text/javascript" }));
  }
  const n = Math.max(1, Math.min(PARALLEL_MAX_WORKERS, navigator.hardwareConcurrency || 4));
  const workers = [];
  try {
    for (let i = 0; i < n; i++) {
      const w = url ? new Worker(url) : new Worker("worker.js?v=" + WORKER_VERSION);
      w.onmessage = (e) => onWorkerMessage(w, e);
      w.onerror = () => failWorker(w);
      workers.push(w);
    }
    // 任一工人就绪失败就整体放弃，不留半个池子
    await Promise.all(workers.map((w) => new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        poolInits.delete(w);
        reject(new Error("工人就绪超时"));
      }, 10000);
      poolInits.set(w, {
        resolve: () => { clearTimeout(timer); resolve(); },
        reject: (err) => { clearTimeout(timer); reject(err); },
      });
      w.postMessage({ type: "init", wasmBytes });
    })));
  } catch (e) {
    for (const w of workers) {
      try { w.terminate(); } catch { /* 同上 */ }
    }
    if (url) URL.revokeObjectURL(url);
    throw e;
  }
  Object.assign(pool, { workers, url, ready: true });
  return pool;
}

// 把 [from, to] 闭区间切成 n 段，尽量均分。
function splitRange(from, to, n) {
  const out = [];
  let cur = from;
  for (let i = 0; i < n; i++) {
    const len = Math.ceil((to - cur + 1) / (n - i));
    out.push([cur, cur + len - 1]);
    cur += len;
  }
  return out;
}

// 并行跑完一段种子再合并。正常交 { sim }，中途被新任务顶掉交
// { cancelled: true }，工人起不来或算错了就抛错，调用方回退同步。
async function runParallel(cfg, routerId, token, onProgress) {
  const from = cfg.sim.seed, to = cfg.sim.seed_to;
  const total = to - from + 1;
  if (typeof Worker === "undefined" || !wasmBytes) {
    throw new Error("并行条件不够，回退同步");
  }
  // 上一轮的分片还在算，工人单线程，新任务只能排在后面。拖滑块时旧结果
  // 已经没用，把池子换新的比回收旧任务快，也比旧版每轮重建快
  if (poolWaiters.size) disposePool();
  const p = await ensurePool();
  if (token !== runToken) return { cancelled: true };
  const n = Math.min(p.workers.length, Math.floor(total / PARALLEL_MIN_CHUNK));
  if (n <= 1) throw new Error("并行条件不够，回退同步");
  // 轮次号让旧任务的结果能被认出来。滑块一拖，上一轮派出去的片段还在路上，
  // 没有这个号就会把上一轮的数算进这一轮
  const seq = ++pool.seq;
  const ranges = splitRange(from, to, n);
  let done = 0;
  const jobs = [];
  for (let i = 0; i < n; i++) {
    const [a, b] = ranges[i];
    const job = Object.assign(clone(cfg), { sim: Object.assign({}, cfg.sim, { seed: a, seed_to: b }) });
    const part = new Promise((resolve, reject) => {
      poolWaiters.set(seq + ":" + i, { worker: p.workers[i], resolve, reject });
      p.workers[i].postMessage({ type: "run", seq, job: i, cfgJson: JSON.stringify(job), routerId });
    }).then((res) => {
      if (!res || !res.ok) {
        throw new Error(((res && res.issues && res.issues[0]) || {}).message || "分片没跑起来");
      }
      done += b - a + 1;
      if (token === runToken) onProgress(done, total);
      return res.sim;
    });
    jobs.push(part);
  }
  try {
    const partials = await Promise.all(jobs);
    if (token !== runToken) return { cancelled: true };
    return { sim: mergeReports(partials) };
  } catch (e) {
    // 被顶掉不算失败，回退同步只会白算一遍旧配置
    if (e && e.cancelled) return { cancelled: true };
    throw e;
  } finally {
    for (let i = 0; i < n; i++) poolWaiters.delete(seq + ":" + i);
  }
}

// ---------------------------------------------------------------------------
// 出结果
// ---------------------------------------------------------------------------

let runToken = 0;
let lastRouterId = 0;

// 同步跑一次。批量与单次都只调一次内核，聚合在内核侧完成。
// 正常交 { sim }，配置不合法交 { issues }，内核抛错就往上抛。
function syncRun(cfg, routerId) {
  const res = kernel.run(cfg, routerId);
  if (!res.ok) return { issues: res.issues };
  return { sim: res.sim, issues: res.issues };
}

// 把当前网址（含全部参数）拷进剪贴板。上下两组按钮共用，按下的那颗变字提示。
async function copyLink(e) {
  try {
    await navigator.clipboard.writeText(location.href);
    e.target.textContent = "已复制";
  } catch (err) {
    e.target.textContent = "复制失败";
  }
  setTimeout(() => (e.target.textContent = "复制配置链接"), 1200);
}

async function run() {
  const token = ++runToken;
  const cfg = applyUi(clone(base));
  lastRouterId = Number(ui.router.value);

  // 这里只按种子范围估个次数，用来先画一句提示，真正跑几个种子由内核说了算。
  const runs = cfg.sim.seed_to ? cfg.sim.seed_to - cfg.sim.seed + 1 : 1;
  // 超 1000 的范围不走并行：分片各自合法，会绕过内核的 seed_range 校验。
  // 回退同步让内核报错，用户看得到红条，而不是浏览器卡死几十秒
  const parallel = runs >= PARALLEL_MIN_RUNS && runs <= 1000;
  setNote(runs > 1 ? (parallel ? "正在并行跑 " + runs + " 个种子…" : "正在跑 " + runs + " 个种子…") : "");
  if (runs > 1) {
    // 先把这句话画到页面上，再开始算，不然页面会一直停在上一张图。
    await new Promise((r) => setTimeout(r, 0));
    if (token !== runToken) return;
  }

  const fail = (issues) => { setNote(""); showIssues(issues); };
  let sim;
  if (parallel) {
    try {
      const r = await runParallel(cfg, lastRouterId, token, (done, total) => {
        if (token === runToken) setNote("正在并行跑 " + done + " / " + total + " 个种子…");
      });
      if (token !== runToken || r.cancelled) return;
      sim = r.sim;
      showIssues([]);
    } catch (e) {
      // 工人起不来就回退同步，保证旧环境照样出数
      if (token !== runToken) return;
      setNote("并行失败，逐个重跑…");
      await new Promise((r) => setTimeout(r, 0));
      if (token !== runToken) return;
      try {
        const fb = syncRun(cfg, lastRouterId);
        if (!fb.sim) { fail(fb.issues); return; }
        showIssues(fb.issues);
        sim = fb.sim;
      } catch (e2) {
        fail([{ severity: "error", message: "仿真没能跑起来：" + e2.message }]);
        return;
      }
    }
  } else {
    try {
      const fb = syncRun(cfg, lastRouterId);
      if (!fb.sim) { fail(fb.issues); return; }
      showIssues(fb.issues);
      sim = fb.sim;
    } catch (e) {
      fail([{ severity: "error", message: "仿真没能跑起来：" + e.message }]);
      return;
    }
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
    .map((i) => `<li>${escHtml(i.message)}</li>`)
    .concat(warns.map((i) => `<li class="warn">${escHtml(i.message)}</li>`))
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
    box.innerHTML = barChartRangePair(
      floors.map((f) => ({
        label: f.label,
        waitMean: f.wait_mean, waitBest: f.wait_best, waitWorst: f.wait_worst,
        d2dMean: f.d2d_mean, d2dBest: f.d2d_best, d2dWorst: f.d2d_worst,
        live: f.live,
      })),
      { digits: 1 }
    );
  } else {
    box.innerHTML = barChartPair(
      floors.map((f) => ({
        label: f.label,
        wait: f.wait_mean, waitP50: f.wait_p50, waitP90: f.wait_p90,
        d2d: f.d2d_mean, d2dP50: f.d2d_p50, d2dP90: f.d2d_p90,
      })),
      { digits: 1 }
    );
  }

  const floorNote = document.getElementById("floor_note");
  floorNote.hidden = runs <= 1;
  floorNote.textContent = runs > 1
    ? "共跑 " + runs + " 次。每层两组：上为等梯，下为门到门。实线是各次的平均，浅色带是最短与最长的那两次。"
    : "";

  const histNote = document.getElementById("hist_note");
  histNote.hidden = runs <= 1;
  histNote.textContent = runs > 1 ? "柱高是 " + runs + " 次运行合计的人次。" : "";

  document.getElementById("chart_hist").innerHTML = histogram(s.wait_hist, s.wait_hist_bin_s);

  const declined = s.floors.reduce((a, f) => a + f.declined, 0);
  const noSpace = s.floors.reduce((a, f) => a + f.no_space, 0);
  const rows2 = [
    ["总人数 / 送达人数", s.pax_total + " / " + s.pax_served],
    ["送达比例", fmt(s.serve_ratio * 100, 1) + "%"],
    ["未完成行程", s.pax_unfinished + " 人"],
    ["走楼梯（含半路改走）", s.stair_users + "（" + fmt(s.stair_share * 100, 1) + "%）"],
    ["其中半路改走楼梯", s.abandoned + " 人"],
    ["按了外呼就走", s.pax_press_and_run + " 人"],
    ["被跳过：位子不够", noSpace + " 次"],
    ["被跳过：人不愿挤", declined + " 次"],
    ["白停一次", s.ghost_stops + " 次"],
    ["满员直接开过", s.full_bypasses + " 次"],
    ["太快停不住", s.overshoots + " 次"],
    ["跑一圈平均用时", fmt(s.rtt_mean) + " 秒"],
    ["超过一小时没算进平均的圈", s.rtt_dropped + " 圈"],
    ["每 5 分钟送走", fmt(s.throughput_5min) + " 人"],
    ["超载卡死", s.stalls + " 次，累计 " + fmt(s.stall_seconds) + " 秒"],
    ["超载被请下车", s.pax_after_stall + " 人"],
    ["因车卡住没人接的外呼", s.calls_orphaned + " 条"],
  ];
  document.getElementById("balance").innerHTML = rows2
    .map((r) => `<tr><td>${escHtml(r[0])}</td><td>${escHtml(r[1])}</td></tr>`)
    .join("");

  setNote(runs > 1
    ? "共跑 " + runs + " 个种子，从 " + s.seed_from + " 到 " + s.seed_to +
      "。秒数和比例是各次的平均值，样本数也是平均值。"
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
    .concat(presets.map((p) => `<option value="${escHtml(p.name)}">${escHtml(p.label)}</option>`))
    .join("");
  const routers = kernel.config(2);
  ui.router.innerHTML = routers
    .map((r, i) => `<option value="${i}">${escHtml(r.label)}</option>`)
    .join("");
  document.getElementById("router_list").innerHTML = routers
    .map((r) => `<li><b>${escHtml(r.label)}</b><span>${escHtml(r.desc)}</span></li>`)
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
  for (const id of LIVE_IDS) {
    ui[id].addEventListener("input", () => { syncLabels(); kick(); });
    ui[id].addEventListener("change", () => { syncLabels(); kick(); });
  }

  document.querySelectorAll('[data-act="run"]').forEach((b) => b.addEventListener("click", () => run()));
  document.querySelectorAll('[data-act="link"]').forEach((b) => b.addEventListener("click", copyLink));

  loadIntoUi(base);

  const hash = location.hash;
  if (hash.startsWith("#s=")) {
    if (fromHash(hash.slice(3))) {
      // 分享链接连预设名一起存了。base 要按它重建，否则预设里那些不在
      // 控件上的字段，比如每股客流和策略细分，会悄悄落回默认配置
      const idx = presets.findIndex((p) => p.name === ui.preset.value);
      if (idx >= 0) base = kernel.config(3 + idx);
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
