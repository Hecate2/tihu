// WASM 绑定的冒烟测试。
//
// 只做四件事，任何一件失败就说明 ABI 契约破了：
//
//   1. 裸加载产物，不许有它没有的导入项
//   2. 连跑两次同一份配置，报告必须逐字节相同
//   3. 每次取回缓冲区都要 free，跑完之后 WASM 线性内存不许一直涨
//   4. 故意喂坏 JSON，必须走 JSON 报错而不是 trap
//
// 用法：node tools/smoke.js [wasm 路径]

const fs = require("fs");
const path = require("path");

const wasmPath =
  process.argv[2] ||
  path.join(__dirname, "..", "target", "wasm32-unknown-unknown", "release-wasm", "tihu_wasm.wasm");

function fail(msg) {
  console.error("冒烟失败：" + msg);
  process.exit(1);
}

// 读出「4 字节小端长度 + 负载」的缓冲区，然后立刻释放。
function take(inst, ptr) {
  if (ptr === 0) fail("拿到的缓冲区指针是空");
  const mem = new Uint8Array(inst.exports.memory.buffer);
  const view = new DataView(inst.exports.memory.buffer);
  const len = view.getUint32(ptr, true);
  if (len > mem.byteLength) fail("长度前缀 " + len + " 超出了 WASM 线性内存");
  const bytes = mem.slice(ptr + 4, ptr + 4 + len);
  const text = Buffer.from(bytes).toString("utf8");
  inst.exports.tihu_free(ptr, len + 4);
  return text;
}

// 把字符串写进 WASM 线性内存，交给 call 回调，用完释放。
function pass(inst, text, call) {
  const bytes = Buffer.from(text, "utf8");
  const ptr = inst.exports.tihu_alloc(bytes.length);
  if (ptr === 0 && bytes.length > 0) fail("tihu_alloc 返回了空指针");
  new Uint8Array(inst.exports.memory.buffer).set(bytes, ptr);
  try {
    return call(ptr, bytes.length);
  } finally {
    inst.exports.tihu_free(ptr, bytes.length);
  }
}

async function main() {
  const module = await WebAssembly.compile(fs.readFileSync(wasmPath));
  const imports = WebAssembly.Module.imports(module);
  if (imports.length > 0) {
    fail(
      "产物带上了导入项，静态加载会找不到它们：" +
        imports.map((i) => i.module + "." + i.name).join(", ")
    );
  }
  const inst = await WebAssembly.instantiate(module, {});
  const e = inst.exports;
  for (const name of ["memory", "tihu_alloc", "tihu_free", "tihu_run", "tihu_config"]) {
    if (!(name in e)) fail("产物没有导出 " + name);
  }

  const config = take(inst, e.tihu_config(0));
  if (!config.includes("\"building\"")) fail("默认配置 JSON 看起来不对");
  console.log("默认配置 " + config.length + " 字节");

  const presets = JSON.parse(take(inst, e.tihu_config(1)));
  console.log("预设 " + presets.map((p) => p.name).join("、")); 

  const before = e.memory.buffer.byteLength;
  const first = pass(inst, config, (p, n) => JSON.parse(take(inst, e.tihu_run(p, n, 0))));
  if (!first.ok) fail("默认配置跑不起来：" + JSON.stringify(first.issues));
  console.log(
    "一趟跑完：送达 " +
      first.sim.pax_served +
      " 人，平均等 " +
      first.sim.wait.mean.toFixed(1) +
      " 秒，P90 " +
      first.sim.wait.p90.toFixed(1) +
      " 秒，层数 " +
      first.sim.floors.length
  );

  // 连跑 30 次，检查内存不涨。漏 free 的话这里会线性上涨。
  const second = pass(inst, config, (p, n) => JSON.parse(take(inst, e.tihu_run(p, n, 0))));
  if (JSON.stringify(first) !== JSON.stringify(second)) fail("同一份配置跑两次结果不同");
  for (let i = 0; i < 28; i++) {
    pass(inst, config, (p, n) => {
      take(inst, e.tihu_run(p, n, 0));
    });
  }
  const after = e.memory.buffer.byteLength;
  if (after > before * 2) fail("跑三十次之后 WASM 内存涨到了两倍以上，八成漏了 free");
  console.log("内存 " + before + " → " + after + " 字节，三十次调用没漏");

  const bad = pass(inst, "{ 这不是 json", (p, n) => JSON.parse(take(inst, e.tihu_run(p, n, 0))));
  if (bad.ok) fail("坏 JSON 不该返回 ok");
  console.log("坏 JSON 走的是 JSON 报错：" + bad.issues[0].code);

  console.log("冒烟通过");
}

main().catch((err) => fail(err.stack || String(err)));
