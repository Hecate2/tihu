// 并行池的工人：每个工人自带一份 WASM 实例，算一段种子。
//
// 启动方式两种，见 docs/ARCHITECTURE.md 第 8 节：静态服务器下 app.js 按
// worker.js 文件名启动，单文件版把本文件内嵌成文本再以 Blob 启动，
// file:// 下双击走后一条。两种都不成就回退主线程同步，见 app.js。
//
// 本文件不碰 DOM，只用 onmessage、postMessage 与 WebAssembly。
// 注意：正文里不许出现脚本结束标记的字面写法，否则单文件版内嵌时会被提前截断。

"use strict";

let ex = null; // 本工人的 WASM 导出表

// 与 app.js 同一份「长度前缀 + UTF-8」胶水。工人侧自带，不依赖主线程。
function take(ptr) {
  if (ptr === 0) throw new Error("内核交回了空指针");
  const view = new DataView(ex.memory.buffer);
  const len = view.getUint32(ptr, true);
  const mem = new Uint8Array(ex.memory.buffer);
  const text = new TextDecoder().decode(mem.slice(ptr + 4, ptr + 4 + len));
  ex.tihu_free(ptr, len + 4);
  return text;
}

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

onmessage = async (e) => {
  const m = e.data || {};
  if (m.type === "init") {
    try {
      // 主线程一律发字节。单文件版走 base64 的是主线程那一侧，不是这里
      const { instance } = await WebAssembly.instantiate(m.wasmBytes, {});
      ex = instance.exports;
      postMessage({ type: "ready" });
    } catch (err) {
      postMessage({ type: "failed", message: String((err && err.message) || err) });
    }
    return;
  }
  if (m.type === "run") {
    // seq 与 job 一起原样带回，主线程靠它认出这是哪一轮哪一片
    if (!ex) {
      postMessage({ type: "error", seq: m.seq, job: m.job, message: "WASM 还没就绪" });
      return;
    }
    try {
      const res = JSON.parse(pass(m.cfgJson, (p, n) => take(ex.tihu_run(p, n, m.routerId))));
      postMessage({ type: "result", seq: m.seq, job: m.job, res });
    } catch (err) {
      postMessage({ type: "error", seq: m.seq, job: m.job, message: String((err && err.message) || err) });
    }
  }
};
