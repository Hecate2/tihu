// 把 web/ 的四个文件与 WASM 产物压成一个能双击打开的 HTML。
//
// 为什么不用 fetch 加载 .wasm：file:// 协议下 fetch 会被浏览器拦掉，
// instantiateStreaming 还会挑剔 MIME。内嵌成 base64 走 WebAssembly.instantiate
// 是唯一在断网 file:// 下稳当的办法。代价是产物大了三分之一，换的是零依赖。
//
// 用法：node tools/embed.js  →  dist/tihu.html

const fs = require("fs");
const path = require("path");

const root = path.join(__dirname, "..");
const webDir = path.join(root, "web");
const distDir = path.join(root, "dist");

const read = (p) => fs.readFileSync(p, "utf8");

let html = read(path.join(webDir, "index.html"));
const css = read(path.join(webDir, "styles.css"));
const charts = read(path.join(webDir, "charts.js"));
const app = read(path.join(webDir, "app.js"));
const wasm = fs.readFileSync(path.join(webDir, "tihu.wasm")).toString("base64");

const swaps = [
  ['<link rel="stylesheet" href="styles.css">', `<style>\n${css}\n</style>`],
  ['<script src="charts.js"></script>', `<script>\n${charts}\n</script>`],
  [
    '<script src="app.js"></script>',
    `<script>window.TIHU_WASM_BASE64 = "${wasm}";</script>\n<script>\n${app}\n</script>`,
  ],
];

for (const [from, to] of swaps) {
  if (!html.includes(from)) {
    console.error("embed 失败：在 index.html 里找不到这一行\n  " + from);
    process.exit(1);
  }
  html = html.replace(from, () => to);
}

if (html.includes('src="app.js"') || html.includes('href="styles.css"')) {
  console.error("embed 失败：还有没内联进去的外部引用");
  process.exit(1);
}

fs.mkdirSync(distDir, { recursive: true });
const out = path.join(distDir, "tihu.html");
fs.writeFileSync(out, html);
const kb = (n) => (n / 1024).toFixed(0) + " KB";
console.log(
  `写好了 ${path.relative(root, out)}：${kb(Buffer.byteLength(html))}` +
    `（其中 WASM 内嵌 ${kb(wasm.length)}）`
);
