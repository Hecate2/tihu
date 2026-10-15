// 手写 SVG 图表。不引任何图表库，理由见 README：网页要在断网的 file:// 下双击可用，
// 外部脚本一律不能有。
//
// 每个函数返回一段 SVG 字符串，调用方塞进 innerHTML 即可。

function esc(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

// 数值到颜色的映射：绿 → 黄 → 红。用于一眼看出哪层最堵。
function heat(t) {
  const x = Math.max(0, Math.min(1, t));
  const stops = [
    [0.0, [62, 207, 142]],
    [0.5, [240, 180, 41]],
    [1.0, [255, 107, 107]],
  ];
  let a = stops[0], b = stops[stops.length - 1];
  for (let i = 0; i < stops.length - 1; i++) {
    if (x >= stops[i][0] && x <= stops[i + 1][0]) { a = stops[i]; b = stops[i + 1]; break; }
  }
  const k = b[0] === a[0] ? 0 : (x - a[0]) / (b[0] - a[0]);
  const c = a[1].map((v, i) => Math.round(v + (b[1][i] - v) * k));
  return `rgb(${c[0]},${c[1]},${c[2]})`;
}

/**
 * 水平条形图，用来画「每一层等多久」。
 * rows: [{ label, value }]，value 为 null 时画成空。
 */
function barChartH(rows, opts) {
  const o = Object.assign({ rowH: 19, barW: 240, labelW: 46, unit: "s", digits: 1, max: null }, opts);
  const max = o.max || Math.max(1e-6, ...rows.map((r) => r.value || 0));
  const w = o.labelW + o.barW + 52;
  const h = rows.length * o.rowH + 22;
  let s = `<svg width="${w}" height="${h}" viewBox="0 0 ${w} ${h}" role="img">`;

  // 三条竖向参考线，让不同图之间能对比
  for (const f of [0.25, 0.5, 0.75, 1]) {
    const x = o.labelW + f * o.barW;
    s += `<line x1="${x}" y1="16" x2="${x}" y2="${h - 6}" stroke="#2a323e" stroke-width="1" stroke-dasharray="${f === 1 ? "0" : "3 3"}"/>`;
  }
  s += `<text x="0" y="10" fill="#97a3b4" font-size="10">平均等待，秒</text>`;
  s += `<text x="${o.labelW + o.barW}" y="10" fill="#97a3b4" font-size="10" text-anchor="end">${max.toFixed(0)}</text>`;

  rows.forEach((r, i) => {
    const y = 18 + i * o.rowH;
    const v = r.value || 0;
    const bw = Math.max(v > 0 ? 1.5 : 0, (v / max) * o.barW);
    const hot = v / max;
    s += `<text x="${o.labelW - 6}" y="${y + 12}" fill="#97a3b4" font-size="11" text-anchor="end">${esc(r.label)}</text>`;
    s += `<rect x="${o.labelW}" y="${y + 2}" width="${o.barW}" height="${o.rowH - 7}" fill="#1e242e" rx="2"/>`;
    if (bw > 0) {
      s += `<rect x="${o.labelW}" y="${y + 2}" width="${bw.toFixed(1)}" height="${o.rowH - 7}" fill="${heat(hot)}" rx="2"/>`;
    }
    s += `<text x="${o.labelW + o.barW + 6}" y="${y + 12}" fill="#e6ebf2" font-size="11" font-variant-numeric="tabular-nums">${v.toFixed(o.digits)}</text>`;
  });
  return s + "</svg>";
}

/**
 * 竖向柱状图，用来画等待时间的分布。
 * bins: 每格的样本数；binS: 格宽（秒）。
 */
function histogram(bins, binS, opts) {
  const o = Object.assign({ w: 460, h: 150, unit: "s" }, opts);
  const padL = 6, padB = 22, padT = 8, padR = 6;
  const innerW = o.w - padL - padR;
  const innerH = o.h - padT - padB;
  const max = Math.max(1, ...bins);
  const bw = innerW / bins.length;
  let s = `<svg width="${o.w}" height="${o.h}" viewBox="0 0 ${o.w} ${o.h}" role="img">`;
  s += `<line x1="${padL}" y1="${padT + innerH}" x2="${padL + innerW}" y2="${padT + innerH}" stroke="#2a323e"/>`;
  bins.forEach((c, i) => {
    const bh = (c / max) * innerH;
    const x = padL + i * bw;
    if (bh > 0) {
      s += `<rect x="${(x + 0.6).toFixed(2)}" y="${(padT + innerH - bh).toFixed(2)}" width="${Math.max(1, bw - 1.2).toFixed(2)}" height="${bh.toFixed(2)}" fill="#4aa3ff" opacity="0.85" rx="1"/>`;
    }
  });
  // 横轴刻度，每 60 秒一个
  for (let t = 0; t <= bins.length * binS; t += 60) {
    const x = padL + (t / (bins.length * binS)) * innerW;
    s += `<text x="${x}" y="${o.h - 7}" fill="#97a3b4" font-size="10" text-anchor="middle">${t}</text>`;
  }
  s += `<text x="${padL}" y="${padT}" fill="#97a3b4" font-size="10">人数</text>`;
  s += `<text x="${padL + innerW}" y="${padT}" fill="#97a3b4" font-size="10" text-anchor="end">峰值 ${max}</text>`;
  return s + "</svg>";
}
