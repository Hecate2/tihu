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
 * 双指标图的共用件：竖向参考线。
 * `top` 是第一行数据顶上的 Y，图例占掉标题下面那一行。
 */
function pairGrid(o, h, top) {
  let s = "";
  for (const f of [0.25, 0.5, 0.75, 1]) {
    const gx = o.labelW + f * o.barW;
    s += `<line x1="${gx}" y1="${top}" x2="${gx}" y2="${h - 6}" stroke="#2a323e" stroke-width="1" stroke-dasharray="${f === 1 ? "0" : "3 3"}"/>`;
  }
  return s;
}

/**
 * 双指标图的图例，画在标题下面那一行。
 * 等梯走绿黄红，门到门走蓝色。细竖线是 P50，粗竖线是 P90。
 */
function pairLegend(o, y) {
  let s = `<rect x="${o.labelW}" y="${y}" width="10" height="8" fill="${heat(0.15)}" rx="2"/>`;
  s += `<text x="${o.labelW + 14}" y="${y + 8}" fill="#97a3b4" font-size="10">等梯均值</text>`;
  s += `<rect x="${o.labelW + 82}" y="${y}" width="10" height="8" fill="#4aa3ff" opacity="0.85" rx="2"/>`;
  s += `<text x="${o.labelW + 96}" y="${y + 8}" fill="#97a3b4" font-size="10">门到门均值</text>`;
  s += `<line x1="${o.labelW + 178}" y1="${y}" x2="${o.labelW + 178}" y2="${y + 8}" stroke="#fff" stroke-width="1" opacity="0.55"/>`;
  s += `<line x1="${o.labelW + 184}" y1="${y}" x2="${o.labelW + 184}" y2="${y + 8}" stroke="#fff" stroke-width="2.5"/>`;
  s += `<text x="${o.labelW + 190}" y="${y + 8}" fill="#97a3b4" font-size="10">细线 P50，粗线 P90</text>`;
  return s;
}

/**
 * 双条形图，用来画「每一层等多久，门到门多久」。
 * rows: [{ label, wait, waitP50, waitP90, d2d, d2dP50, d2dP90 }]。
 * 每层两根条：上为等梯均值，下为门到门均值。条上的细竖线是 P50，
 * 粗竖线是 P90。两根条共用一把刻度，颜色各自按本指标的最大值归一，
 * 免得等梯那根被门到门压得全绿。
 */
function barChartPair(rows, opts) {
  const o = Object.assign({ rowH: 30, barW: 240, labelW: 46, digits: 1, max: null }, opts);
  const max = o.max || Math.max(1e-6, ...rows.map((r) => Math.max(r.d2dP90 || 0, r.waitP90 || 0)));
  const maxWait = Math.max(1e-6, ...rows.map((r) => r.wait || 0));
  const top = 36;
  const w = o.labelW + o.barW + 104;
  const h = top + rows.length * o.rowH + 6;
  const x = (v) => o.labelW + Math.max(0, Math.min(1, v / max)) * o.barW;
  let s = `<svg width="${w}" height="${h}" viewBox="0 0 ${w} ${h}" role="img">`;
  s += pairGrid(o, h, top);
  s += `<text x="0" y="10" fill="#97a3b4" font-size="10">等梯与门到门，秒</text>`;
  s += `<text x="${o.labelW + o.barW}" y="10" fill="#97a3b4" font-size="10" text-anchor="end">${max.toFixed(0)}</text>`;
  s += pairLegend(o, 18);

  // 一行两根条：等梯在上，门到门在下。分位竖线只画在各自那根条的高度里。
  const tick = (v, y1, y2, wide) =>
    `<line x1="${x(v).toFixed(1)}" y1="${y1}" x2="${x(v).toFixed(1)}" y2="${y2}" stroke="#fff" stroke-width="${wide ? "2.5" : "1"}" opacity="${wide ? "1" : "0.55"}"/>`;
  rows.forEach((r, i) => {
    const y = top + i * o.rowH;
    const wv = r.wait || 0, dv = r.d2d || 0;
    const wb = Math.max(wv > 0 ? 1.5 : 0, (wv / max) * o.barW);
    const db = Math.max(dv > 0 ? 1.5 : 0, (dv / max) * o.barW);
    const tip =
      r.label + "：等梯均值 " + wv.toFixed(o.digits) + " 秒，P50 " + (r.waitP50 || 0).toFixed(o.digits) +
      "，P90 " + (r.waitP90 || 0).toFixed(o.digits) + "。门到门均值 " + dv.toFixed(o.digits) +
      " 秒，P50 " + (r.d2dP50 || 0).toFixed(o.digits) + "，P90 " + (r.d2dP90 || 0).toFixed(o.digits);
    s += `<g><title>${esc(tip)}</title>`;
    s += `<text x="${o.labelW - 6}" y="${y + 19}" fill="#97a3b4" font-size="11" text-anchor="end">${esc(r.label)}</text>`;
    s += `<rect x="${o.labelW}" y="${y + 2}" width="${o.barW}" height="${o.rowH - 6}" fill="#1e242e" rx="2"/>`;
    if (wb > 0) {
      s += `<rect x="${o.labelW}" y="${y + 4}" width="${wb.toFixed(1)}" height="8" fill="${heat(wv / maxWait)}" rx="2"/>`;
    }
    s += tick(r.waitP50 || 0, y + 4, y + 12, false);
    s += tick(r.waitP90 || 0, y + 4, y + 12, true);
    if (db > 0) {
      s += `<rect x="${o.labelW}" y="${y + 16}" width="${db.toFixed(1)}" height="8" fill="#4aa3ff" opacity="0.85" rx="2"/>`;
    }
    s += tick(r.d2dP50 || 0, y + 16, y + 24, false);
    s += tick(r.d2dP90 || 0, y + 16, y + 24, true);
    s += `<text x="${o.labelW + o.barW + 6}" y="${y + 19}" fill="#e6ebf2" font-size="11" font-variant-numeric="tabular-nums">${wv.toFixed(o.digits)} / ${dv.toFixed(o.digits)}</text>`;
    s += `</g>`;
  });
  return s + "</svg>";
}

/**
 * 带区间的双条形图，用来画多次运行后的「每层等多久，门到门多久」。
 * rows: [{ label, waitMean, waitBest, waitWorst, d2dMean, d2dBest, d2dWorst, live }]。
 * best 与 worst 是各次运行里该层均值的最短与最长，live 是有乘客从该层出发的次数。
 * 每层两组：上为等梯，下为门到门。浅色带是最短与最长的两次，实线是各次的平均。
 */
function barChartRangePair(rows, opts) {
  const o = Object.assign({ rowH: 32, barW: 240, labelW: 46, digits: 1, max: null }, opts);
  const max = o.max || Math.max(1e-6, ...rows.map((r) => r.d2dWorst || 0));
  const maxWait = Math.max(1e-6, ...rows.map((r) => r.waitWorst || 0));
  const top = 36;
  const w = o.labelW + o.barW + 104;
  const h = top + rows.length * o.rowH + 6;
  const x = (v) => o.labelW + Math.max(0, Math.min(1, v / max)) * o.barW;
  let s = `<svg width="${w}" height="${h}" viewBox="0 0 ${w} ${h}" role="img">`;
  s += pairGrid(o, h, top);
  s += `<text x="0" y="10" fill="#97a3b4" font-size="10">等梯与门到门，秒</text>`;
  s += `<text x="${o.labelW + o.barW}" y="10" fill="#97a3b4" font-size="10" text-anchor="end">${max.toFixed(0)}</text>`;
  s += pairLegend(o, 18);

  const lane = (mean, best, worst, y1, color, bandColor) => {
    const m = mean || 0;
    const b = Math.max(0, Math.min(best == null ? m : best, m));
    const t = Math.max(m, worst == null ? m : worst);
    let g = "";
    const band = x(t) - x(b);
    if (band > 0.5) {
      g += `<rect x="${x(b).toFixed(1)}" y="${y1}" width="${band.toFixed(1)}" height="9" fill="${bandColor}" opacity="0.22" rx="2"/>`;
    }
    g += `<line x1="${x(m).toFixed(1)}" y1="${y1}" x2="${x(m).toFixed(1)}" y2="${y1 + 9}" stroke="${color}" stroke-width="3"/>`;
    return { g, b: b, t: t, m: m };
  };
  rows.forEach((r, i) => {
    const y = top + i * o.rowH;
    const waitColor = heat(maxWait > 0 ? (r.waitMean || 0) / maxWait : 0);
    const wm = lane(r.waitMean, r.waitBest, r.waitWorst, y + 3, waitColor, waitColor);
    const dm = lane(r.d2dMean, r.d2dBest, r.d2dWorst, y + 16, "#4aa3ff", "#4aa3ff");
    const tip =
      r.label + "：等梯平均 " + wm.m.toFixed(o.digits) + " 秒，最短 " + wm.b.toFixed(o.digits) +
      "，最长 " + wm.t.toFixed(o.digits) + "。门到门平均 " + dm.m.toFixed(o.digits) + " 秒，最短 " +
      dm.b.toFixed(o.digits) + "，最长 " + dm.t.toFixed(o.digits) + "，" + (r.live || 0) + " 次运行有人从这里出发";
    s += `<g><title>${esc(tip)}</title>`;
    s += `<text x="${o.labelW - 6}" y="${y + 20}" fill="#97a3b4" font-size="11" text-anchor="end">${esc(r.label)}</text>`;
    s += `<rect x="${o.labelW}" y="${y + 2}" width="${o.barW}" height="${o.rowH - 6}" fill="#1e242e" rx="2"/>`;
    s += wm.g + dm.g;
    s += `<text x="${o.labelW + o.barW + 6}" y="${y + 20}" fill="#e6ebf2" font-size="11" font-variant-numeric="tabular-nums">${wm.m.toFixed(o.digits)} / ${dm.m.toFixed(o.digits)}</text>`;
    s += `</g>`;
  });
  return s + "</svg>";
}

/**
 * 竖向柱状图，用来画等待时间的分布。
 * bins 是每格样本数，binS 是格宽秒数。
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
