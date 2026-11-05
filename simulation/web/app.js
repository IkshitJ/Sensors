'use strict';
// ---------------------------------------------------------------------------
//  Zebra crossing simulator UI. All physics and the Arduino firmware run in the
//  native simulator process; this page only renders frames and sends commands.
//
//  Layout: the sensor strip lies on the zebra crossing and spans both lanes
//  (14 pads x 0.5 m across the road, 1 m along it). People and vehicles move
//  along the lanes: lane A (bottom, y > 0) towards the right, lane B towards
//  the left.
// ---------------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const NUM_PADS = 14, PER_LANE = 7, PAD_W = 0.5;
const STRIP_X0 = 1.5, STRIP_X1 = 2.5, STRIP_Y0 = -3.5;
const KIND_COLOR = ['#4589ff', '#33b1ff', '#ff7eb6', '#be95ff', '#42be65'];
const VEH_COLOR = { car: '#e0e0e0', suv: '#a8a8a8', auto: '#f1c21b', moto: '#ff832b', bicycle: '#08bdba', bus: '#d12771', truck: '#8a3ffc' };
const SPEEDS = [0.25, 0.5, 1, 2, 4, 8, 16, 30];
const PRESETS = { // mirrors presetRates() in world.cpp: people/min, group, vehicles/min, stop, wait
  idle: [0, 0, 0, 0, 0], sparse: [4, 0, 0, 0, 0], normal: [14, .25, 4, .1, 0], rush: [36, .35, 3, .1, 0],
  dense: [75, .4, 0, 0, 0], festival: [140, .5, 0, 0, 0], signal: [0, 0, 6, 0, 0], traffic: [0, 0, 24, .08, 0],
  mixed: [18, .3, 10, .2, 0], waiting: [12, .2, 2, 0, .3],
};
const QUICK = { v2: ['r', 't', 's', 'c', 'd', 'h'], cal: ['t', 'k20', 'n', 'p', 'a', 'h'] };

let frame = null;
let fw = 'v2';
let history = [];
let lastHistT = -1;
let serialLines = [];
let serialPartial = '';

// ------------------------------------------------------------------ server --
function cmd(c) { return fetch('/api/cmd', { method: 'POST', body: c }); }
function markFirmware(name) {
  fw = name;
  document.querySelectorAll('.fw-switch button').forEach((b) => b.classList.toggle('on', b.dataset.fw === name));
  $('calCard').hidden = name !== 'cal';
  renderQuick();
}
function setFirmware(name, scenario) {
  markFirmware(name);
  history = []; lastHistT = -1; serialLines = []; serialPartial = ''; $('serial').textContent = '';
  return fetch('/api/firmware', { method: 'POST', body: JSON.stringify({ fw: name, scenario: scenario || $('scenario').value }) });
}

function connect() {
  const es = new EventSource('/api/stream');
  es.onopen = () => { $('conn').textContent = 'live'; $('conn').classList.add('live'); };
  es.onerror = () => { $('conn').textContent = 'reconnecting…'; $('conn').classList.remove('live'); };
  es.onmessage = (ev) => {
    let f;
    try { f = JSON.parse(ev.data); } catch { return; }
    if (frame && f.t < frame.t) { history = []; lastHistT = -1; }
    frame = f;
    if (f.fw !== fw) markFirmware(f.fw);
    if (f.serial) appendSerial(f.serial);
    if (f.t - lastHistT >= 0.25) { history.push({ t: f.t, truth: f.truth, count: Math.max(0, f.count) }); lastHistT = f.t; if (history.length > 2000) history.shift(); }
    updateStats(f);
  };
}

// ------------------------------------------------------------------- stats --
function updateStats(f) {
  const cal = fw === 'cal';
  const c = f.count < 0 ? 0 : f.count;
  $('sCount').textContent = cal ? '–' : c;
  $('sLaneA').textContent = cal ? '–' : f.laneA;
  $('sLaneB').textContent = cal ? '–' : f.laneB;
  $('sLive').textContent = cal ? 'calibration sketch running: no counting' : (f.live >= 0 ? `live ≈ ${f.live.toFixed(1)} including people on the strip now` : '');
  $('sTruth').textContent = f.truth;
  $('sTruthNote').textContent = `→ ${f.truthA}   ← ${f.truthB}`;
  if (f.truth > 0 && !cal) {
    const acc = 1 - Math.abs(c - f.truth) / f.truth;
    $('sAcc').textContent = (acc * 100).toFixed(1) + '%';
    $('sAcc').className = acc >= 0.9 ? 'acc-good' : acc >= 0.7 ? 'acc-mid' : 'acc-bad';
    $('sAccNote').textContent = 'firmware reports ~0.6 s after a person steps off';
  } else { $('sAcc').textContent = '–'; $('sAcc').className = ''; $('sAccNote').textContent = ''; }
  $('sVeh').textContent = f.vehOver;
  $('sVehNote').textContent = f.veh ? 'vehicle mask ACTIVE on the red pads' : `${f.vehTotal} vehicles spawned; tyres are masked, never counted`;
  $('pauseBtn').textContent = f.paused ? 'Resume' : 'Pause';
}

// ------------------------------------------------------------------ serial --
function appendSerial(text) {
  const parts = (serialPartial + text).split('\n');
  serialPartial = parts.pop();
  const show = $('showTelemetry').checked;
  for (const l of parts) {
    if (!show && l.startsWith('@T,')) continue;
    serialLines.push(l);
  }
  if (serialLines.length > 1500) serialLines.splice(0, serialLines.length - 1500);
  const el = $('serial');
  const atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 30;
  el.textContent = serialLines.join('\n');
  if (atBottom) el.scrollTop = el.scrollHeight;
}
function renderQuick() {
  const q = $('quickCmds');
  q.innerHTML = '';
  for (const c of QUICK[fw] || []) {
    const b = document.createElement('button');
    b.type = 'button'; b.textContent = c;
    b.onclick = () => cmd('serial ' + c);
    q.appendChild(b);
  }
}

// ------------------------------------------------------------------- scene --
function fitCanvas(cv) {
  const r = cv.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(10, Math.round(r.width * dpr)), h = Math.max(10, Math.round(r.height * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  return { w, h, dpr };
}
function heat(kg) {
  if (kg > 250) return '#da1e28';
  const t = Math.min(1, Math.max(0, kg / 160));
  const a = [43, 58, 74], b = [241, 194, 27], c = [255, 131, 43];
  const mix = (p, q, s) => p.map((v, i) => Math.round(v + (q[i] - v) * s));
  const col = t < 0.5 ? mix(a, b, t * 2) : mix(b, c, (t - 0.5) * 2);
  return `rgb(${col[0]},${col[1]},${col[2]})`;
}
function roundRect(g, x, y, w, h, r) {
  r = Math.min(r, w / 2, h / 2);
  g.beginPath(); g.moveTo(x + r, y); g.arcTo(x + w, y, x + w, y + h, r); g.arcTo(x + w, y + h, x, y + h, r);
  g.arcTo(x, y + h, x, y, r); g.arcTo(x, y, x + w, y, r); g.closePath();
}

function drawScene() {
  const cv = $('scene');
  const { w, h, dpr } = fitCanvas(cv);
  const g = cv.getContext('2d');
  const css = getComputedStyle(document.documentElement);
  const C = (n) => css.getPropertyValue(n).trim();
  const s = h / 12.6;                     // px per metre
  const cx = w / 2 - 2 * s, cy = h / 2;   // world (2, 0) in the centre
  const X = (x) => cx + x * s, Y = (y) => cy + y * s;
  g.clearRect(0, 0, w, h);
  // pavements and road
  g.fillStyle = C('--walk'); g.fillRect(0, 0, w, h);
  g.fillStyle = C('--road'); g.fillRect(0, Y(-3.5), w, 7 * s);
  g.fillStyle = C('--road-edge'); g.fillRect(0, Y(-3.5) - 3 * dpr, w, 3 * dpr); g.fillRect(0, Y(3.5), w, 3 * dpr);
  // lane divider
  g.strokeStyle = 'rgba(255,255,255,.5)'; g.lineWidth = 2 * dpr; g.setLineDash([3 * s, 3 * s]);
  g.beginPath(); g.moveTo(0, Y(0)); g.lineTo(w, Y(0)); g.stroke(); g.setLineDash([]);
  // zebra crossing: white bars across both lanes
  g.fillStyle = C('--stripe');
  for (let y = -3.3; y < 3.3; y += 0.9) g.fillRect(X(0), Y(y), 4 * s, 0.45 * s);
  // lane direction labels
  g.fillStyle = 'rgba(255,255,255,.4)'; g.font = `${0.55 * s}px system-ui`; g.textAlign = 'left';
  g.fillText('Lane A  →', X(-10.5), Y(2.0)); g.fillText('←  Lane B', X(10), Y(-1.5));

  // sensor strip: 14 pads stacked across the road
  const f = frame;
  const mask = f && f.fw === 'v2' ? f.mask : 0;
  const fs = Math.max(9 * dpr, 0.2 * s);
  for (let i = 0; i < NUM_PADS; i++) {
    const kg = f ? f.pads[i] : 0;
    const y0 = STRIP_Y0 + i * PAD_W;
    g.globalAlpha = 0.9;
    g.fillStyle = heat(kg);
    g.fillRect(X(STRIP_X0), Y(y0) + 1, (STRIP_X1 - STRIP_X0) * s, PAD_W * s - 2);
    g.globalAlpha = 1;
    if (mask & (1 << i)) {
      g.save(); g.beginPath(); g.rect(X(STRIP_X0), Y(y0) + 1, s, PAD_W * s - 2); g.clip();
      g.strokeStyle = '#fa4d56'; g.lineWidth = 2 * dpr;
      for (let k = -s; k < 2 * s; k += 6 * dpr) { g.beginPath(); g.moveTo(X(STRIP_X0) + k, Y(y0)); g.lineTo(X(STRIP_X0) + k + PAD_W * s, Y(y0 + PAD_W)); g.stroke(); }
      g.restore();
    }
    g.fillStyle = kg > 90 ? '#111' : '#fff';
    g.font = `${fs}px ui-monospace, Menlo, monospace`;
    g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(kg >= 1 ? Math.round(kg) : '·', X((STRIP_X0 + STRIP_X1) / 2), Y(y0 + PAD_W / 2));
    g.fillStyle = 'rgba(255,255,255,.75)'; g.textAlign = 'right';
    g.fillText('P' + i, X(STRIP_X0) - 4 * dpr, Y(y0 + PAD_W / 2));
  }
  g.textAlign = 'left'; g.textBaseline = 'alphabetic';
  g.strokeStyle = '#ffffff'; g.lineWidth = 1.5 * dpr; g.strokeRect(X(STRIP_X0), Y(-3.5), s, 7 * s);
  if (f) {
    // static calibration weights
    for (const [pad, kg] of f.weights) {
      const y0 = STRIP_Y0 + pad * PAD_W;
      g.fillStyle = '#6f6f6f'; g.fillRect(X(1.75), Y(y0 + 0.08), 0.5 * s, 0.34 * s);
      g.fillStyle = '#fff'; g.font = `${0.17 * s}px system-ui`; g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText(kg + 'kg', X(2), Y(y0 + 0.25)); g.textAlign = 'left'; g.textBaseline = 'alphabetic';
    }
    // vehicles
    for (const v of f.veh) {
      const [, type, x, y, len, wid, lane, stopped, wheels] = v;
      const x0 = lane === 0 ? x - len : x, x1 = lane === 0 ? x : x + len;
      const single = type === 'moto' || type === 'bicycle';
      const bw = single ? Math.max(0.35, wid * 0.6) : wid;
      g.fillStyle = 'rgba(0,0,0,.4)';
      for (const wx of wheels) {
        const tw = single ? 0.12 : 0.22, tl = single ? 0.5 : 0.65;
        if (single) g.fillRect(X(wx - tl / 2), Y(y - tw / 2), tl * s, tw * s);
        else { const tr = wid / 2 - 0.12; g.fillRect(X(wx - tl / 2), Y(y - tr - tw / 2), tl * s, tw * s); g.fillRect(X(wx - tl / 2), Y(y + tr - tw / 2), tl * s, tw * s); }
      }
      g.fillStyle = VEH_COLOR[type] || '#ccc';
      g.globalAlpha = single ? 1 : 0.88;
      roundRect(g, X(x0), Y(y - bw / 2), (x1 - x0) * s, bw * s, 0.25 * s); g.fill();
      g.globalAlpha = 1;
      g.fillStyle = 'rgba(20,30,45,.55)';
      if (!single) { const fx = lane === 0 ? x1 - 1.3 : x0 + 0.6; g.fillRect(X(fx), Y(y - bw / 2 + 0.15), 0.7 * s, (bw - 0.3) * s); }
      if (stopped) {
        g.fillStyle = '#ff2a2a';
        const bx = lane === 0 ? X(x0) - 2 * dpr : X(x1) - 1 * dpr;
        g.fillRect(bx, Y(y - bw / 2 + 0.1), 3 * dpr, 0.3 * s); g.fillRect(bx, Y(y + bw / 2 - 0.4), 3 * dpr, 0.3 * s);
      }
    }
    // pedestrians (walking along the lanes)
    for (const p of f.peds) {
      const [, kind, x, y, mass, standing, feet] = p;
      for (let k = 0; k + 1 < feet.length; k += 2) {
        g.fillStyle = 'rgba(255,255,255,.9)';
        g.beginPath(); g.ellipse(X(feet[k]), Y(feet[k + 1]), 0.11 * s, 0.05 * s, 0, 0, 7); g.fill();
      }
      const r = (0.13 + Math.min(1, mass / 110) * 0.13) * s;
      g.fillStyle = KIND_COLOR[kind] || '#888';
      g.beginPath(); g.arc(X(x), Y(y), r, 0, 7); g.fill();
      g.lineWidth = 1.5 * dpr; g.strokeStyle = standing ? '#f1c21b' : 'rgba(0,0,0,.35)'; g.stroke();
    }
  }
  // the counter sign next to the crossing
  drawSign(g, X(2), Y(-3.5) - 6 * dpr, dpr, s);
  // HUD
  if (f) {
    g.fillStyle = 'rgba(0,0,0,.55)'; roundRect(g, 8 * dpr, 8 * dpr, 230 * dpr, 24 * dpr, 6 * dpr); g.fill();
    g.fillStyle = '#fff'; g.font = `${12 * dpr}px system-ui`;
    g.fillText(`t = ${f.t.toFixed(1)} s   ${f.speed}×${f.paused ? '  (paused)' : ''}`, 16 * dpr, 24 * dpr);
  }
}

function drawSign(g, cx, bottom, dpr, s) {
  const f = frame;
  const cal = !f || f.fw === 'cal';
  const total = cal ? '----' : String(Math.max(0, f.count));
  const wSign = Math.max(170 * dpr, 4.6 * s), hSign = Math.max(54 * dpr, 1.45 * s);
  const x = cx - wSign / 2, y = bottom - hSign;
  g.fillStyle = '#0d1117'; roundRect(g, x, y, wSign, hSign, 8 * dpr); g.fill();
  g.strokeStyle = '#30363d'; g.lineWidth = 2 * dpr; g.stroke();
  g.fillStyle = '#8b949e'; g.font = `600 ${Math.max(9 * dpr, hSign * 0.17)}px system-ui`; g.textAlign = 'center';
  g.fillText('PEOPLE CROSSED', cx, y + hSign * 0.25);
  g.fillStyle = '#3fb950'; g.font = `700 ${hSign * 0.42}px ui-monospace, Menlo, monospace`;
  g.fillText(total, cx, y + hSign * 0.66);
  g.fillStyle = '#58a6ff'; g.font = `600 ${Math.max(9 * dpr, hSign * 0.17)}px ui-monospace, Menlo, monospace`;
  g.fillText(cal ? '' : `→ ${f.laneA}    ← ${f.laneB}`, cx, y + hSign * 0.9);
  g.textAlign = 'left';
}

function drawPads() {
  const cv = $('pads');
  const { w, h, dpr } = fitCanvas(cv);
  const g = cv.getContext('2d');
  const css = getComputedStyle(document.documentElement);
  g.clearRect(0, 0, w, h);
  if (!frame) return;
  const max = Math.max(160, ...frame.pads, ...(frame.u || [0]));
  const pad = 22 * dpr, bw = (w - pad) / NUM_PADS;
  g.strokeStyle = css.getPropertyValue('--line'); g.fillStyle = css.getPropertyValue('--muted'); g.font = `${10 * dpr}px system-ui`;
  for (const v of [0, 100, 200, 300].filter((v) => v <= max)) {
    const y = h - 16 * dpr - (v / max) * (h - 26 * dpr);
    g.beginPath(); g.moveTo(pad, y); g.lineTo(w, y); g.stroke(); g.fillText(v, 0, y + 3 * dpr);
  }
  // lane separator
  g.strokeStyle = css.getPropertyValue('--muted'); g.setLineDash([4 * dpr, 3 * dpr]);
  g.beginPath(); g.moveTo(pad + PER_LANE * bw, 4 * dpr); g.lineTo(pad + PER_LANE * bw, h - 14 * dpr); g.stroke(); g.setLineDash([]);
  for (let i = 0; i < NUM_PADS; i++) {
    const kg = frame.pads[i], bh = (Math.min(kg, max) / max) * (h - 26 * dpr);
    const x = pad + i * bw + bw * 0.18;
    g.fillStyle = (frame.mask & (1 << i)) && frame.fw === 'v2' ? '#fa4d56' : heat(kg);
    g.fillRect(x, h - 16 * dpr - bh, bw * 0.64, bh);
    if (frame.u) {
      const u = Math.max(0, frame.u[i]), uy = h - 16 * dpr - (Math.min(u, max) / max) * (h - 26 * dpr);
      g.fillStyle = css.getPropertyValue('--accent'); g.fillRect(x - 2 * dpr, uy - 1.5 * dpr, bw * 0.64 + 4 * dpr, 3 * dpr);
    }
    g.fillStyle = css.getPropertyValue('--muted'); g.textAlign = 'center';
    g.fillText(i, x + bw * 0.32, h - 3 * dpr); g.textAlign = 'left';
  }
}

function drawChart() {
  const cv = $('chart');
  const { w, h, dpr } = fitCanvas(cv);
  const g = cv.getContext('2d');
  const css = getComputedStyle(document.documentElement);
  g.clearRect(0, 0, w, h);
  if (history.length < 2) return;
  const tEnd = history[history.length - 1].t, tStart = Math.max(history[0].t, tEnd - 120);
  const pts = history.filter((p) => p.t >= tStart);
  const lo = Math.min(...pts.map((p) => Math.min(p.truth, p.count))), hi = Math.max(lo + 5, ...pts.map((p) => Math.max(p.truth, p.count)));
  const L = 34 * dpr, B = 18 * dpr;
  const X = (t) => L + ((t - tStart) / Math.max(1, tEnd - tStart)) * (w - L - 6 * dpr);
  const Y = (v) => h - B - ((v - lo) / (hi - lo)) * (h - B - 8 * dpr);
  g.strokeStyle = css.getPropertyValue('--line'); g.fillStyle = css.getPropertyValue('--muted'); g.font = `${10 * dpr}px system-ui`;
  for (let k = 0; k <= 3; k++) {
    const v = Math.round(lo + (hi - lo) * k / 3), y = Y(v);
    g.beginPath(); g.moveTo(L, y); g.lineTo(w, y); g.stroke(); g.fillText(v, 2 * dpr, y + 3 * dpr);
  }
  g.fillText(`last ${Math.round(tEnd - tStart)} s`, L, h - 4 * dpr);
  const line = (key, color, width, dash) => {
    g.strokeStyle = color; g.lineWidth = width * dpr; g.setLineDash(dash.map((d) => d * dpr));
    g.beginPath(); pts.forEach((p, i) => (i ? g.lineTo(X(p.t), Y(p[key])) : g.moveTo(X(p.t), Y(p[key])))); g.stroke(); g.setLineDash([]);
  };
  line('truth', css.getPropertyValue('--muted'), 2, [5, 4]);
  if (fw !== 'cal') line('count', css.getPropertyValue('--accent'), 2.5, []);
  g.fillStyle = css.getPropertyValue('--muted'); g.textAlign = 'right';
  g.fillText('- - truth   ', w - 70 * dpr, h - 4 * dpr);
  g.fillStyle = css.getPropertyValue('--accent'); g.fillText('— firmware', w - 6 * dpr, h - 4 * dpr); g.textAlign = 'left';
}

function render() { drawScene(); drawPads(); drawChart(); requestAnimationFrame(render); }

// --------------------------------------------------------------- controls --
function rateUI(vals) {
  const [ped, grp, veh, stop, wait] = vals;
  $('rPed').value = ped; $('rGrp').value = Math.round(grp * 100); $('rVeh').value = veh;
  $('rStop').value = Math.round(stop * 100); $('rWait').value = Math.round(wait * 100);
  rateLabels();
}
function rateLabels() {
  $('oPed').textContent = $('rPed').value; $('oGrp').textContent = $('rGrp').value + '%';
  $('oVeh').textContent = $('rVeh').value; $('oStop').textContent = $('rStop').value + '%'; $('oWait').textContent = $('rWait').value + '%';
}
function sendRates() {
  rateLabels();
  cmd(`rates ${$('rPed').value} ${$('rGrp').value / 100} ${$('rVeh').value} ${$('rStop').value / 100} ${$('rWait').value / 100}`);
}

function wire() {
  document.querySelectorAll('.fw-switch button').forEach((b) => (b.onclick = () => setFirmware(b.dataset.fw)));
  $('scenario').onchange = () => { cmd('scenario ' + $('scenario').value); rateUI(PRESETS[$('scenario').value] || PRESETS.idle); };
  $('speed').oninput = () => { const v = SPEEDS[+$('speed').value]; $('speedOut').textContent = v + '×'; cmd('speed ' + v); };
  $('pauseBtn').onclick = () => cmd(frame && frame.paused ? 'resume' : 'pause');
  $('resetBtn').onclick = () => { cmd('serial r'); cmd('resettruth'); history = []; lastHistT = -1; };
  $('restartBtn').onclick = () => setFirmware(fw).then(() => cmd('speed ' + SPEEDS[+$('speed').value]));
  document.querySelectorAll('[data-spawn]').forEach((b) => (b.onclick = () => cmd(`spawn ${b.dataset.spawn} ${$('spawnLane').value}`.trim())));
  ['rPed', 'rGrp', 'rVeh', 'rStop', 'rWait'].forEach((id) => ($(id).onchange = sendRates));
  ['rPed', 'rGrp', 'rVeh', 'rStop', 'rWait'].forEach((id) => ($(id).oninput = rateLabels));
  $('serialForm').onsubmit = (e) => { e.preventDefault(); const v = $('serialInput').value; if (v) { cmd('serial ' + v); $('serialInput').value = ''; } };
  $('showTelemetry').onchange = () => { serialLines = []; };
  for (let i = 0; i < NUM_PADS; i++) {
    const o = document.createElement('option');
    o.value = i; o.textContent = `Pad ${i} (lane ${i < PER_LANE ? 'B' : 'A'})`;
    $('calPad').appendChild(o);
  }
  $('calPut').onclick = () => cmd(`weight ${$('calPad').value} ${+$('calKg').value || 0}`);
  $('calClear').onclick = () => cmd('weight clear');
  $('benchBtn').onclick = runBench;
  rateUI(PRESETS.normal);
  renderQuick();
}

// -------------------------------------------------------------- benchmark --
async function runBench() {
  const r = await fetch('/api/bench', { method: 'POST', body: JSON.stringify({ seeds: 3, duration: 600 }) });
  if (!r.ok) return;
  $('benchBtn').disabled = true;
  pollBench();
}
async function pollBench() {
  const b = await (await fetch('/api/bench')).json();
  if (b.running) {
    $('benchStatus').textContent = `running… ${b.done}/${b.total} simulations`;
    $('benchBtn').disabled = true;
    setTimeout(pollBench, 400);
    return;
  }
  $('benchBtn').disabled = false;
  $('benchStatus').textContent = b.error ? 'error: ' + b.error : (b.result ? `${b.result.source === 'results.json' ? 'saved results' : 'finished'}: ${b.result.seeds} seeds × ${b.result.duration / 60} min per scenario` : '');
  if (b.result) renderBench(b.result);
}
function accClass(a) { return a >= 0.9 ? 'acc-good' : a >= 0.7 ? 'acc-mid' : 'acc-bad'; }
function pct(a) { return a === null || a === undefined ? '–' : `<span class="${accClass(a)}">${(a * 100).toFixed(1)}%</span>`; }
function renderBench(res) {
  let html = '<div class="table-wrap"><table><thead><tr><th>Scenario</th><th>Accuracy</th><th></th><th>→ Lane A</th><th>← Lane B</th></tr></thead><tbody>';
  let tT = 0, tE = 0;
  for (const row of res.table) {
    const c = row.v2;
    if (!c) continue;
    html += `<tr><td><b>${row.scenario}</b><div class="scen-desc">${row.description}</div></td>`;
    if (c.accuracy === null) {
      const per = c.vehiclesOverStrip ? (c.counted / c.vehiclesOverStrip) : 0;
      html += `<td class="num ${per < 0.1 ? 'acc-good' : 'acc-bad'}">${c.counted} false counts</td><td class="num scen-desc">from ${c.vehiclesOverStrip} vehicles (${(per * 100).toFixed(1)}%)</td><td></td><td></td>`;
    } else {
      tT += c.truth; tE += Math.abs(c.counted - c.truth);
      const a = Math.max(0, c.accuracy);
      html += `<td style="min-width:140px"><div class="bar v2"><span style="width:${(a * 100).toFixed(1)}%"></span><em></em></div></td>`;
      html += `<td class="num">${pct(c.accuracy)} <span class="scen-desc">${c.counted}/${c.truth} · worst run ${(c.worstRun * 100).toFixed(0)}%</span></td>`;
      const la = c.lanes ? c.lanes.A : null, lb = c.lanes ? c.lanes.B : null;
      html += `<td class="num">${la ? pct(la.accuracy) + ` <span class="scen-desc">${la.counted}/${la.truth}</span>` : '–'}</td>`;
      html += `<td class="num">${lb ? pct(lb.accuracy) + ` <span class="scen-desc">${lb.counted}/${lb.truth}</span>` : '–'}</td>`;
    }
    html += '</tr>';
  }
  if (tT) {
    const a = 1 - tE / tT;
    html += `<tr><td><b>All pedestrian scenarios</b><div class="scen-desc">sum of |error| / people</div></td><td><div class="bar v2"><span style="width:${(a * 100).toFixed(1)}%"></span><em></em></div></td><td class="num">${pct(a)} <span class="scen-desc">${tT} people</span></td><td></td><td></td></tr>`;
  }
  html += '</tbody></table></div><p class="hint" style="margin-top:8px">Dashed line = 90 % target. Accuracy = 1 − |counted − true| / true per scenario, summed over seeds.</p>';
  $('benchTable').innerHTML = html;
}

wire();
connect();
pollBench();
fetch('/api/state').then((r) => r.json()).then((s) => { if (s.fw) markFirmware(s.fw); });
requestAnimationFrame(render);
