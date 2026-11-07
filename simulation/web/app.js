'use strict';
// ---------------------------------------------------------------------------
//  Ram Ghat gate simulator UI. All physics and the Arduino firmware run in the
//  native simulator process; this page only renders frames and sends commands.
//
//  Scene (top-down): Ramghat Marg runs left-right. On the LEFT are the Ram
//  Ghat premises: the plaza, the stone steps and the Shipra river. On the
//  RIGHT the marg leads away towards Dani Gate / Mahakaleshwar. As at
//  Simhastha 2016, a barricade splits the marg into two one-way corridors of
//  7 m: the ENTRY corridor (top, everyone walks <- into the premises) and the
//  EXIT corridor (bottom, everyone walks -> out). Each corridor has its own
//  gate with a sensor strip of 14 plates x 0.5 m across it, 1 m along it,
//  and its own Arduino; the two boards share their counts over a cable.
// ---------------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const NUM_STRIPS = 2, NUM_PADS = 14, PAD_W = 0.5;
const CORR_W = 7, MEDIAN_W = 1, ROAD_Y = MEDIAN_W / 2 + CORR_W;  // 7.5: the outer kerbs
const STRIP_X0 = 1.5, STRIP_X1 = 2.5;
const corrY0 = (c) => (c === 0 ? -ROAD_Y : MEDIAN_W / 2);        // 0 = entry (top), 1 = exit (bottom)
const STRIP_NAME = ['ENTRY', 'EXIT'];
const KIND_COLOR = ['#4589ff', '#33b1ff', '#ff7eb6', '#be95ff', '#42be65'];
const VEH_COLOR = { car: '#e0e0e0', suv: '#a8a8a8', auto: '#f1c21b', moto: '#ff832b', bicycle: '#08bdba', bus: '#d12771', truck: '#8a3ffc' };
const SPEEDS = [0.25, 0.5, 1, 2, 4, 8, 16, 30];
// starting rates: people/min, group, vehicles/min, stop, wait, non-linear, heading into the ghat
// (everyone wanders a little anyway; a quarter weave, cut across, jostle or turn back)
const DEFAULT_RATES = [14, .25, 4, .1, 0, .25, .5];
const QUICK = { v2: ['r', 't', 's', 'c', 'o0', 'o390', 'd', 'h'], cal: ['t', 'k20', 'n', 'p', 'a', 'h'] };

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
// (Re)start the simulator, then give it the rates set on this page.
function setFirmware(name) {
  markFirmware(name);
  history = []; lastHistT = -1; serialLines = []; serialPartial = ''; $('serial').textContent = '';
  return fetch('/api/firmware', { method: 'POST', body: JSON.stringify({ fw: name, scenario: 'idle' }) });
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
    if (f.t - lastHistT >= 0.25) {
      history.push({ t: f.t, truth: f.truthInside, count: f.inside });
      lastHistT = f.t;
      if (history.length > 2000) history.shift();
    }
    updateStats(f);
  };
}

// ------------------------------------------------------------------- stats --
function updateStats(f) {
  const cal = fw === 'cal';
  const c = f.count < 0 ? 0 : f.count;
  $('sInside').textContent = cal ? '–' : f.inside;
  $('sIn').textContent = cal ? '–' : f.in;
  $('sOut').textContent = cal ? '–' : f.out;
  $('sCount').textContent = cal ? '–' : c;
  $('sLive').textContent = cal ? 'calibration sketch running: no counting' : (f.live >= 0 ? `live ≈ ${f.live.toFixed(1)} incl. people on the strip now` : '\u00a0');
  $('sTruthInside').textContent = f.truthInside;
  $('sTruthInsideNote').textContent = `went in ${f.truthIn} · came out ${f.truthOut}`;
  if (!cal && f.truthInside > 0) {
    const acc = Math.max(0, 1 - Math.abs(f.inside - f.truthInside) / f.truthInside);
    $('sInsideAcc').textContent = (acc * 100).toFixed(1) + '%';
    $('sInsideAcc').className = acc >= 0.95 ? 'acc-good' : acc >= 0.85 ? 'acc-mid' : 'acc-bad';
    $('sInsideAccNote').textContent = `board shows ${f.inside}, really ${f.truthInside} inside`;
  } else {
    $('sInsideAcc').textContent = '–'; $('sInsideAcc').className = '';
    $('sInsideAccNote').textContent = cal ? '' : 'nobody inside yet';
  }
  updateAlert(f, cal);
  $('sTruth').textContent = f.truth;
  $('sTruthNote').textContent = f.turnedBack ? `${f.turnedBack} stepped on & turned back (not crossings)` : '';
  if (f.truth > 0 && !cal) {
    const acc = 1 - Math.abs(c - f.truth) / f.truth;
    $('sAcc').textContent = (acc * 100).toFixed(1) + '%';
    $('sAcc').className = acc >= 0.9 ? 'acc-good' : acc >= 0.7 ? 'acc-mid' : 'acc-bad';
    $('sAccNote').textContent = `firmware counted ${c} crossings`;
  } else { $('sAcc').textContent = '–'; $('sAcc').className = ''; $('sAccNote').textContent = ''; }
  $('sVeh').textContent = f.vehOver;
  $('sVehNote').textContent = f.vehActive ? 'vehicle mask ACTIVE on the red plates' : `${f.vehTotal} vehicles spawned; tyres are masked, never counted`;
  $('pauseBtn').textContent = f.paused ? 'Resume' : 'Pause';
  updatePedHint(f);
}

// Nobody waits at the gates: everyone pushes on and the crowd slows down. Show
// how packed each corridor is at its gate (crowd-safety bands, people / m2).
function updatePedHint(f) {
  const el = $('pedHint');
  const dens = f && Array.isArray(f.density) ? f.density : [0, 0];
  const back = f && Array.isArray(f.backlog) ? f.backlog : [0, 0];
  const d = Math.max(dens[0], dens[1]);
  const both = `entry ${dens[0].toFixed(1)} · exit ${dens[1].toFixed(1)} people/m² at the gates`;
  const backed = [0, 1].filter((c) => back[c] > 0).map((c) => `${back[c].toLocaleString('en-IN')} more backed up along the ${STRIP_NAME[c].toLowerCase()} corridor beyond the screen`).join(' · ');
  el.classList.remove('warn', 'danger');
  if (d >= 6) { el.textContent = `${both}: CRUSH DANGER` + (backed ? ' · ' + backed : ''); el.classList.add('danger'); }
  else if (d >= 4) { el.textContent = `${both}: packed, everyone shuffling` + (backed ? ' · ' + backed : ''); el.classList.add('warn'); }
  else el.textContent = 'exact people / min (0 – 10,000)' + (d >= 1 ? ` · ${both}` : '') + (backed ? ' · ' + backed : '');
}

// The overcrowding alert is raised by the FIRMWARE (it drives the siren pin
// and prints @A lines); the page only shows it.
let alertWas = false;
function updateAlert(f, cal) {
  const limit = f.limit > 0 ? f.limit : +$('crowdLimit').value || 3000;
  const on = !cal && !!f.alert;
  const frac = cal ? 0 : Math.max(0, Math.min(1, f.inside / limit));
  $('limitFill').style.width = (frac * 100).toFixed(1) + '%';
  $('limitFill').className = on ? 'over' : frac >= 0.8 ? 'near' : '';
  $('limitState').textContent = cal ? '–' : on ? 'ALERT' : frac >= 0.8 ? `${Math.round(frac * 100)}% full` : 'ok';
  $('limitState').className = 'limit-state' + (on ? ' over' : frac >= 0.8 ? ' near' : '');
  $('counterCard').classList.toggle('alarm', on);
  $('crowdAlert').hidden = !on;
  if (on) $('crowdAlertText').textContent =
    `${f.inside.toLocaleString('en-IN')} people inside Ram Ghat, limit ${limit.toLocaleString('en-IN')}. Hold the entry gate on Ramghat Marg; keep the exit gate open.`;
  if (document.activeElement !== $('crowdLimit') && f.limit > 0 && +$('crowdLimit').value !== f.limit) $('crowdLimit').value = f.limit;
  if (on && !alertWas) beep();
  alertWas = on;
}
let audio = null;
function beep() {
  try {
    audio = audio || new (window.AudioContext || window.webkitAudioContext)();
    if (audio.state === 'suspended') return;  // browsers allow sound only after a click on the page
    for (let i = 0; i < 3; i++) {
      const o = audio.createOscillator(), g = audio.createGain();
      o.frequency.value = 880; o.connect(g); g.connect(audio.destination);
      const t = audio.currentTime + i * 0.35;
      g.gain.setValueAtTime(0.15, t); g.gain.setValueAtTime(0, t + 0.2);
      o.start(t); o.stop(t + 0.22);
    }
  } catch { /* no audio: the banner is enough */ }
}

// ------------------------------------------------------------------ serial --
function appendSerial(text) {
  const parts = (serialPartial + text).split('\n');
  serialPartial = parts.pop();
  const show = $('showTelemetry').checked;
  for (const l of parts) {
    const body = l.replace(/^\[\w+\] /, '');  // "[ENTRY] " / "[EXIT] ": which board printed it
    if (!show && (body.startsWith('@T,') || body.startsWith('@E,') || body.startsWith('@X,'))) continue;
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
  const s = h / 16.5;                     // px per metre: the two 7 m corridors and the barricade fill the height
  const cx = w / 2 - 2 * s, cy = h / 2;   // world (2, 0) in the centre
  const X = (x) => cx + x * s, Y = (y) => cy + y * s;
  const xL = -cx / s, xR = (w - cx) / s;  // visible world x range
  const f = frame;
  g.clearRect(0, 0, w, h);

  // ---- RIGHT: Ramghat Marg, kerb to kerb, split into two one-way corridors --
  g.fillStyle = C('--walk'); g.fillRect(X(STRIP_X1), 0, w - X(STRIP_X1), h);
  for (let c = 0; c < NUM_STRIPS; c++) {
    g.fillStyle = C('--road'); g.fillRect(X(STRIP_X1), Y(corrY0(c)), w - X(STRIP_X1), CORR_W * s);
  }
  g.fillStyle = C('--road-edge');
  g.fillRect(X(STRIP_X1), Y(-ROAD_Y) - 3 * dpr, w, 3 * dpr); g.fillRect(X(STRIP_X1), Y(ROAD_Y), w, 3 * dpr);
  // painted arrows: one way in each corridor
  g.save();
  g.fillStyle = 'rgba(255,255,255,.28)';
  for (let c = 0; c < NUM_STRIPS; c++) {
    const d = c === 0 ? -1 : 1, yc = corrY0(c) + CORR_W / 2;
    for (let x = Math.ceil(xR) - 1; x > STRIP_X1 + 2; x -= 6) {
      for (const yy of [yc - 2.2, yc + 2.2]) {
        g.beginPath();
        g.moveTo(X(x + d * 0.9), Y(yy)); g.lineTo(X(x - d * 0.5), Y(yy - 0.5)); g.lineTo(X(x - d * 0.2), Y(yy));
        g.lineTo(X(x - d * 0.5), Y(yy + 0.5)); g.closePath(); g.fill();
      }
    }
  }
  // corridor names, painted on the road
  g.textAlign = 'center'; g.textBaseline = 'middle';
  const mx = (Math.max(5.5, xR - 6) + 4) / 2 + 2.5;
  g.fillStyle = 'rgba(255,255,255,.6)'; g.font = `700 ${0.66 * s}px system-ui`;
  g.fillText('◀   ENTRY  ·  प्रवेश', X(mx), Y(corrY0(0) + 3.0));
  g.fillText('EXIT  ·  निकास   ▶', X(mx), Y(corrY0(1) + 3.0));
  g.font = `${0.36 * s}px system-ui`; g.fillStyle = 'rgba(255,255,255,.45)';
  g.fillText('one way · to Ram Ghat · रामघाट मार्ग', X(mx), Y(corrY0(0) + 3.8));
  g.fillText('one way · to Dani Gate · रामघाट मार्ग', X(mx), Y(corrY0(1) + 3.8));
  g.restore();
  // road sign, pinned to the top-right corner
  {
    const sh = Math.max(34 * dpr, 0.62 * s), sw = sh * 3.8, sx = w - sw - 8 * dpr, sy = 8 * dpr;
    g.fillStyle = '#0b6e3a'; roundRect(g, sx, sy, sw, sh, 0.12 * s); g.fill();
    g.strokeStyle = '#fff'; g.lineWidth = 1.5 * dpr; roundRect(g, sx + 2 * dpr, sy + 2 * dpr, sw - 4 * dpr, sh - 4 * dpr, 0.1 * s); g.stroke();
    g.fillStyle = '#fff'; g.textAlign = 'left'; g.textBaseline = 'middle';
    g.font = `700 ${sh * 0.29}px system-ui`; g.fillText('← Ram Ghat  ·  Shipra', sx + sh * 0.17, sy + sh * 0.32);
    g.font = `600 ${sh * 0.25}px system-ui`; g.fillText('Dani Gate · Mahakaleshwar →', sx + sh * 0.17, sy + sh * 0.7);
    g.textBaseline = 'alphabetic';
  }

  // ---- LEFT: Ram Ghat premises (plaza, steps, Shipra river) ------------------
  const riverX = -11.25, stepsX = -6.0;
  const gl = g.createLinearGradient(0, 0, X(STRIP_X0), 0);
  gl.addColorStop(0, '#d8b98b'); gl.addColorStop(1, '#e6cfa6');
  g.fillStyle = gl; g.fillRect(0, 0, X(STRIP_X0), h);
  // paving joints on the plaza
  g.strokeStyle = 'rgba(120,90,50,.18)'; g.lineWidth = 1 * dpr;
  for (let x = stepsX; x < STRIP_X0; x += 1) { g.beginPath(); g.moveTo(X(x), 0); g.lineTo(X(x), h); g.stroke(); }
  for (let y = -9; y <= 9; y += 1) { g.beginPath(); g.moveTo(X(stepsX), Y(y)); g.lineTo(X(STRIP_X0), Y(y)); g.stroke(); }
  // ghat steps going down to the river
  for (let i = 0, x = stepsX; x > riverX; i++, x -= 0.55) {
    g.fillStyle = i % 2 ? '#c9a473' : '#bb9461';
    g.fillRect(X(x - 0.55), 0, 0.55 * s + 1, h);
    g.fillStyle = 'rgba(90,60,30,.25)'; g.fillRect(X(x - 0.55), 0, 2 * dpr, h);
  }
  // the Shipra
  const gr = g.createLinearGradient(0, 0, X(riverX), 0);
  gr.addColorStop(0, '#1f5f8b'); gr.addColorStop(1, '#3a86b4');
  g.fillStyle = gr; g.fillRect(0, 0, X(riverX), h);
  g.strokeStyle = 'rgba(255,255,255,.25)'; g.lineWidth = 1.2 * dpr;
  const tt = f ? f.t : 0;
  for (let y = -9; y < 9; y += 0.9) {
    g.beginPath();
    for (let x = xL; x <= riverX; x += 0.1) {
      const yy = y + 0.12 * Math.sin(x * 3 + tt * 1.2 + y);
      if (x === xL) g.moveTo(X(x), Y(yy)); else g.lineTo(X(x), Y(yy));
    }
    g.stroke();
  }
  g.save(); g.translate(X((xL + riverX) / 2), Y(0)); g.rotate(-Math.PI / 2);
  g.fillStyle = 'rgba(255,255,255,.85)'; g.font = `600 ${0.36 * s}px system-ui`; g.textAlign = 'center';
  g.fillText('SHIPRA · क्षिप्रा', 0, 0.12 * s); g.restore();
  // the premises' name, painted on the plaza
  g.fillStyle = 'rgba(80,45,10,.38)'; g.textAlign = 'center';
  const nx = Math.max(xL + 3.2, -3.0);
  g.font = `800 ${0.62 * s}px system-ui`; g.fillText('RAM GHAT', X(nx), Y(5.6));
  g.font = `600 ${0.32 * s}px system-ui`; g.fillText('राम घाट · Simhastha Kumbh, Ujjain', X(nx), Y(6.15));
  // premises boundary along the kerb lines, with the gates at the strips
  g.strokeStyle = '#7a4b1c'; g.lineWidth = 3 * dpr; g.setLineDash([6 * dpr, 4 * dpr]);
  g.beginPath(); g.moveTo(X(stepsX), Y(-ROAD_Y)); g.lineTo(X(STRIP_X0), Y(-ROAD_Y)); g.moveTo(X(stepsX), Y(ROAD_Y)); g.lineTo(X(STRIP_X0), Y(ROAD_Y)); g.stroke();
  g.setLineDash([]);

  // ---- the barricade between the corridors, from the plaza to the far end --
  {
    const bx0 = X(-3.0), bw = w - bx0;
    g.fillStyle = C('--walk'); g.fillRect(bx0, Y(-MEDIAN_W / 2), bw, MEDIAN_W * s);
    g.strokeStyle = '#8a9099'; g.lineWidth = Math.max(2 * dpr, 0.08 * s);
    g.beginPath(); g.moveTo(bx0, Y(0)); g.lineTo(w, Y(0)); g.stroke();
    g.fillStyle = '#5c636d';
    for (let x = -3.0; x < xR; x += 1.5) { g.beginPath(); g.arc(X(x), Y(0), Math.max(2.5 * dpr, 0.11 * s), 0, 7); g.fill(); }
    g.save(); g.fillStyle = 'rgba(40,45,52,.75)'; g.font = `600 ${0.3 * s}px system-ui`; g.textAlign = 'left'; g.textBaseline = 'middle';
    g.fillText('BARRICADE · entry and exit kept apart', X(4.2), Y(0) - 0.34 * s); g.restore();
  }
  // gate pillars at the kerbs and at the barricade ends
  for (const py of [-ROAD_Y, -MEDIAN_W / 2, MEDIAN_W / 2, ROAD_Y]) {
    g.fillStyle = '#a0522d'; g.fillRect(X(STRIP_X0) - 0.15 * s, Y(py) - 0.3 * s, 1.3 * s, 0.6 * s);
    g.fillStyle = '#f5b700'; g.beginPath(); g.arc(X(2), Y(py), 0.16 * s, 0, 7); g.fill();
  }
  g.fillStyle = 'rgba(80,45,10,.8)'; g.font = `700 ${0.3 * s}px system-ui`; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText('ENTRY GATE', X(0.2), Y(-ROAD_Y - 0.4));
  g.fillText('EXIT GATE', X(0.2), Y(ROAD_Y + 0.4));
  g.textAlign = 'left'; g.textBaseline = 'alphabetic';

  // ---- sensor strips: 14 plates across each corridor, in its gate -----------
  const mask = f && f.fw === 'v2' ? f.mask : 0;
  const fs = Math.max(9 * dpr, 0.2 * s);
  for (let c = 0; c < NUM_STRIPS; c++) {
    for (let i = 0; i < NUM_PADS; i++) {
      const k = c * NUM_PADS + i;
      const kg = f ? f.pads[k] : 0;
      const y0 = corrY0(c) + i * PAD_W;
      g.globalAlpha = 0.92;
      g.fillStyle = heat(kg);
      g.fillRect(X(STRIP_X0), Y(y0) + 1, (STRIP_X1 - STRIP_X0) * s, PAD_W * s - 2);
      g.globalAlpha = 1;
      if (mask & (1 << k)) {
        g.save(); g.beginPath(); g.rect(X(STRIP_X0), Y(y0) + 1, s, PAD_W * s - 2); g.clip();
        g.strokeStyle = '#fa4d56'; g.lineWidth = 2 * dpr;
        for (let q = -s; q < 2 * s; q += 6 * dpr) { g.beginPath(); g.moveTo(X(STRIP_X0) + q, Y(y0)); g.lineTo(X(STRIP_X0) + q + PAD_W * s, Y(y0 + PAD_W)); g.stroke(); }
        g.restore();
      }
      g.fillStyle = kg > 90 ? '#111' : '#fff';
      g.font = `${fs}px ui-monospace, Menlo, monospace`;
      g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText(kg >= 1 ? Math.round(kg) : '·', X((STRIP_X0 + STRIP_X1) / 2), Y(y0 + PAD_W / 2));
    }
    g.textAlign = 'left'; g.textBaseline = 'alphabetic';
    g.strokeStyle = '#ffffff'; g.lineWidth = 1.5 * dpr; g.strokeRect(X(STRIP_X0), Y(corrY0(c)), s, CORR_W * s);
  }

  if (f) {
    // static calibration weights
    for (const [pad, kg] of f.weights) {
      const y0 = corrY0(Math.floor(pad / NUM_PADS)) + (pad % NUM_PADS) * PAD_W;
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
    // pilgrims
    for (const p of f.peds) {
      const [, kind, x, y, mass, standing, feet, path, dir] = p;
      for (let k = 0; k + 1 < feet.length; k += 2) {
        g.fillStyle = 'rgba(255,255,255,.9)';
        g.beginPath(); g.ellipse(X(feet[k]), Y(feet[k + 1]), 0.11 * s, 0.05 * s, 0, 0, 7); g.fill();
      }
      const r = (0.13 + Math.min(1, mass / 110) * 0.13) * s;
      g.fillStyle = KIND_COLOR[kind] || '#888';
      g.beginPath(); g.arc(X(x), Y(y), r, 0, 7); g.fill();
      g.lineWidth = 1.5 * dpr; g.strokeStyle = standing ? '#f1c21b' : 'rgba(0,0,0,.35)'; g.stroke();
      // walking direction: a small chevron on the leading side
      // (orange = heading into the ghat, white = coming out)
      const d = dir < 0 ? -1 : 1;
      g.fillStyle = d < 0 ? '#ff9f1a' : '#ffffff';
      g.beginPath();
      g.moveTo(X(x) + d * (r + 5 * dpr), Y(y));
      g.lineTo(X(x) + d * (r + 1 * dpr), Y(y) - 3.5 * dpr);
      g.lineTo(X(x) + d * (r + 1 * dpr), Y(y) + 3.5 * dpr);
      g.closePath(); g.fill();
      if (path > 0) { // non-linear walker: dashed ring
        g.beginPath(); g.arc(X(x), Y(y), r + 3 * dpr, 0, 7);
        g.setLineDash([3 * dpr, 2.5 * dpr]); g.strokeStyle = '#f1c21b'; g.lineWidth = 1.5 * dpr; g.stroke(); g.setLineDash([]);
      }
    }
  }
  // the gates' counter board, hanging over the plaza by the entry gate
  drawSign(g, X(STRIP_X0) - 0.6 * s - Math.max(95 * dpr, 1.8 * s), Y(-ROAD_Y) + Math.max(60 * dpr, 1.2 * s) + 0.5 * s, dpr, s);
  // HUD
  if (f) {
    g.fillStyle = 'rgba(0,0,0,.55)'; roundRect(g, 8 * dpr, 8 * dpr, 230 * dpr, 24 * dpr, 6 * dpr); g.fill();
    g.fillStyle = '#fff'; g.font = `${12 * dpr}px system-ui`;
    g.fillText(`t = ${f.t.toFixed(1)} s   ${f.speed}×${f.paused ? '  (paused)' : ''}`, 16 * dpr, 24 * dpr);
    const hidden = (f.pedsTotal || 0) - f.peds.length;
    if (hidden > 0) {  // only a sample of the people is drawn
      const txt = `${f.pedsTotal.toLocaleString('en-IN')} people on this stretch · drawing every ${Math.round(f.pedsTotal / f.peds.length)}th`;
      g.font = `600 ${12 * dpr}px system-ui`;
      const tw = g.measureText(txt).width + 20 * dpr, qy = h - 32 * dpr;
      g.fillStyle = 'rgba(0,0,0,.6)'; roundRect(g, w - tw - 8 * dpr, qy, tw, 24 * dpr, 6 * dpr); g.fill();
      g.fillStyle = '#fff'; g.fillText(txt, w - tw + 2 * dpr, qy + 16 * dpr);
    }
  }
}

function drawSign(g, cx, bottom, dpr, s) {
  const f = frame;
  const cal = !f || f.fw === 'cal';
  const inside = cal ? '----' : String(f.inside);
  const alarm = !cal && f.alert;
  const blink = alarm && Math.floor(performance.now() / 450) % 2 === 0;
  const wSign = Math.max(190 * dpr, 3.6 * s), hSign = Math.max(60 * dpr, 1.2 * s);
  const x = cx - wSign / 2, y = bottom - hSign;
  g.fillStyle = blink ? '#5c0b0b' : '#0d1117'; roundRect(g, x, y, wSign, hSign, 8 * dpr); g.fill();
  g.strokeStyle = alarm ? '#ff4d4f' : '#f0a020'; g.lineWidth = (alarm ? 3 : 2) * dpr; g.stroke();
  g.fillStyle = alarm ? '#ffb3b3' : '#c9d1d9'; g.font = `600 ${Math.max(9 * dpr, hSign * 0.15)}px system-ui`; g.textAlign = 'center';
  g.fillText(alarm ? 'OVERCROWDED · HOLD ENTRY' : 'INSIDE RAM GHAT', cx, y + hSign * 0.23);
  g.fillStyle = alarm ? '#ff4d4f' : '#f0a020'; g.font = `700 ${hSign * 0.4}px ui-monospace, Menlo, monospace`;
  g.fillText(inside, cx, y + hSign * 0.62);
  g.fillStyle = '#58a6ff'; g.font = `600 ${Math.max(9 * dpr, hSign * 0.15)}px ui-monospace, Menlo, monospace`;
  g.fillText(cal ? '' : `← in ${f.in}   out ${f.out} →`, cx, y + hSign * 0.88);
  g.textAlign = 'left';
}

function drawPads() {
  const cv = $('pads');
  const { w, h, dpr } = fitCanvas(cv);
  const g = cv.getContext('2d');
  const css = getComputedStyle(document.documentElement);
  g.clearRect(0, 0, w, h);
  if (!frame) return;
  const n = NUM_STRIPS * NUM_PADS;
  const max = Math.max(160, ...frame.pads, ...(frame.u || [0]));
  const pad = 22 * dpr, gap = 16 * dpr, bw = (w - pad - gap) / n;
  const barX = (i) => pad + i * bw + (i >= NUM_PADS ? gap : 0);
  g.strokeStyle = css.getPropertyValue('--line'); g.fillStyle = css.getPropertyValue('--muted'); g.font = `${10 * dpr}px system-ui`;
  for (const v of [0, 100, 200, 300].filter((v) => v <= max)) {
    const y = h - 16 * dpr - (v / max) * (h - 36 * dpr);
    g.beginPath(); g.moveTo(pad, y); g.lineTo(w, y); g.stroke(); g.fillText(v, 0, y + 3 * dpr);
  }
  for (let i = 0; i < n; i++) {
    const kg = frame.pads[i], bh = (Math.min(kg, max) / max) * (h - 36 * dpr);
    const x = barX(i) + bw * 0.18;
    g.fillStyle = (frame.mask & (1 << i)) && frame.fw === 'v2' ? '#fa4d56' : heat(kg);
    g.fillRect(x, h - 16 * dpr - bh, bw * 0.64, bh);
    if (frame.u) {
      const u = Math.max(0, frame.u[i]), uy = h - 16 * dpr - (Math.min(u, max) / max) * (h - 36 * dpr);
      g.fillStyle = css.getPropertyValue('--accent'); g.fillRect(x - 2 * dpr, uy - 1.5 * dpr, bw * 0.64 + 4 * dpr, 3 * dpr);
    }
    g.fillStyle = css.getPropertyValue('--muted'); g.textAlign = 'center';
    g.fillText(i % NUM_PADS, x + bw * 0.32, h - 3 * dpr); g.textAlign = 'left';
  }
  // which strip is which
  g.font = `600 ${10 * dpr}px system-ui`; g.fillStyle = css.getPropertyValue('--muted');
  for (let c = 0; c < NUM_STRIPS; c++) g.fillText(`${STRIP_NAME[c]} GATE`, barX(c * NUM_PADS) + bw * 0.18, 10 * dpr);
  // the gap between the strips
  g.fillStyle = css.getPropertyValue('--line'); g.fillRect(barX(NUM_PADS) - gap / 2 - dpr, 14 * dpr, 2 * dpr, h - 30 * dpr);
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
  if (fw !== 'cal') line('count', '#f0a020', 2.5, []);
  g.fillStyle = css.getPropertyValue('--muted'); g.textAlign = 'right';
  g.fillText('- - truth   ', w - 70 * dpr, h - 4 * dpr);
  g.fillStyle = '#f0a020'; g.fillText('— firmware', w - 6 * dpr, h - 4 * dpr); g.textAlign = 'left';
}

function render() { drawScene(); drawPads(); drawChart(); requestAnimationFrame(render); }

// --------------------------------------------------------------- controls --
// People/min runs 0..10,000 on a cubic scale so the low end stays usable:
// slider 100 -> 10/min, 240 -> ~140, 360 -> ~470, 500 -> 1,250, 1000 -> 10,000
const PED_MAX = 10000;
const pedFromSlider = (p) => Math.round(PED_MAX * Math.pow(p / 1000, 3));
const sliderFromPed = (v) => Math.round(1000 * Math.cbrt(Math.max(0, v) / PED_MAX));
const pedRate = () => Math.max(0, Math.min(PED_MAX, Math.round(+$('nPed').value || 0)));
const vehRate = () => ($('vehOn').checked ? +$('rVeh').value : 0);
function rateUI(vals) {
  const [ped, grp, veh, stop, wait, nonlin = 0, inbound = 0.5] = vals;
  $('nPed').value = ped; $('rPed').value = sliderFromPed(ped);
  $('rGrp').value = Math.round(grp * 100); if (veh > 0) $('rVeh').value = veh;
  $('rStop').value = Math.round(stop * 100); $('rWait').value = Math.round(wait * 100);
  $('rNonlin').value = Math.round(nonlin * 100); $('rInb').value = Math.round(inbound * 100);
  rateLabels();
}
function rateLabels() {
  const ped = pedRate();
  $('oPed').textContent = ped.toLocaleString('en-IN');
  $('oGrp').textContent = $('rGrp').value + '%';
  $('oVeh').textContent = $('vehOn').checked ? $('rVeh').value : 'off';
  $('rVeh').disabled = !$('vehOn').checked;
  $('vehOnNote').textContent = $('vehOn').checked ? `on: ${$('rVeh').value} vehicles/min` : 'off: no vehicles arrive unless you spawn one';
  $('oStop').textContent = $('rStop').value + '%'; $('oWait').textContent = $('rWait').value + '%';
  $('oNonlin').textContent = $('rNonlin').value + '%';
  $('oInb').textContent = `${$('rInb').value}% in · ${100 - $('rInb').value}% out`;
  updatePedHint(frame);
}
function sendRates() {
  rateLabels();
  return cmd(`rates ${pedRate()} ${$('rGrp').value / 100} ${vehRate()} ${$('rStop').value / 100} ${$('rWait').value / 100}`);
}
// Send every rate on this page to the simulator (after loading the page or
// restarting the simulator), and the alert limit to the board.
function applyLocal(withLimit = true) {
  const p = sendRates()
    .then(() => cmd(`nonlin ${$('rNonlin').value / 100}`))
    .then(() => cmd(`inbound ${$('rInb').value / 100}`));
  return withLimit && fw === 'v2' ? p.then(() => cmd(`serial l${+$('crowdLimit').value || 3000}`)) : p;
}

function wire() {
  document.querySelectorAll('.fw-switch button').forEach((b) => (b.onclick = () => setFirmware(b.dataset.fw).then(() => applyLocal())));
  $('vehOn').onchange = () => { rateLabels(); cmd(`vehicles ${vehRate()}`); };
  $('rVeh').onchange = () => { rateLabels(); cmd(`vehicles ${vehRate()}`); };
  $('rPed').oninput = () => { $('nPed').value = pedFromSlider(+$('rPed').value); rateLabels(); };
  $('rPed').onchange = () => { $('nPed').value = pedFromSlider(+$('rPed').value); sendRates(); };
  $('nPed').onchange = () => { $('nPed').value = pedRate(); $('rPed').value = sliderFromPed(pedRate()); sendRates(); };
  $('crowdForm').onsubmit = (e) => {
    e.preventDefault();
    const n = Math.max(1, Math.min(100000, Math.round(+$('crowdN').value || 0)));
    $('crowdN').value = n;
    cmd(`spawn crowd ${n} ${$('spawnLane').value}`.trim());
  };
  $('crowdLimit').onchange = () => {
    const v = Math.max(1, Math.round(+$('crowdLimit').value || 3000));
    $('crowdLimit').value = v; cmd(`serial l${v}`);
  };
  // the browser only allows the alarm sound after a click on the page
  document.addEventListener('pointerdown', () => { try { audio = audio || new (window.AudioContext || window.webkitAudioContext)(); audio.resume(); } catch { /* ignore */ } }, { once: true });
  $('speed').oninput = () => { const v = SPEEDS[+$('speed').value]; $('speedOut').textContent = v + '×'; cmd('speed ' + v); };
  $('pauseBtn').onclick = () => cmd(frame && frame.paused ? 'resume' : 'pause');
  // fresh start: counts to zero and an empty ghat (o0 = headcount of 0)
  $('resetBtn').onclick = () => { cmd('serial r').then(() => cmd('resettruth')).then(() => cmd('serial o0')); history = []; lastHistT = -1; };
  $('restartBtn').onclick = () => setFirmware(fw).then(() => cmd('speed ' + SPEEDS[+$('speed').value])).then(applyLocal);
  document.querySelectorAll('[data-spawn]').forEach((b) => (b.onclick = () => cmd(`spawn ${b.dataset.spawn} ${$('spawnLane').value}`.trim())));
  ['rGrp', 'rStop', 'rWait'].forEach((id) => ($(id).onchange = sendRates));
  ['rGrp', 'rVeh', 'rStop', 'rWait', 'rNonlin', 'rInb'].forEach((id) => ($(id).oninput = rateLabels));
  $('rNonlin').onchange = () => { rateLabels(); cmd(`nonlin ${$('rNonlin').value / 100}`); };
  $('rInb').onchange = () => { rateLabels(); cmd(`inbound ${$('rInb').value / 100}`); };
  $('serialForm').onsubmit = (e) => { e.preventDefault(); const v = $('serialInput').value; if (v) { cmd('serial ' + v); $('serialInput').value = ''; } };
  $('showTelemetry').onchange = () => { serialLines = []; };
  for (let i = 0; i < NUM_PADS; i++) {
    const o = document.createElement('option');
    o.value = i; o.textContent = `Entry plate ${i}`;
    $('calPad').appendChild(o);
  }
  $('calPut').onclick = () => cmd(`weight ${$('calPad').value} ${+$('calKg').value || 0}`);
  $('calClear').onclick = () => cmd('weight clear');
  $('benchBtn').onclick = runBench;
  rateUI(DEFAULT_RATES);
  renderQuick();
  applyLocal(false);  // the page's rates go to the simulator; vehicles stay off until switched on (the limit is read from the board)
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
  let html = '<div class="table-wrap"><table><thead><tr><th>Scenario</th><th>Crossings</th><th></th><th>← Entry gate</th><th>→ Exit gate</th><th>Inside at end: true / counted</th></tr></thead><tbody>';
  let tT = 0, tE = 0;
  for (const row of res.table) {
    const c = row.v2;
    if (!c) continue;
    const extra = c.turnedBack ? `${c.turnedBack} stepped on & turned back` : '';
    html += `<tr><td><b>${row.scenario}</b><div class="scen-desc">${row.description}${extra ? '<br>' + extra : ''}</div></td>`;
    const ins = c.inside;
    const insideCell = ins ? `<td class="num">${ins.truthEnd.map((t, i) => `${t} / ${ins.countedEnd[i]}`).join(' · ')}<div class="scen-desc">mean error ${ins.errEndMean.toFixed(1)} people</div></td>` : '<td></td>';
    if (c.turnedBack > c.truth) { // mostly people who did NOT cross
      const per = c.counted / Math.max(1, c.turnedBack + c.truth);
      html += `<td class="num acc-bad" colspan="2">counted ${c.counted} for ${c.turnedBack + c.truth} people on the strip</td><td class="num scen-desc" colspan="2">${per.toFixed(2)} counts per person · only ${c.truth} crossed · in/out cancel</td>${insideCell}`;
    } else if (c.accuracy === null) {
      const per = c.vehiclesOverStrip ? (c.counted / c.vehiclesOverStrip) : 0;
      html += `<td class="num ${per < 0.1 ? 'acc-good' : 'acc-bad'}">${c.counted} false counts</td><td class="num scen-desc">from ${c.vehiclesOverStrip} vehicles (${(per * 100).toFixed(1)}%)</td><td></td><td></td><td></td>`;
    } else {
      tT += c.truth; tE += Math.abs(c.counted - c.truth);
      const a = Math.max(0, c.accuracy);
      html += `<td style="min-width:130px"><div class="bar v2"><span style="width:${(a * 100).toFixed(1)}%"></span><em></em></div></td>`;
      html += `<td class="num">${pct(c.accuracy)} <span class="scen-desc">${c.counted}/${c.truth}</span></td>`;
      const di = c.dirs ? c.dirs.in : null, dout = c.dirs ? c.dirs.out : null;
      html += `<td class="num">${di ? pct(di.accuracy) + ` <span class="scen-desc">${di.counted}/${di.truth}</span>` : '–'}</td>`;
      html += `<td class="num">${dout ? pct(dout.accuracy) + ` <span class="scen-desc">${dout.counted}/${dout.truth}</span>` : '–'}</td>`;
      html += insideCell;
    }
    html += '</tr>';
  }
  if (tT) {
    const a = 1 - tE / tT;
    html += `<tr><td><b>All pedestrian scenarios</b><div class="scen-desc">sum of |error| / people</div></td><td><div class="bar v2"><span style="width:${(a * 100).toFixed(1)}%"></span><em></em></div></td><td class="num">${pct(a)} <span class="scen-desc">${tT} people</span></td><td></td><td></td><td></td></tr>`;
  }
  html += '</tbody></table></div><p class="hint" style="margin-top:8px">Dashed line = 90 % target. Accuracy = 1 − |counted − true| / true per scenario, summed over seeds. "Inside at end" = net people who went into the premises during each 10-minute run (one pair per seed).</p>';
  $('benchTable').innerHTML = html;
}

wire();
connect();
pollBench();
fetch('/api/state').then((r) => r.json()).then((s) => { if (s.fw) markFirmware(s.fw); });
requestAnimationFrame(render);
