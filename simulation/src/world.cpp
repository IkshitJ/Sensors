#include "world.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace sim {

static const double kPi = 3.14159265358979323846;
static const double STRIP_XC = (STRIP_X0 + STRIP_X1) / 2;

static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

Rates presetRates(const std::string& n, bool* ok) {
  Rates r;
  if (ok) *ok = true;
  if (n == "idle") {
  } else if (n == "sparse") {
    r.pedPerMin = 4;
  } else if (n == "normal") {
    r.pedPerMin = 14; r.groupFrac = 0.25; r.vehPerMin = 4; r.stopFrac = 0.1;
  } else if (n == "rush") {
    r.pedPerMin = 36; r.groupFrac = 0.35; r.vehPerMin = 3; r.stopFrac = 0.1;
  } else if (n == "dense") {
    r.pedPerMin = 75; r.groupFrac = 0.4;
  } else if (n == "festival") {
    r.pedPerMin = 140; r.groupFrac = 0.5;
  } else if (n == "signal") {  // platoons released by a traffic signal every 60 s
    r.burstPeriod = 60; r.burstSize = 45; r.burstWindow = 14; r.vehPerMin = 6;
  } else if (n == "traffic") {
    r.vehPerMin = 24; r.stopFrac = 0.08;
  } else if (n == "mixed") {
    r.pedPerMin = 18; r.groupFrac = 0.3; r.vehPerMin = 10; r.stopFrac = 0.2;
  } else if (n == "waiting") {
    r.pedPerMin = 12; r.groupFrac = 0.2; r.waitFrac = 0.3; r.vehPerMin = 2;
  // ---- non-linear walking (Maha Kumbh-style crowds) --------------------------
  } else if (n == "weave") {       // everyone meanders, alone
    r.pedPerMin = 6; r.nonlinFrac = 1; r.nonlinStyle = WEAVE;
  } else if (n == "drift") {       // everyone cuts diagonally, often across lanes
    r.pedPerMin = 6; r.nonlinFrac = 1; r.nonlinStyle = DRIFT;
  } else if (n == "jostle") {      // everyone jostles: random heading, sidesteps
    r.pedPerMin = 6; r.nonlinFrac = 1; r.nonlinStyle = JOSTLE;
  } else if (n == "uturn") {       // everyone steps onto the strip and turns back
    r.pedPerMin = 6; r.nonlinFrac = 1; r.nonlinStyle = UTURN;
  } else if (n == "meander") {     // all non-linear (realistic mix), groups, some vehicles
    r.pedPerMin = 14; r.groupFrac = 0.25; r.vehPerMin = 4; r.stopFrac = 0.1; r.nonlinFrac = 1;
  } else if (n == "kumbh") {       // festival crowd, 60 % non-linear
    r.pedPerMin = 140; r.groupFrac = 0.5; r.nonlinFrac = 0.6;
  // ---- tidal flows at the ghat gate ------------------------------------------
  } else if (n == "snan") {        // bathing rush: 85 % heading INTO the ghat, 40 % non-linear
    r.pedPerMin = 60; r.groupFrac = 0.4; r.nonlinFrac = 0.4; r.inboundFrac = 0.85;
  } else if (n == "return") {      // after the snan: 85 % coming OUT, 40 % non-linear
    r.pedPerMin = 60; r.groupFrac = 0.4; r.nonlinFrac = 0.4; r.inboundFrac = 0.15;
    r.startInside = 1200;  // the ghat is full from the snan
  } else {
    if (ok) *ok = false;
  }
  return r;
}

World::World(uint32_t seed) : rng_(seed) {}

static double uni(std::mt19937& g, double a, double b) { return std::uniform_real_distribution<double>(a, b)(g); }
static double nrm(std::mt19937& g, double m, double s) { return std::normal_distribution<double>(m, s)(g); }


// ---------------------------------------------------------------------------
//  Pedestrians (pilgrims walking to and from the ghat, in both directions)
// ---------------------------------------------------------------------------
// Footsteps keep their rhythm (cadence in real time); what the crowd changes
// is their LENGTH. The crowd clock integrates the crowd speed factor (1 on a
// free road, 0.1 in a crush): a walker's progress is speed x (clock - clock0),
// so everyone slows down as the crowd thickens around them and speeds up
// again as it thins, shuffling with short steps at a near-normal rhythm.
double World::crowdClock(double t, int c) const { return clockVal_[c] + (t - clockT_) * cfVal_[c]; }

double World::gaitTime(const Pedestrian& p, double t) const {
  if (!p.hasPause || t < p.pauseStart) return t;
  if (t < p.pauseStart + p.pauseDur) return p.pauseStart;
  return t - p.pauseDur;
}

// Walking speed relative to free flow, from the crowd density (people / m2).
// Weidmann's (1993) speed-density curve, but with the closer packing of
// Indian festival crowds: Weidmann's jam density of 5.4 / m2 comes from
// European data; pilgrims at a ghat keep moving at 6-8 / m2. In such a crowd
// nobody stops: they shuffle on at ~0.1-0.15 m/s, however packed it gets.
static const double CRAWL = 0.10;        // shuffling speed, relative to free walking
static double weidmann(double rho) {
  if (rho < 0.05) return 1.0;
  return std::max(CRAWL, 1.0 - std::exp(-1.913 * (1.0 / rho - 1.0 / JAM_DENSITY)));
}

// Body centre at real time t: between the body positions of the footsteps.
void World::bodyPos(const Pedestrian& p, double t, double& x, double& y) const {
  double te = gaitTime(p, t);
  int n = (int)p.feet.size();
  if (n == 0) { x = p.xStart; y = p.y0; return; }
  double u = (te - p.t0) * p.cadence;
  if (u <= 0) { x = p.feet[0].bx; y = p.feet[0].by; return; }
  int k = (int)std::floor(u);
  if (k >= n - 1) { x = p.feet[n - 1].bx; y = p.feet[n - 1].by; return; }
  double f = u - k;
  x = p.feet[k].bx + f * (p.feet[k + 1].bx - p.feet[k].bx);
  y = p.feet[k].by + f * (p.feet[k + 1].by - p.feet[k].by);
}

// Census of each corridor: the crowd in one does not slow the other down.
void World::crowdFactor(double t) const {
  // In huge crowds this is asked for every arrival: counting tens of thousands
  // of people each time stalls the simulation. Re-count at most every 0.25 s.
  if (peds_.size() > 400 && std::fabs(t - cfT_) < 0.25) return;
  // density of people near each strip (10 m of corridor), and the number at
  // the end of the screen where the corridor is entered (room for the
  // backed-up crowd to push on): the entry corridor is entered at WALK_X1,
  // the exit corridor at WALK_X0
  int n[NUM_STRIPS] = {0, 0}, nEdge[NUM_STRIPS] = {0, 0};
  for (const Pedestrian& p : peds_) {
    if (t < p.t0 - 2.0) continue;
    double x, y;
    bodyPos(p, t, x, y);
    int c = p.corr;
    if (std::fabs(x - STRIP_XC) < 5.0) n[c]++;
    if (c == STRIP_ENTRY ? x > WALK_X1 - EDGE_ZONE : x < WALK_X0 + EDGE_ZONE) nEdge[c]++;
  }
  cfT_ = t;
  for (int c = 0; c < NUM_STRIPS; c++) {
    cfRho_[c] = n[c] / (10.0 * CORR_W);
    cfVal_[c] = weidmann(cfRho_[c]);
    // room at the edge; none while the stretch around the strip is packed
    // solid (a walker's speed is set when they enter, so people entering
    // faster than the crowd ahead would otherwise pile up on it beyond crush
    // density)
    double stripRoom = JAM_DENSITY * 10.0 * CORR_W - n[c];
    edgeRoom_[c] = std::min(JAM_DENSITY * EDGE_ZONE * CORR_W - nEdge[c], stripRoom);
  }
}

bool World::edgeFull(int c, int n) { return edgeRoom_[c] < n; }

void World::admitHeld(double t) {
  crowdFactor(t);
  for (int c = 0; c < NUM_STRIPS; c++) {
    while (!held_[c].empty() && !edgeFull(c, held_[c].front().n)) {
      Held h = held_[c].front();
      held_[c].pop_front();
      backlog_[c] -= h.n;
      if (h.n >= 2) spawnGroup(t, h.n, h.dir);
      else spawnPedestrian(t, h.kind, h.dir, h.pause, h.path);
    }
  }
}

// along-walk coordinates of the strip (distance from the pedestrian's entry point)
static double stripA0(int dir) { return dir > 0 ? STRIP_X0 - WALK_X0 : WALK_X1 - STRIP_X1; }
static double stripA1(int dir) { return stripA0(dir) + (STRIP_X1 - STRIP_X0); }

PathStyle World::randomPath() {
  if (rates_.nonlinStyle > STRAIGHT && rates_.nonlinStyle < NUM_PATH_STYLES) return (PathStyle)rates_.nonlinStyle;
  double u = uni(rng_, 0, 1);
  if (u < 0.35) return WEAVE;
  if (u < 0.65) return DRIFT;
  if (u < 0.90) return JOSTLE;
  return UTURN;
}

// ---------------------------------------------------------------------------
//  Footstep generation, one step at a time. Works in walk coordinates: a =
//  distance along the lane in the walking direction (x = xStart + dir*a), y =
//  across the road, theta = heading relative to the lane direction (0 =
//  straight on, +-pi = walking back). Cadence stays constant (footstep k lands
//  at k/cadence, real time) so the force model and pedContacts() need no
//  changes; the step LENGTH follows the crowd (crowd clock), and for the
//  non-linear styles its DIRECTION varies:
//   STRAIGHT : a near-straight line along the lane
//   WEAVE    : sinusoidal meander, heading swinging +-10..30 deg
//   DRIFT    : diagonal lane change of 1-4 m somewhere near the strip
//   JOSTLE   : crowd jostling: random-walk heading, uneven step length and
//              occasional sidesteps (shuffles with little forward progress)
//   UTURN    : walks up to the strip, steps onto/near it, turns round and
//              walks back (does NOT cross)
// ---------------------------------------------------------------------------
void World::setupPath(Pedestrian& p) {
  std::mt19937& g = rng_;
  PathState& q = p.ps;
  const double yLo = corrY0(p.corr) + 0.15, yHi = corrY1(p.corr) - 0.15;
  const double aS0 = stripA0(p.dir), aSC = aS0 + 0.5 * (STRIP_X1 - STRIP_X0);
  q.a = 0.15 + nrm(g, 0, 0.02);
  q.y = p.y0;
  q.th = 0;
  q.yPref = p.y0;
  // natural walking drifts across the corridor every few metres (towards a
  // gap, away from a slow group); people walking together keep their line
  q.aRetarget = p.group >= 0 ? 1e9 : uni(g, 1.5, 8.0);
  if (p.path == WEAVE) {
    double amp = uni(g, 0.17, 0.52), lam = uni(g, 3.0, 8.0);  // 10-30 deg, 3-8 m wavelength
    q.wK = 2 * kPi / lam;
    q.wAy = std::tan(amp) / q.wK;
    q.wPh = uni(g, 0, 2 * kPi);
  } else if (p.path == DRIFT) {
    // a diagonal cut of 1-4 m across the corridor
    double yT = p.y0 + (uni(g, 0, 1) < 0.5 ? -1 : 1) * uni(g, 1.0, 4.0);
    if (yT < yLo + 0.1 || yT > yHi - 0.1) yT = 2 * p.y0 - yT;  // the other way if that runs into the barricade
    yT = clampd(yT, yLo + 0.1, yHi - 0.1);
    q.dDy = yT - p.y0;
    double ang = uni(g, 0.35, 0.79);  // 20-45 deg at the steepest point
    q.dL = std::max(1.0, 1.5 * std::fabs(q.dDy) / std::tan(ang));
    q.dA0 = aSC - q.dL / 2 + uni(g, -q.dL / 2 - 1.0, q.dL / 2 + 1.0);
  } else if (p.path == UTURN) {
    q.aTurn = aS0 + uni(g, -0.35, 0.75);  // front foot lands just before / on the strip
    q.turnSign = uni(g, 0, 1) < 0.5 ? -1 : 1;
  }
}

void World::genFeet(Pedestrian& p, double t) {
  int need = (int)std::floor((gaitTime(p, t + 1.0) - p.t0) * p.cadence) + 3;
  while (!p.ps.done && (int)p.feet.size() < need) genFoot(p);
}

void World::genFoot(Pedestrian& p) {
  std::mt19937& g = rng_;
  PathState& q = p.ps;
  const double d = p.dir, c = p.cadence;
  const double yLo = corrY0(p.corr) + 0.15, yHi = corrY1(p.corr) - 0.15;
  int k = (int)p.feet.size();
  double tOn = p.t0 + k / c;
  double x = p.xStart + d * q.a;
  if (x < WALK_X0 - 0.01 || x > WALK_X1 + 0.01 || k >= 20000) {  // left the view: the path is complete
    q.done = true;
    p.pastStrip = true;
    p.tEnd = (k ? p.feet.back().tOn : p.t0) + p.stance + 1.0;
    return;
  }
  // the length of this step: the free step, scaled by the crowd speed factor
  // over the step (the crowd clock), never shorter than a shuffle
  double tr = tOn + (p.hasPause && tOn >= p.pauseStart ? p.pauseDur : 0);  // real time of this step
  double L = p.speed * (crowdClock(tr + 1 / c, p.corr) - crowdClock(tr, p.corr));
  if (L < 0.03) L = 0.03;
  double by, stepA, stepY;
  if (p.path == STRAIGHT) {
    // natural walking: the heading wanders a few degrees from step to step
    // (a mean-reverting random walk) and is pulled gently towards the spot
    // across the corridor the walker is drifting to, which changes every few
    // metres. In a crush the steps are short, so the drift shrinks with them.
    by = q.y;
    if (q.a >= q.aRetarget) {
      q.yPref = uni(g, yLo + 0.3, yHi - 0.3);
      q.aRetarget = q.a + uni(g, 4.0, 14.0);
    }
    double pull = clampd(0.25 * (q.yPref - q.y), -0.22, 0.22);
    q.th = clampd(0.78 * q.th + 0.22 * pull + nrm(g, 0, 0.055), -0.35, 0.35);
    if ((q.y > yHi - 0.3 && q.th > 0) || (q.y < yLo + 0.3 && q.th < 0)) q.th = -0.5 * q.th;  // off the barricade
    stepA = L * std::cos(q.th) + nrm(g, 0, 0.02);
    stepY = L * std::sin(q.th);
  } else if (p.path == WEAVE || p.path == DRIFT) {
    auto yProfile = [&](double a) {
      if (p.path == WEAVE) return p.y0 + q.wAy * (std::sin(q.wK * a + q.wPh) - std::sin(q.wPh));
      double u = clampd((a - q.dA0) / q.dL, 0, 1);
      return p.y0 + q.dDy * u * u * (3 - 2 * u);
    };
    double y0 = yProfile(q.a);
    by = clampd(y0, yLo, yHi);
    double slope = (yProfile(q.a + 0.01) - yProfile(q.a - 0.01)) / 0.02;
    q.th = (by == y0) ? std::atan(slope) : 0.0;
    stepA = L * std::cos(q.th) + nrm(g, 0, 0.02);
    stepY = 0;  // follows the profile
  } else if (p.path == JOSTLE) {
    by = q.y;
    // heading: mean-reverting random walk (pushed around by the crowd)
    if (k > 0) q.th = clampd(0.75 * q.th + nrm(g, 0, 0.14), -0.6, 0.6);
    if ((q.y > yHi - 0.3 && q.th > 0) || (q.y < yLo + 0.3 && q.th < 0)) q.th = -0.5 * q.th;
    double f = clampd(nrm(g, 1, 0.18), 0.55, 1.25);  // uneven step length
    if (uni(g, 0, 1) < 0.12) {                       // sidestep / shuffle
      stepA = 0.15 * L;
      stepY = (uni(g, 0, 1) < 0.5 ? -1 : 1) * uni(g, 0.18, 0.35);
    } else {
      stepA = L * f * std::cos(q.th);
      stepY = L * f * std::sin(q.th);
    }
  } else {  // UTURN
    by = q.y;
    if (q.turnStep == 0 && q.a >= q.aTurn) q.turnStep = 1;
    double sl = L;
    if (q.turnStep >= 1 && q.turnStep <= 3) {  // pivot: 60 deg per step, short steps
      q.th = q.turnSign * q.turnStep * kPi / 3;
      sl = 0.3 * L;
      q.turnStep++;
    } else if (q.turnStep > 3) {
      q.th = q.turnSign * kPi;
    }
    stepA = sl * std::cos(q.th);
    stepY = sl * std::sin(q.th);
  }
  double th = q.th;
  double hx = d * std::cos(th), hy = std::sin(th);
  double px = -d * std::sin(th), py = std::cos(th);  // left of the heading (0,1 when straight)
  double side = (k % 2 == 0) ? -1.0 : 1.0;
  double fx = x + side * 0.09 * px;
  double fy = clampd(by + side * 0.09 * py + nrm(g, 0, 0.015), yLo - 0.1, yHi + 0.1);
  Footfall ff;
  ff.x = fx; ff.y = fy; ff.tOn = tOn;
  ff.hx = hx; ff.hy = hy; ff.bx = x; ff.by = by;
  ff.step = std::fabs(stepA);
  p.feet.push_back(ff);
  q.a += stepA;
  q.y = clampd(q.y + stepY, yLo, yHi);

  // ---- bookkeeping for the strip ----
  // waiting on the strip: stop on the first foot past the chosen spot
  if (p.pauseDur > 0 && !p.hasPause && d * (fx - p.pauseX) >= 0) {
    p.hasPause = true;
    p.pauseStart = tOn + 0.5 * p.stance;
    p.pauseX = fx;
    p.pauseY[0] = fy;
    p.pauseY[1] = fy - side * 0.18;
  }
  if (fx + 0.1 >= STRIP_X0 && fx - 0.1 <= STRIP_X1) {  // a foot on the strip
    p.touchesStrip = true;
    p.tLastOnStrip = std::max(p.tLastOnStrip, tOn + p.stance);
  }
  if (fx + 0.25 >= STRIP_X0 && fx - 0.25 <= STRIP_X1) {  // a sole can load a plate
    if (p.tPadOn < 0) p.tPadOn = tOn;
    p.tPadOff = std::max(p.tPadOff, tOn + p.stance);
  }
  // crossed: the body is beyond the strip in the walking direction; turned
  // back: a U-turner walking away from it again. Either way the strip is
  // behind them.
  double aBody = d * (x - p.xStart);
  if (aBody > stripA1(p.dir) + 0.3) { p.crosses = true; p.pastStrip = true; }
  if (p.path == UTURN && q.turnStep > 3 && aBody < stripA0(p.dir) - 0.3) p.pastStrip = true;
}

void World::buildPedestrian(Pedestrian& p, double forcedSpeed) {
  std::mt19937& g = rng_;
  switch (p.kind) {
    case ADULT:
      p.mass = clampd(nrm(g, 72, 13), 45, 135);
      if (uni(g, 0, 1) < 0.25) p.mass += uni(g, 2, 10);  // bag / backpack
      p.speed = clampd(nrm(g, 1.35, 0.17), 0.9, 1.9);
      p.cadence = clampd(nrm(g, 1.85, 0.1), 1.6, 2.1);
      break;
    case TEEN:
      p.mass = clampd(nrm(g, 54, 9), 35, 85);
      p.speed = clampd(nrm(g, 1.4, 0.18), 0.9, 2.0);
      p.cadence = clampd(nrm(g, 1.9, 0.1), 1.65, 2.2);
      break;
    case CHILD:
      p.mass = clampd(nrm(g, 24, 7), 12, 40);
      p.speed = clampd(nrm(g, 1.15, 0.2), 0.7, 1.7);
      p.cadence = clampd(nrm(g, 2.3, 0.15), 1.9, 2.7);
      break;
    case ELDER:
      p.mass = clampd(nrm(g, 68, 12), 42, 110);
      p.speed = clampd(nrm(g, 0.95, 0.12), 0.6, 1.2);
      p.cadence = clampd(nrm(g, 1.65, 0.1), 1.4, 1.9);
      break;
    case RUNNER:
      p.mass = clampd(nrm(g, 70, 11), 45, 110);
      p.speed = clampd(nrm(g, 3.0, 0.4), 2.2, 4.2);
      p.cadence = clampd(nrm(g, 2.8, 0.15), 2.4, 3.2);
      break;
  }
  if (forcedSpeed > 0) p.speed = forcedSpeed * clampd(nrm(g, 1, 0.02), 0.95, 1.05);  // walking together
  // dir is chosen by the spawner: -1 walks from the marg (right) into the
  // ghat premises (left) along the entry corridor, +1 walks out of the
  // premises onto the marg along the exit corridor
  p.corr = corridorOf(p.dir);
  p.xStart = p.dir > 0 ? WALK_X0 : WALK_X1;
  // (the crowd in their corridor slows everyone down through its crowd clock as they go)
  p.clock0 = crowdClock(p.t0, p.corr);

  // step around a vehicle that is standing on the strip when we get there
  double tArr = p.t0 + std::fabs(STRIP_XC - p.xStart) / (p.speed * cfVal_[p.corr]);
  for (const Vehicle& v : vehs_) {
    if (v.stopDur <= 0) continue;
    double tS = v.t0 + std::fabs(v.stopFront - v.x0) / v.v;
    if (tArr < tS - 2.0 || tArr > tS + v.stopDur + 1.0) continue;
    double lo = v.centreY - v.wid / 2 - 0.35, hi = v.centreY + v.wid / 2 + 0.35;
    if (p.y0 > lo && p.y0 < hi) {
      double a = lo, b = hi;
      bool aOk = a > corrY0(p.corr) + 0.2, bOk = b < corrY1(p.corr) - 0.2;
      p.y0 = (aOk && (!bOk || p.y0 - lo < hi - p.y0)) ? a : (bOk ? b : p.y0);
    }
  }

  p.stepLen = p.speed / p.cadence;
  const double c = p.cadence;
  if (p.kind == RUNNER) {
    p.stance = 0.7 / c;  // ~35 % of the stride: flight phase in between
    p.shapeA = 1; p.shapeB = 0;
    p.shapeNorm = 1.0 / (c * p.stance * (2.0 / kPi));
  } else {
    p.stance = (p.kind == ELDER ? 1.3 : 1.2) / c;  // ~60-65 % stance, double support
    p.shapeA = 1.21 * clampd(nrm(g, 1, 0.06), 0.85, 1.15);
    p.shapeB = 0.46 * clampd(nrm(g, 1, 0.15), 0.6, 1.4);
    // normalise so every step delivers exactly body-weight x step-period of impulse
    p.shapeNorm = 1.0 / (c * p.stance * (2 * p.shapeA / kPi + 2 * p.shapeB / (3 * kPi)));
  }
  // waiting on the strip: where (the stop happens on the first foot past it)
  p.hasPause = false;
  if (p.pauseDur > 0) p.pauseX = uni(g, STRIP_X0 + 0.25, STRIP_X1 - 0.25);
  p.feet.clear();
  p.touchesStrip = p.crosses = p.pastStrip = false;
  p.tLastOnStrip = p.tPadOn = p.tPadOff = -1;
  p.tEnd = 1e18;
  setupPath(p);
  genFeet(p, p.t0);  // the first steps, so they can be drawn before they set off
}

static PedKind randomKind(std::mt19937& g, const Rates& r) {
  double u = uni(g, 0, 1);
  if (u < r.childFrac) return CHILD;
  if (u < r.childFrac + r.runnerFrac) return RUNNER;
  if (u < r.childFrac + r.runnerFrac + r.elderFrac) return ELDER;
  if (u < r.childFrac + r.runnerFrac + r.elderFrac + 0.10) return TEEN;
  return ADULT;
}

int World::randomDir() { return uni(rng_, 0, 1) < rates_.inboundFrac ? -1 : 1; }

// Walking OUT needs n people inside who are not already walking: otherwise
// these are new pilgrims arriving from the marg, walking IN.
int World::pickDir(int dir, int n) {
  int d = dir != 0 ? (dir < 0 ? -1 : 1) : randomDir();
  if (d > 0 && atGhat() < n) d = -1;
  return d;
}

void World::takeFromGhat(Pedestrian& p) {
  // a random person among those at the ghat (anonymous ones first in proportion)
  int total = atGhat();
  int k = (int)uni(rng_, 0, total);
  if (k >= total) k = total - 1;
  if (k < anonInside_) { anonInside_--; return; }  // details unknown: keep the random ones
  k -= anonInside_;
  p.kind = (PedKind)atGhat_[k].kind;
  p.mass = atGhat_[k].mass;
  atGhat_[k] = atGhat_.back();
  atGhat_.pop_back();
}

void World::setInside(int n) {
  if (n < 0) n = 0;
  int delta = n - insideNow_;
  anonInside_ += delta;
  // fewer than counted: the extra people leave from those at the ghat
  while (anonInside_ < 0 && !atGhat_.empty()) { atGhat_.pop_back(); anonInside_++; }
  if (anonInside_ < 0) { delta -= anonInside_; anonInside_ = 0; }  // the rest are walking on screen
  insideNow_ += delta;
}

void World::spawnPedestrian(double t, int kind, int dir, double pause, int path) {
  Pedestrian p;
  p.id = nextId_++;
  p.kind = kind < 0 ? randomKind(rng_, rates_) : (PedKind)kind;
  // pilgrims use the whole width of the marg, in both directions; only people
  // who are inside can walk out
  p.dir = pickDir(dir, 1);
  p.corr = corridorOf(p.dir);
  // the end of their corridor is packed solid: they wait beyond the edge of
  // the screen and push on as soon as there is room (admitHeld)
  crowdFactor(t);
  if (edgeFull(p.corr, 1)) {
    held_[p.corr].push_back({t, kind, p.dir, path, 1, pause});
    backlog_[p.corr]++;
    return;
  }
  edgeRoom_[p.corr] -= 1;
  double keepMass = -1;
  if (p.dir > 0) {
    int kindWas = p.kind;
    p.mass = -1;
    takeFromGhat(p);
    // a returning pilgrim keeps who they were, unless a kind was asked for
    if (p.mass > 0 && kind < 0) keepMass = p.mass;
    else p.kind = (PedKind)kindWas;
  }
  p.y0 = corrY(p.corr, 0);
  p.t0 = t;
  if (pause < 0) pause = 0;
  if (pause == 0 && p.kind != RUNNER && uni(rng_, 0, 1) < rates_.waitFrac) pause = uni(rng_, 3, 25);
  // path style (only touches the RNG when non-linear walking is enabled, so
  // the original scenarios replay exactly)
  if (path >= 0 && path < NUM_PATH_STYLES) p.path = (PathStyle)path;
  else if (rates_.nonlinFrac > 0 && uni(rng_, 0, 1) < rates_.nonlinFrac) p.path = randomPath();
  if (p.path == UTURN && p.kind == RUNNER) p.path = WEAVE;
  if (p.path == UTURN) pause = 0;  // they turn back instead of waiting
  p.pauseDur = pause;
  buildPedestrian(p);
  if (keepMass > 0) p.mass = keepMass;  // the same pilgrim walking back
  peds_.push_back(p);
}

void World::spawnCrowd(double t, int n, int dir) {
  // a surge: n people arrive together and push onto the marg at once, spread
  // over its width and a few seconds; nobody waits at the gate. The denser it
  // gets, the slower everyone shuffles (crowdFactor).
  for (int i = 0; i < n; i++) spawnPedestrian(t + uni(rng_, 0, 1.0), -1, dir);
}

// Where across corridor c a walker (or a group `width` wide) starts: anywhere
// on its width (the corridor is one-way, so no lanes form inside it).
double World::corrY(int c, double width) {
  double lo = corrY0(c) + 0.3, hi = corrY1(c) - 0.3 - width;
  return uni(rng_, lo, std::max(lo + 0.01, hi));
}

void World::spawnGroup(double t, int n, int dir) {
  if (n < 2) n = 2;
  if (n > 6) n = 6;
  dir = pickDir(dir, n);  // a group walks the same way; out only if all n are inside
  int c = corridorOf(dir);
  crowdFactor(t);
  if (edgeFull(c, n)) {  // packed solid at their end of the corridor: back up beyond the screen
    held_[c].push_back({t, -1, dir, -1, n, 0});
    backlog_[c] += n;
    return;
  }
  edgeRoom_[c] -= n;
  int gid = nextGroup_++;
  double spacing = uni(rng_, 0.48, 0.65);  // walking side by side
  double width = spacing * (n - 1);
  double base = corrY(c, width);
  double groupSpeed = clampd(nrm(rng_, 1.25, 0.15), 0.85, 1.7);  // free walking; the crowd clock slows them
  bool family = uni(rng_, 0, 1) < 0.25;
  // a non-linear group jostles: everyone keeps roughly abreast but is pushed
  // around individually (random heading, uneven steps, sidesteps)
  bool jostle = rates_.nonlinFrac > 0 && uni(rng_, 0, 1) < rates_.nonlinFrac;
  for (int i = 0; i < n; i++) {
    Pedestrian p;
    p.id = nextId_++;
    p.group = gid;
    p.dir = dir;
    if (jostle) p.path = JOSTLE;
    p.kind = (family && i == n - 1) ? CHILD : (uni(rng_, 0, 1) < 0.12 ? TEEN : ADULT);
    p.y0 = clampd(base + i * spacing + nrm(rng_, 0, 0.04), corrY0(c) + 0.2, corrY1(c) - 0.2);
    p.t0 = t + uni(rng_, 0, 0.45);  // not perfectly abreast
    double keepMass = -1;
    if (dir > 0) {  // returning from the ghat: the same people who went in
      int kindWas = p.kind;
      p.mass = -1;
      takeFromGhat(p);
      if (p.mass > 0) keepMass = p.mass;
      else p.kind = (PedKind)kindWas;
    }
    buildPedestrian(p, groupSpeed);
    if (keepMass > 0) p.mass = keepMass;
    peds_.push_back(p);
  }
}

void World::pedContacts(const Pedestrian& p, double t, double* xs, double* ys, double* fs, int& n) const {
  n = 0;
  if (p.hasPause && t >= p.pauseStart && t < p.pauseStart + p.pauseDur) {
    double sway = 0.03 * std::sin(2 * kPi * 0.25 * t + p.id);
    for (int i = 0; i < 2; i++) {
      xs[n] = p.pauseX;
      ys[n] = p.pauseY[i];
      fs[n] = p.mass * 0.5 * (1 + (i == 0 ? sway : -sway));
      n++;
    }
    return;
  }
  double te = gaitTime(p, t);
  if (te < p.t0) return;
  int kf = (int)std::floor((te - p.t0) * p.cadence);
  for (int k = kf - 2; k <= kf; k++) {
    if (k < 0 || k >= (int)p.feet.size()) continue;
    const Footfall& f = p.feet[k];
    double s = (te - f.tOn) / p.stance;
    if (s < 0 || s > 1) continue;
    double g = p.shapeNorm * (p.shapeA * std::sin(kPi * s) + p.shapeB * std::sin(3 * kPi * s));
    // heel-to-toe roll of the centre of pressure along the walking direction:
    // ~20 cm in a normal stride, but a shuffling crowd with 7 cm steps plants
    // the foot flat and rolls no further than it steps
    double roll = std::min(0.20, f.step);
    xs[n] = f.x + f.hx * roll * (s - 0.5);
    ys[n] = f.y + f.hy * roll * (s - 0.5);
    fs[n] = p.mass * std::max(0.0, g);
    n++;
  }
}

// ---------------------------------------------------------------------------
//  Vehicles
// ---------------------------------------------------------------------------
std::string World::randomVehicleType() {
  if (const char* only = getenv("SIM_VTYPE")) return only;  // diagnostics: one type only
  double u = uni(rng_, 0, 1);
  if (u < 0.42) return "car";
  if (u < 0.66) return "moto";
  if (u < 0.76) return "auto";
  if (u < 0.85) return "suv";
  if (u < 0.91) return "bicycle";
  if (u < 0.96) return "truck";
  return "bus";
}

struct VehSpec {
  const char* type;
  double massLo, massHi, payLo, payHi, len, wid, track, patch, tyreW, vLo, vHi;
  bool singleTrack;
  int nAxles;
  double axOff[3], axShare[3];
  int axWheels[3];  // 1 = single central wheel, 2 = left+right
};

static const VehSpec kSpecs[] = {
    {"car", 1150, 1800, 70, 300, 4.3, 1.75, 1.52, 0.16, 0.20, 15, 45, false, 2, {0.9, 3.5, 0}, {0.57, 0.43, 0}, {2, 2, 0}},
    {"suv", 1800, 2500, 70, 400, 4.8, 1.90, 1.62, 0.18, 0.24, 15, 45, false, 2, {0.95, 3.75, 0}, {0.53, 0.47, 0}, {2, 2, 0}},
    {"auto", 330, 400, 70, 350, 2.7, 1.40, 1.15, 0.10, 0.11, 10, 35, false, 2, {0.4, 2.0, 0}, {0.28, 0.72, 0}, {1, 2, 0}},
    {"moto", 120, 180, 60, 150, 2.0, 0.80, 0.0, 0.09, 0.12, 8, 45, true, 2, {0.35, 1.7, 0}, {0.45, 0.55, 0}, {1, 1, 0}},
    {"bicycle", 10, 16, 35, 95, 1.75, 0.60, 0.0, 0.05, 0.04, 8, 22, true, 2, {0.3, 1.35, 0}, {0.4, 0.6, 0}, {1, 1, 0}},
    {"bus", 11000, 16500, 0, 0, 11.0, 2.50, 2.00, 0.22, 0.50, 10, 35, false, 2, {2.4, 8.4, 0}, {0.35, 0.65, 0}, {2, 2, 0}},
    {"truck", 7000, 25000, 0, 0, 9.0, 2.50, 2.00, 0.22, 0.50, 8, 35, false, 3, {1.5, 6.0, 7.3}, {0.28, 0.36, 0.36}, {2, 2, 2}},
};

static const VehSpec* findSpec(const std::string& t) {
  for (const VehSpec& s : kSpecs)
    if (t == s.type) return &s;
  return nullptr;
}

void World::spawnVehicle(double t, const std::string& type, int lane, double stopDur) {
  const VehSpec* s = findSpec(type);
  if (!s) return;
  if (lane < 0) lane = uni(rng_, 0, 1) < 0.5 ? 0 : 1;
  Vehicle v;
  v.id = nextId_++;
  v.type = s->type;
  v.mass = uni(rng_, s->massLo, s->massHi) + (s->payHi > 0 ? uni(rng_, s->payLo, s->payHi) : 0);
  v.len = s->len;
  v.wid = s->wid;
  v.lane = lane;
  v.v = uni(rng_, s->vLo, s->vHi) / 3.6;
  v.patch = s->patch;
  v.tyreW = s->tyreW;
  v.singleTrack = s->singleTrack;
  v.bouncePhase = uni(rng_, 0, 2 * kPi);
  // lane 0 drives +x along the exit corridor, lane 1 drives -x along the entry corridor
  int c = lane == 0 ? STRIP_EXIT : STRIP_ENTRY;
  v.centreY = corrY0(c) + (s->singleTrack ? uni(rng_, 0.6, CORR_W - 0.6) : CORR_W / 2 + nrm(rng_, 0, 0.4));
  for (int a = 0; a < s->nAxles; a++) v.axles.push_back({s->axOff[a], v.mass * s->axShare[a]});
  v.x0 = lane == 0 ? -30.0 : 34.0;
  v.t0 = t;
  if (stopDur > 0) {
    // stops with its front wheels standing on the strip
    double wheelX = STRIP_XC + uni(rng_, -0.35, 0.35);
    v.stopFront = lane == 0 ? wheelX + s->axOff[0] : wheelX - s->axOff[0];
    v.stopDur = stopDur;
  }
  double dist = 64.0 + v.len;
  v.tEnd = t + dist / v.v + (v.stopDur > 0 ? v.stopDur : 0) + 0.5;
  vehTotal_++;
  if (trace) fprintf(stderr, "%9.3f VEH %-7s lane %c %5.0fkg %4.1fkm/h stop=%.1fs\n", t, v.type.c_str(), lane ? 'B' : 'A', v.mass, v.v * 3.6, v.stopDur);
  vehs_.push_back(v);
}

double World::vehFront(const Vehicle& v, double t) const {
  double s = v.lane == 0 ? 1.0 : -1.0;
  double f = v.x0 + s * v.v * (t - v.t0);
  if (v.stopDur > 0) {
    double tS = v.t0 + std::fabs(v.stopFront - v.x0) / v.v;
    if (t >= tS) f = (t < tS + v.stopDur) ? v.stopFront : v.stopFront + s * v.v * (t - tS - v.stopDur);
  }
  return f;
}

// ---------------------------------------------------------------------------
void World::setStaticWeight(int pad, double kg) {
  for (StaticWeight& w : weights_)
    if (w.pad == pad) { w.kg = kg; return; }
  weights_.push_back({pad, kg});
}

void World::setRates(const Rates& r) {
  // a changed rate restarts its arrival process from "now" (update() schedules
  // it): otherwise switching from a scenario without vehicles never brings any
  if (r.pedPerMin != rates_.pedPerMin) nextPed_ = 1e18;
  if (r.vehPerMin != rates_.vehPerMin) nextVeh_ = 1e18;
  if (r.burstPeriod != rates_.burstPeriod) nextBurst_ = 1e18;
  rates_ = r;
}

void World::update(double t) {
  // the crowd clocks: everyone's gait runs at their corridor's crowd speed factor
  if (t > clockT_) {
    for (int c = 0; c < NUM_STRIPS; c++) clockVal_[c] += (t - clockT_) * cfVal_[c];
    clockT_ = t;
    crowdFactor(t);  // refresh the rates for the next interval
  }
  // schedule each arrival process on its own (same random draws, in the same
  // order, as when they all start together)
  if (nextPed_ > 1e17 && rates_.pedPerMin > 0)
    nextPed_ = t + std::exponential_distribution<double>(rates_.pedPerMin / 60.0)(rng_);
  if (nextVeh_ > 1e17 && rates_.vehPerMin > 0)
    nextVeh_ = t + std::exponential_distribution<double>(rates_.vehPerMin / 60.0)(rng_);
  if (nextBurst_ > 1e17 && rates_.burstPeriod > 0) nextBurst_ = t + 2.0;
  if (spawning_) {
    while (t >= nextPed_) {
      if (uni(rng_, 0, 1) < rates_.groupFrac) {
        double u = uni(rng_, 0, 1);
        spawnGroup(nextPed_, u < 0.55 ? 2 : (u < 0.85 ? 3 : 4));
      } else {
        spawnPedestrian(nextPed_);
      }
      nextPed_ += std::exponential_distribution<double>(rates_.pedPerMin / 60.0)(rng_);
    }
    while (t >= nextVeh_) {
      double stop = uni(rng_, 0, 1) < rates_.stopFrac ? uni(rng_, 2, 20) : 0;
      spawnVehicle(nextVeh_, randomVehicleType(), -1, stop);
      nextVeh_ += std::exponential_distribution<double>(rates_.vehPerMin / 60.0)(rng_);
    }
    while (t >= nextBurst_) {
      int n = (int)rates_.burstSize;
      for (int i = 0; i < n;) {
        double ts = nextBurst_ + uni(rng_, 0, rates_.burstWindow);
        if (uni(rng_, 0, 1) < 0.35) {
          int g = uni(rng_, 0, 1) < 0.7 ? 2 : 3;
          spawnGroup(ts, g);
          i += g;
        } else {
          spawnPedestrian(ts);
          i++;
        }
      }
      nextBurst_ += rates_.burstPeriod;
    }
  }
  admitHeld(t);  // the backed-up crowd pushes on wherever there is room
  for (Pedestrian& p : peds_) genFeet(p, t);  // everyone's next footsteps, at the crowd's pace
  for (Pedestrian& p : peds_) {
    if (!p.counted && p.touchesStrip && p.pastStrip && gaitTime(p, t) >= p.tLastOnStrip) {
      p.counted = true;
      if (trace) fprintf(stderr, "%9.3f PED %d dir %s %s mass %.0f speed %.2f path %d\n", t, p.id, p.dir < 0 ? "IN " : "OUT",
                         p.crosses ? "crossed" : "turned-back", p.mass, p.speed, (int)p.path);
      if (p.crosses) {
        truth_[p.dir < 0 ? DIR_IN : DIR_OUT]++;  // into / out of the ghat premises
        insideNow_ += p.dir < 0 ? 1 : -1;
        truthStyle_[p.path]++;
      } else {
        touchedOnly_++;  // stepped on the strip but turned back
      }
    }
  }
  for (Vehicle& v : vehs_) {
    double f = vehFront(v, t);
    bool past = v.lane == 0 ? f - v.len > STRIP_X1 : f + v.len < STRIP_X0;
    if (!v.counted && past) {
      v.counted = true;
      vehOverStrip_++;
    }
  }
  // clean-up (keep 1 s of history: the ADC averages over its conversion window)
  for (const Pedestrian& p : peds_) {
    if (!p.ps.done || gaitTime(p, t) <= p.tEnd + 1.0) continue;
    // reached the ghat (went in), or turned back on the way out: still inside
    bool inside = p.dir < 0 ? p.crosses : !p.crosses;
    if (inside) atGhat_.push_back({(int)p.kind, p.mass});
  }
  peds_.erase(std::remove_if(peds_.begin(), peds_.end(), [&](const Pedestrian& p) { return p.ps.done && gaitTime(p, t) > p.tEnd + 1.0; }), peds_.end());
  vehs_.erase(std::remove_if(vehs_.begin(), vehs_.end(), [&](const Vehicle& v) { return t > v.tEnd + 1.0; }), vehs_.end());
}

// Distribute force f uniformly over [y0, y1] across the plates of whichever
// strip it falls on. The load stands at x along the plate; a rigid plate on
// cells at both edges hands the marg-side pair the share (x - X0) / depth,
// the ghat-side pair the rest.
static void addSpanY(double out[NUM_PADS_ALL], double* marg, double y0, double y1, double f, double x) {
  double w = y1 - y0;
  if (w <= 0) return;
  double u = (x - STRIP_X0) / (STRIP_X1 - STRIP_X0);
  u = u < 0 ? 0 : (u > 1 ? 1 : u);
  for (int c = 0; c < NUM_STRIPS; c++) {
    double sy0 = corrY0(c);
    int p0 = std::max(0, (int)std::floor((y0 - sy0) / PAD_W));
    int p1 = std::min(NUM_PADS - 1, (int)std::floor((y1 - sy0) / PAD_W));
    for (int p = p0; p <= p1; p++) {
      double a = sy0 + p * PAD_W, b = a + PAD_W;
      double ov = std::min(b, y1) - std::max(a, y0);
      if (ov > 0) {
        out[c * NUM_PADS + p] += f * ov / w;
        if (marg) marg[c * NUM_PADS + p] += f * ov / w * u;
      }
    }
  }
}

void World::padForces(double t, double out[NUM_PADS_ALL], double marg[NUM_PADS_ALL]) const {
  for (int i = 0; i < NUM_PADS_ALL; i++) out[i] = 0;
  if (marg)
    for (int i = 0; i < NUM_PADS_ALL; i++) marg[i] = 0;
  double xs[4], ys[4], fs[4];
  int n;
  for (const Pedestrian& p : peds_) {
    if (t < p.t0 || p.tPadOn < 0) continue;
    double te = gaitTime(p, t);
    if (te < p.tPadOn - 0.01 || te > p.tPadOff + 0.01) continue;  // not on the strip now
    pedContacts(p, t, xs, ys, fs, n);
    for (int i = 0; i < n; i++) {
      // the loaded part of the sole spans ~12 cm around the centre of pressure,
      // so a foot on the plate edge shares its load with the road
      double frac = (std::min(xs[i] + 0.06, STRIP_X1) - std::max(xs[i] - 0.06, STRIP_X0)) / 0.12;
      if (frac <= 0) continue;
      addSpanY(out, marg, ys[i] - 0.045, ys[i] + 0.045, fs[i] * std::min(1.0, frac), xs[i]);  // ~9 cm wide
    }
  }
  for (const Vehicle& v : vehs_) {
    if (t < v.t0) continue;
    const VehSpec* s = findSpec(v.type);
    double front = vehFront(v, t);
    double dirS = v.lane == 0 ? 1.0 : -1.0;
    double rear = front - dirS * v.len;
    if (std::max(front, rear) < STRIP_X0 - 1 || std::min(front, rear) > STRIP_X1 + 1) continue;
    bool moving = std::fabs(vehFront(v, t + 0.05) - front) > 1e-6;
    // suspension bounce while rolling; idle-engine vibration when stopped
    double bounce = 1.0 + (moving ? 0.05 * std::sin(2 * kPi * 1.6 * t + v.bouncePhase)
                                  : 0.01 * std::sin(2 * kPi * 12.0 * t + v.bouncePhase));
    for (size_t a = 0; a < v.axles.size(); a++) {
      double wx = front - dirS * v.axles[a].offset;
      double ovx = std::min(wx + v.patch / 2, STRIP_X1) - std::max(wx - v.patch / 2, STRIP_X0);
      if (ovx <= 0) continue;
      double frac = std::min(1.0, ovx / v.patch);
      int nw = s->axWheels[a];
      for (int w = 0; w < nw; w++) {
        double wy = nw == 1 ? v.centreY : v.centreY + (w == 0 ? -1 : 1) * s->track / 2;
        double load = v.axles[a].load / nw * frac * bounce;
        addSpanY(out, marg, wy - v.tyreW / 2, wy + v.tyreW / 2, load, wx);
      }
    }
  }
  for (const StaticWeight& w : weights_)  // calibration weights sit in the plate centre
    if (w.pad >= 0 && w.pad < NUM_PADS_ALL) {
      out[w.pad] += w.kg;
      if (marg) marg[w.pad] += 0.5 * w.kg;
    }
}

double World::vibrationKg(double t) const {
  double s = 0;
  for (const Vehicle& v : vehs_) {
    double c = vehFront(v, t) - (v.lane == 0 ? 1 : -1) * v.len / 2;
    if (std::fabs(c - STRIP_XC) < 10 && t >= v.t0) s += v.mass > 5000 ? 1.2 : 0.2;
  }
  return s;
}

void World::snapshot(double t, std::vector<PedView>& pv, std::vector<VehView>& vv) const {
  pv.clear();
  vv.clear();
  for (const Pedestrian& p : peds_) {
    PedView o;
    o.id = p.id;
    o.kind = p.kind;
    o.dir = p.dir;
    o.mass = p.mass;
    bodyPos(p, t, o.x, o.y);
    o.path = p.path;
    o.standing = p.hasPause && t >= p.pauseStart && t < p.pauseStart + p.pauseDur;
    double xs[4], ys[4], fs[4];
    int n = 0;
    if (t >= p.t0) pedContacts(p, t, xs, ys, fs, n);
    o.nf = 0;
    for (int i = 0; i < n && o.nf < 4; i++) {
      if (fs[i] < 1) continue;
      o.fx[o.nf] = xs[i];
      o.fy[o.nf] = ys[i];
      o.nf++;
    }
    pv.push_back(o);
  }
  for (const Vehicle& v : vehs_) {
    VehView o;
    o.id = v.id;
    o.type = v.type.c_str();
    o.len = v.len;
    o.wid = v.wid;
    o.lane = v.lane;
    double f = vehFront(v, t);
    double dirS = v.lane == 0 ? 1.0 : -1.0;
    o.x = f;
    o.y = v.centreY;
    o.stopped = v.stopDur > 0 && f == v.stopFront;
    o.nw = 0;
    for (size_t a = 0; a < v.axles.size() && o.nw < 4; a++) o.wheelX[o.nw++] = f - dirS * v.axles[a].offset;
    vv.push_back(o);
  }
}

}  // namespace sim
