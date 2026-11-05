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
  } else {
    if (ok) *ok = false;
  }
  return r;
}

World::World(uint32_t seed) : rng_(seed) {}

static double uni(std::mt19937& g, double a, double b) { return std::uniform_real_distribution<double>(a, b)(g); }
static double nrm(std::mt19937& g, double m, double s) { return std::normal_distribution<double>(m, s)(g); }

void World::scheduleNext(double t) {
  nextPed_ = rates_.pedPerMin > 0 ? t + std::exponential_distribution<double>(rates_.pedPerMin / 60.0)(rng_) : 1e18;
  nextVeh_ = rates_.vehPerMin > 0 ? t + std::exponential_distribution<double>(rates_.vehPerMin / 60.0)(rng_) : 1e18;
  nextBurst_ = rates_.burstPeriod > 0 ? t + 2.0 : 1e18;
}

// ---------------------------------------------------------------------------
//  Pedestrians (walk along their lane, like the vehicles)
// ---------------------------------------------------------------------------
double World::gaitTime(const Pedestrian& p, double t) const {
  if (!p.hasPause || t < p.pauseStart) return t;
  if (t < p.pauseStart + p.pauseDur) return p.pauseStart;
  return t - p.pauseDur;
}

// Weidmann (1993) fundamental diagram: walking speed relative to free flow.
static double weidmann(double rho) {
  if (rho < 0.05) return 1.0;
  return std::max(0.2, 1.0 - std::exp(-1.913 * (1.0 / rho - 1.0 / 5.4)));
}

double World::crowdFactor(double t) const {
  // density of people near the crossing (10 m of road, both lanes)
  int n = 0;
  for (const Pedestrian& p : peds_) {
    if (t < p.t0) continue;
    double x = p.xStart + p.dir * (gaitTime(p, t) - p.t0) * p.speed;
    if (std::fabs(x - STRIP_XC) < 5.0) n++;
  }
  return weidmann(n / (10.0 * 2 * ROAD_Y));
}

static double laneLo(int lane) { return lane == 0 ? 0.3 : -ROAD_Y + 0.3; }
static double laneHi(int lane) { return lane == 0 ? ROAD_Y - 0.3 : -0.3; }

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
  else p.speed *= crowdFactor(p.t0);  // crowds walk slower (shorter steps)
  p.dir = p.lane == 0 ? 1 : -1;
  p.xStart = p.dir > 0 ? WALK_X0 : WALK_X1;

  // step around a vehicle that is standing on the crossing when we get there
  double tArr = p.t0 + std::fabs(STRIP_XC - p.xStart) / p.speed;
  for (const Vehicle& v : vehs_) {
    if (v.stopDur <= 0) continue;
    double tS = v.t0 + std::fabs(v.stopFront - v.x0) / v.v;
    if (tArr < tS - 2.0 || tArr > tS + v.stopDur + 1.0) continue;
    double lo = v.centreY - v.wid / 2 - 0.35, hi = v.centreY + v.wid / 2 + 0.35;
    if (p.y0 > lo && p.y0 < hi) {
      double a = lo, b = hi;
      bool aOk = a > -ROAD_Y + 0.2, bOk = b < ROAD_Y - 0.2;
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
  p.slope = nrm(g, 0, 0.015);
  p.feet.clear();
  for (int k = 0;; k++) {
    double along = 0.15 + k * p.stepLen + nrm(g, 0, 0.02);
    double x = p.xStart + p.dir * along;
    if (x < WALK_X0 - 0.01 || x > WALK_X1 + 0.01) break;
    double side = (k % 2 == 0) ? -1.0 : 1.0;
    double y = clampd(p.y0 + p.slope * (x - p.xStart), -ROAD_Y + 0.1, ROAD_Y - 0.1) + side * 0.09 + nrm(g, 0, 0.015);
    p.feet.push_back({x, y, p.t0 + k / c});
  }
  // waiting on the strip
  p.hasPause = false;
  if (p.pauseDur > 0) {
    double target = uni(g, STRIP_X0 + 0.25, STRIP_X1 - 0.25);
    for (size_t k = 0; k < p.feet.size(); k++) {
      if (p.dir * (p.feet[k].x - target) >= 0) {
        p.hasPause = true;
        p.pauseStart = p.feet[k].tOn + 0.5 * p.stance;
        p.pauseX = p.feet[k].x;
        p.pauseY[0] = p.feet[k].y;
        double side = (k % 2 == 0) ? -1.0 : 1.0;
        p.pauseY[1] = p.feet[k].y - side * 0.18;
        break;
      }
    }
  }
  // ground truth: when does the last foot leave the strip?
  p.touchesStrip = false;
  p.tLastOnStrip = -1;
  for (const Footfall& f : p.feet) {
    if (f.x + 0.1 < STRIP_X0 || f.x - 0.1 > STRIP_X1) continue;
    p.touchesStrip = true;
    double off = f.tOn + p.stance;
    if (p.hasPause && off > p.pauseStart) off += p.pauseDur;
    p.tLastOnStrip = std::max(p.tLastOnStrip, off);
  }
  p.tEnd = p.t0 + p.feet.size() / c + p.stance + (p.hasPause ? p.pauseDur : 0) + 1.0;
}

static PedKind randomKind(std::mt19937& g, const Rates& r) {
  double u = uni(g, 0, 1);
  if (u < r.childFrac) return CHILD;
  if (u < r.childFrac + r.runnerFrac) return RUNNER;
  if (u < r.childFrac + r.runnerFrac + r.elderFrac) return ELDER;
  if (u < r.childFrac + r.runnerFrac + r.elderFrac + 0.10) return TEEN;
  return ADULT;
}

void World::spawnPedestrian(double t, int kind, int lane, double pause) {
  Pedestrian p;
  p.id = nextId_++;
  p.kind = kind < 0 ? randomKind(rng_, rates_) : (PedKind)kind;
  p.lane = lane >= 0 ? lane : (uni(rng_, 0, 1) < 0.5 ? 0 : 1);
  p.y0 = uni(rng_, laneLo(p.lane), laneHi(p.lane));
  p.t0 = t;
  if (pause < 0) pause = 0;
  if (pause == 0 && p.kind != RUNNER && uni(rng_, 0, 1) < rates_.waitFrac) pause = uni(rng_, 3, 25);
  p.pauseDur = pause;
  buildPedestrian(p);
  peds_.push_back(p);
}

void World::spawnGroup(double t, int n, int lane) {
  if (n < 2) n = 2;
  if (n > 6) n = 6;
  int gid = nextGroup_++;
  if (lane < 0) lane = uni(rng_, 0, 1) < 0.5 ? 0 : 1;
  double spacing = uni(rng_, 0.48, 0.65);  // walking side by side
  double width = spacing * (n - 1);
  double lo = laneLo(lane), hi = std::max(lo + 0.01, laneHi(lane) - width);
  double base = width < laneHi(lane) - lo ? uni(rng_, lo, hi) : lo;
  double groupSpeed = clampd(nrm(rng_, 1.25, 0.15), 0.85, 1.7) * crowdFactor(t);
  bool family = uni(rng_, 0, 1) < 0.25;
  for (int i = 0; i < n; i++) {
    Pedestrian p;
    p.id = nextId_++;
    p.group = gid;
    p.lane = lane;
    p.kind = (family && i == n - 1) ? CHILD : (uni(rng_, 0, 1) < 0.12 ? TEEN : ADULT);
    p.y0 = clampd(base + i * spacing + nrm(rng_, 0, 0.04), -ROAD_Y + 0.2, ROAD_Y - 0.2);
    p.t0 = t + uni(rng_, 0, 0.45);  // not perfectly abreast
    buildPedestrian(p, groupSpeed);
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
    xs[n] = f.x + p.dir * (-0.10 + 0.20 * s);  // heel-to-toe roll along the walking direction
    ys[n] = f.y;
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
  double sign = lane == 0 ? 1.0 : -1.0;
  v.centreY = sign * (s->singleTrack ? uni(rng_, 0.4, 3.1) : 1.75 + nrm(rng_, 0, 0.15));
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

void World::update(double t) {
  if (nextPed_ > 1e17 && nextVeh_ > 1e17 && nextBurst_ > 1e17) scheduleNext(t);
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
  for (Pedestrian& p : peds_) {
    if (!p.counted && p.touchesStrip && t >= p.tLastOnStrip) {
      p.counted = true;
      truth_[p.lane]++;
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
  peds_.erase(std::remove_if(peds_.begin(), peds_.end(), [&](const Pedestrian& p) { return t > p.tEnd + 1.0; }), peds_.end());
  vehs_.erase(std::remove_if(vehs_.begin(), vehs_.end(), [&](const Vehicle& v) { return t > v.tEnd + 1.0; }), vehs_.end());
}

static void addSpanY(double out[NUM_PADS], double y0, double y1, double f) {
  // distribute force f uniformly over [y0, y1] across the pads
  double w = y1 - y0;
  if (w <= 0) return;
  int p0 = std::max(0, (int)std::floor((y0 - STRIP_Y0) / PAD_W));
  int p1 = std::min(NUM_PADS - 1, (int)std::floor((y1 - STRIP_Y0) / PAD_W));
  for (int p = p0; p <= p1; p++) {
    double a = STRIP_Y0 + p * PAD_W, b = a + PAD_W;
    double ov = std::min(b, y1) - std::max(a, y0);
    if (ov > 0) out[p] += f * ov / w;
  }
}

void World::padForces(double t, double out[NUM_PADS]) const {
  for (int i = 0; i < NUM_PADS; i++) out[i] = 0;
  double xs[4], ys[4], fs[4];
  int n;
  for (const Pedestrian& p : peds_) {
    if (t < p.t0) continue;
    pedContacts(p, t, xs, ys, fs, n);
    for (int i = 0; i < n; i++) {
      // the loaded part of the sole spans ~12 cm around the centre of pressure,
      // so a foot on the plate edge shares its load with the road
      double frac = (std::min(xs[i] + 0.06, STRIP_X1) - std::max(xs[i] - 0.06, STRIP_X0)) / 0.12;
      if (frac <= 0) continue;
      addSpanY(out, ys[i] - 0.045, ys[i] + 0.045, fs[i] * std::min(1.0, frac));  // ~9 cm wide
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
        addSpanY(out, wy - v.tyreW / 2, wy + v.tyreW / 2, load);
      }
    }
  }
  for (const StaticWeight& w : weights_)
    if (w.pad >= 0 && w.pad < NUM_PADS) out[w.pad] += w.kg;
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
    o.lane = p.lane;
    o.mass = p.mass;
    double te = gaitTime(p, t);
    double x = p.xStart + p.dir * std::max(0.0, te - p.t0) * p.speed;
    x = clampd(x, WALK_X0 - 0.3, WALK_X1 + 0.3);
    o.x = x;
    o.y = clampd(p.y0 + p.slope * (x - p.xStart), -ROAD_Y + 0.1, ROAD_Y - 0.1);
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
