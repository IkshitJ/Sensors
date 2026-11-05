// =============================================================================
//  Simulation harness: runs a REAL Arduino sketch (setup()/loop()) against the
//  emulated HX711 chips, which sample the physical world model.
//
//  The firmware is treated as a black box: its pedestrian count is read only
//  from what it prints on Serial, exactly like a data logger would.
//
//  Usage:
//    zebra_<fw> --bench --scenario dense --duration 600 --seed 1   -> JSON result
//    zebra_<fw> --interactive [--scenario normal] [--seed 1]       -> JSON frames
//        stdin commands: see handleCommand()
// =============================================================================
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "../arduino/Arduino.h"
#include "../arduino/sim_hooks.h"
#include "world.h"

// provided by the sketch wrapper (sketch_*.cpp)
void setup();
void loop();
__attribute__((weak)) void sim_report() {}
extern const char* SIM_FIRMWARE_ID;

using namespace sim;

// ---------------------------------------------------------------- hardware --
struct HwProfile {
  const char* name;
  double periodUs;      // HX711 conversion period (RATE pin: low=10 SPS, high=80 SPS)
  int subSamples;       // sinc-filter averaging inside one conversion
  double capacityKg;    // per pad (4 load cells)
  double countsPerKg;   // nominal bridge sensitivity in ADC counts
  double elecNoise;     // counts RMS
};

// v1: 4 x 50 kg bar cells, RATE pin low (10 SPS). Polarity/scale chosen so the
//     sketch's own CALIBRATION_FACTOR (-7050) is correct (best case for v1).
static const HwProfile HW_V1 = {"v1: 4x50kg cells, 10 SPS", 100000, 16, 200, -7050, 21};
// v2: 4 x 1000 kg shear-beam cells (2 mV/V, 5 V), RATE pin high (80 SPS).
//     2 mV/V * 5 V = 10 mV at 4000 kg; HX711 gain 128 -> 2.328 nV/count -> 1073.7 counts/kg
static const HwProfile HW_V2 = {"v2: 4x1000kg cells, 80 SPS", 12500, 4, 4000, 1073.7, 39};

struct HxDevice {
  int dout = -1, pad = -1;
  double phaseUs = 0;
  long long lastK = -1;
  double countsPerKg = 1;
  double offsetCounts = 0;
  double driftAmp = 0, driftPeriod = 1200, driftPhase = 0;
};

class Harness : public SimHooks {
 public:
  Harness(const HwProfile& hw, uint32_t seed) : world(seed * 7919u + 13u), hw_(hw), rng_(seed * 104729u + 7u) {
    for (int i = 0; i < NUM_PADS; i++) {
      HxDevice d;
      d.dout = 22 + 2 * i;  // DOUT pins 22, 24, ... 48 (SCK = DOUT + 1)
      d.pad = i;
      d.phaseUs = std::uniform_real_distribution<double>(0, hw.periodUs)(rng_);
      d.countsPerKg = hw.countsPerKg * (1 + std::normal_distribution<double>(0, 0.02)(rng_));  // cell tolerance
      d.offsetCounts = std::uniform_real_distribution<double>(-60000, 60000)(rng_);
      d.driftAmp = std::uniform_real_distribution<double>(0.3, 1.5)(rng_);  // kg, temperature
      d.driftPeriod = std::uniform_real_distribution<double>(900, 2400)(rng_);
      d.driftPhase = std::uniform_real_distribution<double>(0, 6.283)(rng_);
      devs_.push_back(d);
    }
  }

  // ---- SimHooks
  uint64_t nowUs() const override { return now_; }
  void advanceUs(uint64_t us) override { now_ += us; }
  bool hxReady(int pin) override {
    HxDevice* d = dev(pin);
    if (!d) return false;
    return currentK(*d) > d->lastK;
  }
  uint64_t hxNextReadyUs(int pin) override {
    HxDevice* d = dev(pin);
    if (!d) return now_ + 1000;
    long long k = currentK(*d);
    return (uint64_t)(d->phaseUs + (k + 1) * hw_.periodUs) + 1;
  }
  long hxRead(int pin) override {
    HxDevice* d = dev(pin);
    if (!d) return 0;
    long long k = currentK(*d);
    d->lastK = k;
    return conversion(*d, k);
  }
  void serialTx(const char* data, size_t n) override {
    for (size_t i = 0; i < n; i++) {
      char c = data[i];
      txAll_.push_back(c);
      if (c == '\n') {
        onLine(line_);
        line_.clear();
      } else if (c != '\r') {
        line_.push_back(c);
      }
    }
  }

  // ---- firmware state as seen through its Serial output
  long fwTotal = -1;
  double fwLive = -1;
  int fwVeh = 0, fwMask = 0;
  long fwLaneA = 0, fwLaneB = 0;
  double fwU[NUM_PADS] = {0};
  bool fwHasU = false;

  World world;
  std::string txAll_;

  double lastForces[NUM_PADS] = {0};  // most recent physical load (for display)
  bool trace = false;

 private:
  HxDevice* dev(int pin) {
    for (auto& d : devs_)
      if (d.dout == pin) return &d;
    return nullptr;
  }
  long long currentK(const HxDevice& d) const {
    double t = (double)now_ - d.phaseUs;
    if (t < 0) return -1;
    return (long long)std::floor(t / hw_.periodUs);
  }
  long conversion(const HxDevice& d, long long k) {
    // The sigma-delta ADC averages the bridge signal over the conversion period.
    double tEnd = (d.phaseUs + k * hw_.periodUs) * 1e-6;
    double tStart = tEnd - hw_.periodUs * 1e-6;
    double f[NUM_PADS];
    double sum = 0;
    for (int s = 0; s < hw_.subSamples; s++) {
      double ts = tStart + (s + 0.5) * (tEnd - tStart) / hw_.subSamples;
      world.padForces(ts, f);
      // mechanical overload stop: the plate bottoms out at 150 % of rated load
      double fp = std::min(f[d.pad], hw_.capacityKg * 1.5);
      sum += fp;
    }
    double kg = sum / hw_.subSamples;
    lastForces[d.pad] = kg;
    double vib = world.vibrationKg(tEnd);
    kg += std::normal_distribution<double>(0, 0.3 + vib)(rng_);  // mechanical noise
    kg += d.driftAmp * std::sin(6.2832 * tEnd / d.driftPeriod + d.driftPhase);
    double counts = d.offsetCounts + d.countsPerKg * kg + std::normal_distribution<double>(0, hw_.elecNoise)(rng_);
    if (std::uniform_real_distribution<double>(0, 1)(rng_) < 2e-5) counts += (1 << 19);  // rare bit error
    if (counts > 8388607) counts = 8388607;    // 24-bit saturation
    if (counts < -8388608) counts = -8388608;
    return (long)std::llround(counts);
  }

  void onLine(const std::string& l) {
    if (trace && (getenv("TRACE_T") || l.rfind("@T,", 0) != 0)) fprintf(stderr, "%9.3f FW  %s\n", now_ * 1e-6, l.c_str());
    if (l.rfind("@T,", 0) == 0) {
      // @T,ms,total,live,vehActive,mask,laneA,laneB,pad0..pad13
      std::vector<double> v;
      std::stringstream ss(l.substr(3));
      std::string tok;
      while (std::getline(ss, tok, ',')) v.push_back(std::atof(tok.c_str()));
      if (v.size() >= 7) {
        fwTotal = (long)v[1];
        fwLive = v[2];
        fwVeh = (int)v[3];
        fwMask = (int)v[4];
        fwLaneA = (long)v[5];
        fwLaneB = (long)v[6];
      }
      if (v.size() >= 7 + NUM_PADS) {
        for (int i = 0; i < NUM_PADS; i++) fwU[i] = v[7 + i];
        fwHasU = true;
      }
    } else if (l.rfind("@E,", 0) == 0) {
      // @E,ms,lane,people,estimate,total
      std::vector<std::string> v;
      std::stringstream ss(l.substr(3));
      std::string tok;
      while (std::getline(ss, tok, ',')) v.push_back(tok);
      if (v.size() >= 5) {
        fwTotal = std::atol(v[4].c_str());
        if (v[1] == "A") fwLaneA += std::atol(v[2].c_str());
        if (v[1] == "B") fwLaneB += std::atol(v[2].c_str());
      }
    }
  }

  HwProfile hw_;
  std::vector<HxDevice> devs_;
  std::mt19937 rng_;
  uint64_t now_ = 0;
  std::string line_;
};

static Harness* H = nullptr;
static bool g_trace = false;
static double g_physImp = 0;

// One iteration of the Arduino main() loop: loop(), then serialEvent().
static const uint64_t kLoopOverheadUs = 20;
static uint64_t lastWorldUpdateUs = 0;
static void runFor(uint64_t untilUs) {
  while (H->nowUs() < untilUs) {
    loop();
    if (sim_serial_rx_available()) serialEvent();
    H->advanceUs(kLoopOverheadUs);
    if (H->nowUs() - lastWorldUpdateUs >= 1000) {
      lastWorldUpdateUs = H->nowUs();
      int before = H->world.truthCrossed();
      { double f[NUM_PADS]; H->world.padForces(H->nowUs() * 1e-6, f); for (int i = 0; i < NUM_PADS; i++) g_physImp += f[i] * 0.001; }
      H->world.update(H->nowUs() * 1e-6);
      if (H->trace && H->world.truthCrossed() != before)
        fprintf(stderr, "%9.3f TRUTH %d\n", H->nowUs() * 1e-6, H->world.truthCrossed());
    }
  }
}

static std::string jsonEscape(const std::string& s) {
  std::string o;
  for (char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': break;
      case '\t': o += "\\t"; break;
      default:
        if ((unsigned char)c < 0x20) break;
        o += c;
    }
  }
  return o;
}

// ------------------------------------------------------------------ bench --
static int runBench(const std::string& scenario, double duration, uint32_t seed) {
  bool ok = false;
  Rates r = presetRates(scenario, &ok);
  if (!ok) {
    fprintf(stderr, "unknown scenario %s\n", scenario.c_str());
    return 2;
  }
  setup();
  // let the firmware boot/tare on an empty strip, then start traffic
  runFor(H->nowUs() + 2000000);
  if (getenv("SIM_NOVEH")) r.vehPerMin = 0;
  if (getenv("SIM_WAIT")) r.waitFrac = atof(getenv("SIM_WAIT"));
  if (getenv("SIM_NOSTOP")) r.stopFrac = 0;
  H->world.setRates(r);
  H->world.resetTruth();
  long startCount = H->fwTotal < 0 ? 0 : H->fwTotal;
  long startA = H->fwLaneA, startB = H->fwLaneB;
  uint64_t end = H->nowUs() + (uint64_t)(duration * 1e6);
  runFor(end);
  H->world.setSpawning(false);
  // drain: everyone finishes crossing, firmware closes its last event
  uint64_t guard = H->nowUs() + 120000000ULL;
  while (!H->world.empty() && H->nowUs() < guard) runFor(H->nowUs() + 500000);
  runFor(H->nowUs() + 3000000);
  long counted = (H->fwTotal < 0 ? 0 : H->fwTotal) - startCount;
  sim_report();
  if (g_trace) fprintf(stderr, "PHYS imp/person=%.1f\n", g_physImp / std::max(1, H->world.truthCrossed()));
  int truth = H->world.truthCrossed();
  double acc = truth > 0 ? 1.0 - std::fabs((double)counted - truth) / truth : (counted == 0 ? 1.0 : 0.0);
  printf("{\"firmware\":\"%s\",\"scenario\":\"%s\",\"seed\":%u,\"duration\":%.0f,\"truth\":%d,\"counted\":%ld,"
         "\"accuracy\":%.4f,\"vehicles\":%d,\"vehiclesOverStrip\":%d,\"truthA\":%d,\"truthB\":%d,"
         "\"countedA\":%ld,\"countedB\":%ld}\n",
         SIM_FIRMWARE_ID, scenario.c_str(), seed, duration, truth, counted, acc, H->world.vehiclesTotal(),
         H->world.vehiclesOverStrip(), H->world.truthLane(0), H->world.truthLane(1), H->fwLaneA - startA,
         H->fwLaneB - startB);
  return 0;
}

// ------------------------------------------------------------ interactive --
static std::mutex g_cmdMu;
static std::deque<std::string> g_cmds;
static std::atomic<bool> g_eof(false);

static void stdinThread() {
  std::string line;
  while (std::getline(std::cin, line)) {
    std::lock_guard<std::mutex> lk(g_cmdMu);
    g_cmds.push_back(line);
  }
  g_eof = true;
}

static double g_speed = 1.0;
static bool g_paused = false;

static void handleCommand(const std::string& cmdLine) {
  std::stringstream ss(cmdLine);
  std::string cmd;
  ss >> cmd;
  double t = H->nowUs() * 1e-6;
  World& w = H->world;
  if (cmd == "scenario") {
    std::string n;
    ss >> n;
    bool ok = false;
    Rates r = presetRates(n, &ok);
    if (ok) w.setRates(r);
  } else if (cmd == "rates") {
    Rates r = w.rates();
    ss >> r.pedPerMin >> r.groupFrac >> r.vehPerMin >> r.stopFrac >> r.waitFrac;
    r.burstPeriod = 0;
    w.setRates(r);
  } else if (cmd == "spawn") {
    std::string what;
    ss >> what;
    std::string laneArg;
    int n = 3;
    if (what == "group") ss >> n;
    ss >> laneArg;
    int lane = laneArg == "A" ? 0 : (laneArg == "B" ? 1 : -1);
    if (what == "ped") w.spawnPedestrian(t, ADULT, lane);
    else if (what == "child") w.spawnPedestrian(t, CHILD, lane);
    else if (what == "elder") w.spawnPedestrian(t, ELDER, lane);
    else if (what == "runner") w.spawnPedestrian(t, RUNNER, lane);
    else if (what == "waiter") w.spawnPedestrian(t, ADULT, lane, 8);
    else if (what == "group") w.spawnGroup(t, n, lane);
    else if (what == "parkcar") w.spawnVehicle(t, "car", lane, 12);
    else w.spawnVehicle(t, what, lane, 0);
  } else if (cmd == "speed") {
    ss >> g_speed;
    if (g_speed < 0.05) g_speed = 0.05;
    if (g_speed > 50) g_speed = 50;
  } else if (cmd == "pause") {
    g_paused = true;
  } else if (cmd == "resume") {
    g_paused = false;
  } else if (cmd == "serial") {
    std::string rest;
    std::getline(ss, rest);
    if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
    sim_serial_push(rest + "\n");
  } else if (cmd == "weight") {
    std::string a;
    ss >> a;
    if (a == "clear") w.clearStaticWeights();
    else {
      double kg = 0;
      ss >> kg;
      w.setStaticWeight(std::atoi(a.c_str()), kg);
    }
  } else if (cmd == "resettruth") {
    w.resetTruth();
  }
}

static void emitFrame(size_t& txSent) {
  std::vector<World::PedView> pv;
  std::vector<World::VehView> vv;
  double t = H->nowUs() * 1e-6;
  H->world.snapshot(t, pv, vv);
  std::string o;
  o.reserve(4096);
  char b[256];
  snprintf(b, sizeof b, "{\"t\":%.3f,\"speed\":%.2f,\"paused\":%s,\"fw\":\"%s\",\"truth\":%d,\"truthA\":%d,\"truthB\":%d,\"vehTotal\":%d,\"vehOver\":%d,", t,
           g_speed, g_paused ? "true" : "false", SIM_FIRMWARE_ID, H->world.truthCrossed(), H->world.truthLane(0),
           H->world.truthLane(1), H->world.vehiclesTotal(), H->world.vehiclesOverStrip());
  o += b;
  snprintf(b, sizeof b, "\"count\":%ld,\"laneA\":%ld,\"laneB\":%ld,\"live\":%.2f,\"veh\":%d,\"mask\":%d,", H->fwTotal,
           H->fwLaneA, H->fwLaneB, H->fwLive, H->fwVeh, H->fwMask);
  o += b;
  o += "\"pads\":[";
  for (int i = 0; i < NUM_PADS; i++) {
    snprintf(b, sizeof b, "%s%.1f", i ? "," : "", H->lastForces[i]);
    o += b;
  }
  o += "],\"u\":";
  if (H->fwHasU) {
    o += "[";
    for (int i = 0; i < NUM_PADS; i++) {
      snprintf(b, sizeof b, "%s%.0f", i ? "," : "", H->fwU[i]);
      o += b;
    }
    o += "]";
  } else {
    o += "null";
  }
  o += ",\"peds\":[";
  for (size_t i = 0; i < pv.size(); i++) {
    const auto& p = pv[i];
    snprintf(b, sizeof b, "%s[%d,%d,%.2f,%.2f,%.0f,%d,[", i ? "," : "", p.id, p.kind, p.x, p.y, p.mass, p.standing ? 1 : 0);
    o += b;
    for (int f = 0; f < p.nf; f++) {
      snprintf(b, sizeof b, "%s%.2f,%.2f", f ? "," : "", p.fx[f], p.fy[f]);
      o += b;
    }
    o += "]]";
  }
  o += "],\"veh\":[";
  for (size_t i = 0; i < vv.size(); i++) {
    const auto& v = vv[i];
    snprintf(b, sizeof b, "%s[%d,\"%s\",%.2f,%.2f,%.2f,%.2f,%d,%d,[", i ? "," : "", v.id, v.type, v.x, v.y, v.len, v.wid,
             v.lane, v.stopped ? 1 : 0);
    o += b;
    for (int k = 0; k < v.nw; k++) {
      snprintf(b, sizeof b, "%s%.2f", k ? "," : "", v.wheelX[k]);
      o += b;
    }
    o += "]]";
  }
  o += "],\"weights\":[";
  const auto& ws = H->world.weights();
  for (size_t i = 0; i < ws.size(); i++) {
    snprintf(b, sizeof b, "%s[%d,%.2f]", i ? "," : "", ws[i].pad, ws[i].kg);
    o += b;
  }
  o += "],\"serial\":\"";
  if (H->txAll_.size() > txSent) {
    o += jsonEscape(H->txAll_.substr(txSent));
    txSent = H->txAll_.size();
  }
  if (H->txAll_.size() > (1u << 20)) {  // keep memory bounded
    H->txAll_.clear();
    txSent = 0;
  }
  o += "\"}\n";
  fwrite(o.data(), 1, o.size(), stdout);
  fflush(stdout);
}

static int runInteractive(const std::string& scenario) {
  std::thread(stdinThread).detach();
  size_t txSent = 0;
  setup();
  bool ok = false;
  Rates r = presetRates(scenario, &ok);
  if (ok) H->world.setRates(r);
  const double frameReal = 0.04;  // 25 fps
  auto next = std::chrono::steady_clock::now();
  while (!g_eof) {
    {
      std::lock_guard<std::mutex> lk(g_cmdMu);
      while (!g_cmds.empty()) {
        handleCommand(g_cmds.front());
        g_cmds.pop_front();
      }
    }
    if (!g_paused) runFor(H->nowUs() + (uint64_t)(frameReal * g_speed * 1e6));
    emitFrame(txSent);
    next += std::chrono::microseconds((long long)(frameReal * 1e6));
    auto now = std::chrono::steady_clock::now();
    if (next > now) std::this_thread::sleep_until(next);
    else next = now;  // can't keep up at this speed; don't spiral
  }
  return 0;
}

int main(int argc, char** argv) {
  std::string mode = "interactive", scenario = "normal", hw = "";
  double duration = 600;
  uint32_t seed = 1;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--bench") mode = "bench";
    else if (a == "--interactive") mode = "interactive";
    else if (a == "--scenario" && i + 1 < argc) scenario = argv[++i];
    else if (a == "--duration" && i + 1 < argc) duration = std::atof(argv[++i]);
    else if (a == "--seed" && i + 1 < argc) seed = (uint32_t)std::atol(argv[++i]);
    else if (a == "--hw" && i + 1 < argc) hw = argv[++i];
    else if (a == "--trace") g_trace = true;
  }
  if (hw.empty()) hw = "v2";
  static Harness harness(hw == "v1" ? HW_V1 : HW_V2, seed);
  H = &harness;
  H->trace = g_trace;
  H->world.trace = g_trace;
  g_sim = H;
  if (mode == "bench") return runBench(scenario, duration, seed);
  return runInteractive(scenario);
}
