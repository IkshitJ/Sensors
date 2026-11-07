#include <algorithm>
// =============================================================================
//  Simulation harness: runs REAL Arduino sketches (setup()/loop()) against the
//  emulated HX711 chips, which sample the physical world model.
//
//  The installation has one board per gate: the ENTRY board on the plates of
//  the entry corridor and the EXIT board on those of the exit corridor, joined
//  by a serial cable (Serial1 to Serial1). Both run here, in one process, on
//  one shared world and one virtual clock, each with its own 28 HX711s.
//
//  The firmware is treated as a black box: its counts are read only from what
//  it prints on Serial, exactly like a data logger would.
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
#include "sketch_boards.h"
#include "world.h"

__attribute__((weak)) void sim_report() {}

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
// v3: each plate's 4 x 1000 kg shear-beam cells (2 mV/V, 5 V) are wired as TWO
//     bridges of 2 cells (ghat-side edge, marg-side edge), RATE pin high (80 SPS).
//     2 mV/V * 5 V = 10 mV at 2000 kg per bridge; HX711 gain 128 -> 2.328 nV/count
//     -> 2147.4 counts/kg
static const HwProfile HW_V2 = {"v4: 2 strips x 14 plates x 2 bridges of 2x1000kg cells, 80 SPS", 12500, 4, 2000, 2147.4, 39};

// HX711 wiring, must match the sketches: ghat-side bridge of plate i on DOUT
// 22+2i, marg-side bridge on A0,A2,..,A14 (54..68) then 2,4,..,12.
static int doutPin(int pad, int half) {
  if (half == 0) return 22 + 2 * pad;
  return pad < 8 ? 54 + 2 * pad : 2 + 2 * (pad - 8);
}

struct HxDevice {
  int dout = -1, pad = -1;  // pad: 0..27, the plate in the world (strip * 14 + plate)
  int half = 0;  // 0 = ghat-side cell pair, 1 = marg-side cell pair
  double phaseUs = 0;
  long long lastK = -1;
  double countsPerKg = 1;
  double offsetCounts = 0;
  double driftAmp = 0, driftPeriod = 1200, driftPhase = 0;
};

// what one board has told us on its USB serial
struct FwState {
  long total = -1;
  double live = -1;
  int veh = 0, mask = 0;
  long in = 0, out = 0, inside = 0;
  bool alert = false;
  long limit = 0;
  int alertChanges = 0;
  double u[NUM_PADS] = {0};
  bool hasU = false;
  std::string line;
};

class Harness : public SimHooks {
 public:
  Harness(const HwProfile& hw, uint32_t seed, int nBoards)
      : world(seed * 7919u + 13u), nBoards_(nBoards), hw_(hw), rng_(seed * 104729u + 7u) {
    // board b is wired to the plates of strip b (the calibration sketch, a
    // single board, sits on the entry gate's plates)
    for (int b = 0; b < nBoards_; b++)
      for (int k = 0; k < 2 * NUM_PADS; k++) {
        HxDevice d;
        d.pad = b * NUM_PADS + k % NUM_PADS;
        d.half = k / NUM_PADS;
        d.dout = doutPin(k % NUM_PADS, d.half);  // SCK = DOUT + 1
        d.phaseUs = std::uniform_real_distribution<double>(0, hw.periodUs)(rng_);
        d.countsPerKg = hw.countsPerKg * (1 + std::normal_distribution<double>(0, 0.02)(rng_));  // cell tolerance
        d.offsetCounts = std::uniform_real_distribution<double>(-60000, 60000)(rng_);
        d.driftAmp = std::uniform_real_distribution<double>(0.3, 1.5)(rng_);  // kg, temperature
        d.driftPeriod = std::uniform_real_distribution<double>(900, 2400)(rng_);
        d.driftPhase = std::uniform_real_distribution<double>(0, 6.283)(rng_);
        devs_[b].push_back(d);
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
  void serialTx(int port, const char* data, size_t n) override {
    int b = g_board;
    if (port == 1) {  // the link cable: straight into the other board's Serial1
      if (nBoards_ > 1) sim_serial_push(1 - b, 1, std::string(data, n));
      return;
    }
    for (size_t i = 0; i < n; i++) {
      char c = data[i];
      if (fw[b].line.empty() && nBoards_ > 1 && c != '\n') tagLine(b);
      if (c == '\n') {
        onLine(b, fw[b].line);
        txAll_ += '\n';
        fw[b].line.clear();
      } else if (c != '\r') {
        fw[b].line.push_back(c);
        txAll_ += c;
      }
    }
  }

  // ---- firmware state as seen through its Serial output
  FwState fw[SIM_MAX_BOARDS];
  int nBoards() const { return nBoards_; }
  // the installation's figures: crossings on all strips; in / out / inside /
  // alert as the ENTRY board (board 0) reports them (the EXIT board agrees,
  // the link keeps them within 250 ms of each other)
  long fwTotal() const { long s = 0; for (int b = 0; b < nBoards_; b++) s += fw[b].total < 0 ? 0 : fw[b].total; return s; }
  double fwLive() const { double s = 0; for (int b = 0; b < nBoards_; b++) s += fw[b].live < 0 ? 0 : fw[b].live; return s; }
  bool fwKnown() const { return fw[0].total >= 0; }
  int fwVeh() const { for (int b = 0; b < nBoards_; b++) if (fw[b].veh) return 1; return 0; }
  long fwMask() const { long m = 0; for (int b = 0; b < nBoards_; b++) m |= (long)fw[b].mask << (b * NUM_PADS); return m; }
  bool fwHasU() const { for (int b = 0; b < nBoards_; b++) if (!fw[b].hasU) return false; return true; }

  World world;
  std::string txAll_;

  double lastForces[NUM_PADS_ALL] = {0};  // most recent physical load (for display)
  bool trace = false;

 private:
  void tagLine(int b) {
    std::string tag = std::string("[") + SIM_BOARDS[b].name + "] ";
    txAll_ += tag;
  }
  HxDevice* dev(int pin) {
    for (auto& d : devs_[g_board])
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
    double f[NUM_PADS_ALL], fm[NUM_PADS_ALL];
    double sum = 0;
    for (int s = 0; s < hw_.subSamples; s++) {
      double ts = tStart + (s + 0.5) * (tEnd - tStart) / hw_.subSamples;
      world.padForces(ts, f, fm);
      double mine = d.half == 0 ? f[d.pad] - fm[d.pad] : fm[d.pad];  // this bridge's share
      // mechanical overload stop: the cells bottom out at 150 % of rated load
      sum += std::min(mine, hw_.capacityKg * 1.5);
    }
    double kg = sum / hw_.subSamples;
    double vib = world.vibrationKg(tEnd);
    kg += std::normal_distribution<double>(0, 0.3 + vib)(rng_);  // mechanical noise
    kg += d.driftAmp * std::sin(6.2832 * tEnd / d.driftPeriod + d.driftPhase);
    double counts = d.offsetCounts + d.countsPerKg * kg + std::normal_distribution<double>(0, hw_.elecNoise)(rng_);
    if (std::uniform_real_distribution<double>(0, 1)(rng_) < 2e-5) counts += (1 << 19);  // rare bit error
    if (counts > 8388607) counts = 8388607;    // 24-bit saturation
    if (counts < -8388608) counts = -8388608;
    return (long)std::llround(counts);
  }

  void onLine(int b, const std::string& l) {
    FwState& s = fw[b];
    if (trace && (getenv("TRACE_T") || l.rfind("@T,", 0) != 0)) fprintf(stderr, "%9.3f FW%d %s\n", now_ * 1e-6, b, l.c_str());
    if (l.rfind("@T,", 0) == 0) {
      // @T,ms,total,live,vehActive,mask,in,out,inside,alert,limit,plate0..plate13
      std::vector<double> v;
      std::stringstream ss(l.substr(3));
      std::string tok;
      while (std::getline(ss, tok, ',')) v.push_back(std::atof(tok.c_str()));
      if (v.size() >= 10) {
        s.total = (long)v[1];
        s.live = v[2];
        s.veh = (int)v[3];
        s.mask = (int)v[4];
        s.in = (long)v[5];
        s.out = (long)v[6];
        s.inside = (long)v[7];
        s.alert = v[8] != 0;
        s.limit = (long)v[9];
      }
      if (v.size() >= 10 + NUM_PADS) {
        for (int i = 0; i < NUM_PADS; i++) s.u[i] = v[10 + i];
        s.hasU = true;
      }
    } else if (l.rfind("@A,", 0) == 0) {
      // @A,ms,alert,inside,limit
      std::vector<std::string> v;
      std::stringstream ss(l.substr(3));
      std::string tok;
      while (std::getline(ss, tok, ',')) v.push_back(tok);
      if (v.size() >= 4) {
        s.alert = std::atoi(v[1].c_str()) != 0;
        s.inside = std::atol(v[2].c_str());
        s.limit = std::atol(v[3].c_str());
        s.alertChanges++;
      }
    } else if (l.rfind("@E,", 0) == 0) {
      // @E,ms,half,people,estimate,total,in,out,inside
      std::vector<std::string> v;
      std::stringstream ss(l.substr(3));
      std::string tok;
      while (std::getline(ss, tok, ',')) v.push_back(tok);
      if (v.size() >= 8) {
        s.total = std::atol(v[4].c_str());
        s.in = std::atol(v[5].c_str());
        s.out = std::atol(v[6].c_str());
        s.inside = std::atol(v[7].c_str());
      }
    }
  }

  int nBoards_;
  HwProfile hw_;
  std::vector<HxDevice> devs_[SIM_MAX_BOARDS];
  std::mt19937 rng_;
  uint64_t now_ = 0;
};

static Harness* H = nullptr;
static bool g_trace = false;
static double g_physImp = 0;

// One iteration of every board's Arduino main() loop. Each board's sketch runs
// with g_board set to it, so the emulated core routes pins and serial ports
// to that board's chips and cables.
static const uint64_t kLoopOverheadUs = 20;
static uint64_t lastWorldUpdateUs = 0;
static void runFor(uint64_t untilUs) {
  while (H->nowUs() < untilUs) {
    for (int b = 0; b < H->nBoards(); b++) {
      g_board = b;
      SIM_BOARDS[b].loop();
    }
    g_board = 0;
    H->advanceUs(kLoopOverheadUs);
    if (H->nowUs() - lastWorldUpdateUs >= 1000) {
      lastWorldUpdateUs = H->nowUs();
      int before = H->world.truthCrossed();
      {
        double f[NUM_PADS_ALL];
        H->world.padForces(H->nowUs() * 1e-6, f);
        for (int i = 0; i < NUM_PADS_ALL; i++) { g_physImp += f[i] * 0.001; H->lastForces[i] = f[i]; }
      }
      H->world.update(H->nowUs() * 1e-6);
      if (H->trace && H->world.truthCrossed() != before)
        fprintf(stderr, "%9.3f TRUTH %d\n", H->nowUs() * 1e-6, H->world.truthCrossed());
    }
  }
}

static void setupBoards() {
  for (int b = 0; b < H->nBoards(); b++) {
    g_board = b;
    SIM_BOARDS[b].setup();
  }
  g_board = 0;
}

// a line typed on the Serial Monitor goes to every board (there is one
// monitor per board in reality; the page shows them together)
static void serialToAll(const std::string& s) {
  for (int b = 0; b < H->nBoards(); b++) sim_serial_push(b, 0, s);
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

// A headcount ("o<n>": n people inside now) is a fact about the world as well
// as a command to the boards: the operator counted them. Apply it to all.
static bool isCounterFw() { return std::string(SIM_FIRMWARE_ID) == "v2"; }
static void headcount(int n) {
  H->world.setInside(n);
  if (isCounterFw()) serialToAll("o" + std::to_string(n) + "\n");
}

// ------------------------------------------------------------------ bench --
static int runBench(const std::string& scenario, double duration, uint32_t seed) {
  bool ok = false;
  Rates r = presetRates(scenario, &ok);
  if (!ok) {
    fprintf(stderr, "unknown scenario %s\n", scenario.c_str());
    return 2;
  }
  setupBoards();
  // let the firmware boot/tare on empty strips, then start traffic
  runFor(H->nowUs() + 2000000);
  if (getenv("SIM_NOVEH")) r.vehPerMin = 0;
  if (getenv("SIM_WAIT")) r.waitFrac = atof(getenv("SIM_WAIT"));
  if (getenv("SIM_NOSTOP")) r.stopFrac = 0;
  // add non-linear walkers to ANY scenario: SIM_NONLIN=0.3 [SIM_NONLIN_STYLE=1..4]
  if (getenv("SIM_NONLIN")) r.nonlinFrac = atof(getenv("SIM_NONLIN"));
  if (getenv("SIM_NONLIN_STYLE")) r.nonlinStyle = atoi(getenv("SIM_NONLIN_STYLE"));
  if (getenv("SIM_INBOUND")) r.inboundFrac = atof(getenv("SIM_INBOUND"));
  if (getenv("SIM_PED")) r.pedPerMin = atof(getenv("SIM_PED"));  // any people/min, e.g. SIM_PED=2000
  H->world.setRates(r);
  if (r.startInside > 0) {  // e.g. "return": the ghat is full when the counting starts
    headcount(r.startInside);
    runFor(H->nowUs() + 1000000);
  }
  H->world.resetTruth();
  long startCount = H->fwTotal();
  long startIn = H->fw[0].in, startOut = H->fw[0].out, startInside = H->fw[0].inside;
  int truthInside0 = H->world.truthInside();
  // occupancy error, sampled once a second while traffic flows
  double occErrSum = 0, occErrMax = 0;
  long occSamples = 0;
  uint64_t end = H->nowUs() + (uint64_t)(duration * 1e6);
  while (H->nowUs() < end) {
    runFor(std::min<uint64_t>(end, H->nowUs() + 1000000));
    double e = std::fabs((double)(H->fw[0].inside - startInside) - (H->world.truthInside() - truthInside0));
    occErrSum += e;
    occErrMax = std::max(occErrMax, e);
    occSamples++;
  }
  H->world.setSpawning(false);
  // drain: everyone finishes crossing, firmware closes its last event
  uint64_t guard = H->nowUs() + 120000000ULL;
  while (!H->world.empty() && H->nowUs() < guard) runFor(H->nowUs() + 500000);
  runFor(H->nowUs() + 3000000);
  long counted = H->fwTotal() - startCount;
  sim_report();
  if (g_trace) fprintf(stderr, "PHYS imp/person=%.1f\n", g_physImp / std::max(1, H->world.truthCrossed()));
  int truth = H->world.truthCrossed();
  double acc = truth > 0 ? 1.0 - std::fabs((double)counted - truth) / truth : (counted == 0 ? 1.0 : 0.0);
  printf("{\"firmware\":\"%s\",\"scenario\":\"%s\",\"seed\":%u,\"duration\":%.0f,\"truth\":%d,\"counted\":%ld,"
         "\"accuracy\":%.4f,\"vehicles\":%d,\"vehiclesOverStrip\":%d,\"truthIn\":%d,\"truthOut\":%d,"
         "\"countedIn\":%ld,\"countedOut\":%ld,\"truthInside\":%d,\"countedInside\":%ld,"
         "\"insideErrMean\":%.2f,\"insideErrMax\":%.0f,\"touchedNotCrossed\":%d,"
         "\"nonlinFrac\":%.2f,\"inboundFrac\":%.2f,"
         "\"truthByPath\":{\"straight\":%d,\"weave\":%d,\"drift\":%d,\"jostle\":%d,\"uturn\":%d}}\n",
         SIM_FIRMWARE_ID, scenario.c_str(), seed, duration, truth, counted, acc, H->world.vehiclesTotal(),
         H->world.vehiclesOverStrip(), H->world.truthDir(DIR_IN), H->world.truthDir(DIR_OUT), H->fw[0].in - startIn,
         H->fw[0].out - startOut, H->world.truthInside() - truthInside0, H->fw[0].inside - startInside,
         occSamples ? occErrSum / occSamples : 0.0, occErrMax, H->world.touchedNotCrossed(), r.nonlinFrac,
         r.inboundFrac, H->world.truthStyle(STRAIGHT), H->world.truthStyle(WEAVE), H->world.truthStyle(DRIFT),
         H->world.truthStyle(JOSTLE), H->world.truthStyle(UTURN));
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
    if (ok) {
      w.setRates(r);
      if (r.startInside > 0) headcount(r.startInside);
    }
  } else if (cmd == "rates") {
    Rates r = w.rates();
    ss >> r.pedPerMin >> r.groupFrac >> r.vehPerMin >> r.stopFrac >> r.waitFrac;
    r.burstPeriod = 0;
    w.setRates(r);
  } else if (cmd == "nonlin") {  // nonlin <fraction 0..1> [style -1..4]
    Rates r = w.rates();
    double f = 0;
    int style = -1;
    ss >> f;
    if (ss >> style) r.nonlinStyle = style;
    r.nonlinFrac = f < 0 ? 0 : (f > 1 ? 1 : f);
    w.setRates(r);
  } else if (cmd == "vehicles") {  // vehicles <per minute>, 0 = none (other rates unchanged)
    Rates r = w.rates();
    double v = 0;
    ss >> v;
    r.vehPerMin = v < 0 ? 0 : v;
    w.setRates(r);
  } else if (cmd == "inbound") {  // inbound <fraction heading into the ghat 0..1>
    Rates r = w.rates();
    double f = 0.5;
    ss >> f;
    r.inboundFrac = f < 0 ? 0 : (f > 1 ? 1 : f);
    w.setRates(r);
  } else if (cmd == "spawn") {
    // spawn <what> [n for group] [in|out]   (no direction = random)
    std::string what;
    ss >> what;
    std::string dirArg;
    int n = 3;
    if (what == "group" || what == "crowd") ss >> n;
    ss >> dirArg;
    int dir = dirArg == "in" ? -1 : (dirArg == "out" ? 1 : 0);
    int lane = dir < 0 ? 1 : (dir > 0 ? 0 : -1);  // vehicles: the corridor driving that way
    if (what == "ped") w.spawnPedestrian(t, ADULT, dir);
    else if (what == "child") w.spawnPedestrian(t, CHILD, dir);
    else if (what == "elder") w.spawnPedestrian(t, ELDER, dir);
    else if (what == "runner") w.spawnPedestrian(t, RUNNER, dir);
    else if (what == "waiter") w.spawnPedestrian(t, ADULT, dir, 8);
    else if (what == "weaver") w.spawnPedestrian(t, ADULT, dir, 0, WEAVE);
    else if (what == "drifter") w.spawnPedestrian(t, ADULT, dir, 0, DRIFT);
    else if (what == "jostler") w.spawnPedestrian(t, ADULT, dir, 0, JOSTLE);
    else if (what == "uturn") w.spawnPedestrian(t, ADULT, dir, 0, UTURN);
    else if (what == "group") w.spawnGroup(t, n, dir);
    else if (what == "crowd") w.spawnCrowd(t, n < 1 ? 1 : (n > 100000 ? 100000 : n), dir);
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
    // a headcount typed on the Serial monitor also sets how many are inside
    if (isCounterFw() && rest.size() > 1 && (rest[0] == 'o' || rest[0] == 'O') && isdigit((unsigned char)rest[1]))
      w.setInside(std::atoi(rest.c_str() + 1));
    serialToAll(rest + "\n");
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
  // a browser can't draw tens of thousands of people 25 times a second: send
  // every k-th person (picked by id, so the same people stay visible and
  // walk the whole way). All of them are still simulated and weighed.
  const size_t MAX_DRAW = 2000;
  static size_t every = 1;
  size_t pedsTotal = pv.size();
  while (pedsTotal / every > MAX_DRAW) every++;
  while (every > 1 && pedsTotal / (every - 1) < MAX_DRAW * 8 / 10) every--;  // hysteresis: no flicker
  if (every > 1)
    pv.erase(std::remove_if(pv.begin(), pv.end(), [&](const World::PedView& p) { return (uint32_t)(p.id * 2654435761u) % every != 0; }), pv.end());
  std::string o;
  o.reserve(4096);
  char b[256];
  const FwState& e = H->fw[0];
  snprintf(b, sizeof b, "{\"t\":%.3f,\"speed\":%.2f,\"paused\":%s,\"fw\":\"%s\",\"boards\":%d,\"truth\":%d,\"truthIn\":%d,\"truthOut\":%d,\"truthInside\":%d,\"vehTotal\":%d,\"vehOver\":%d,", t,
           g_speed, g_paused ? "true" : "false", SIM_FIRMWARE_ID, H->nBoards(), H->world.truthCrossed(), H->world.truthDir(DIR_IN),
           H->world.truthDir(DIR_OUT), H->world.truthInside(), H->world.vehiclesTotal(), H->world.vehiclesOverStrip());
  o += b;
  snprintf(b, sizeof b, "\"turnedBack\":%d,\"nonlin\":%.2f,\"inbound\":%.2f,\"atGhat\":%d,\"pedsTotal\":%zu,\"density\":[%.2f,%.2f],\"backlog\":[%ld,%ld],\"alert\":%s,\"limit\":%ld,",
           H->world.touchedNotCrossed(), H->world.rates().nonlinFrac, H->world.rates().inboundFrac, H->world.atGhat(), pedsTotal,
           H->world.density(t, STRIP_ENTRY), H->world.density(t, STRIP_EXIT), H->world.backlog(STRIP_ENTRY), H->world.backlog(STRIP_EXIT),
           e.alert ? "true" : "false", e.limit);
  o += b;
  snprintf(b, sizeof b, "\"count\":%ld,\"in\":%ld,\"out\":%ld,\"inside\":%ld,\"live\":%.2f,\"vehActive\":%d,\"mask\":%ld,", H->fwKnown() ? H->fwTotal() : -1,
           e.in, e.out, e.inside, H->fwKnown() ? H->fwLive() : -1.0, H->fwVeh(), H->fwMask());
  o += b;
  o += "\"pads\":[";
  for (int i = 0; i < NUM_PADS_ALL; i++) {
    snprintf(b, sizeof b, "%s%.1f", i ? "," : "", H->lastForces[i]);
    o += b;
  }
  o += "],\"u\":";
  if (H->fwHasU()) {
    o += "[";
    for (int i = 0; i < NUM_PADS_ALL; i++) {
      int bd = i / NUM_PADS;
      double u = bd < H->nBoards() ? H->fw[bd].u[i % NUM_PADS] : 0;
      snprintf(b, sizeof b, "%s%.0f", i ? "," : "", u);
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
    snprintf(b, sizeof b, "],%d,%d]", p.path, p.dir);
    o += b;
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
  setupBoards();
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
  static Harness harness(hw == "v1" ? HW_V1 : HW_V2, seed, std::min(SIM_NUM_BOARDS, SIM_MAX_BOARDS));
  H = &harness;
  H->trace = g_trace;
  H->world.trace = g_trace;
  g_sim = H;
  if (mode == "bench") return runBench(scenario, duration, seed);
  return runInteractive(scenario);
}
