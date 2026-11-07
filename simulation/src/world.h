// =============================================================================
//  Physical world model of the road and the sensor strips.
//
//  Setting: Ramghat Marg, the approach road to Ram Ghat on the Shipra river,
//  Ujjain (Simhastha Kumbh). The ghat premises are on the LEFT (-x), the marg
//  continues to the RIGHT (+x, towards Dani Gate / Mahakaleshwar).
//
//  Arrangement (as the ghats were run at Simhastha 2016: entry and exit kept
//  apart by barricades so that incoming and outgoing crowds never meet): the
//  marg is split lengthwise by a barricade into two one-way corridors, each
//  7 m wide, with a gate in each at the premises boundary.
//
//  Coordinates (metres):  x runs ALONG the road, y runs ACROSS it.
//    ENTRY corridor: y in [-7.5, -0.5], everyone walks INTO the ghat (dir -1, <-)
//    barricade:      y in [-0.5,  0.5]
//    EXIT corridor:  y in [ 0.5,  7.5], everyone walks OUT of it (dir +1, ->)
//  (Vehicles, when present, drive along a corridor in its direction.)
//  Sensor strips: one in each gate, spanning the corridor's whole width:
//  14 plates of 0.5 m x 1.0 m deep (x in [1.5, 2.5]). Plate index p of strip c
//  is pad c*14 + p in the flat arrays below.
//  Each plate stands on 4 load cells wired as TWO bridges: the ghat-side pair
//  (x = 1.5 edge) and the marg-side pair (x = 2.5 edge). A load at x splits
//  between them in proportion to its distance from each edge, so the firmware
//  sees where on the plate the weight is and which way it moves.
//
//  Every footfall produces the classic double-hump vertical ground reaction
//  force with the centre of pressure rolling heel-to-toe; runners have a
//  flight phase. Vehicles: each axle puts its wheel loads on tyre patches.
// =============================================================================
#pragma once
#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <vector>

namespace sim {

constexpr int NUM_STRIPS = 2;                       // 0 = entry gate, 1 = exit gate
constexpr int STRIP_ENTRY = 0, STRIP_EXIT = 1;
constexpr int NUM_PADS = 14;                        // plates per strip
constexpr int NUM_PADS_ALL = NUM_STRIPS * NUM_PADS;
constexpr int PADS_PER_LANE = 7;
constexpr double PAD_W = 0.5;                       // across the road
constexpr double CORR_W = 7.0;                      // each corridor, barricade to kerb
constexpr double MEDIAN_W = 1.0;                    // the barricade between them
constexpr double ROAD_Y = MEDIAN_W / 2 + CORR_W;    // 7.5: the outer kerbs are at +-ROAD_Y
constexpr double STRIP_X0 = 1.5, STRIP_X1 = 2.5;    // 1 m deep, in the gate
constexpr double CROSS_X0 = 0.0, CROSS_X1 = 4.0;    // the gate area
constexpr double WALK_X0 = -11.0, WALK_X1 = 19.5;   // pedestrians enter/leave the view here
inline double corrY0(int c) { return c == STRIP_ENTRY ? -ROAD_Y : MEDIAN_W / 2; }  // across-the-road extent of corridor c
inline double corrY1(int c) { return corrY0(c) + CORR_W; }
inline int corridorOf(int dir) { return dir < 0 ? STRIP_ENTRY : STRIP_EXIT; }
// Crowd physics. Bodies cannot overlap: the densest a crowd packs is about
// 8 people / m2 (crush density at Hajj and Kumbh stampedes). Nobody queues at
// the gate: when the end of a corridor on screen is packed solid, the crowd
// backs up along the marg BEYOND the edge of the screen and pushes on as soon
// as there is room. Speed falls with density (see weidmann() in world.cpp).
constexpr double JAM_DENSITY = 8.0;                  // people / m2
constexpr double EDGE_ZONE = 5.0;                    // metres of road at each screen edge checked for room

enum PedKind { ADULT = 0, TEEN, CHILD, ELDER, RUNNER };

// How a pedestrian's path bends. Nobody walks a ruler-straight line:
//   STRAIGHT : natural walking: the heading wanders a few degrees and the
//              walker drifts across the corridor towards gaps every few metres
//   WEAVE    : sinusoidal meander, heading swinging +-10..30 deg
//   DRIFT    : diagonal cut of 1-4 m across the corridor somewhere near the strip
//   JOSTLE   : crowd jostling: random-walk heading, uneven step length and
//              occasional sidesteps (shuffles with little forward progress)
//   UTURN    : walks up to the strip, steps onto/near it, turns round and
//              walks back (does NOT cross)
enum PathStyle { STRAIGHT = 0, WEAVE, DRIFT, JOSTLE, UTURN, NUM_PATH_STYLES };

struct Footfall {
  double x, y;   // footprint centre
  double tOn;    // touchdown (gait-time)
  double hx = 1, hy = 0;  // walking heading (unit vector) during this step
  double bx = 0, by = 0;  // body centre at touchdown
  double step = 0.73;     // length of this step (the heel-to-toe roll is no longer than it)
};

// The state of a walker's path, advanced one footstep at a time as they go
// (the step length follows the crowd around them, so the footsteps cannot be
// laid out in advance)
struct PathState {
  double a = 0.15, y = 0, th = 0;   // distance along the walk, across the road, heading
  double yPref = 0, aRetarget = 0;  // STRAIGHT: where across the corridor they are drifting to, and when to pick anew
  int turnStep = 0;                 // UTURN: pivot progress
  double wAy = 0, wK = 0, wPh = 0;  // WEAVE
  double dA0 = 0, dL = 1, dDy = 0;  // DRIFT
  double aTurn = 0, turnSign = 1;   // UTURN
  bool done = false;                // the last footstep (leaving the view) has been laid
};

struct Pedestrian {
  int id = 0;
  PedKind kind = ADULT;
  double mass = 70;
  double speed = 1.35, cadence = 1.85, stepLen = 0.73, stance = 0.65;  // speed, stepLen: free walking
  double clock0 = 0;              // their corridor's crowd clock when they set off: progress = speed x (clock - clock0)
  PathState ps;
  double shapeA = 1.21, shapeB = 0.46, shapeNorm = 1.0;
  double y0 = 1.75;               // where across the corridor they start
  int dir = 1;                    // -1 = INTO the ghat premises (<-), +1 = OUT of them (->)
  int corr = 1;                   // the corridor they walk in (= corridorOf(dir))
  double xStart = WALK_X0, t0 = 0;
  // optional stop (waiting) on the strip
  bool hasPause = false;
  double pauseStart = 0, pauseDur = 0;
  double pauseX = 0, pauseY[2] = {0, 0};
  std::vector<Footfall> feet;
  double tLastOnStrip = -1;  // gait time the last foot (so far) leaves the strip
  double tPadOn = -1, tPadOff = -1;  // gait-time window in which any foot can load a plate
  double tEnd = 1e18;        // despawn time (gait time), known once the path is done
  bool touchesStrip = false;
  bool crosses = false;      // reached the far side of the strip
  bool pastStrip = false;    // will not touch the strip again (crossed, turned back, or done)
  bool counted = false;
  int group = -1;
  PathStyle path = STRAIGHT;
};

struct Axle {
  double offset;  // metres behind the vehicle front
  double load;    // kg on the whole axle
};

struct Vehicle {
  int id = 0;
  std::string type;
  double mass = 1500, len = 4.3, wid = 1.75;
  int lane = 0;  // 0 = drives +x (exit corridor), 1 = drives -x (entry corridor)
  double v = 8;  // m/s
  double x0 = -30, t0 = 0;
  double centreY = 1.75, tyreW = 0.2, patch = 0.16;
  bool singleTrack = false;
  std::vector<Axle> axles;
  double stopFront = -1e9, stopDur = 0;  // stops with front at stopFront for stopDur
  double bouncePhase = 0;
  double tEnd = 0;
  bool counted = false;
};

struct StaticWeight {
  int pad;  // 0..27: strip * 14 + plate
  double kg;
};

struct Rates {
  double pedPerMin = 0, groupFrac = 0, vehPerMin = 0, stopFrac = 0, waitFrac = 0;
  double childFrac = 0.06, runnerFrac = 0.02, elderFrac = 0.06;
  double burstPeriod = 0, burstSize = 0, burstWindow = 0;  // platoons released by a signal
  // non-linear walking: fraction of people who do not walk straight, and the
  // style they use (-1 = realistic mix of WEAVE/DRIFT/JOSTLE/UTURN)
  double nonlinFrac = 0;
  int nonlinStyle = -1;
  double inboundFrac = 0.5;  // share of people/groups heading INTO the ghat premises
  int startInside = 0;       // people already inside the premises when the scenario starts
};

// directions for ground truth and spawning
enum { DIR_IN = 0, DIR_OUT = 1 };

Rates presetRates(const std::string& name, bool* ok = nullptr);

class World {
 public:
  explicit World(uint32_t seed);
  bool trace = false;

  void setRates(const Rates& r);
  const Rates& rates() const { return rates_; }
  void setSpawning(bool on) { spawning_ = on; }

  void update(double t);  // spawning, ground truth and clean-up
  // total load per plate (both strips, 28 entries); if marg != nullptr, also
  // the part carried by each plate's marg-side cell pair (the ghat-side pair
  // carries out - marg)
  void padForces(double t, double out[NUM_PADS_ALL], double marg[NUM_PADS_ALL] = nullptr) const;
  double vibrationKg(double t) const;  // ground vibration from heavy traffic nearby

  // manual spawning. dir: 0 = random (rates.inboundFrac), -1 = into the ghat
  // premises (walks <- in the entry corridor), +1 = out of them (-> in the exit corridor)
  // path: -1 = drawn from the rates (nonlinFrac / nonlinStyle), else a PathStyle
  void spawnPedestrian(double t, int kind = -1, int dir = 0, double pause = 0, int path = -1);
  void spawnGroup(double t, int n, int dir = 0);
  void spawnVehicle(double t, const std::string& type, int lane = -1, double stopDur = 0);
  void setStaticWeight(int pad, double kg);
  void clearStaticWeights() { weights_.clear(); }

  // ground truth
  int truthCrossed() const { return truth_[0] + truth_[1]; }
  int truthDir(int d) const { return truth_[d]; }               // DIR_IN / DIR_OUT
  // The premises are closed: new pilgrims only arrive from Ramghat Marg (the
  // right). Everyone walking OUT is somebody who went in earlier (or was
  // inside at the start) and now returns, so the people inside never go negative.
  int truthInside() const { return insideNow_; }   // people inside the premises now
  int atGhat() const { return (int)atGhat_.size() + anonInside_; }  // inside, not walking on screen
  void setInside(int n);                           // headcount (serial o<n>): n people inside now
  // crowd density in corridor c around its strip (people / m2)
  double density(double t, int c) const { crowdFactor(t); return cfRho_[c]; }
  long backlog(int c) const { return backlog_[c]; }  // arrived, backed up beyond the edge of the screen
  int vehiclesOverStrip() const { return vehOverStrip_; }
  int vehiclesTotal() const { return vehTotal_; }
  bool empty() const { return peds_.empty() && vehs_.empty(); }
  // n people arriving at once (a surge): dir 0 = random, -1 in, +1 out
  void spawnCrowd(double t, int n, int dir = 0);
  // non-linear walking diagnostics
  int touchedNotCrossed() const { return touchedOnly_; }        // stepped on, turned back
  int truthStyle(int s) const { return truthStyle_[s]; }        // crossings per path style
  void resetTruth() {
    truth_[0] = truth_[1] = 0; vehOverStrip_ = 0; vehTotal_ = 0;
    touchedOnly_ = 0;
    for (int& v : truthStyle_) v = 0;
  }

  // rendering
  struct PedView { int id, kind, dir, path; double x, y, mass; bool standing; double fx[4], fy[4]; int nf; };
  struct VehView { int id; const char* type; double x, y, len, wid; int lane; double wheelX[4]; int nw; bool stopped; };
  void snapshot(double t, std::vector<PedView>& p, std::vector<VehView>& v) const;
  const std::vector<StaticWeight>& weights() const { return weights_; }

 private:
  double gaitTime(const Pedestrian& p, double t) const;
  double crowdClock(double t, int c) const;  // integral of corridor c's crowd speed factor: runs slower as the crowd thickens
  void setupPath(Pedestrian& p);      // draw the path's parameters
  void genFeet(Pedestrian& p, double t);  // lay footsteps up to 1 s ahead of t
  void genFoot(Pedestrian& p);        // lay the next footstep
  void bodyPos(const Pedestrian& p, double t, double& x, double& y) const;
  PathStyle randomPath();
  int randomDir();
  int pickDir(int dir, int n);        // OUT only if n people are inside to return
  void takeFromGhat(Pedestrian& p);   // a returning pilgrim is the same person
  void crowdFactor(double t) const;   // census of both corridors (cached)
  double corrY(int c, double width);  // across-the-corridor position for a walker (or a group `width` wide)
  bool edgeFull(int c, int n);        // no room for n more at the screen edge where corridor c is entered
  void admitHeld(double t);            // the backed-up crowd pushes on where there is room
  void pedContacts(const Pedestrian& p, double t, double* xs, double* ys, double* fs, int& n) const;
  double vehFront(const Vehicle& v, double t) const;
  void buildPedestrian(Pedestrian& p, double forcedSpeed = -1);
  std::string randomVehicleType();

  std::mt19937 rng_;
  Rates rates_;
  bool spawning_ = true;
  std::vector<Pedestrian> peds_;
  std::vector<Vehicle> vehs_;
  std::vector<StaticWeight> weights_;
  int nextId_ = 1, nextGroup_ = 1;
  double nextPed_ = 1e18, nextVeh_ = 1e18, nextBurst_ = 1e18;
  int truth_[2] = {0, 0};
  int vehOverStrip_ = 0, vehTotal_ = 0;
  int touchedOnly_ = 0;
  // people inside the premises: walking on screen are in peds_, the rest are
  // down at the ghat (kind + mass, so the same person walks back out)
  struct GhatPerson { int kind; double mass; };
  std::vector<GhatPerson> atGhat_;
  int anonInside_ = 0;  // inside from a headcount / scenario start (no details)
  int insideNow_ = 0;
  // per corridor: crowdFactor() cache for huge crowds, and the crowd clock
  // (clockVal_ = integral of the speed factor up to clockT_)
  mutable double cfT_ = -1e9;
  mutable double cfVal_[NUM_STRIPS] = {1.0, 1.0}, cfRho_[NUM_STRIPS] = {0, 0};
  double clockT_ = 0, clockVal_[NUM_STRIPS] = {0, 0};
  mutable double edgeRoom_[NUM_STRIPS] = {1e9, 1e9};  // people who still fit at the screen edge where each corridor is entered
  struct Held { double t; int kind, dir, path, n; double pause; };  // n >= 2: a group
  std::deque<Held> held_[NUM_STRIPS];
  long backlog_[NUM_STRIPS] = {0, 0};
  int truthStyle_[NUM_PATH_STYLES] = {0, 0, 0, 0, 0};
};

}  // namespace sim
