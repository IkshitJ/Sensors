// =============================================================================
//  Physical world model of the road and the sensor strip.
//
//  Coordinates (metres):  x runs ALONG the road (direction of travel),
//                         y runs ACROSS the road.
//  Road: y in [-3.5, 3.5].  Lane A (y > 0) travels towards +x,
//                           lane B (y < 0) travels towards -x.
//  Zebra crossing: x in [0, 4], over both lanes.
//  Sensor strip: lies ON the zebra crossing, rotated so it spans the whole road
//  width: 14 pads of 0.5 m (y in [-3.5, 3.5]) x 1.0 m deep (x in [1.5, 2.5]).
//  Pads 0-6 cover lane B, pads 7-13 cover lane A.
//
//  Pedestrians move along the lanes just like the vehicles (lane A ->, lane B
//  <-) and step across the strip's 1 m depth. Every footfall produces the
//  classic double-hump vertical ground reaction force with the centre of
//  pressure rolling heel-to-toe; runners have a flight phase.
//  Vehicles: each axle puts its wheel loads on tyre contact patches.
// =============================================================================
#pragma once
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace sim {

constexpr int NUM_PADS = 14;
constexpr int PADS_PER_LANE = 7;
constexpr double PAD_W = 0.5;                       // across the road
constexpr double STRIP_Y0 = -3.5, STRIP_Y1 = 3.5;
constexpr double STRIP_X0 = 1.5, STRIP_X1 = 2.5;    // 1 m deep, middle of the crossing
constexpr double CROSS_X0 = 0.0, CROSS_X1 = 4.0;    // painted zebra crossing
constexpr double ROAD_Y = 3.5;
constexpr double WALK_X0 = -11.0, WALK_X1 = 15.0;   // pedestrians enter/leave the view here

enum PedKind { ADULT = 0, TEEN, CHILD, ELDER, RUNNER };

struct Footfall {
  double x, y;   // footprint centre
  double tOn;    // touchdown (gait-time)
};

struct Pedestrian {
  int id = 0;
  PedKind kind = ADULT;
  double mass = 70;
  double speed = 1.35, cadence = 1.85, stepLen = 0.73, stance = 0.65;
  double shapeA = 1.21, shapeB = 0.46, shapeNorm = 1.0;
  double y0 = 1.75, slope = 0.0;  // lateral path y(x) = y0 + slope*(x - xStart)
  int dir = 1;                    // +1 = lane A (->), -1 = lane B (<-)
  int lane = 0;                   // 0 = A, 1 = B
  double xStart = WALK_X0, t0 = 0;
  // optional stop (waiting) on the strip
  bool hasPause = false;
  double pauseStart = 0, pauseDur = 0;
  double pauseX = 0, pauseY[2] = {0, 0};
  std::vector<Footfall> feet;
  double tLastOnStrip = -1;  // real time the last foot leaves the strip
  double tEnd = 0;           // despawn time
  bool touchesStrip = false;
  bool counted = false;
  int group = -1;
};

struct Axle {
  double offset;  // metres behind the vehicle front
  double load;    // kg on the whole axle
};

struct Vehicle {
  int id = 0;
  std::string type;
  double mass = 1500, len = 4.3, wid = 1.75;
  int lane = 0;  // 0 = lane A (+x), 1 = lane B (-x)
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
  int pad;
  double kg;
};

struct Rates {
  double pedPerMin = 0, groupFrac = 0, vehPerMin = 0, stopFrac = 0, waitFrac = 0;
  double childFrac = 0.06, runnerFrac = 0.02, elderFrac = 0.06;
  double burstPeriod = 0, burstSize = 0, burstWindow = 0;  // platoons released by a signal
};

Rates presetRates(const std::string& name, bool* ok = nullptr);

class World {
 public:
  explicit World(uint32_t seed);
  bool trace = false;

  void setRates(const Rates& r) { rates_ = r; }
  const Rates& rates() const { return rates_; }
  void setSpawning(bool on) { spawning_ = on; }

  void update(double t);  // spawning, ground truth and clean-up
  void padForces(double t, double out[NUM_PADS]) const;
  double vibrationKg(double t) const;  // ground vibration from heavy traffic nearby

  // manual spawning (lane -1 = random)
  void spawnPedestrian(double t, int kind = -1, int lane = -1, double pause = 0);
  void spawnGroup(double t, int n, int lane = -1);
  void spawnVehicle(double t, const std::string& type, int lane = -1, double stopDur = 0);
  void setStaticWeight(int pad, double kg);
  void clearStaticWeights() { weights_.clear(); }

  // ground truth
  int truthCrossed() const { return truth_[0] + truth_[1]; }
  int truthLane(int lane) const { return truth_[lane]; }
  int vehiclesOverStrip() const { return vehOverStrip_; }
  int vehiclesTotal() const { return vehTotal_; }
  void resetTruth() { truth_[0] = truth_[1] = 0; vehOverStrip_ = 0; vehTotal_ = 0; }
  bool empty() const { return peds_.empty() && vehs_.empty(); }

  // rendering
  struct PedView { int id, kind, lane; double x, y, mass; bool standing; double fx[4], fy[4]; int nf; };
  struct VehView { int id; const char* type; double x, y, len, wid; int lane; double wheelX[4]; int nw; bool stopped; };
  void snapshot(double t, std::vector<PedView>& p, std::vector<VehView>& v) const;
  const std::vector<StaticWeight>& weights() const { return weights_; }

 private:
  double gaitTime(const Pedestrian& p, double t) const;
  double crowdFactor(double t) const;
  void pedContacts(const Pedestrian& p, double t, double* xs, double* ys, double* fs, int& n) const;
  double vehFront(const Vehicle& v, double t) const;
  void buildPedestrian(Pedestrian& p, double forcedSpeed = -1);
  void scheduleNext(double t);
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
};

}  // namespace sim
