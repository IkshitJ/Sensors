/*
 * ============================================================================
 *  PedestrianCounter.h — counting engine for the zebra-crossing sensor strip
 * ============================================================================
 *
 *  Called once per frame (80 Hz, 12.5 ms) with the tared weight of every pad.
 *  Pure computation: no I/O, no dynamic memory. The sketch runs one engine per
 *  lane (7 pads each, ~1.8 KB RAM each).
 *
 *  PIPELINE (per frame)
 *  ────────────────────
 *   1. Median-of-3 per pad ......... removes single-sample ADC glitches.
 *   2. Vehicle detector ............ a tyre = a pad (or pad pair) gaining
 *                                    >250 kg within 0.3 s; crowds can't load a
 *                                    pad that fast. Runs on the NEWEST raw
 *                                    samples and masks only the pads under the
 *                                    tyre, RETROACTIVELY inside a 0.8 s delay
 *                                    line, so a tyre's ramp-up never reaches
 *                                    the counter. Parked tyres are absorbed
 *                                    into the pad's zero after 3 s.
 *   3. Auto-zero ................... drift tracking, re-zero after negative
 *                                    steps, absorbs static loads (parked tyre,
 *                                    debris, someone standing still > 15 s).
 *   4. Wheel gate .................. a pad pulse shorter than a human stance,
 *                                    one without the footstep's mid-stance dip
 *                                    (a wheel's load is flat), or two short
 *                                    pulses one wheelbase apart (front + rear
 *                                    wheel) is erased from the delay line
 *                                    before it is counted: bicycles, motorbikes,
 *                                    auto-rickshaws.
 *   5. Gait smoothing .............. a 2-stage low-pass removes the double-hump
 *                                    ripple of every footstep.
 *   6. Arrival measure ............. the rising part of the load is accumulated
 *                                    over overlapping 2-pad windows minus the
 *                                    shared pads (inclusion-exclusion), so each
 *                                    person adds ~1 body weight no matter how
 *                                    their feet straddle pad borders. This
 *                                    measures THROUGHPUT; v1 measured the peak
 *                                    load (occupancy) and missed people who
 *                                    crossed one after another.
 *                                    Exact for isolated people and small groups,
 *                                    but arrivals and departures overlapping on
 *                                    the same pads partly cancel in a crowd.
 *   7. Moving impulse .............. load x time, counted only on pads that saw
 *                                    a new footstep within 1 s and minus any
 *                                    load that was standing still (waiting
 *                                    people, a stopped wheel). Can't cancel, so
 *                                    it holds up in crowds.
 *   8. Sessions .................... activity separated by >250 ms of silence.
 *                                    people = arrivals/V1 for small sessions,
 *                                    impulse/I1 for big ones, blended between
 *                                    0.8 and 2.5 people. Small sessions are
 *                                    rounded (exact for singles and groups);
 *                                    big crowd sessions keep fractions (no
 *                                    rounding bias) and are committed in chunks
 *                                    (v1 lost the whole flow count whenever a
 *                                    crowd thinned out).
 *   9. Self-calibration ............ V1 and I1 (per-person arrival and impulse)
 *                                    are learned from sessions that are clearly
 *                                    one person, adapting to the local
 *                                    population and walking speed.
 * ============================================================================
 */
#pragma once
#include <Arduino.h>
#include <math.h>

#ifndef PC_MAX_PADS
#define PC_MAX_PADS 8
#endif

namespace pc {
// ---- delay line (frames of 12.5 ms) -------------------------------------------------
static const uint8_t DELAY_FRAMES = 64;            // 0.8 s look-ahead
// ---- pad episodes ------------------------------------------------------------------------
static const float ON_KG = 6.0f;                   // pad becomes active
static const float OFF_KG = 3.0f;                  // pad becomes idle (hysteresis)
static const uint8_t OFF_FRAMES = 4;               // 50 ms below OFF ends an episode
static const uint16_t SHORT_FRAMES = 26;           // <325 ms: a wheel, not a foot
static const uint16_t PAIR_MAX_FRAMES = 40;        // <500 ms pulses: check for a wheel pair
static const uint16_t GAIT_CHECK_FRAMES = 60;      // pulses <750 ms must show a footstep dip
static const float GAIT_DIP_LO = 0.40f;            // footstep mid-stance dip, relative
static const float GAIT_DIP_HI = 0.88f;            //   to the peak load...
static const float GAIT_RECOVER = 0.12f;           // ...followed by a recovery
static const uint8_t WHEEL_MIN_LAG = 6;            // two-wheeler: a 2nd short pulse on the
static const uint8_t WHEEL_MAX_LAG = 56;           //   same/next pad 75..700 ms later (rear wheel)
// ---- gait smoothing & arrival measure ------------------------------------------------------------
static const float SMOOTH_ALPHA = 0.10f;           // 2 x EMA, tau ~ 0.12 s
static const float BACKLASH_KG = 0.8f;             // dead-band against noise and sway
// ---- sessions & counting ---------------------------------------------------------------
static const uint16_t SESSION_GAP_FRAMES = 20;     // 250 ms of silence closes a session
static const float ACTIVE_KG = 2.0f;               // smoothed load that keeps a session open
static const float DEFAULT_V1 = 68.0f;             // kg of arrival per person (learned)
static const float V1_MIN = 40.0f, V1_MAX = 110.0f;
static const float DEFAULT_I1 = 51.0f;             // kg*s of impulse per person (learned)
static const float I1_MIN = 25.0f, I1_MAX = 110.0f;
static const float LEARN_ALPHA = 0.03f;            // learning rate
static const float BLEND_LO = 0.8f;                // session size (people) where the
static const float BLEND_HI = 2.5f;                //   estimate blends arrivals -> impulse
static const float FRESH_RISE_KG = 8.0f;           // a new footstep on the pad...
static const uint8_t FRESH_FRAMES = 80;            // ...keeps its load "moving" for 1 s
static const float FLAT_BAND_KG = 6.0f;            // load (>15 kg) within +-3 kg...
static const float FLAT_MIN_KG = 15.0f;
static const uint8_t FLAT_FRAMES = 24;             // ...for 0.3 s is standing still
static const float VALLEY_ALPHA = 0.04f;              // valley tracker rise rate (tau ~0.3 s)
static const float ROUND_LIMIT = 3.5f;             // sessions above this keep fractions
static const float ROUND_BIAS = 0.5f;
static const float MIN_PERSON = 0.2f;              // below this a session is noise
static const float LARGE_COMMIT = 20.0f;           // commit long crowd sessions in chunks
// ---- vehicles ------------------------------------------------------------------------------
static const float VEH_TRIGGER_KG = 250.0f;        // one pad gaining this much...
static const uint8_t VEH_RISE_FRAMES = 24;         // ...within 0.3 s => tyre
static const float VEH_PAIR_TRIGGER_KG = 260.0f;   // tyre on a pad border: pair sum...
static const float VEH_PAIR_RISE_KG = 200.0f;      // ...rising this much...
static const uint8_t VEH_PAIR_RISE_FRAMES = 8;     // ...within 0.1 s
static const float VEH_CONFIRM_KG = 40.0f;         // previous sample agrees (no glitch)
static const float VEH_TRACK_KG = 180.0f;          // keep tracking the tyre...
static const float VEH_TRACK_PAIR_KG = 230.0f;     // ...also on a pad boundary
static const int8_t VEH_LATCH_SPAN = 0;             // neighbour pads masked around a tyre
static const uint8_t VEH_POST_FRAMES = 6;         // 150 ms tail after the last tyre
static const uint16_t PARK_FRAMES = 240;           // 3 s stationary => parked: absorb
static const uint8_t VEH_RETRO_FRAMES = 32;        // mask the last 0.4 s (tyre ramp-up)
static const float PARK_BAND_KG = 15.0f;
// ---- auto-zero ----------------------------------------------------------------------------
static const uint16_t IDLE_FRAMES = 160;           // 2 s idle before drift tracking
static const float IDLE_ALPHA = 0.004f;
static const float NEG_KG = -6.0f;                 // load removed below zero...
static const uint8_t NEG_FRAMES = 40;              // ...for 0.5 s => re-zero
static const float STATIC_MIN_KG = 8.0f;           // static load...
static const float STATIC_BAND_KG = 4.0f;          // ...that doesn't move...
static const uint16_t STATIC_FRAMES = 1200;        // ...for 15 s => absorb into zero
}  // namespace pc

class PedestrianCounter {
 public:
  struct Event {  // filled when a session closes
    uint16_t people;
    float estimate;
    bool fractional;
  };

  void begin(uint8_t numPads) {
    n_ = numPads > PC_MAX_PADS ? PC_MAX_PADS : numPads;
    total_ = 0;
    frac_ = 0;
    v1_ = pc::DEFAULT_V1;
    i1_ = pc::DEFAULT_I1;
    vehEvents_ = 0;
    transients_ = 0;
    learned_ = 0;
    resetSignal();
  }

  // Call after the HX711s were re-tared: forget all signal history.
  void resetSignal() {
    frame_ = 0;
    ringPos_ = 0;
    for (uint8_t k = 0; k < pc::DELAY_FRAMES; k++) {
      ringMask_[k] = 0;
      for (uint8_t i = 0; i < PC_MAX_PADS; i++) ring_[k][i] = 0;
    }
    for (uint8_t i = 0; i < PC_MAX_PADS; i++) {
      base_[i] = 0;
      for (uint8_t k = 0; k < 4; k++) rh_[i][k] = 0;
      hold_[i] = 0;
      u_[i] = 0;
      s1_[i] = s2_[i] = 0;
      qPad_[i] = qWin_[i] = valley_[i] = 0;
      lastRise_[i] = 0;
      staticLoad_[i] = 0;
      flatMin_[i] = flatMax_[i] = 0;
      flatStart_[i] = 0;
      ne_[i].active = false;
      ne_[i].start = ne_[i].dur = 0;
      ne_[i].lastStart = 0;
      ne_[i].lastDur = 0xFFFF;
      negCnt_[i] = 0;
      idleCnt_[i] = 0;
      staticCnt_[i] = 0;
      staticRef_[i] = 0;
      parkMin_[i] = parkMax_[i] = 0;
      parkStart_[i] = 0;
      ref_[i] = 0;
      latchUntil_[i] = 0;
    }
    vehActive_ = false;
    latched_ = 0;
    sess_.open = false;
    sess_.v = 0;
    sess_.imp = 0;
  }

  void resetCount() {
    total_ = 0;
    frac_ = 0;
  }

  // kg[i]: tared, calibrated pad weight. overload[i]: ADC saturated.
  // Returns true when a crossing session closed (see lastEvent()).
  bool processFrame(const float* kg, const bool* overload) {
    frame_++;
    float m[PC_MAX_PADS];

    // 1. baseline + median-of-3
    for (uint8_t i = 0; i < n_; i++) {
      float r = kg[i] - base_[i];
      rh_[i][3] = rh_[i][2];
      rh_[i][2] = rh_[i][1];
      rh_[i][1] = rh_[i][0];
      rh_[i][0] = r;
      m[i] = median3(rh_[i][0], rh_[i][1], rh_[i][2]);
    }

    // 2. vehicles, 3. auto-zero (both on the newest samples)
    detectVehicle(overload, m);
    for (uint8_t i = 0; i < n_; i++) autoZero(i, m);

    // pop the oldest frame from the delay line, push the newest
    uint8_t mask = ringMask_[ringPos_];
    float out[PC_MAX_PADS];
    for (uint8_t i = 0; i < n_; i++) {
      out[i] = fromQ(ring_[ringPos_][i]);
      ring_[ringPos_][i] = toQ(m[i]);
    }
    ringMask_[ringPos_] = vehActive_ ? latched_ : 0;
    curSlot_ = ringPos_;
    ringPos_ = (uint8_t)((ringPos_ + 1) % pc::DELAY_FRAMES);

    // 4. transient gate on the newest side (edits the delay line)
    for (uint8_t i = 0; i < n_; i++) trackNewest(i, m[i]);

    // 5-6. smoothing and arrival measure on the delayed, cleaned signal
    bool anyActive = false;
    float sum = 0;
    uint8_t activePads = 0;
    for (uint8_t i = 0; i < n_; i++) {
      bool masked = mask & (1 << i);
      float u = masked ? hold_[i] : out[i];
      if (!masked) hold_[i] = u;
      u_[i] = u;
      s1_[i] += pc::SMOOTH_ALPHA * (u - s1_[i]);
      s2_[i] += pc::SMOOTH_ALPHA * (s1_[i] - s2_[i]);
      if (u > pc::ON_KG || s2_[i] > pc::ACTIVE_KG) {
        anyActive = true;
        activePads |= (1 << i);
      }
      // only load that recently changed counts as moving traffic: someone
      // standing still (or a parked tyre) stops adding impulse
      // valley tracker: follows drops at once, rises only slowly
      // on the unsmoothed signal every footstep shows up, even in a packed crowd
      if (u < valley_[i]) valley_[i] = u;
      else valley_[i] += pc::VALLEY_ALPHA * (u - valley_[i]);
      if (u - valley_[i] > pc::FRESH_RISE_KG) lastRise_[i] = frame_;
{
        // load that was standing still when this footstep arrived (a waiting
        // person, a parked wheel) is not part of the moving traffic
        // walking load always ripples; a load that stays flat is standing still
        if (u > flatMax_[i]) flatMax_[i] = u;
        if (u < flatMin_[i]) flatMin_[i] = u;
        if (flatMax_[i] - flatMin_[i] > pc::FLAT_BAND_KG || u < pc::FLAT_MIN_KG) {
          flatStart_[i] = frame_;
          flatMin_[i] = flatMax_[i] = u;
        }
        bool flat = frame_ - flatStart_[i] >= pc::FLAT_FRAMES;
        bool fresh = !flat && frame_ - lastRise_[i] < pc::FRESH_FRAMES;
        if (!fresh) staticLoad_[i] = s2_[i] > pc::ACTIVE_KG ? s2_[i] : 0;
        else if (s2_[i] < staticLoad_[i]) staticLoad_[i] = s2_[i];
        float moving = s2_[i] - staticLoad_[i];
        if (fresh && moving > pc::ACTIVE_KG) sum += moving;
      }
    }
    float arrivals = 0;
    for (uint8_t i = 0; i + 1 < n_; i++) arrivals += rise(qWin_[i], s2_[i] + s2_[i + 1]);
    for (uint8_t i = 1; i + 1 < n_; i++) arrivals -= rise(qPad_[i], s2_[i]);

    // 7. sessions
    bool closed = false;
    if (anyActive) {
      if (!sess_.open) {
        sess_.open = true;
        sess_.v = 0;
        sess_.imp = 0;
        sess_.start = frame_;
        sess_.pads = 0;
        sess_.peak = 0;
        sess_.large = false;
        sess_.vehicle = false;
      }
      sess_.last = frame_;
      sess_.pads |= activePads;
      if (sum > sess_.peak) sess_.peak = sum;
      if (mask) sess_.vehicle = true;
    }
    if (sess_.open) {
      sess_.v += arrivals;
      sess_.imp += sum * 0.0125f;
      float est = sessionEstimate();
      if (est > pc::LARGE_COMMIT) {  // long crowd session: commit in chunks of 10
        float keep = (est - 10.0f) / est;
        total_ += 10;
        sess_.v *= keep;
        sess_.imp *= keep;
        sess_.start = frame_ - (uint32_t)((frame_ - sess_.start) * keep);
        sess_.large = true;
      }
      if (!anyActive && frame_ - sess_.last > pc::SESSION_GAP_FRAMES) {
        closeSession();
        closed = true;
      }
    }
    return closed;
  }

  // ---- accessors -------------------------------------------------------------
  uint32_t total() const { return total_; }
  float liveEstimate() const {
    if (!sess_.open) return (float)total_;
    float est = sessionEstimate();
    if (!sess_.large && est < pc::ROUND_LIMIT) est = (est < pc::MIN_PERSON) ? 0 : (float)(uint16_t)(est + pc::ROUND_BIAS);
    return (float)total_ + est;
  }
  const Event& lastEvent() const { return event_; }
  bool vehicleActive() const { return vehActive_; }
  uint8_t vehicleMask() const { return vehActive_ ? latched_ : 0; }
  uint16_t vehicleEvents() const { return vehEvents_; }
  uint16_t transientsRejected() const { return transients_; }
  uint16_t singlesLearned() const { return learned_; }
  float personVariation() const { return v1_; }
  float personImpulse() const { return i1_; }
  float padLoad(uint8_t i) const { return u_[i]; }
  float padBaseline(uint8_t i) const { return base_[i]; }

 public:
 private:
  struct NewEp {  // episode tracker on the newest (undelayed) samples
    bool active;
    bool gait;          // saw the mid-stance dip of a human footstep
    float peak, valley;
    uint32_t start;
    uint16_t dur;
    uint8_t below;
    uint32_t lastStart;
    uint16_t lastDur;
  };
  struct Session {
    bool open, large, vehicle;
    uint32_t start, last;
    uint8_t pads;
    float v, imp, peak;
  };

  // delay line storage: 0.1 kg steps, +-3200 kg (heavier = a tyre, masked anyway)
  static int16_t toQ(float kg) {
    float q = kg * 10.0f;
    if (q > 32000.0f) q = 32000.0f;
    if (q < -32000.0f) q = -32000.0f;
    return (int16_t)(q < 0 ? q - 0.5f : q + 0.5f);
  }
  static float fromQ(int16_t q) { return q * 0.1f; }
  float ringMin(uint8_t i) const {
    int16_t lo = ring_[0][i];
    for (uint8_t k = 1; k < pc::DELAY_FRAMES; k++)
      if (ring_[k][i] < lo) lo = ring_[k][i];
    return fromQ(lo);
  }

  static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
  }

  // positive variation of x with a dead-band (backlash operator)
  static float rise(float& q, float x, float band = pc::BACKLASH_KG) {
    if (x > q + band) {
      float inc = x - band - q;
      q = x - band;
      return inc;
    }
    if (x < q - band) q = x + band;
    return 0;
  }

  // Erase pad i's samples from frame f0 onwards from the delay line.
  void eraseFromRing(uint8_t i, uint32_t f0) {
    uint32_t age = frame_ - f0;
    if (age >= pc::DELAY_FRAMES) age = pc::DELAY_FRAMES - 1;
    for (uint32_t a = 0; a <= age; a++) {
      uint8_t slot = (uint8_t)((curSlot_ + pc::DELAY_FRAMES - a) % pc::DELAY_FRAMES);
      ring_[slot][i] = 0;
    }
  }

  void trackNewest(uint8_t i, float v) {
    NewEp& e = ne_[i];
    if (vehActive_ && (latched_ & (1 << i))) {  // vehicle detector owns this pad
      e.active = false;
      return;
    }
    if (!e.active) {
      if (v > pc::ON_KG) {
        e.active = true;
        e.start = frame_;
        e.dur = 0;
        e.below = 0;
        e.gait = false;
        e.peak = e.valley = v;
      }
      return;
    }
    e.dur++;
    // footstep load = double hump: a shallow dip (to ~50-90 % of the peak) and a
    // recovery. A wheel is flat on the pad, and the gap between two wheels
    // drops to (nearly) zero.
    if (v > e.peak) { e.peak = v; e.valley = v; }
    else if (v < e.valley) e.valley = v;
    else if (!e.gait && e.valley > pc::GAIT_DIP_LO * e.peak && e.valley < pc::GAIT_DIP_HI * e.peak &&
             v > e.valley + pc::GAIT_RECOVER * e.peak) e.gait = true;
    if (v < pc::OFF_KG) {
      if (++e.below < pc::OFF_FRAMES) return;
    } else {
      e.below = 0;
      return;
    }
    // episode ended
    e.active = false;
    uint16_t dur = e.dur > pc::OFF_FRAMES ? (uint16_t)(e.dur - pc::OFF_FRAMES) : 0;
    uint32_t prevStart = e.lastStart;
    uint16_t prevDur = e.lastDur;
    e.lastStart = e.start;
    e.lastDur = dur;
    if (dur >= pc::GAIT_CHECK_FRAMES) return;  // long enough: footsteps / standing
    // shorter than any human stance, or a pulse without the footstep dip:
    // wheel(s) rolling over the 1 m strip
    bool erase = dur < pc::SHORT_FRAMES || !e.gait;
    // two-wheeler: front and rear wheel each cross the 1 m strip as a short
    // pulse on the same (or neighbouring) pad, one wheelbase apart
    for (int8_t j = (int8_t)i - 1; j <= (int8_t)i + 1; j++) {
      if (j < 0 || j >= (int8_t)n_) continue;
      NewEp& o = ne_[j];
      uint32_t ostart = (j == (int8_t)i) ? prevStart : o.lastStart;
      uint16_t odur = (j == (int8_t)i) ? prevDur : o.lastDur;
      if (j != (int8_t)i && o.active) continue;
      if (dur >= pc::PAIR_MAX_FRAMES) break;
      if (odur >= pc::PAIR_MAX_FRAMES) continue;
      int32_t lag = (int32_t)(e.start - ostart);
      if (lag < pc::WHEEL_MIN_LAG || lag > pc::WHEEL_MAX_LAG) continue;
      erase = true;
      if (j != (int8_t)i) { eraseFromRing(j, ostart); o.lastDur = 0xFFFF; }
      else eraseFromRing(i, ostart);
    }
    if (erase) {
      eraseFromRing(i, e.start);
      transients_++;
    }
  }

  void closeSession() {
    sess_.open = false;
    float est = sessionEstimate();
    uint16_t people = 0;
    bool fractional = sess_.large || est >= pc::ROUND_LIMIT;
    if (fractional) {
      frac_ += est;
      people = (uint16_t)frac_;
      frac_ -= people;
    } else if (est >= pc::MIN_PERSON) {
      people = (uint16_t)(est + pc::ROUND_BIAS);
      if (people < 1) people = 1;
    }
    total_ += people;
    event_.people = people;
    event_.estimate = est;
    event_.fractional = fractional;

    // self-calibration from sessions that are unmistakably one person
    uint32_t dur = sess_.last - sess_.start;
    uint8_t p = sess_.pads;
    bool contiguous = p && ((p & (p + (p & -p))) == 0);  // one run of set bits
    uint8_t bits = 0;
    for (uint8_t b = p; b; b >>= 1) bits += b & 1;
    if (!sess_.vehicle && !sess_.large && contiguous && bits <= 2 && dur >= 40 && dur <= 180 &&
        sess_.peak < 130.0f && est > 0.5f && est < 1.6f) {
      v1_ += pc::LEARN_ALPHA * (sess_.v - v1_);
      i1_ += pc::LEARN_ALPHA * (sess_.imp - i1_);
      v1_ = constrain(v1_, pc::V1_MIN, pc::V1_MAX);
      i1_ = constrain(i1_, pc::I1_MIN, pc::I1_MAX);
      learned_++;
    }
    sess_.v = 0;
    sess_.imp = 0;
  }

  // People in the current session. Two independent measures:
  //  - arrivals / V1: exact for individuals and small groups, but arrivals and
  //    departures overlapping on the same pads partly cancel in a crowd;
  //  - impulse / I1 : moving load x time can't cancel, so it holds in crowds.
  // Small sessions use arrivals, big ones impulse, blended in between.
  float sessionEstimate() const {
    float nV = sess_.v / v1_;
    if (nV < 0) nV = 0;
    float nI = sess_.imp / i1_;
    float n = nV > nI ? nV : nI;
    if (sess_.large || n >= pc::BLEND_HI) return nI;
    if (n <= pc::BLEND_LO) return nV;
    float w = (n - pc::BLEND_LO) / (pc::BLEND_HI - pc::BLEND_LO);
    return nV + w * (nI - nV);
  }

  // Shift pad i's zero by d kg, keeping the whole pipeline consistent.
  // keepOutput: the delayed output must not move (parked tyre being absorbed).
  void shiftPad(uint8_t i, float d, bool keepOutput, float* m) {
    base_[i] += d;
    for (uint8_t k = 0; k < 4; k++) rh_[i][k] -= d;
    for (uint8_t k = 0; k < pc::DELAY_FRAMES; k++) ring_[k][i] = toQ(fromQ(ring_[k][i]) - d);
    m[i] -= d;
    ref_[i] -= keepOutput ? 0 : d;
    if (!keepOutput) {
      hold_[i] -= d;
      s1_[i] -= d;
      s2_[i] -= d;
      qPad_[i] -= d;
      valley_[i] -= d;
      staticLoad_[i] -= d;
      flatMin_[i] -= d;
      flatMax_[i] -= d;
      if (i > 0) qWin_[i - 1] -= d;
      if (i + 1 < n_) qWin_[i] -= d;
    }
    staticCnt_[i] = 0;
    negCnt_[i] = 0;
  }

  // Latch pad i (and its neighbours) as covered by a tyre. Newly latched pads are
  // also masked retroactively in the delay line, hiding the tyre's ramp-up.
  void latchAround(uint8_t i) {
    for (int8_t j = (int8_t)i - pc::VEH_LATCH_SPAN; j <= (int8_t)i + pc::VEH_LATCH_SPAN; j++) {
      if (j < 0 || j >= (int8_t)n_) continue;
      if (!(latched_ & (1 << j))) {
        latched_ |= (1 << j);
        ref_[j] = ringMin(j);
        // retroactive mask: only the recent past, where the tyre's ramp-up is
        for (uint8_t a = 1; a <= pc::VEH_RETRO_FRAMES; a++)
          ringMask_[(uint8_t)((ringPos_ + pc::DELAY_FRAMES - a) % pc::DELAY_FRAMES)] |= (1 << j);
        parkMin_[j] = parkMax_[j] = rh_[j][0];
        parkStart_[j] = frame_;
      }
      latchUntil_[j] = frame_ + pc::VEH_POST_FRAMES;
    }
  }

  void detectVehicle(const bool* overload, float* m) {
    // A tyre adds >250 kg to a pad within a few tenths of a second. People can
    // only load a 0.5 m pad that much by accumulating over seconds, so the
    // trigger is the RISE, not the level (works even on a pad that reads
    // negative right after a parked car drove off).
    uint8_t past = (uint8_t)((ringPos_ + pc::DELAY_FRAMES - pc::VEH_RISE_FRAMES) % pc::DELAY_FRAMES);
    uint8_t pastPair = (uint8_t)((ringPos_ + pc::DELAY_FRAMES - pc::VEH_PAIR_RISE_FRAMES) % pc::DELAY_FRAMES);
    uint8_t trigPads = 0;
    for (uint8_t i = 0; i < n_; i++) {
      float r0 = rh_[i][0];
      bool confirm = rh_[i][1] > pc::VEH_CONFIRM_KG;  // not a lone ADC glitch
      if (overload[i] || (confirm && r0 - fromQ(ring_[past][i]) > pc::VEH_TRIGGER_KG)) {
        trigPads |= (1 << i);
      }
      // a tyre straddling two pads splits its load: check adjacent pairs too
      if (i + 1 < n_ && (confirm || rh_[i + 1][1] > pc::VEH_CONFIRM_KG)) {
        float pair = r0 + rh_[i + 1][0];
        float pairPast = fromQ(ring_[pastPair][i]) + fromQ(ring_[pastPair][i + 1]);
        if (pair > pc::VEH_PAIR_TRIGGER_KG && pair - pairPast > pc::VEH_PAIR_RISE_KG) trigPads |= (3 << i);
      }
    }
    if (trigPads && !vehActive_) {
      vehActive_ = true;
      vehEvents_++;
      latched_ = 0;
      // reference = the pad's lowest level in the delay line: a tyre only ever
      // adds load, so this is the level before the vehicle (or between vehicles)
      for (uint8_t i = 0; i < n_; i++) ref_[i] = ringMin(i);
    }
    if (!vehActive_) return;

    // follow the tyre(s): only pads under or next to a tyre are masked, so
    // pedestrians elsewhere on the strip keep being counted
    for (uint8_t i = 0; i < n_; i++) {
      float d = rh_[i][0] - ref_[i];
      bool tyre = (trigPads & (1 << i)) || d > pc::VEH_TRACK_KG;
      if (i + 1 < n_ && d + (rh_[i + 1][0] - ref_[i + 1]) > pc::VEH_TRACK_PAIR_KG) {
        tyre = true;
        latchAround(i + 1);
      }
      if (tyre) latchAround(i);
    }

    // parked vehicle: a tyre standing still on a pad is absorbed into that
    // pad's zero, so people stepping next to it are counted again
    for (uint8_t i = 0; i < n_; i++) {
      if (!(latched_ & (1 << i))) continue;
      float r = rh_[i][0];
      if (r < parkMin_[i]) parkMin_[i] = r;
      if (r > parkMax_[i]) parkMax_[i] = r;
      if (parkMax_[i] - parkMin_[i] > pc::PARK_BAND_KG) {
        parkStart_[i] = frame_;
        parkMin_[i] = parkMax_[i] = r;
      } else if (frame_ - parkStart_[i] > pc::PARK_FRAMES) {
        shiftPad(i, r - ref_[i], true, m);
        parkStart_[i] = frame_;
        parkMin_[i] = parkMax_[i] = rh_[i][0];
      }
    }

    // release pads the tyre has left; the event ends when none is latched
    for (uint8_t i = 0; i < n_; i++)
      if ((latched_ & (1 << i)) && (int32_t)(frame_ - latchUntil_[i]) > 0) latched_ &= (uint8_t)~(1 << i);
    if (!latched_) vehActive_ = false;
  }

  void autoZero(uint8_t i, float* m) {
    if (vehActive_ && (latched_ & (1 << i))) return;
    float v = m[i];
    // load removed that had been absorbed (or drift below zero): re-zero
    if (v < pc::NEG_KG) {
      if (++negCnt_[i] >= pc::NEG_FRAMES) shiftPad(i, v, false, m);
    } else {
      negCnt_[i] = 0;
    }
    // slow drift tracking while the pad is idle
    if (fabs(v) < 3.0f && !ne_[i].active) {
      if (idleCnt_[i] < pc::IDLE_FRAMES) idleCnt_[i]++;
      else if ((frame_ & 31) == 0) shiftPad(i, 32 * pc::IDLE_ALPHA * v, false, m);  // batched: cheap on AVR
    } else {
      idleCnt_[i] = 0;
    }
    // static load (debris, someone standing still for a long time): absorb
    if (v > pc::STATIC_MIN_KG) {
      if (staticCnt_[i] == 0) staticRef_[i] = v;
      if (fabs(v - staticRef_[i]) < pc::STATIC_BAND_KG) {
        if (++staticCnt_[i] >= pc::STATIC_FRAMES) shiftPad(i, v, false, m);
      } else {
        staticCnt_[i] = 0;
      }
    } else {
      staticCnt_[i] = 0;
    }
  }

  uint8_t n_ = 0;
  uint32_t frame_ = 0;
  uint32_t total_ = 0;
  float frac_ = 0;
  float v1_ = pc::DEFAULT_V1, i1_ = pc::DEFAULT_I1;
  uint16_t vehEvents_ = 0, transients_ = 0, learned_ = 0;
  Event event_ = {0, 0, false};

  float base_[PC_MAX_PADS];
  float rh_[PC_MAX_PADS][4];  // raw history, [0] = newest
  int16_t ring_[pc::DELAY_FRAMES][PC_MAX_PADS];  // 0.1 kg units: halves the RAM
  uint8_t ringMask_[pc::DELAY_FRAMES];
  uint8_t ringPos_ = 0, curSlot_ = 0;
  float hold_[PC_MAX_PADS], u_[PC_MAX_PADS];
  float s1_[PC_MAX_PADS], s2_[PC_MAX_PADS];
  float qPad_[PC_MAX_PADS], qWin_[PC_MAX_PADS], valley_[PC_MAX_PADS];
  uint32_t lastRise_[PC_MAX_PADS];
  float staticLoad_[PC_MAX_PADS];
  float flatMin_[PC_MAX_PADS], flatMax_[PC_MAX_PADS];
  uint32_t flatStart_[PC_MAX_PADS];
  NewEp ne_[PC_MAX_PADS];
  Session sess_;

  bool vehActive_ = false;
  uint8_t latched_ = 0;
  uint32_t parkStart_[PC_MAX_PADS];
  float ref_[PC_MAX_PADS], parkMin_[PC_MAX_PADS], parkMax_[PC_MAX_PADS];
  uint32_t latchUntil_[PC_MAX_PADS];

  uint8_t negCnt_[PC_MAX_PADS];
  uint16_t idleCnt_[PC_MAX_PADS], staticCnt_[PC_MAX_PADS];
  float staticRef_[PC_MAX_PADS];
};
