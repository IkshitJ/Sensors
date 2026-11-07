/*
 * ============================================================================
 *  DirectionTracker.h — which way are people walking over the strip?
 * ============================================================================
 *
 *  Each 0.5 m x 1.0 m plate stands on 4 load cells wired as TWO bridges:
 *    G = the pair along the GHAT-side edge of the plate
 *    M = the pair along the MARG-side edge
 *  A load at fraction u of the plate depth (0 = ghat edge, 1 = marg edge)
 *  puts u of its weight on M and (1 - u) on G. So per plate:
 *    S = G + M          total load (what the counting engine uses)
 *    U = M / S          centre of pressure along the walking direction
 *
 *  A pilgrim's weight moves across the plate as they walk over it: heel to
 *  toe on every footstep, then onto the next foot. We accumulate that motion
 *  as "transport":
 *        dT = dM - U * dS        (per plate, per 12.5 ms frame)
 *  For a single walker dT = S * dU exactly: the load times how far its centre
 *  moved. People stepping ON or OFF an empty plate add nothing (their own
 *  position is the centre), so only movement ALONG the plate counts.
 *    dT < 0 : weight moving towards the ghat  -> walking INTO the premises
 *    dT > 0 : weight moving towards the marg  -> walking OUT
 *  One person crossing a plate produces about (body weight x 0.8 of the plate
 *  depth) of transport. That constant, T1, is learned from single walkers,
 *  exactly like the counting engine learns V1 / I1.
 *
 *  The counting engine decides HOW MANY people crossed (it works on S). Its
 *  signal runs through a 0.8 s delay line, so the transport goes through a
 *  matching delay before it is booked. When the engine commits n people, the
 *  transport that arrived with them splits them:
 *   - small sessions (1-3 people): the SIGN of r = transport / (n T1) is
 *     reliable, its size is not (it depends on where the feet landed), so the
 *     direction is decided: all n go out if r > +0.12, all in if r < -0.12,
 *     split evenly near 0. Someone who steps on and turns back moves their
 *     weight there AND back: r ~ 0, so they add as much "in" as "out" and the
 *     occupancy is unchanged;
 *   - crowd flows: forward and backward transport are summed per plate in
 *     50 ms bins, and the crowd is split by their RATIO rho, which does not
 *     depend on how much transport one person makes. Every walker's own
 *     backward wobble (eps, learned from single walkers, ~15 %) is taken
 *     out:  out share = (rho - eps) / (1 - 2 eps).
 *     LIMIT: with several people on one plate, U is their average position,
 *     and someone stepping on at the ghat edge or off at the marg edge
 *     shifts U backwards by as much as their walking shifted it forwards. On
 *     a plate carrying n people at once each one's net transport is diluted
 *     to 1/n: at 2-3 per plate (~300 people/min on a 7 m road) the split is
 *     still usable, in a crush (4-5 per plate) the ratio tends to 50 / 50
 *     whichever way the crowd walks and the direction is lost. One row of
 *     plates cannot tell direction in a crush; a second row further along
 *     the marg (arrival order between the rows) would.
 *     The direction never books more people than the engine counted:
 *     in + out = crossings.
 *
 *  Pure computation, no I/O, no dynamic memory (~0.3 KB RAM per 7 plates).
 * ============================================================================
 */
#pragma once
#include <Arduino.h>
#include <math.h>

#ifndef DT_MAX_PADS
#define DT_MAX_PADS 8
#endif

namespace dt {
static const uint8_t BIN_FRAMES = 4;      // transport is booked in 50 ms bins...
static const uint8_t DELAY_BINS = 16;     // ...0.8 s late: matches the counter's delay line
static const float FRAME_S = 0.0125f;     // 80 Hz
static const float MIN_KG = 8.0f;         // a plate must carry this much to measure motion
static const float MAX_KG = 1500.0f;      // tyres are masked by the engine; a packed crowd puts ~300 kg on a plate
static const float DEFAULT_T1 = 35.0f;    // average kg of transport per person crossing (learned)
static const float T1_MIN = 10.0f, T1_MAX = 110.0f;
static const float LEARN_ALPHA = 0.03f;
static const float DECIDE_R = 0.12f;      // small sessions: |r| above this decides the direction
static const float DEFAULT_EPS = 0.15f;   // share of one walker's transport running backwards (learned)
static const float EPS_MAX = 0.4f;

}  // namespace dt

class DirectionTracker {
 public:
  void begin(uint8_t numPads) {
    n_ = numPads > DT_MAX_PADS ? DT_MAX_PADS : numPads;
    t1_ = dt::DEFAULT_T1;
    eps_ = dt::DEFAULT_EPS;
    learned_ = 0;
    resetCount();
    resetSignal();
  }

  void resetSignal() {
    for (uint8_t i = 0; i < DT_MAX_PADS; i++) {
      for (uint8_t k = 0; k < 3; k++) hg_[i][k] = hm_[i][k] = 0;
      prevS_[i] = prevM_[i] = 0;
      plateBin_[i] = 0;
      valid_[i] = false;
    }
    for (uint8_t k = 0; k < dt::DELAY_BINS; k++) posBins_[k] = negBins_[k] = loadBins_[k] = timeBins_[k] = 0;
    binPos_ = 0;
    frameInBin_ = 0;
    loadBin_ = timeBin_ = 0;
    pendPos_ = pendNeg_ = pendLoad_ = pendTime_ = 0;
  }

  void resetCount() {
    in_ = out_ = 0;
    inFrac_ = outFrac_ = 0;
  }

  // g[i], m[i]: tared kg on the ghat-side and marg-side cell pairs of plate i.
  // skipMask: plates the counting engine has masked as a vehicle tyre.
  void processFrame(const float* g, const float* m, uint8_t skipMask) {
    for (uint8_t i = 0; i < n_; i++) {
      // median of 3 per channel: a single ADC bit error must not look like motion
      hg_[i][2] = hg_[i][1]; hg_[i][1] = hg_[i][0]; hg_[i][0] = g[i];
      hm_[i][2] = hm_[i][1]; hm_[i][1] = hm_[i][0]; hm_[i][0] = m[i];
      float G = median3(hg_[i][0], hg_[i][1], hg_[i][2]);
      float M = median3(hm_[i][0], hm_[i][1], hm_[i][2]);
      float S = G + M;
      bool masked = skipMask & (1 << i);
      bool ok = !masked && S < dt::MAX_KG;
      if (ok && valid_[i]) {
        float sAvg = 0.5f * (S + prevS_[i]);
        if (sAvg > dt::MIN_KG) {
          float u = 0.5f * (M + prevM_[i]) / sAvg;
          if (u < 0) u = 0;
          if (u > 1) u = 1;
          plateBin_[i] += (M - prevM_[i]) - u * (S - prevS_[i]);
          loadBin_ += sAvg * dt::FRAME_S;  // plate load x time: how crowded the loaded plates are
          timeBin_ += dt::FRAME_S;
        }
      }
      valid_[i] = !masked;
      prevS_[i] = S;
      prevM_[i] = M;
    }
    // Every 50 ms: the transport of each plate goes to the "towards the marg"
    // (+) or "towards the ghat" (-) total, then through the 0.8 s delay line.
    if (++frameInBin_ >= dt::BIN_FRAMES) {
      frameInBin_ = 0;
      float pos = 0, neg = 0;
      for (uint8_t i = 0; i < n_; i++) {
        if (plateBin_[i] > 0) pos += plateBin_[i];
        else neg -= plateBin_[i];
        plateBin_[i] = 0;
      }
      pendPos_ += posBins_[binPos_];
      pendNeg_ += negBins_[binPos_];
      pendLoad_ += loadBins_[binPos_];
      pendTime_ += timeBins_[binPos_];
      posBins_[binPos_] = pos;
      negBins_[binPos_] = neg;
      loadBins_[binPos_] = loadBin_;
      timeBins_[binPos_] = timeBin_;
      loadBin_ = timeBin_ = 0;
      binPos_ = (uint8_t)((binPos_ + 1) % dt::DELAY_BINS);
    }
  }

  // The counting engine just committed `people` (may be fractional for crowd
  // flows). `share` = fraction of the pending transport that belongs to them:
  // 1 when the crossing session closed, less when a long crowd session commits
  // a chunk. `single` = the engine is sure this was one person (learn T1).
  // `small` = a small, rounded session (1-3 people): decide its direction.
  // `kgPerPerson` = the engine's V1 (people per plate, for diagnostics).
  void commit(float people, float share, bool single, bool small, float kgPerPerson) {
    if (share > 1) share = 1;
    if (share < 0) share = 0;
    float pos = pendPos_ * share, neg = pendNeg_ * share;
    float load = pendLoad_ * share, time = pendTime_ * share;
    pendPos_ -= pos;
    pendNeg_ -= neg;
    pendLoad_ -= load;
    pendTime_ -= time;
    float used = pos - neg;  // net transport, + = towards the marg (out)
    lastTransport_ = used;
    lastOccupancy_ = time > 0.01f && kgPerPerson > 1 ? load / time / kgPerPerson : 0;
    if (people <= 0) return;
    if (single && pos + neg > 1.0f) {
      // T1  = AVERAGE net transport of one person (no threshold: unbiased)
      // eps = share of a walker's transport that runs BACKWARDS (foot-to-foot
      //       load shifts, heel strike): the reach of the crowd ratio below
      t1_ += dt::LEARN_ALPHA * (fabs(used) - t1_);
      t1_ = constrain(t1_, dt::T1_MIN, dt::T1_MAX);
      float back = (pos < neg ? pos : neg) / (pos + neg);
      eps_ += dt::LEARN_ALPHA * (back - eps_);
      eps_ = constrain(eps_, 0.0f, dt::EPS_MAX);
      learned_++;
    }
    float outShare;
    if (small) {
      // One person, or a family walking together: the SIGN of the transport is
      // reliable, its size is not (it depends on where the feet landed). Only
      // a near-zero reading is ambiguous (someone stepping on and turning back,
      // two people passing each other): split it evenly.
      float r = used / (people * t1_);
      if (r >= dt::DECIDE_R) outShare = 1;
      else if (r <= -dt::DECIDE_R) outShare = 0;
      else outShare = 0.5f + 0.5f * r / dt::DECIDE_R;
    } else {
      // Crowd flow: the ratio of forward to backward transport, corrected
      // for every walker's backward wobble (see the limit at the top).
      float tot = pos + neg;
      if (tot < 1.0f) outShare = 0.5f;
      else outShare = (pos / tot - eps_) / (1.0f - 2.0f * eps_);
    }
    if (outShare > 1) outShare = 1;
    if (outShare < 0) outShare = 0;
    inFrac_ += people * (1 - outShare);
    outFrac_ += people * outShare;
    roll();
  }

  // ---- accessors -----------------------------------------------------------
  uint32_t in() const { return in_; }      // people who walked INTO the premises
  uint32_t out() const { return out_; }    // people who walked OUT
  float inExact() const { return in_ + inFrac_; }
  float outExact() const { return out_ + outFrac_; }
  float transportPerPerson() const { return t1_; }
  uint16_t singlesLearned() const { return learned_; }
  float pendingTransport() const { return pendPos_ - pendNeg_; }
  float lastTransport() const { return lastTransport_; }  // kg moved by the last commit (+ = out)
  float lastOccupancy() const { return lastOccupancy_; }  // people per loaded plate during the last commit
  float backwardShare() const { return eps_; }

 private:
  void roll() {
    while (inFrac_ >= 1.0f) { inFrac_ -= 1.0f; in_++; }
    while (outFrac_ >= 1.0f) { outFrac_ -= 1.0f; out_++; }
  }
  static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
  }

  uint8_t n_ = 0;
  float hg_[DT_MAX_PADS][3], hm_[DT_MAX_PADS][3];
  float prevS_[DT_MAX_PADS], prevM_[DT_MAX_PADS];
  bool valid_[DT_MAX_PADS];
  float plateBin_[DT_MAX_PADS];
  float loadBin_ = 0, timeBin_ = 0;
  float posBins_[dt::DELAY_BINS], negBins_[dt::DELAY_BINS], loadBins_[dt::DELAY_BINS], timeBins_[dt::DELAY_BINS];
  uint8_t binPos_ = 0, frameInBin_ = 0;
  float pendPos_ = 0, pendNeg_ = 0, pendLoad_ = 0, pendTime_ = 0;
  float lastTransport_ = 0, lastOccupancy_ = 0;
  float t1_ = dt::DEFAULT_T1;
  float eps_ = dt::DEFAULT_EPS;
  uint16_t learned_ = 0;
  uint32_t in_ = 0, out_ = 0;
  float inFrac_ = 0, outFrac_ = 0;
};
