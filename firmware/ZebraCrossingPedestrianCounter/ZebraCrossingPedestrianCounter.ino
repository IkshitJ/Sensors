/*
 * ============================================================================
 *  PROTOTYPE : COUNTING PEOPLE IN KUMBH — v4 (entry / exit gate counters)
 *  Ram Ghat gates on Ramghat Marg, Ujjain (Simhastha Kumbh)
 * ============================================================================
 *
 *  Board:     Arduino Mega 2560 (or Teensy 4.x / ESP32 for CPU headroom)
 *  Library:   "HX711 Arduino Library" by Bogdan Necula (Library Manager)
 *  Files:     ZebraCrossingPedestrianCounter.ino  (this file: hardware, I/O)
 *             PedestrianCounter.h                 (how MANY people crossed)
 *             DirectionTracker.h                  (which WAY they walked; only
 *                                                  needed on a two-way strip)
 *
 *  LAYOUT (as the ghats were run at Simhastha 2016: entry and exit kept
 *  apart by barricades, so that incoming and outgoing crowds never meet)
 *   Ramghat Marg is split by a barricade into two one-way corridors, each
 *   7 m wide: the ENTRY corridor (everyone walks towards the ghat) and the
 *   EXIT corridor (everyone walks away from it). Each corridor has its own
 *   gate, and in each gate a strip of 14 steel plates (0.5 m across x 1.0 m
 *   along the corridor) spans the full 7 m width.
 *   ONE BOARD PER STRIP, both running this sketch:
 *     BOARD_ROLE = ROLE_ENTRY  counts everyone who crosses as IN
 *     BOARD_ROLE = ROLE_EXIT   counts everyone who crosses as OUT
 *   The two boards are joined by a serial cable (Serial1, pins 18/19, GND):
 *   each sends its running total to the other 4 times a second, so BOTH know
 *   how many people are INSIDE the premises (in - out + the headcount).
 *   The direction no longer has to be measured: on a barricaded one-way
 *   corridor it is a property of the gate, not of the signal. The direction
 *   tracker still runs, with one job: someone who steps onto the strip and
 *   turns back moves their weight there and back, so their session is
 *   dropped instead of counted. (ROLE_TWO_WAY keeps the v3 behaviour for a
 *   single strip across a two-way road, where the tracker splits the
 *   crossings in/out from the load pattern.)
 *   OVERCROWDING ALERT: when the people inside reach the limit (300 by
 *   default) ALERT_PIN goes HIGH (drive a relay for a siren / beacon / gate
 *   signal) and the board prints an alarm. It clears below 95 % of the limit.
 *
 *  HARDWARE (per board)
 *   - 14 plates, each on 4 x 1000 kg shear-beam load cells
 *   - the 4 cells of a plate are wired as TWO bridges, two junction boxes:
 *       G = the 2 cells under the GHAT-side edge  -> its own HX711
 *       M = the 2 cells under the MARG-side edge  -> its own HX711
 *     G + M = the weight on the plate; M / (G + M) = where on the plate it is.
 * *   - 28 HX711, RATE pin tied to VCC => 80 samples/s
 *   - Serial1 (TX1 pin 18, RX1 pin 19) crossed over to the other board
 *
 *  SERIAL (115200 baud)
 *   Human-readable event lines, plus machine-readable telemetry for a logger:
 *     @T,<ms>,<total>,<live>,<vehicleActive>,<vehicleMask>,<in>,<out>,<inside>,<alert 0|1>,<limit>,<plate0..plate13 kg>
 *     @E,<ms>,<half 1|2>,<peopleThisEvent>,<estimate>,<total>,<in>,<out>,<inside>,<net>,<speed>,<occupancy>
 *       (net = direction of this event in people: + out, - in; fixed by the
 *        gate on a one-way board; speed = the crowd's walking speed relative
 *        to free walking, from its density; occupancy = people per loaded
 *        plate during the event)
 *     @A,<ms>,<alert 0|1>,<inside>,<limit>      when the overcrowding alert changes
 *     @X,<role E|X|W>,<total>                   the link message, 4 x per second
 *                                               (sent on Serial1, echoed here)
 *   Commands: r=reset counts  t=tare  s=status  c=calibration info
 *             o<n>=set people inside now (e.g. o0, o2500)
 *             l<n>=overcrowding limit (e.g. l300)
 *             d=debug line on/off  m=telemetry on/off  h=help
 * ============================================================================
 */

#include <HX711.h>
#define PC_MAX_PADS 7  // plates per engine (keeps RAM low on the Mega)
#define DT_MAX_PADS 7
#include "PedestrianCounter.h"
#include "DirectionTracker.h"

// ============================================================================
//  CONFIGURATION
// ============================================================================

// Which gate this board stands in. Flash the ENTRY board with ROLE_ENTRY and
// the EXIT board with ROLE_EXIT (nothing else differs between them).
#define ROLE_TWO_WAY 0   // v3: one strip across a two-way road, direction measured
#define ROLE_ENTRY   1   // the strip in the entry corridor: everyone crossing walks IN
#define ROLE_EXIT    2   // the strip in the exit corridor: everyone crossing walks OUT
#ifndef BOARD_ROLE
#define BOARD_ROLE ROLE_ENTRY
#endif
#define ONE_WAY (BOARD_ROLE != ROLE_TWO_WAY)

#define LINK          Serial1   // to the other gate's board (TX1 18 -> its RX1 19 and back, GND to GND)
#define LINK_MS       250UL     // how often the running total is sent over the link

#define NUM_PLATES 14
#define HALF       7   // plates per counting engine (two engines cover the width)

// HX711 pin pairs {DOUT, SCK}. Plate 0 is at one kerb, plate 13 at the other.
// Ghat-side cell pairs: pins 22-49 (as in v2).
const uint8_t PINS_G[NUM_PLATES][2] = {
  {22, 23}, {24, 25}, {26, 27}, {28, 29}, {30, 31}, {32, 33}, {34, 35},
  {36, 37}, {38, 39}, {40, 41}, {42, 43}, {44, 45}, {46, 47}, {48, 49},
};
// Marg-side cell pairs: analog pins A0-A15 (as digital I/O), then 2-13.
const uint8_t PINS_M[NUM_PLATES][2] = {
  {A0, A1}, {A2, A3}, {A4, A5}, {A6, A7}, {A8, A9}, {A10, A11}, {A12, A13},
  {A14, A15}, {2, 3}, {4, 5}, {6, 7}, {8, 9}, {10, 11}, {12, 13},
};

// Calibration factors in ADC counts per kg, one per bridge (2 cells each).
// Nominal for 2 x 1000 kg, 2 mV/V cells at 5 V, gain 128 is ~2147.4.
// Measure each plate with HX711_Calibration.ino and paste its 'a' output here.
float CAL_G[NUM_PLATES] = {
  2147.4, 2147.4, 2147.4, 2147.4, 2147.4, 2147.4, 2147.4,
  2147.4, 2147.4, 2147.4, 2147.4, 2147.4, 2147.4, 2147.4,
};
float CAL_M[NUM_PLATES] = {
  2147.4, 2147.4, 2147.4, 2147.4, 2147.4, 2147.4, 2147.4,
  2147.4, 2147.4, 2147.4, 2147.4, 2147.4, 2147.4, 2147.4,
};

// Overcrowding alert: people inside the premises at which the alert fires.
// Set to the safe capacity of the area these gates serve (l<n> changes it).
#define CROWD_LIMIT_DEFAULT 300L    // people inside the premises that trigger the alert
#define ALERT_PIN       50          // HIGH while overcrowded: relay for siren / beacon / gate signal
#define ALERT_REPEAT_MS 10000UL     // repeat the alarm line this often while it lasts

#define FRAME_US        12500UL   // 80 Hz processing frame (matches HX711 at 80 SPS)
#define TELEMETRY_MS    250UL
#define DEBUG_MS        1000UL
#define HX711_MAX       8388607L
#define HX711_MIN      (-8388608L)

// ============================================================================
//  GLOBALS
// ============================================================================

HX711 scaleG[NUM_PLATES], scaleM[NUM_PLATES];
PedestrianCounter engine[2];    // plates 0-6 and 7-13: how many crossed
DirectionTracker direction[2];  // same plates: which way they walked

float kgG[NUM_PLATES], kgM[NUM_PLATES];   // tared kg per bridge
float plateKg[NUM_PLATES];                // G + M: what the counting engine sees
bool plateOverload[NUM_PLATES];

long insideBase = 0;        // headcount inside when the count started (set with o<n>)
long crowdLimit = CROWD_LIMIT_DEFAULT;  // set with l<n>
bool alertOn = false;
uint32_t lastAlertMs = 0;

// the other gate's board, over the link
long remoteTotal = 0;       // its running total (people through its gate since its last reset)
long remoteBase = 0;        // its total when our counts were last reset
bool linkSeen = false;      // a valid message has arrived
uint32_t lastLinkRxMs = 0, lastLinkTxMs = 0;
char linkBuf[24];
uint8_t linkLen = 0;

uint32_t lastFrameUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastDebugMs = 0;
uint16_t maxFrameUs = 0;  // worst processing time of one frame (CPU budget check)
bool telemetryOn = true;
bool debugOn = false;

char cmdBuf[16];
uint8_t cmdLen = 0;

// ============================================================================
//  HELPERS
// ============================================================================

// people who stepped onto a one-way strip and turned back (not crossings)
uint32_t turnedBack[2] = {0, 0};
uint32_t totalCount() { return engine[0].total() + engine[1].total() - turnedBack[0] - turnedBack[1]; }
float liveCount() { return engine[0].liveEstimate() + engine[1].liveEstimate() - turnedBack[0] - turnedBack[1]; }
// people through the OTHER gate since our last reset (0 until its board talks)
long remoteCount() { return linkSeen ? remoteTotal - remoteBase : 0; }
#if BOARD_ROLE == ROLE_ENTRY
uint32_t inCount() { return totalCount(); }
uint32_t outCount() { return (uint32_t)remoteCount(); }
long insideCount() { return insideBase + (long)totalCount() - remoteCount(); }
#elif BOARD_ROLE == ROLE_EXIT
uint32_t inCount() { return (uint32_t)remoteCount(); }
uint32_t outCount() { return totalCount(); }
long insideCount() { return insideBase + remoteCount() - (long)totalCount(); }
#else
uint32_t inCount() { return direction[0].in() + direction[1].in(); }
uint32_t outCount() { return direction[0].out() + direction[1].out(); }
long insideCount() {
  // from the exact (fractional) totals so rounding of either side can't drift
  float net = direction[0].inExact() + direction[1].inExact() - direction[0].outExact() - direction[1].outExact();
  return insideBase + (long)(net < 0 ? net - 0.5f : net + 0.5f);
}
#endif
// Nobody can be inside fewer than 0 people: when direction errors would push
// the count below 0, the premises must be empty, so the count restarts at 0
// (the same as an automatic o0 headcount).
void keepInsideNonNegative() {
  long n = insideCount();
  if (n < 0) insideBase -= n;
}
uint16_t vehicleMask() { return (uint16_t)engine[0].vehicleMask() | ((uint16_t)engine[1].vehicleMask() << HALF); }
float plateLoad(uint8_t i) { return i < HALF ? engine[0].padLoad(i) : engine[1].padLoad(i - HALF); }

void tareAll() {
  for (uint8_t i = 0; i < NUM_PLATES; i++) {
    scaleG[i].tare(20);
    scaleM[i].tare(20);
    kgG[i] = kgM[i] = plateKg[i] = 0;
    plateOverload[i] = false;
  }
  for (uint8_t h = 0; h < 2; h++) {
    engine[h].resetSignal();
    direction[h].resetSignal();
  }
}

bool readBridge(HX711& s, float& kg) {
  if (!s.is_ready()) return false;  // keep previous sample; chips aren't synchronised
  long raw = s.read();
  kg = (float)(raw - s.get_offset()) / s.get_scale();
  return raw >= HX711_MAX || raw <= HX711_MIN;
}

void readPlates() {
  for (uint8_t i = 0; i < NUM_PLATES; i++) {
    bool ovG = readBridge(scaleG[i], kgG[i]);
    bool ovM = readBridge(scaleM[i], kgM[i]);
    plateKg[i] = kgG[i] + kgM[i];
    plateOverload[i] = ovG || ovM;
  }
}

// ---- the link to the other gate's board ------------------------------------
// Each board sends "@X,<role>,<total>" 4 times a second. Totals, not deltas:
// a lost line costs nothing, the next one carries the same information.
void sendLink() {
  LINK.print(F("@X,"));
  LINK.print(BOARD_ROLE == ROLE_ENTRY ? 'E' : (BOARD_ROLE == ROLE_EXIT ? 'X' : 'W'));
  LINK.print(',');
  LINK.println(totalCount());
}

void onLinkLine(const char* line) {
  if (line[0] != '@' || line[1] != 'X' || line[2] != ',' || line[4] != ',') return;
  long total = atol(line + 5);
  if (total < remoteBase) remoteBase = 0;  // the other board was reset
  remoteTotal = total;
  linkSeen = true;
  lastLinkRxMs = millis();
}

void readLink() {
  while (LINK.available()) {
    char c = (char)LINK.read();
    if (c == '\r') continue;
    if (c == '\n') {
      linkBuf[linkLen] = 0;
      if (linkLen) onLinkLine(linkBuf);
      linkLen = 0;
    } else if (linkLen < sizeof(linkBuf) - 1) {
      linkBuf[linkLen++] = c;
    } else {
      linkLen = 0;  // garbage: start over
    }
  }
}

// The engine of half h may have added people this frame: split them into
// in / out with the direction tracker. On a one-way corridor the direction is
// the gate's, so the tracker has one job left: to catch people who stepped
// onto the strip and TURNED BACK. Their weight goes onto the plate and back
// off the same side, so their net transport is near zero or runs the wrong
// way, while anyone who crossed moved their weight clearly the gate's way.
// Checked on small sessions (up to 4 counted), which is what a turn-back
// looks like to the engine; a crowd flow is taken as crossing.
void bookPeople(uint8_t h, bool closed, uint32_t before) {
  PedestrianCounter& c = engine[h];
  uint32_t added = c.total() - before;
  if (closed) {
    const PedestrianCounter::Event& e = c.lastEvent();
    // crowd flows keep fractions: split the estimate, not the rounded count
    float people = e.fractional ? e.estimate : (float)added;
    bool single = !e.fractional && e.people == 1 && e.estimate > 0.6f && e.estimate < 1.4f;
    bool small = !e.fractional && added <= 3;
    direction[h].commit(people, 1.0f, single, small, c.personVariation());
#if ONE_WAY
    if (!e.fractional && added > 0 && added <= 4) {
      float r = direction[h].lastTransport() / (people * direction[h].transportPerPerson());
      bool crossed = BOARD_ROLE == ROLE_EXIT ? r >= dt::DECIDE_R : r <= -dt::DECIDE_R;
      if (!crossed) turnedBack[h] += added;
    }
#endif
  } else if (added > 0) {
    // a long crowd session committed a chunk; it keeps running
    float open = c.liveEstimate() - (float)c.total();
    if (open < 0) open = 0;
    direction[h].commit((float)added, (float)added / ((float)added + open), false, false, c.personVariation());
  }
}

void printAlert() {
  Serial.print(F("@A,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(alertOn ? 1 : 0);
  Serial.print(',');
  Serial.print(insideCount());
  Serial.print(',');
  Serial.println(crowdLimit);
  if (alertOn) {
    Serial.print(F("!!! OVERCROWDING ALERT: "));
    Serial.print(insideCount());
    Serial.print(F(" people inside Ram Ghat (limit "));
    Serial.print(crowdLimit);
    Serial.println(F(") - hold entry at the gate !!!"));
  } else {
    Serial.print(F("*** Overcrowding cleared: "));
    Serial.print(insideCount());
    Serial.println(F(" people inside ***"));
  }
}

// Alert at the limit, clear below 95 % of it, so it doesn't flicker when the
// count hovers around the limit.
void checkOvercrowding() {
  long n = insideCount();
  bool was = alertOn;
  if (!alertOn && n >= crowdLimit) alertOn = true;
  else if (alertOn && n < crowdLimit - crowdLimit / 20) alertOn = false;
  if (alertOn != was) {
    digitalWrite(ALERT_PIN, alertOn ? HIGH : LOW);
    lastAlertMs = millis();
    printAlert();
  } else if (alertOn && millis() - lastAlertMs >= ALERT_REPEAT_MS) {
    lastAlertMs = millis();
    printAlert();
  }
}

void printTelemetry() {
  Serial.print(F("@T,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(totalCount());
  Serial.print(',');
  Serial.print(liveCount(), 1);
  Serial.print(',');
  Serial.print((engine[0].vehicleActive() || engine[1].vehicleActive()) ? 1 : 0);
  Serial.print(',');
  Serial.print(vehicleMask());
  Serial.print(',');
  Serial.print(inCount());
  Serial.print(',');
  Serial.print(outCount());
  Serial.print(',');
  Serial.print(insideCount());
  Serial.print(',');
  Serial.print(alertOn ? 1 : 0);
  Serial.print(',');
  Serial.print(crowdLimit);
  for (uint8_t i = 0; i < NUM_PLATES; i++) {
    Serial.print(',');
    Serial.print(plateLoad(i), 0);
  }
  Serial.println();
}

void printEvent(uint8_t h) {
  const PedestrianCounter::Event& e = engine[h].lastEvent();
  Serial.print(F("@E,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(h + 1);
  Serial.print(',');
  Serial.print(e.people);
  Serial.print(',');
  Serial.print(e.estimate, 2);
  Serial.print(',');
  Serial.print(totalCount());
  Serial.print(',');
  Serial.print(inCount());
  Serial.print(',');
  Serial.print(outCount());
  Serial.print(',');
  Serial.print(insideCount());
  Serial.print(',');
  Serial.print(direction[h].lastTransport() / direction[h].transportPerPerson(), 2);  // net people, + = out
  Serial.print(',');
  Serial.print(engine[h].speedFactor(), 2);  // crowd walking speed, relative to free walking
  Serial.print(',');
  Serial.println(direction[h].lastOccupancy(), 2);  // people per loaded plate
  if (e.people > 0) {
    Serial.print(F("+"));
    Serial.print(e.people);
    Serial.print(e.fractional ? F(" (crowd flow, est ") : F(" (est "));
    Serial.print(e.estimate, 2);
    Serial.print(F(")   CROSSED: "));
    Serial.print(totalCount());
    Serial.print(F("  [in <- "));
    Serial.print(inCount());
    Serial.print(F(" | out -> "));
    Serial.print(outCount());
    Serial.print(F("]   INSIDE RAM GHAT: "));
    Serial.println(insideCount());
  }
}

void printDebug() {
  uint16_t mask = vehicleMask();
  Serial.print(F("["));
  for (uint8_t i = 0; i < NUM_PLATES; i++) {
    float w = plateLoad(i);
    if (mask & (1 << i)) Serial.print(F(" VEH"));
    else if (w > 3) { Serial.print(' '); if (w < 100) Serial.print(' '); if (w < 10) Serial.print(' '); Serial.print(w, 0); }
    else Serial.print(F("  __"));
  }
  Serial.print(F(" ] crossed="));
  Serial.print(totalCount());
  Serial.print(F(" inside="));
  Serial.print(insideCount());
  Serial.print(F(" live~"));
  Serial.println(liveCount(), 1);
}

void printHalfStatus(uint8_t h) {
  PedestrianCounter& c = engine[h];
  Serial.print(h == 0 ? F("  Plates 0-6         : ") : F("  Plates 7-13        : "));
  Serial.print(c.total() - turnedBack[h]);
#if !ONE_WAY
  Serial.print(F(" crossed, in ")); Serial.print(direction[h].in());
  Serial.print(F(" / out ")); Serial.print(direction[h].out());
#else
  Serial.print(F(" crossed, turned back ")); Serial.print(turnedBack[h]);
#endif
  Serial.print(F("  (vehicle events ")); Serial.print(c.vehicleEvents());
  Serial.print(F(", transients rejected ")); Serial.print(c.transientsRejected());
  Serial.print(F(", learned V1 ")); Serial.print(c.personVariation(), 1);
  Serial.print(F(" / I1 ")); Serial.print(c.personImpulse(), 1);
  Serial.print(F(" / T1 ")); Serial.print(direction[h].transportPerPerson(), 1);
  Serial.print(F(" / eps ")); Serial.print(direction[h].backwardShare(), 2);
  Serial.print(F(", crowd ")); Serial.print(c.crowdDensity(), 1); Serial.print(F("/m2 at ")); Serial.print(c.speedFactor() * 100, 0); Serial.print(F("% speed"));
  Serial.print(F(" from ")); Serial.print(c.singlesLearned()); Serial.println(F(" singles)"));
}

void printRole() {
#if BOARD_ROLE == ROLE_ENTRY
  Serial.println(F("  ENTRY gate board: everyone crossing walks IN (turn-backs dropped); OUT comes from the exit board over the link"));
#elif BOARD_ROLE == ROLE_EXIT
  Serial.println(F("  EXIT gate board: everyone crossing walks OUT (turn-backs dropped); IN comes from the entry board over the link"));
#else
  Serial.println(F("  Two-way strip: direction measured from the load pattern"));
#endif
}

void printLinkStatus() {
#if ONE_WAY
  Serial.print(F("  Link to other gate  : "));
  if (!linkSeen) Serial.println(F("NO DATA YET (cable? other board off?)"));
  else {
    Serial.print(remoteCount()); Serial.print(F(" through it since reset, last message "));
    Serial.print((millis() - lastLinkRxMs) / 1000); Serial.println(F(" s ago"));
  }
#endif
}

void printStatus() {
  Serial.println(F("\n=== STATUS ==="));
  printRole();
  Serial.print(F("  PEOPLE CROSSED HERE : ")); Serial.println(totalCount());
  Serial.print(F("  IN  (<- to ghat)    : ")); Serial.println(inCount());
  Serial.print(F("  OUT (-> to marg)    : ")); Serial.println(outCount());
  Serial.print(F("  INSIDE RAM GHAT     : ")); Serial.print(insideCount());
  Serial.print(F("  (started at ")); Serial.print(insideBase); Serial.println(F(")"));
  printLinkStatus();
  Serial.print(F("  OVERCROWDING LIMIT  : ")); Serial.print(crowdLimit);
  Serial.println(alertOn ? F("   *** ALERT ON ***") : F("   (ok)"));
  printHalfStatus(0);
  printHalfStatus(1);
  Serial.print(F("  Live estimate       : ")); Serial.println(liveCount(), 1);
  Serial.print(F("  Worst frame time    : ")); Serial.print(maxFrameUs); Serial.println(F(" us of 12500"));
  Serial.println(F("==============\n"));
}

void printCalibration() {
  Serial.println(F("\n=== CALIBRATION ==="));
  for (uint8_t i = 0; i < NUM_PLATES; i++) {
    Serial.print(F("  Plate ")); Serial.print(i);
    Serial.print(F(": ghat-side ")); Serial.print(scaleG[i].get_scale(), 1);
    Serial.print(F(" / marg-side ")); Serial.print(scaleM[i].get_scale(), 1);
    Serial.print(F(" counts/kg, auto-zero "));
    Serial.print(i < HALF ? engine[0].padBaseline(i) : engine[1].padBaseline(i - HALF), 2);
    Serial.println(F(" kg"));
  }
  Serial.println(F("===================\n"));
}

void printHelp() {
  Serial.println(F("Commands: r=reset counts  t=tare (clear the strip!)  s=status  c=calibration"));
  Serial.println(F("          o<n>=people inside now, e.g. o0 or o2500  l<n>=overcrowding limit, e.g. l300"));
  Serial.println(F("          d=debug  m=telemetry  h=help"));
}

void runCommand(const char* cmd) {
  switch (cmd[0]) {
    case 'r': case 'R':
      for (uint8_t h = 0; h < 2; h++) {
        engine[h].resetCount();
        direction[h].resetCount();
        turnedBack[h] = 0;
      }
      insideBase = 0;
      remoteBase = remoteTotal;  // the other gate's count restarts from now too
      Serial.println(F("\n*** COUNTS RESET TO ZERO ***\n"));
      break;
    case 't': case 'T':
      Serial.println(F("\n*** TARING ALL PLATES - keep the strip clear ***"));
      tareAll();
      Serial.println(F("*** TARE COMPLETE ***\n"));
      break;
    case 'o': case 'O': {
      // set the current headcount inside; keeps in/out totals
      long want = atol(cmd + 1);
      insideBase += want - insideCount();
      Serial.print(F("\n*** PEOPLE INSIDE SET TO ")); Serial.print(insideCount()); Serial.println(F(" ***\n"));
      break;
    }
    case 'l': case 'L': {
      long lim = atol(cmd + 1);
      if (lim <= 0) { Serial.println(F("Usage: l<people>, e.g. l400")); break; }
      crowdLimit = lim;
      Serial.print(F("\n*** OVERCROWDING LIMIT SET TO ")); Serial.print(crowdLimit); Serial.println(F(" ***\n"));
      break;
    }
    case 's': case 'S': printStatus(); break;
    case 'c': case 'C': printCalibration(); break;
    case 'd': case 'D': debugOn = !debugOn; break;
    case 'm': case 'M': telemetryOn = !telemetryOn; break;
    case 'h': case 'H': case '?': printHelp(); break;
    default: break;
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    // one-key commands act at once; 'o' and 'l' wait for their number and Enter
    if (cmdLen == 0 && c != 'o' && c != 'O' && c != 'l' && c != 'L' && c != '\n') {
      char one[2] = {c, 0};
      runCommand(one);
      continue;
    }
    if (c == '\n') {
      cmdBuf[cmdLen] = 0;
      if (cmdLen) runCommand(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  LINK.begin(115200);
  Serial.println(F("================================================"));
  Serial.println(F("  PROTOTYPE : COUNTING PEOPLE IN KUMBH v4"));
#if BOARD_ROLE == ROLE_ENTRY
  Serial.println(F("  Ram Ghat ENTRY gate, Ramghat Marg"));
#elif BOARD_ROLE == ROLE_EXIT
  Serial.println(F("  Ram Ghat EXIT gate, Ramghat Marg"));
#else
  Serial.println(F("  Ram Ghat gate, Ramghat Marg - two-way strip"));
#endif
  Serial.println(F("  14 plates x 2 bridges (keep the strip clear)"));
  Serial.println(F("================================================"));
  printRole();

  for (uint8_t i = 0; i < NUM_PLATES; i++) {
    scaleG[i].begin(PINS_G[i][0], PINS_G[i][1]);
    scaleG[i].set_scale(CAL_G[i]);
    scaleM[i].begin(PINS_M[i][0], PINS_M[i][1]);
    scaleM[i].set_scale(CAL_M[i]);
  }
  for (uint8_t h = 0; h < 2; h++) {
    engine[h].begin(HALF);
    direction[h].begin(HALF);
  }
  tareAll();
  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_PIN, LOW);

  Serial.println(F("  System READY. 80 Hz frames, vehicle masking on."));
  Serial.print(F("  Overcrowding alert at ")); Serial.print(crowdLimit); Serial.println(F(" people inside (pin 50)."));
#if ONE_WAY
  Serial.println(F("  Link to the other gate's board on Serial1 (pins 18/19)."));
#endif
  printHelp();
  Serial.println();
  lastFrameUs = micros();
}

void loop() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_US) {
    lastFrameUs += FRAME_US;
    if ((uint32_t)(nowUs - lastFrameUs) > 4 * FRAME_US) lastFrameUs = nowUs;  // resync after a stall

    readPlates();
    bool closed[2];
    for (uint8_t h = 0; h < 2; h++) {
      uint8_t p0 = h * HALF;
      uint32_t before = engine[h].total();
      closed[h] = engine[h].processFrame(&plateKg[p0], &plateOverload[p0]);
      direction[h].processFrame(&kgG[p0], &kgM[p0], engine[h].vehicleMask());
      bookPeople(h, closed[h], before);
    }
    uint32_t took = micros() - nowUs;
    if (took > maxFrameUs) maxFrameUs = took > 65535UL ? 65535 : (uint16_t)took;
    if (closed[0] || closed[1]) keepInsideNonNegative();
    for (uint8_t h = 0; h < 2; h++)
      if (closed[h]) printEvent(h);
    checkOvercrowding();
  }

  uint32_t nowMs = millis();
#if ONE_WAY
  readLink();
  keepInsideNonNegative();  // the other gate's count may have moved the total
  if ((uint32_t)(nowMs - lastLinkTxMs) >= LINK_MS) {
    lastLinkTxMs = nowMs;
    sendLink();
  }
#endif
  if (telemetryOn && (uint32_t)(nowMs - lastTelemetryMs) >= TELEMETRY_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry();
  }
  if (debugOn && (uint32_t)(nowMs - lastDebugMs) >= DEBUG_MS) {
    lastDebugMs = nowMs;
    printDebug();
  }
  handleSerial();
}
