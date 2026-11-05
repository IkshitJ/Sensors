/*
 * ============================================================================
 *  ZEBRA CROSSING PEDESTRIAN COUNTER — v2 (full-width strip)
 * ============================================================================
 *
 *  Board:     Arduino Mega 2560
 *  Library:   "HX711 Arduino Library" by Bogdan Necula (Library Manager)
 *  Files:     ZebraCrossingPedestrianCounter.ino  (this file: hardware, I/O)
 *             PedestrianCounter.h                 (counting engine)
 *
 *  LAYOUT
 *   The sensor strip lies ON the zebra crossing and spans the full road width
 *   (both lanes): 14 pads of 0.5 m across the road x 1.0 m along the road.
 *     pads 0-6  -> lane B (traffic and people moving  <-)
 *     pads 7-13 -> lane A (traffic and people moving  ->)
 *   People move along the lanes like the vehicles and step over the strip.
 *   Each lane has its own counting engine, so the board reports how many
 *   people passed in each direction plus the total.
 *
 *  HARDWARE
 *   - 14 pads, each with 4 x 1000 kg shear-beam load cells on a steel plate
 *   - 14 HX711, RATE pin tied to VCC => 80 samples/s
 *
 *  SERIAL (115200 baud)
 *   Human-readable event lines, plus machine-readable telemetry for a logger:
 *     @T,<ms>,<total>,<live>,<vehicleActive>,<vehicleMask>,<laneA>,<laneB>,<pad0..pad13 kg>
 *     @E,<ms>,<lane A|B>,<peopleThisEvent>,<estimate>,<total>
 *   Commands: r=reset counts  t=tare  s=status  c=calibration info
 *             d=debug line on/off  m=telemetry on/off  h=help
 * ============================================================================
 */

#include <HX711.h>
#define PC_MAX_PADS 7  // pads per lane engine (keeps RAM low on the Mega)
#include "PedestrianCounter.h"

// ============================================================================
//  CONFIGURATION
// ============================================================================

#define NUM_SENSORS   14
#define PADS_PER_LANE 7

// HX711 pin pairs {DOUT, SCK}; pad 0 is at the lane-B kerb, pad 13 at lane A's
const uint8_t SENSOR_PINS[NUM_SENSORS][2] = {
  {22, 23}, {24, 25}, {26, 27}, {28, 29}, {30, 31}, {32, 33}, {34, 35},
  {36, 37}, {38, 39}, {40, 41}, {42, 43}, {44, 45}, {46, 47}, {48, 49},
};

// Per-pad calibration factor in ADC counts per kg.
// Nominal for 4 x 1000 kg, 2 mV/V cells at 5 V, gain 128 is ~1073.7.
// Measure each pad with HX711_Calibration.ino and paste its 'a' output here.
float CALIBRATION_FACTORS[NUM_SENSORS] = {
  1073.7, 1073.7, 1073.7, 1073.7, 1073.7, 1073.7, 1073.7,
  1073.7, 1073.7, 1073.7, 1073.7, 1073.7, 1073.7, 1073.7,
};

#define FRAME_US        12500UL   // 80 Hz processing frame (matches HX711 at 80 SPS)
#define TELEMETRY_MS    250UL
#define DEBUG_MS        1000UL
#define HX711_MAX       8388607L
#define HX711_MIN      (-8388608L)

// ============================================================================
//  GLOBALS
// ============================================================================

HX711 scales[NUM_SENSORS];
PedestrianCounter laneB;  // pads 0-6,  people moving <-
PedestrianCounter laneA;  // pads 7-13, people moving ->

float padKg[NUM_SENSORS];
bool padOverload[NUM_SENSORS];

uint32_t lastFrameUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastDebugMs = 0;
uint16_t maxFrameUs = 0;  // worst processing time of one frame (CPU budget check)
bool telemetryOn = true;
bool debugOn = false;

// ============================================================================
//  HELPERS
// ============================================================================

uint32_t totalCount() { return laneA.total() + laneB.total(); }
float liveCount() { return laneA.liveEstimate() + laneB.liveEstimate(); }
uint16_t vehicleMask() { return (uint16_t)laneB.vehicleMask() | ((uint16_t)laneA.vehicleMask() << PADS_PER_LANE); }
float padLoad(uint8_t i) { return i < PADS_PER_LANE ? laneB.padLoad(i) : laneA.padLoad(i - PADS_PER_LANE); }

void tareAll() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    scales[i].tare(20);
    padKg[i] = 0;
    padOverload[i] = false;
  }
  laneA.resetSignal();
  laneB.resetSignal();
}

void readPads() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!scales[i].is_ready()) continue;  // keep previous sample; chips aren't synchronised
    long raw = scales[i].read();
    padOverload[i] = (raw >= HX711_MAX || raw <= HX711_MIN);
    padKg[i] = (float)(raw - scales[i].get_offset()) / scales[i].get_scale();
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
  Serial.print((laneA.vehicleActive() || laneB.vehicleActive()) ? 1 : 0);
  Serial.print(',');
  Serial.print(vehicleMask());
  Serial.print(',');
  Serial.print(laneA.total());
  Serial.print(',');
  Serial.print(laneB.total());
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(',');
    Serial.print(padLoad(i), 0);
  }
  Serial.println();
}

void printEvent(char lane, PedestrianCounter& c) {
  const PedestrianCounter::Event& e = c.lastEvent();
  Serial.print(F("@E,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(lane);
  Serial.print(',');
  Serial.print(e.people);
  Serial.print(',');
  Serial.print(e.estimate, 2);
  Serial.print(',');
  Serial.println(totalCount());
  if (e.people > 0) {
    Serial.print(lane == 'A' ? F("Lane A (->): +") : F("Lane B (<-): +"));
    Serial.print(e.people);
    Serial.print(e.fractional ? F(" (crowd flow, est ") : F(" (est "));
    Serial.print(e.estimate, 2);
    Serial.print(F(")   PEOPLE CROSSED: "));
    Serial.print(totalCount());
    Serial.print(F("  [-> "));
    Serial.print(laneA.total());
    Serial.print(F(" | <- "));
    Serial.print(laneB.total());
    Serial.println(']');
  }
}

void printDebug() {
  uint16_t mask = vehicleMask();
  Serial.print(F("B<-["));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (i == PADS_PER_LANE) Serial.print(F(" ] A->["));
    float w = padLoad(i);
    if (mask & (1 << i)) Serial.print(F(" VEH"));
    else if (w > 3) { Serial.print(' '); if (w < 100) Serial.print(' '); if (w < 10) Serial.print(' '); Serial.print(w, 0); }
    else Serial.print(F("  __"));
  }
  Serial.print(F(" ] crossed="));
  Serial.print(totalCount());
  Serial.print(F(" live~"));
  Serial.println(liveCount(), 1);
}

void printLaneStatus(const __FlashStringHelper* name, PedestrianCounter& c) {
  Serial.print(name);
  Serial.print(c.total());
  Serial.print(F("  (vehicle events ")); Serial.print(c.vehicleEvents());
  Serial.print(F(", transients rejected ")); Serial.print(c.transientsRejected());
  Serial.print(F(", learned V1 ")); Serial.print(c.personVariation(), 1);
  Serial.print(F(" / I1 ")); Serial.print(c.personImpulse(), 1);
  Serial.print(F(" from ")); Serial.print(c.singlesLearned()); Serial.println(F(" singles)"));
}

void printStatus() {
  Serial.println(F("\n=== STATUS ==="));
  Serial.print(F("  PEOPLE CROSSED      : ")); Serial.println(totalCount());
  printLaneStatus(F("  Lane A (->)         : "), laneA);
  printLaneStatus(F("  Lane B (<-)         : "), laneB);
  Serial.print(F("  Live estimate       : ")); Serial.println(liveCount(), 1);
  Serial.print(F("  Worst frame time    : ")); Serial.print(maxFrameUs); Serial.println(F(" us of 12500"));
  Serial.println(F("==============\n"));
}

void printCalibration() {
  Serial.println(F("\n=== CALIBRATION ==="));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("  Pad ")); Serial.print(i);
    Serial.print(i < PADS_PER_LANE ? F(" (B)") : F(" (A)"));
    Serial.print(F(": factor ")); Serial.print(scales[i].get_scale(), 1);
    Serial.print(F(" counts/kg, offset ")); Serial.print(scales[i].get_offset());
    float base = i < PADS_PER_LANE ? laneB.padBaseline(i) : laneA.padBaseline(i - PADS_PER_LANE);
    Serial.print(F(", auto-zero ")); Serial.print(base, 2); Serial.println(F(" kg"));
  }
  Serial.println(F("===================\n"));
}

void printHelp() {
  Serial.println(F("Commands: r=reset counts  t=tare (clear the strip!)  s=status  c=calibration  d=debug  m=telemetry  h=help"));
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'r': case 'R':
        laneA.resetCount();
        laneB.resetCount();
        Serial.println(F("\n*** COUNTS RESET TO ZERO ***\n"));
        break;
      case 't': case 'T':
        Serial.println(F("\n*** TARING ALL PADS - keep the strip clear ***"));
        tareAll();
        Serial.println(F("*** TARE COMPLETE ***\n"));
        break;
      case 's': case 'S': printStatus(); break;
      case 'c': case 'C': printCalibration(); break;
      case 'd': case 'D': debugOn = !debugOn; break;
      case 'm': case 'M': telemetryOn = !telemetryOn; break;
      case 'h': case 'H': case '?': printHelp(); break;
      default: break;
    }
  }
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("================================================"));
  Serial.println(F("  ZEBRA CROSSING PEDESTRIAN COUNTER v2"));
  Serial.println(F("  14 pads across both lanes (keep the strip clear)"));
  Serial.println(F("================================================"));

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    scales[i].begin(SENSOR_PINS[i][0], SENSOR_PINS[i][1]);
    scales[i].set_scale(CALIBRATION_FACTORS[i]);
  }
  laneB.begin(PADS_PER_LANE);
  laneA.begin(PADS_PER_LANE);
  tareAll();

  Serial.println(F("  System READY. 80 Hz frames, vehicle masking on."));
  printHelp();
  Serial.println();
  lastFrameUs = micros();
}

void loop() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_US) {
    lastFrameUs += FRAME_US;
    if ((uint32_t)(nowUs - lastFrameUs) > 4 * FRAME_US) lastFrameUs = nowUs;  // resync after a stall

    readPads();
    bool closedB = laneB.processFrame(&padKg[0], &padOverload[0]);
    bool closedA = laneA.processFrame(&padKg[PADS_PER_LANE], &padOverload[PADS_PER_LANE]);
    uint32_t took = micros() - nowUs;
    if (took > maxFrameUs) maxFrameUs = took > 65535UL ? 65535 : (uint16_t)took;
    if (closedB) printEvent('B', laneB);
    if (closedA) printEvent('A', laneA);
  }

  uint32_t nowMs = millis();
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
