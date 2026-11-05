/*
 * ============================================================================
 *  ZEBRA CROSSING PEDESTRIAN COUNTER
 * ============================================================================
 *  
 *  System: Chain of pressure sensors (load cells + HX711) across a zebra crossing
 *  Purpose: Count pedestrians crossing, reject vehicle weight
 *  
 *  HARDWARE SETUP:
 *  ───────────────
 *  - 8 load cell sensor pads arranged in a strip across the zebra crossing
 *  - Each pad is ~0.5m wide, covering a ~4m wide crossing
 *  - Each load cell connects to an HX711 amplifier module
 *  - HX711 modules connect to Arduino Mega (needs many digital pins)
 *  - Optional: LCD display (I2C) for live count
 *  
 *  LOGIC OVERVIEW:
 *  ───────────────
 *  1. Read weight from each sensor pad continuously
 *  2. Per-pad vehicle rejection: if a single pad reads > VEHICLE_THRESHOLD_KG,
 *     that pad's reading is flagged as a vehicle tyre → discard
 *  3. Sum all valid (pedestrian) pad weights → total pedestrian mass on crossing
 *  4. When crossing becomes occupied (total > MIN_DETECTION_KG) and then clears
 *     (total drops back below), we calculate:
 *       people_in_batch = peak_total_weight / AVG_PERSON_WEIGHT_KG
 *  5. Accumulate into a running total
 *  
 *  DENSE CROWD HANDLING:
 *  ─────────────────────
 *  - In a dense crowd, the crossing never fully clears between waves
 *  - We use a "rolling batch" approach: sample peak weight periodically,
 *    compute instantaneous person count, and integrate over time
 *  - A "crossing event" ends after sustained low readings (debounce)
 *  
 *  VEHICLE vs PEDESTRIAN:
 *  ──────────────────────
 *  - Car tyre contact patch ≈ 0.03 m² → ~375 kg on one pad (1500kg / 4 tyres)
 *  - Truck tyre → 1000+ kg on one pad
 *  - Single person on one pad → 30-90 kg typically
 *  - Two people sharing one pad → 60-180 kg
 *  - Dense crowd on one pad (0.5m wide, ~1m deep, ~3 people) → ~195 kg
 *  - THRESHOLD: >250 kg on a SINGLE pad = likely vehicle → reject that pad
 *  
 * ============================================================================
 */

#include <HX711.h>
// Uncomment if using LCD display:
// #include <LiquidCrystal_I2C.h>

// ============================================================================
//  CONFIGURATION — Tune these for your deployment
// ============================================================================

// Number of sensor pads across the crossing
#define NUM_SENSORS         8

// Average person weight (kg) — used to estimate pedestrian count
#define AVG_PERSON_WEIGHT_KG  65.0

// Vehicle rejection threshold per single sensor pad (kg)
// If ONE pad reads more than this, it's a vehicle tyre, not humans
#define VEHICLE_THRESHOLD_KG  250.0

// Minimum total weight (kg) to consider the crossing "occupied"
#define MIN_DETECTION_KG      20.0

// Debounce: crossing must stay below MIN_DETECTION_KG for this many ms
// to be considered "cleared" (handles gaps between dense crowd waves)
#define CLEAR_DEBOUNCE_MS     2000

// How often to sample sensors (ms)
#define SAMPLE_INTERVAL_MS    100

// For continuous dense crowd: integration interval (ms)
// Every this many ms during an active crossing, we snapshot the count
#define INTEGRATION_INTERVAL_MS  1000

// Moving average filter depth (number of readings to average per sensor)
#define FILTER_DEPTH          5

// HX711 calibration factor — YOU MUST CALIBRATE THIS FOR YOUR LOAD CELLS
// Use the calibration sketch (below) to find this value
// Positive or negative depending on wiring
#define CALIBRATION_FACTOR    -7050.0

// Minimum weight per sensor to consider it "active" (noise floor, kg)
#define NOISE_FLOOR_KG        2.0

// ============================================================================
//  PIN DEFINITIONS — Arduino Mega recommended for 8 sensors
// ============================================================================

// HX711 pin pairs: {DOUT, SCK} for each sensor
// Arranged sequentially on digital pins
const uint8_t SENSOR_PINS[NUM_SENSORS][2] = {
  {22, 23},   // Sensor 0: DOUT=22, SCK=23
  {24, 25},   // Sensor 1
  {26, 27},   // Sensor 2
  {28, 29},   // Sensor 3
  {30, 31},   // Sensor 4
  {32, 33},   // Sensor 5
  {34, 35},   // Sensor 6
  {36, 37},   // Sensor 7
};

// ============================================================================
//  GLOBAL STATE
// ============================================================================

HX711 scales[NUM_SENSORS];

// Per-sensor reading buffer for moving average filter
float sensorBuffer[NUM_SENSORS][FILTER_DEPTH];
int   bufferIndex[NUM_SENSORS];
bool  bufferFilled[NUM_SENSORS];

// Current filtered weight per sensor (kg)
float sensorWeight[NUM_SENSORS];

// Crossing state
bool     crossingOccupied      = false;
float    peakWeightThisCrossing = 0.0;
float    currentTotalWeight     = 0.0;

// Timing
unsigned long lastSampleTime       = 0;
unsigned long lastIntegrationTime  = 0;
unsigned long lastOccupiedTime     = 0;

// Counters
unsigned long totalPedestrianCount = 0;
float         fractionalAccumulator = 0.0;  // accumulates fractional people

// Per-sensor flags
bool sensorIsVehicle[NUM_SENSORS];

// Integration-based counting for dense crowds
float integratedPersonSeconds = 0.0;

// ============================================================================
//  LCD (Optional) — Uncomment if using I2C LCD
// ============================================================================
// LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================================================
//  SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("================================================"));
  Serial.println(F("  ZEBRA CROSSING PEDESTRIAN COUNTER"));
  Serial.println(F("  Initializing sensors..."));
  Serial.println(F("================================================"));

  // Initialize all HX711 load cell amplifiers
  for (int i = 0; i < NUM_SENSORS; i++) {
    scales[i].begin(SENSOR_PINS[i][0], SENSOR_PINS[i][1]);
    scales[i].set_scale(CALIBRATION_FACTOR);
    scales[i].tare();  // Zero out with no load

    // Initialize filter buffers
    for (int j = 0; j < FILTER_DEPTH; j++) {
      sensorBuffer[i][j] = 0.0;
    }
    bufferIndex[i]  = 0;
    bufferFilled[i] = false;
    sensorWeight[i] = 0.0;
    sensorIsVehicle[i] = false;

    Serial.print(F("  Sensor "));
    Serial.print(i);
    Serial.println(F(" initialized and tared."));
  }

  // Optional LCD
  // lcd.init();
  // lcd.backlight();
  // lcd.setCursor(0, 0);
  // lcd.print("Pedestrian Cntr");
  // lcd.setCursor(0, 1);
  // lcd.print("Count: 0");

  Serial.println(F("================================================"));
  Serial.println(F("  System READY. Monitoring crossing..."));
  Serial.print(F("  Vehicle threshold: "));
  Serial.print(VEHICLE_THRESHOLD_KG);
  Serial.println(F(" kg/pad"));
  Serial.print(F("  Avg person weight: "));
  Serial.print(AVG_PERSON_WEIGHT_KG);
  Serial.println(F(" kg"));
  Serial.print(F("  Sensors: "));
  Serial.println(NUM_SENSORS);
  Serial.println(F("================================================\n"));

  lastSampleTime      = millis();
  lastIntegrationTime = millis();
}

// ============================================================================
//  MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  // --- Sample sensors at defined interval ---
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    readAllSensors();
    applyVehicleFilter();
    computeTotalWeight();
    updateCrossingState(now);
  }

  // --- Integration tick for dense crowd counting ---
  if (crossingOccupied && (now - lastIntegrationTime >= INTEGRATION_INTERVAL_MS)) {
    lastIntegrationTime = now;
    integrateCrowdCount();
  }
}

// ============================================================================
//  SENSOR READING WITH MOVING AVERAGE FILTER
// ============================================================================

void readAllSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    float rawWeight = 0.0;

    // Read from HX711 (returns weight based on calibration factor)
    if (scales[i].is_ready()) {
      rawWeight = scales[i].get_units(1);  // Single reading (fast for dense sampling)

      // Clamp negative readings to zero (sensor noise when unloaded)
      if (rawWeight < 0) rawWeight = 0.0;
    }

    // Push into moving average buffer
    sensorBuffer[i][bufferIndex[i]] = rawWeight;
    bufferIndex[i] = (bufferIndex[i] + 1) % FILTER_DEPTH;
    if (bufferIndex[i] == 0) bufferFilled[i] = true;

    // Compute filtered value
    int count = bufferFilled[i] ? FILTER_DEPTH : bufferIndex[i];
    float sum = 0.0;
    for (int j = 0; j < count; j++) {
      sum += sensorBuffer[i][j];
    }
    sensorWeight[i] = (count > 0) ? (sum / count) : 0.0;

    // Apply noise floor
    if (sensorWeight[i] < NOISE_FLOOR_KG) {
      sensorWeight[i] = 0.0;
    }
  }
}

// ============================================================================
//  VEHICLE DETECTION & REJECTION
// ============================================================================
//
//  A car tyre concentrates ~375 kg on one pad (small contact area).
//  A truck tyre can exceed 1000 kg on one pad.
//  Even in a dense crowd, one 0.5m pad won't see > 250 kg from pedestrians.
//  
//  Additional heuristic: vehicles activate fewer adjacent sensors with very
//  high readings, while crowds activate many sensors with moderate readings.
//
// ============================================================================

void applyVehicleFilter() {
  int  highReadingCount = 0;
  int  activeCount      = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorIsVehicle[i] = false;

    if (sensorWeight[i] > NOISE_FLOOR_KG) {
      activeCount++;
    }

    // Primary check: single pad exceeds vehicle threshold
    if (sensorWeight[i] > VEHICLE_THRESHOLD_KG) {
      sensorIsVehicle[i] = true;
      highReadingCount++;
    }
  }

  // Secondary heuristic: if only 1-2 pads are active but showing very high
  // weight, it's almost certainly a vehicle (pedestrians spread across pads)
  if (activeCount <= 2 && highReadingCount > 0) {
    // Mark ALL active sensors as vehicle — a single car covers 1-2 pads
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorWeight[i] > NOISE_FLOOR_KG) {
        sensorIsVehicle[i] = true;
      }
    }
  }

  // Adjacency check: if a sensor is flagged as vehicle, check its neighbors
  // Vehicle weight often "bleeds" to adjacent pads via the road surface
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorIsVehicle[i]) {
      // Check left neighbor
      if (i > 0 && sensorWeight[i - 1] > VEHICLE_THRESHOLD_KG * 0.3) {
        sensorIsVehicle[i - 1] = true;
      }
      // Check right neighbor
      if (i < NUM_SENSORS - 1 && sensorWeight[i + 1] > VEHICLE_THRESHOLD_KG * 0.3) {
        sensorIsVehicle[i + 1] = true;
      }
    }
  }
}

// ============================================================================
//  COMPUTE TOTAL PEDESTRIAN WEIGHT
// ============================================================================

void computeTotalWeight() {
  currentTotalWeight = 0.0;
  int vehiclePadCount = 0;
  int pedestrianPadCount = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorIsVehicle[i]) {
      vehiclePadCount++;
      continue;  // Skip vehicle-flagged pads entirely
    }
    currentTotalWeight += sensorWeight[i];
    if (sensorWeight[i] > NOISE_FLOOR_KG) {
      pedestrianPadCount++;
    }
  }

  // Track peak weight during this crossing event
  if (currentTotalWeight > peakWeightThisCrossing) {
    peakWeightThisCrossing = currentTotalWeight;
  }

  // Debug output (every sample)
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint >= 500) {  // Print every 500ms to avoid flooding
    lastDebugPrint = millis();
    printSensorDebug(vehiclePadCount, pedestrianPadCount);
  }
}

// ============================================================================
//  CROSSING STATE MACHINE
// ============================================================================
//
//  States:
//    IDLE → crossing is empty
//    OCCUPIED → people (or vehicles, filtered out) are on the crossing
//
//  Transitions:
//    IDLE → OCCUPIED: totalWeight > MIN_DETECTION_KG
//    OCCUPIED → IDLE: totalWeight < MIN_DETECTION_KG for CLEAR_DEBOUNCE_MS
//      → On this transition, finalize the batch count
//
// ============================================================================

void updateCrossingState(unsigned long now) {

  if (!crossingOccupied) {
    // === IDLE STATE ===
    if (currentTotalWeight >= MIN_DETECTION_KG) {
      // Crossing just became occupied
      crossingOccupied        = true;
      peakWeightThisCrossing  = currentTotalWeight;
      integratedPersonSeconds = 0.0;
      lastIntegrationTime     = now;
      lastOccupiedTime        = now;

      Serial.println(F(">>> CROSSING OCCUPIED — Pedestrians detected"));
    }

  } else {
    // === OCCUPIED STATE ===
    if (currentTotalWeight >= MIN_DETECTION_KG) {
      lastOccupiedTime = now;  // Reset debounce timer
    }

    // Check if crossing has been clear long enough (debounce)
    if (now - lastOccupiedTime >= CLEAR_DEBOUNCE_MS) {
      // Crossing cleared — finalize this batch
      finalizeCrossingEvent();
      crossingOccupied       = false;
      peakWeightThisCrossing = 0.0;
    }
  }
}

// ============================================================================
//  DENSE CROWD INTEGRATION
// ============================================================================
//
//  For dense crowds that may occupy the crossing for extended periods:
//  - Every INTEGRATION_INTERVAL_MS, snapshot the current person estimate
//  - This handles the case where people continuously flow through
//  - The "person-seconds" metric captures throughput even if the total
//    weight stays roughly constant as new people replace those who left
//
//  Heuristic: if weight is roughly stable (±20%) for >5s, assume
//  people are flowing through (not standing still). In that case,
//  count the estimated people as having crossed every ~3 seconds
//  (average zebra crossing time).
//
// ============================================================================

// Crossing time estimate (seconds) — how long a pedestrian takes to cross
#define AVG_CROSSING_TIME_S   4.0

// Weight stability threshold for "flowing crowd" detection
#define WEIGHT_STABILITY_PCT  0.20  // ±20%

// Minimum time of stable weight to trigger flow counting (ms)
#define FLOW_DETECT_TIME_MS   5000

// Track weight history for flow detection
float    prevIntegrationWeight = 0.0;
unsigned long stableWeightStart = 0;
bool     flowDetected = false;

void integrateCrowdCount() {
  float instantPersons = currentTotalWeight / AVG_PERSON_WEIGHT_KG;

  if (instantPersons < 0.5) return;  // No meaningful reading

  // Check weight stability for flow detection
  float weightDelta = abs(currentTotalWeight - prevIntegrationWeight);
  float stabilityThreshold = prevIntegrationWeight * WEIGHT_STABILITY_PCT;

  if (prevIntegrationWeight > MIN_DETECTION_KG && weightDelta < stabilityThreshold) {
    // Weight is stable
    if (stableWeightStart == 0) {
      stableWeightStart = millis();
    }
    if (millis() - stableWeightStart >= FLOW_DETECT_TIME_MS) {
      flowDetected = true;
    }
  } else {
    stableWeightStart = 0;
    flowDetected = false;
  }

  prevIntegrationWeight = currentTotalWeight;

  // If flow detected (continuous stream of people), accumulate
  if (flowDetected) {
    // People are flowing through: each "batch" on the crossing is replaced
    // every AVG_CROSSING_TIME_S seconds
    float intervalSeconds = INTEGRATION_INTERVAL_MS / 1000.0;
    float peopleThisTick = instantPersons * (intervalSeconds / AVG_CROSSING_TIME_S);
    integratedPersonSeconds += peopleThisTick;

    Serial.print(F("  [FLOW] ~"));
    Serial.print(instantPersons, 1);
    Serial.print(F(" people on crossing, throughput this tick: "));
    Serial.println(peopleThisTick, 2);
  }
}

// ============================================================================
//  FINALIZE CROSSING EVENT
// ============================================================================

void finalizeCrossingEvent() {
  float estimatedPeople;

  if (integratedPersonSeconds > 0 && flowDetected) {
    // Dense crowd / continuous flow mode
    // Use the integrated count (already accumulated via flow detection)
    estimatedPeople = integratedPersonSeconds;
    Serial.println(F("  [MODE: Dense crowd / continuous flow]"));
  } else {
    // Single batch mode: use peak weight
    estimatedPeople = peakWeightThisCrossing / AVG_PERSON_WEIGHT_KG;
    Serial.println(F("  [MODE: Single batch crossing]"));
  }

  // Accumulate fractional people (e.g., 2.7 people → 2 now, 0.7 carried over)
  fractionalAccumulator += estimatedPeople;
  unsigned long wholePeople = (unsigned long)fractionalAccumulator;
  fractionalAccumulator -= wholePeople;

  // Ensure at least 1 person if we detected meaningful weight
  if (wholePeople == 0 && estimatedPeople >= 0.3) {
    wholePeople = 1;
    fractionalAccumulator = 0.0;
  }

  totalPedestrianCount += wholePeople;

  // Report
  Serial.println(F(""));
  Serial.println(F("─────────────────────────────────────────────"));
  Serial.print(F("  CROSSING EVENT COMPLETE"));
  Serial.println(F(""));
  Serial.print(F("  Peak weight on crossing: "));
  Serial.print(peakWeightThisCrossing, 1);
  Serial.println(F(" kg"));
  Serial.print(F("  Estimated people this event: "));
  Serial.println(wholePeople);
  Serial.print(F("  ★ TOTAL PEDESTRIANS COUNTED: "));
  Serial.println(totalPedestrianCount);
  Serial.println(F("─────────────────────────────────────────────\n"));

  // Update LCD
  // lcd.setCursor(0, 1);
  // lcd.print("Count: ");
  // lcd.print(totalPedestrianCount);
  // lcd.print("    ");  // Clear trailing chars

  // Reset flow detection state
  prevIntegrationWeight = 0.0;
  stableWeightStart     = 0;
  flowDetected          = false;
}

// ============================================================================
//  DEBUG / SERIAL OUTPUT
// ============================================================================

void printSensorDebug(int vehiclePads, int pedestrianPads) {
  Serial.print(F("["));
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorIsVehicle[i]) {
      Serial.print(F(" X"));           // Vehicle-flagged
      Serial.print(sensorWeight[i], 0);
    } else if (sensorWeight[i] > NOISE_FLOOR_KG) {
      Serial.print(F(" P"));           // Pedestrian weight
      Serial.print(sensorWeight[i], 0);
    } else {
      Serial.print(F("  __"));         // Empty pad
    }
    if (i < NUM_SENSORS - 1) Serial.print(F(" |"));
  }
  Serial.print(F(" ] Total: "));
  Serial.print(currentTotalWeight, 1);
  Serial.print(F("kg | ~"));
  Serial.print(currentTotalWeight / AVG_PERSON_WEIGHT_KG, 1);
  Serial.print(F(" ppl | Veh:"));
  Serial.print(vehiclePads);
  Serial.print(F(" | RunTotal: "));
  Serial.println(totalPedestrianCount);
}

// ============================================================================
//  SERIAL COMMANDS (for live tuning / querying)
// ============================================================================
//
//  Send these via Serial Monitor:
//    'r' — Reset total count to zero
//    't' — Tare (re-zero) all sensors
//    's' — Print current status summary
//    'c' — Print calibration info
//
// ============================================================================

void serialEvent() {
  while (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'r':
      case 'R':
        totalPedestrianCount = 0;
        fractionalAccumulator = 0.0;
        Serial.println(F("\n*** COUNT RESET TO ZERO ***\n"));
        break;

      case 't':
      case 'T':
        Serial.println(F("\n*** TARING ALL SENSORS — Remove all weight! ***"));
        for (int i = 0; i < NUM_SENSORS; i++) {
          scales[i].tare();
          for (int j = 0; j < FILTER_DEPTH; j++) {
            sensorBuffer[i][j] = 0.0;
          }
          bufferIndex[i]  = 0;
          bufferFilled[i] = false;
        }
        Serial.println(F("*** TARE COMPLETE ***\n"));
        break;

      case 's':
      case 'S':
        Serial.println(F("\n=== STATUS SUMMARY ==="));
        Serial.print(F("  Total pedestrians counted: "));
        Serial.println(totalPedestrianCount);
        Serial.print(F("  Crossing occupied: "));
        Serial.println(crossingOccupied ? "YES" : "NO");
        Serial.print(F("  Current total weight: "));
        Serial.print(currentTotalWeight, 1);
        Serial.println(F(" kg"));
        Serial.print(F("  Flow mode active: "));
        Serial.println(flowDetected ? "YES" : "NO");
        Serial.println(F("======================\n"));
        break;

      case 'c':
      case 'C':
        Serial.println(F("\n=== CALIBRATION INFO ==="));
        Serial.print(F("  Calibration factor: "));
        Serial.println(CALIBRATION_FACTOR);
        Serial.print(F("  Vehicle threshold: "));
        Serial.print(VEHICLE_THRESHOLD_KG);
        Serial.println(F(" kg/pad"));
        Serial.print(F("  Avg person weight: "));
        Serial.print(AVG_PERSON_WEIGHT_KG);
        Serial.println(F(" kg"));
        Serial.print(F("  Noise floor: "));
        Serial.print(NOISE_FLOOR_KG);
        Serial.println(F(" kg"));
        Serial.println(F("========================\n"));
        break;
    }
  }
}
