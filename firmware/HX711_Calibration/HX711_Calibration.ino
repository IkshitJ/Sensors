/*
 * ============================================================================
 *  HX711 CALIBRATION — all 14 pads (v2, full-width strip)
 * ============================================================================
 *
 *  Run this BEFORE deploying the pedestrian counter. Each pad gets its own
 *  factor (load cells differ by a few percent).
 *
 *  1. Upload, open Serial Monitor at 115200 baud, line ending "Newline".
 *  2. With the pad EMPTY send:  t          (tare the selected pad)
 *  3. Put a known weight on it and send e.g.:  k20     (20 kg reference)
 *     -> the factor is computed automatically from 32 averaged samples.
 *     (fine-tune manually with + - (1 %) and = _ (0.1 %) if you like)
 *  4. Send n to move to the next pad, repeat.
 *  5. Send a to print all factors as a C array; paste it over
 *     CALIBRATION_FACTORS in ZebraCrossingPedestrianCounter.ino.
 *
 *  Use a reference of at least 20 kg: with 4 x 1000 kg cells one ADC count is
 *  ~1 g, but plate creep and noise are ~0.3 kg.
 * ============================================================================
 */

#include <HX711.h>

#define NUM_SENSORS 14
const uint8_t SENSOR_PINS[NUM_SENSORS][2] = {
  {22, 23}, {24, 25}, {26, 27}, {28, 29}, {30, 31}, {32, 33}, {34, 35},
  {36, 37}, {38, 39}, {40, 41}, {42, 43}, {44, 45}, {46, 47}, {48, 49},
};

HX711 scales[NUM_SENSORS];
float factors[NUM_SENSORS];
uint8_t cur = 0;
char lineBuf[24];
uint8_t lineLen = 0;
unsigned long lastPrint = 0;

void printHelp() {
  Serial.println(F("Commands:  n/p next/prev pad   t tare pad   k<kg> auto-calibrate (e.g. k20)"));
  Serial.println(F("           + - adjust 1%   = _ adjust 0.1%   a print all factors   h help"));
}

void printAll() {
  Serial.println(F("\nfloat CALIBRATION_FACTORS[NUM_SENSORS] = {"));
  Serial.print(F("  "));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(factors[i], 1);
    Serial.print(i < NUM_SENSORS - 1 ? F(", ") : F(","));
    if (i == 6) Serial.print(F("\n  "));
  }
  Serial.println(F("\n};\n"));
}

void selectPad(int8_t p) {
  if (p < 0) p = NUM_SENSORS - 1;
  if (p >= NUM_SENSORS) p = 0;
  cur = (uint8_t)p;
  Serial.print(F("\n>>> Pad ")); Serial.print(cur); Serial.println(F(" selected"));
}

void runCommand(const char* cmd) {
  switch (cmd[0]) {
    case 'n': case 'N': selectPad(cur + 1); break;
    case 'p': case 'P': selectPad((int8_t)cur - 1); break;
    case 't': case 'T':
      scales[cur].tare(32);
      Serial.print(F("*** Pad ")); Serial.print(cur); Serial.println(F(" tared ***"));
      break;
    case 'k': case 'K': {
      float kg = atof(cmd + 1);
      if (kg <= 0) { Serial.println(F("Usage: k<known kg>, e.g. k20")); break; }
      long avg = scales[cur].read_average(32);
      float f = (float)(avg - scales[cur].get_offset()) / kg;
      if (fabs(f) < 1) { Serial.println(F("No load detected - is the weight on the selected pad?")); break; }
      factors[cur] = f;
      scales[cur].set_scale(f);
      Serial.print(F("*** Pad ")); Serial.print(cur); Serial.print(F(" factor = ")); Serial.print(f, 1);
      Serial.println(F(" counts/kg ***"));
      break;
    }
    case '+': factors[cur] *= 1.01f; scales[cur].set_scale(factors[cur]); break;
    case '-': factors[cur] *= 0.99f; scales[cur].set_scale(factors[cur]); break;
    case '=': factors[cur] *= 1.001f; scales[cur].set_scale(factors[cur]); break;
    case '_': factors[cur] *= 0.999f; scales[cur].set_scale(factors[cur]); break;
    case 'a': case 'A': printAll(); break;
    case 'h': case 'H': case '?': printHelp(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== HX711 CALIBRATION (14 pads) ==="));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    scales[i].begin(SENSOR_PINS[i][0], SENSOR_PINS[i][1]);
    factors[i] = 1073.7f;  // nominal for 4 x 1000 kg, 2 mV/V, 5 V, gain 128
    scales[i].set_scale(factors[i]);
    scales[i].tare(20);
  }
  Serial.println(F("All pads tared with the strip empty."));
  printHelp();
  selectPad(0);
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    bool single = (lineLen == 0 && c != 'k' && c != 'K' && c != '\n');
    if (single) {  // one-key commands act immediately
      char cmd[2] = {c, 0};
      runCommand(cmd);
      continue;
    }
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      if (lineLen) runCommand(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    long avg = scales[cur].read_average(8);
    Serial.print(F("Pad ")); Serial.print(cur);
    Serial.print(F(" | raw ")); Serial.print(avg);
    Serial.print(F(" | ")); Serial.print((float)(avg - scales[cur].get_offset()) / factors[cur], 2);
    Serial.print(F(" kg | factor ")); Serial.println(factors[cur], 1);
  }
}
