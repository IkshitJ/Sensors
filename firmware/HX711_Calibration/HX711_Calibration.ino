/*
 * ============================================================================
 *  HX711 CALIBRATION — 14 plates x 2 bridges (v3, two-way gate counter)
 * ============================================================================
 *
 *  Each plate's 4 load cells are wired as two bridges, each with its own
 *  HX711: G (the 2 cells under the ghat-side edge) and M (the 2 cells under
 *  the marg-side edge). This sketch calibrates both bridges of a plate at once.
 *
 *  1. Upload, open Serial Monitor at 115200 baud, line ending "Newline".
 *  2. With the plate EMPTY send:  t          (tare both bridges of the plate)
 *  3. Put a known weight in the CENTRE of the plate and send e.g.:  k20
 *     -> each bridge carries half of it; both factors are computed from 32
 *        averaged samples. (Off-centre? The two factors come out unequal but
 *        G + M still reads the true weight; centre it for correct direction.)
 *  4. Send n to move to the next plate, repeat.
 *  5. Send a to print all factors as two C arrays; paste them over CAL_G and
 *     CAL_M in ZebraCrossingPedestrianCounter.ino.
 *
 *  Use a reference of at least 20 kg: plate creep and noise are ~0.3 kg.
 * ============================================================================
 */

#include <HX711.h>

#define NUM_PLATES 14
// must match ZebraCrossingPedestrianCounter.ino
const uint8_t PINS_G[NUM_PLATES][2] = {
  {22, 23}, {24, 25}, {26, 27}, {28, 29}, {30, 31}, {32, 33}, {34, 35},
  {36, 37}, {38, 39}, {40, 41}, {42, 43}, {44, 45}, {46, 47}, {48, 49},
};
const uint8_t PINS_M[NUM_PLATES][2] = {
  {A0, A1}, {A2, A3}, {A4, A5}, {A6, A7}, {A8, A9}, {A10, A11}, {A12, A13},
  {A14, A15}, {2, 3}, {4, 5}, {6, 7}, {8, 9}, {10, 11}, {12, 13},
};

HX711 scaleG[NUM_PLATES], scaleM[NUM_PLATES];
float facG[NUM_PLATES], facM[NUM_PLATES];
uint8_t cur = 0;
char lineBuf[24];
uint8_t lineLen = 0;
unsigned long lastPrint = 0;

void printHelp() {
  Serial.println(F("Commands:  n/p next/prev plate   t tare plate   k<kg> calibrate with weight in plate centre (e.g. k20)"));
  Serial.println(F("           + - adjust both 1%   = _ adjust both 0.1%   a print all factors   h help"));
}

void printArray(const __FlashStringHelper* name, const float* f) {
  Serial.print(F("float "));
  Serial.print(name);
  Serial.println(F("[NUM_PLATES] = {"));
  Serial.print(F("  "));
  for (uint8_t i = 0; i < NUM_PLATES; i++) {
    Serial.print(f[i], 1);
    Serial.print(i < NUM_PLATES - 1 ? F(", ") : F(","));
    if (i == 6) Serial.print(F("\n  "));
  }
  Serial.println(F("\n};"));
}

void printAll() {
  Serial.println();
  printArray(F("CAL_G"), facG);
  printArray(F("CAL_M"), facM);
  Serial.println();
}

void selectPlate(int8_t p) {
  if (p < 0) p = NUM_PLATES - 1;
  if (p >= NUM_PLATES) p = 0;
  cur = (uint8_t)p;
  Serial.print(F("\n>>> Plate ")); Serial.print(cur); Serial.println(F(" selected"));
}

void scaleBoth(float k) {
  facG[cur] *= k;
  facM[cur] *= k;
  scaleG[cur].set_scale(facG[cur]);
  scaleM[cur].set_scale(facM[cur]);
}

void runCommand(const char* cmd) {
  switch (cmd[0]) {
    case 'n': case 'N': selectPlate(cur + 1); break;
    case 'p': case 'P': selectPlate((int8_t)cur - 1); break;
    case 't': case 'T':
      scaleG[cur].tare(32);
      scaleM[cur].tare(32);
      Serial.print(F("*** Plate ")); Serial.print(cur); Serial.println(F(" tared (both bridges) ***"));
      break;
    case 'k': case 'K': {
      float kg = atof(cmd + 1);
      if (kg <= 0) { Serial.println(F("Usage: k<known kg>, e.g. k20 (weight in the plate centre)")); break; }
      long g = scaleG[cur].read_average(32) - scaleG[cur].get_offset();
      long m = scaleM[cur].read_average(32) - scaleM[cur].get_offset();
      float fg = (float)g / (kg / 2), fm = (float)m / (kg / 2);
      if (fabs(fg) < 1 || fabs(fm) < 1) { Serial.println(F("No load on one bridge - is the weight in the centre of the selected plate?")); break; }
      facG[cur] = fg;
      facM[cur] = fm;
      scaleG[cur].set_scale(fg);
      scaleM[cur].set_scale(fm);
      Serial.print(F("*** Plate ")); Serial.print(cur);
      Serial.print(F(": ghat-side ")); Serial.print(fg, 1);
      Serial.print(F(", marg-side ")); Serial.print(fm, 1);
      Serial.println(F(" counts/kg ***"));
      break;
    }
    case '+': scaleBoth(1.01f); break;
    case '-': scaleBoth(0.99f); break;
    case '=': scaleBoth(1.001f); break;
    case '_': scaleBoth(0.999f); break;
    case 'a': case 'A': printAll(); break;
    case 'h': case 'H': case '?': printHelp(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== HX711 CALIBRATION (14 plates x 2 bridges) ==="));
  for (uint8_t i = 0; i < NUM_PLATES; i++) {
    scaleG[i].begin(PINS_G[i][0], PINS_G[i][1]);
    scaleM[i].begin(PINS_M[i][0], PINS_M[i][1]);
    facG[i] = facM[i] = 2147.4f;  // nominal for 2 x 1000 kg, 2 mV/V, 5 V, gain 128
    scaleG[i].set_scale(facG[i]);
    scaleM[i].set_scale(facM[i]);
    scaleG[i].tare(20);
    scaleM[i].tare(20);
  }
  Serial.println(F("All plates tared with the strip empty."));
  printHelp();
  selectPlate(0);
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
    long g = scaleG[cur].read_average(8) - scaleG[cur].get_offset();
    long m = scaleM[cur].read_average(8) - scaleM[cur].get_offset();
    float kg = g / facG[cur] + m / facM[cur];
    Serial.print(F("Plate ")); Serial.print(cur);
    Serial.print(F(" | ghat-side ")); Serial.print(g / facG[cur], 2);
    Serial.print(F(" kg + marg-side ")); Serial.print(m / facM[cur], 2);
    Serial.print(F(" kg = ")); Serial.print(kg, 2);
    Serial.println(F(" kg"));
  }
}
