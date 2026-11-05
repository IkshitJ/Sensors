/*
 * ============================================================================
 *  HX711 LOAD CELL CALIBRATION SKETCH
 * ============================================================================
 *  
 *  Run this BEFORE deploying the main pedestrian counter!
 *  
 *  Steps:
 *  1. Upload this sketch to your Arduino
 *  2. Open Serial Monitor at 115200 baud
 *  3. Place a known weight (e.g., 10 kg) on the sensor
 *  4. Adjust the calibration_factor until the reading matches the known weight
 *  5. Send '+' or '-' via Serial to adjust (or '=' / '_' for fine adjustment)
 *  6. Note down the final calibration_factor
 *  7. Update CALIBRATION_FACTOR in the main sketch
 *
 *  Repeat for each sensor if they have different characteristics.
 *  
 * ============================================================================
 */

#include <HX711.h>

// Pins for the sensor being calibrated
#define DOUT_PIN  22
#define SCK_PIN   23

HX711 scale;

float calibration_factor = -7050.0;  // Starting value — adjust from here

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== HX711 CALIBRATION ==="));
  Serial.println(F("Place a known weight on the sensor."));
  Serial.println(F("Commands:"));
  Serial.println(F("  '+' / '-'  : adjust by ±100"));
  Serial.println(F("  '=' / '_'  : adjust by ±10"));
  Serial.println(F("  't'        : tare (re-zero)"));
  Serial.println(F("=========================\n"));

  scale.begin(DOUT_PIN, SCK_PIN);
  scale.set_scale();
  scale.tare();

  Serial.println(F("Tared. Place known weight now.\n"));
}

void loop() {
  scale.set_scale(calibration_factor);

  Serial.print(F("Reading: "));
  Serial.print(scale.get_units(10), 2);   // Average of 10 readings
  Serial.print(F(" kg  |  Calibration Factor: "));
  Serial.println(calibration_factor);

  // Handle serial commands
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '+': calibration_factor += 100; break;
      case '-': calibration_factor -= 100; break;
      case '=': calibration_factor += 10;  break;
      case '_': calibration_factor -= 10;  break;
      case 't':
      case 'T':
        scale.tare();
        Serial.println(F("*** Tared ***"));
        break;
    }
  }

  delay(250);
}
