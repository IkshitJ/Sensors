// =============================================================================
//  HX711 library emulation — same public API as the widely used
//  "HX711 Arduino Library" by Bogdan Necula (bogde/HX711).
//
//  The chip itself (24-bit sigma-delta ADC, 10/80 SPS data-ready timing,
//  saturation, noise, bridge offset) is modelled by the simulator; this class
//  only implements the library logic on top of it, exactly like the real one.
// =============================================================================
#pragma once
#include "Arduino.h"

class HX711 {
 public:
  void begin(uint8_t dout, uint8_t pd_sck, uint8_t gain = 128);
  bool is_ready();
  void set_gain(uint8_t gain = 128);
  long read();
  void wait_ready(unsigned long delay_ms = 0);
  bool wait_ready_retry(int retries = 3, unsigned long delay_ms = 0);
  bool wait_ready_timeout(unsigned long timeout = 1000, unsigned long delay_ms = 0);
  long read_average(uint8_t times = 10);
  double get_value(uint8_t times = 1);
  float get_units(uint8_t times = 1);
  void tare(uint8_t times = 10);
  void set_scale(float scale = 1.f);
  float get_scale();
  void set_offset(long offset = 0);
  long get_offset();
  void power_down();
  void power_up();

 private:
  int dout_ = -1;
  int sck_ = -1;
  uint8_t gain_ = 1;
  long offset_ = 0;
  float scale_ = 1.f;
};
