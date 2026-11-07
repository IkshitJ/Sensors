// The boards a sketch wrapper (sketch_*.cpp) hands to the harness: one
// setup()/loop() pair per Arduino in the installation.
#pragma once

struct SimBoard {
  const char* name;
  void (*setup)();
  void (*loop)();
};

extern const SimBoard SIM_BOARDS[];
extern const int SIM_NUM_BOARDS;
extern const char* SIM_FIRMWARE_ID;
