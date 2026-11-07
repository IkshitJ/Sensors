// Compiles the calibration sketch unmodified: one board, wired to the plates
// of the entry gate (calibrate the exit gate's plates by moving the board).
#include "../arduino/Arduino.h"
#include "sketch_boards.h"
#include "../../firmware/HX711_Calibration/HX711_Calibration.ino"
const char* SIM_FIRMWARE_ID = "cal";
const SimBoard SIM_BOARDS[] = {{"CAL", setup, loop}};
const int SIM_NUM_BOARDS = 1;
