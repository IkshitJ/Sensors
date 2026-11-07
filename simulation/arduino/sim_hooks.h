// Interface between the emulated Arduino core and the simulator.
//
// Several boards can run in one process (one sketch instance each, see
// sketch_v2.cpp): g_board says which one is executing right now. Every board
// has its USB Serial (port 0) and a Serial1 (port 1); the simulator wires
// Serial1 of one board to Serial1 of another, like the cable between them.
#pragma once
#include <cstdint>
#include <cstddef>
#include <string>

constexpr int SIM_MAX_BOARDS = 2;

struct SimHooks {
  virtual ~SimHooks() {}
  virtual uint64_t nowUs() const = 0;
  virtual void advanceUs(uint64_t us) = 0;            // burn virtual CPU time
  virtual bool hxReady(int doutPin) = 0;              // DOUT low = data ready (on the current board)
  virtual uint64_t hxNextReadyUs(int doutPin) = 0;    // when next conversion completes
  virtual long hxRead(int doutPin) = 0;               // clock out 24 bits (consumes)
  virtual void serialTx(int port, const char* data, size_t n) = 0;  // bytes leaving the current board
};

extern SimHooks* g_sim;
extern int g_board;  // the board whose sketch is running now
void sim_serial_push(const std::string& s);                      // bytes typed into the Serial Monitor of board 0
void sim_serial_push(int board, int port, const std::string& s);  // bytes arriving at a board's port
int sim_serial_rx_available();                                   // on the current board's USB serial
