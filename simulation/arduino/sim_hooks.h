// Interface between the emulated Arduino core and the simulator.
#pragma once
#include <cstdint>
#include <cstddef>
#include <string>

struct SimHooks {
  virtual ~SimHooks() {}
  virtual uint64_t nowUs() const = 0;
  virtual void advanceUs(uint64_t us) = 0;            // burn virtual CPU time
  virtual bool hxReady(int doutPin) = 0;              // DOUT low = data ready
  virtual uint64_t hxNextReadyUs(int doutPin) = 0;    // when next conversion completes
  virtual long hxRead(int doutPin) = 0;               // clock out 24 bits (consumes)
  virtual void serialTx(const char* data, size_t n) = 0;
};

extern SimHooks* g_sim;
void sim_serial_push(const std::string& s);  // bytes typed into the Serial Monitor
int sim_serial_rx_available();
