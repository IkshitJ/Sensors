// Emulated Arduino core: virtual clock, Serial (with 115200-baud TX blocking),
// and the HX711 library logic.
#include "Arduino.h"
#include "HX711.h"
#include "sim_hooks.h"

#include <algorithm>
#include <deque>
#include <random>

SimHooks* g_sim = nullptr;
int g_board = 0;
HardwareSerial Serial(0), Serial1(1);

// one receive buffer and one UART transmit queue per board and port
static std::deque<char> g_rx[SIM_MAX_BOARDS][2];
static double g_txQueueEndUs[SIM_MAX_BOARDS][2] = {{0, 0}, {0, 0}};  // when the UART finishes sending queued bytes
static const double kUsPerByte = 1e6 * 10.0 / 115200.0;  // 8N1 at 115200 baud
static const double kTxBufferBytes = 64;     // AVR HardwareSerial TX ring buffer
static std::mt19937 g_arduinoRng(1);

void sim_serial_push(const std::string& s) { sim_serial_push(0, 0, s); }
void sim_serial_push(int board, int port, const std::string& s) {
  if (board < 0 || board >= SIM_MAX_BOARDS || port < 0 || port > 1) return;
  for (char c : s) g_rx[board][port].push_back(c);
}
int sim_serial_rx_available() { return (int)g_rx[g_board][0].size(); }

__attribute__((weak)) void serialEvent() {}

unsigned long millis() { return (unsigned long)(g_sim->nowUs() / 1000ULL); }
unsigned long micros() { return (unsigned long)(g_sim->nowUs()); }
void delay(unsigned long ms) { g_sim->advanceUs((uint64_t)ms * 1000ULL); }
void delayMicroseconds(unsigned int us) { g_sim->advanceUs(us); }
void yield() {}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return 0; }
long random(long howbig) { return howbig <= 0 ? 0 : (long)(g_arduinoRng() % (unsigned long)howbig); }
long random(long lo, long hi) { return hi <= lo ? lo : lo + random(hi - lo); }
void randomSeed(unsigned long s) { g_arduinoRng.seed((unsigned)s); }

// ---------------------------------------------------------------- Serial ----
int HardwareSerial::available() { return (int)g_rx[g_board][port_].size(); }
int HardwareSerial::read() {
  std::deque<char>& q = g_rx[g_board][port_];
  if (q.empty()) return -1;
  char c = q.front();
  q.pop_front();
  return (unsigned char)c;
}
int HardwareSerial::peek() {
  const std::deque<char>& q = g_rx[g_board][port_];
  return q.empty() ? -1 : (unsigned char)q.front();
}

size_t HardwareSerial::write(uint8_t c) {
  // Model the real UART: once the 64-byte buffer is full, write() blocks.
  double& qEnd = g_txQueueEndUs[g_board][port_];
  double now = (double)g_sim->nowUs();
  if (qEnd < now) qEnd = now;
  qEnd += kUsPerByte;
  double backlog = qEnd - now;
  double maxBacklog = kTxBufferBytes * kUsPerByte;
  if (backlog > maxBacklog) g_sim->advanceUs((uint64_t)(backlog - maxBacklog));
  char ch = (char)c;
  g_sim->serialTx(port_, &ch, 1);
  return 1;
}
size_t HardwareSerial::write(const char* s) {
  size_t n = 0;
  while (*s) n += write((uint8_t)*s++);
  return n;
}
size_t HardwareSerial::printNumber(unsigned long n, int base) {
  char buf[8 * sizeof(long) + 1];
  char* p = &buf[sizeof(buf) - 1];
  *p = '\0';
  if (base < 2) base = 10;
  do {
    unsigned long m = n;
    n /= (unsigned long)base;
    char c = (char)(m - (unsigned long)base * n);
    *--p = c < 10 ? (char)(c + '0') : (char)(c + 'A' - 10);
  } while (n);
  return write(p);
}
size_t HardwareSerial::printSigned(long n, int base) {
  if (base == 10 && n < 0) return write('-') + printNumber((unsigned long)(-n), 10);
  return printNumber((unsigned long)n, base);
}
size_t HardwareSerial::print(double n, int digits) {
  if (std::isnan(n)) return write("nan");
  if (std::isinf(n)) return write("inf");
  char buf[64];
  snprintf(buf, sizeof(buf), "%.*f", digits < 0 ? 0 : digits, n);
  return write(buf);
}

// ----------------------------------------------------------------- HX711 ----
// Library logic mirrors bogde/HX711. Bit-banging 25-27 clock pulses on an AVR
// takes roughly 60 us per read; that CPU time is charged to the virtual clock.
static const uint64_t kReadCostUs = 60;

void HX711::begin(uint8_t dout, uint8_t pd_sck, uint8_t gain) {
  dout_ = dout;
  sck_ = pd_sck;
  set_gain(gain);
}
bool HX711::is_ready() { return g_sim->hxReady(dout_); }
void HX711::set_gain(uint8_t gain) {
  switch (gain) {
    case 128: gain_ = 1; break;
    case 64: gain_ = 3; break;
    case 32: gain_ = 2; break;
  }
}
void HX711::wait_ready(unsigned long delay_ms) {
  while (!is_ready()) {
    uint64_t next = g_sim->hxNextReadyUs(dout_);
    uint64_t now = g_sim->nowUs();
    uint64_t step = next > now ? next - now : 1;
    if (delay_ms) step = std::max<uint64_t>(step, delay_ms * 1000ULL);
    g_sim->advanceUs(step);
  }
}
bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
  for (int i = 0; i < retries; i++) {
    if (is_ready()) return true;
    delay(delay_ms ? delay_ms : 1);
  }
  return is_ready();
}
bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (is_ready()) return true;
    delay(delay_ms ? delay_ms : 1);
  }
  return false;
}
long HX711::read() {
  wait_ready();
  long v = g_sim->hxRead(dout_);
  g_sim->advanceUs(kReadCostUs);
  return v;
}
long HX711::read_average(uint8_t times) {
  if (times == 0) times = 1;
  long sum = 0;
  for (uint8_t i = 0; i < times; i++) {
    sum += read();
    delay(0);
  }
  return sum / times;
}
double HX711::get_value(uint8_t times) { return (double)(read_average(times) - offset_); }
float HX711::get_units(uint8_t times) { return (float)(get_value(times) / scale_); }
void HX711::tare(uint8_t times) { set_offset(read_average(times)); }
void HX711::set_scale(float scale) { scale_ = scale; }
float HX711::get_scale() { return scale_; }
void HX711::set_offset(long offset) { offset_ = offset; }
long HX711::get_offset() { return offset_; }
void HX711::power_down() {}
void HX711::power_up() {}
