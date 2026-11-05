// =============================================================================
//  Arduino core emulation for native (desktop) builds of real Arduino sketches.
//
//  The sketches in firmware/ and legacy/ are compiled UNMODIFIED against this
//  header. Time is virtual (driven by the simulator), Serial is routed to the
//  simulator, and HX711 chips are emulated in HX711.h.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <math.h>
#include <stdlib.h>

typedef uint8_t byte;
typedef bool boolean;

#define F(s) (s)
#define __FlashStringHelper char  // AVR keeps F() strings in flash; here they're plain strings
#define PROGMEM
#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#ifndef PI
#define PI 3.14159265358979323846
#endif

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void yield();
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
long random(long howbig);
long random(long howsmall, long howbig);
void randomSeed(unsigned long seed);

template <class T, class L, class H>
inline T constrain(T x, L lo, H hi) { return x < (T)lo ? (T)lo : (x > (T)hi ? (T)hi : x); }

class HardwareSerial {
 public:
  void begin(unsigned long, int = 0) {}
  void end() {}
  int available();
  int read();
  int peek();
  void flush() {}
  operator bool() const { return true; }

  size_t write(uint8_t c);
  size_t write(const char* s);

  size_t print(const char* s) { return write(s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned char n, int base = DEC) { return printNumber((unsigned long)n, base); }
  size_t print(int n, int base = DEC) { return printSigned((long)n, base); }
  size_t print(unsigned int n, int base = DEC) { return printNumber((unsigned long)n, base); }
  size_t print(long n, int base = DEC) { return printSigned(n, base); }
  size_t print(unsigned long n, int base = DEC) { return printNumber(n, base); }
  size_t print(double n, int digits = 2);

  size_t println() { return write("\r\n"); }
  template <class T>
  size_t println(T v) { size_t n = print(v); return n + println(); }
  template <class T>
  size_t println(T v, int fmt) { size_t n = print(v, fmt); return n + println(); }

 private:
  size_t printSigned(long n, int base);
  size_t printNumber(unsigned long n, int base);
};

extern HardwareSerial Serial;

// Weak default; a sketch's own serialEvent() overrides it (as on a real Mega).
void serialEvent();
