#pragma once

// HC-SR04 ultrasonic range finder, interrupt-driven on the Echo pin. trigger()
// fires a 10 us pulse on Trig (non-blocking); the Echo edge ISR timestamps the
// rising and falling edges; lastDistanceM() converts the pulse width to metres,
// smooths it over a 5-sample moving average, and falls back to MAX_DISTANCE if
// no echo returns within the 30 ms timeout (out-of-range or missing sensor).

#include <cstdint>

#include "moving_average.h"

class UltrasonicSensor {
 public:
  UltrasonicSensor(uint8_t trig_pin, uint8_t echo_pin);

  void begin();      // configure pins and attach the Echo interrupt
  void trigger();    // start a measurement (non-blocking)
  float lastDistanceM();  // filtered distance; MAX on timeout

  // Called from the Echo ISR; public so the static trampoline can reach it.
  void onEchoEdge();

  static constexpr float kMaxDistanceM = 4.0f;       // HC-SR04 practical max
  static constexpr uint32_t kTimeoutUs = 30000;      // 30 ms echo timeout
  static constexpr float kSpeedOfSoundMps = 343.0f;  // at ~20 C

 private:
  uint8_t trig_pin_;
  uint8_t echo_pin_;
  volatile uint32_t echo_start_us_ = 0;
  volatile uint32_t echo_end_us_ = 0;
  volatile bool echo_ready_ = false;
  volatile bool measuring_ = false;
  uint32_t trigger_time_us_ = 0;
  MovingAverage<5> filter_;
};
