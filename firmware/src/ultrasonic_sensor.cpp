#include "ultrasonic_sensor.h"

#include <Arduino.h>

namespace {
// ISR trampoline: attachInterruptArg passes the sensor instance as `arg`.
void IRAM_ATTR echoIsr(void* arg) { static_cast<UltrasonicSensor*>(arg)->onEchoEdge(); }
}  // namespace

UltrasonicSensor::UltrasonicSensor(uint8_t trig_pin, uint8_t echo_pin)
    : trig_pin_(trig_pin), echo_pin_(echo_pin) {}

void UltrasonicSensor::begin() {
  pinMode(trig_pin_, OUTPUT);
  pinMode(echo_pin_, INPUT);
  digitalWrite(trig_pin_, LOW);
  attachInterruptArg(digitalPinToInterrupt(echo_pin_), echoIsr, this, CHANGE);
}

void UltrasonicSensor::trigger() {
  // 10 us trigger pulse (datasheet). Kept short enough not to need a busy wait.
  measuring_ = true;
  echo_ready_ = false;
  trigger_time_us_ = micros();
  digitalWrite(trig_pin_, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin_, LOW);
}

void IRAM_ATTR UltrasonicSensor::onEchoEdge() {
  if (digitalRead(echo_pin_) == HIGH) {
    echo_start_us_ = micros();
  } else {
    echo_end_us_ = micros();
    echo_ready_ = true;
    measuring_ = false;
  }
}

float UltrasonicSensor::lastDistanceM() {
  if (echo_ready_) {
    const uint32_t width_us = echo_end_us_ - echo_start_us_;
    // distance = (pulse_width * speed_of_sound) / 2, us -> s
    const float distance = (static_cast<float>(width_us) * 1e-6f * kSpeedOfSoundMps) * 0.5f;
    echo_ready_ = false;
    return filter_.push(distance <= kMaxDistanceM ? distance : kMaxDistanceM);
  }
  if (measuring_ && (micros() - trigger_time_us_) > kTimeoutUs) {
    measuring_ = false;
    return filter_.push(kMaxDistanceM);  // no echo: out of range or no sensor
  }
  return filter_.average();
}
