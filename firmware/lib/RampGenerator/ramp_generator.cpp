#include "ramp_generator.h"

#include <cmath>

RampGenerator::RampGenerator(float max_accel) : max_accel_(std::fabs(max_accel)) {}

void RampGenerator::setMaxAccel(float max_accel) { max_accel_ = std::fabs(max_accel); }

void RampGenerator::setTarget(float target_velocity) { target_ = target_velocity; }

void RampGenerator::reset(float velocity) {
  current_ = velocity;
  target_ = velocity;
}

float RampGenerator::update(float dt) {
  const float max_delta = max_accel_ * dt;
  const float diff = target_ - current_;
  if (diff > max_delta) {
    current_ += max_delta;
  } else if (diff < -max_delta) {
    current_ -= max_delta;
  } else {
    current_ = target_;  // within one step of the target: snap to it
  }
  return current_;
}
