#include "pid_controller.h"

#include <cmath>

PidController::PidController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd) {}

void PidController::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PidController::setIntegralLimit(float limit) { integral_limit_ = limit; }

void PidController::reset() {
  integral_ = 0.0f;
  prev_error_ = 0.0f;
  last_error_ = 0.0f;
  has_prev_ = false;
}

float PidController::update(float setpoint, float measurement, float dt) {
  const float error = setpoint - measurement;
  last_error_ = error;

  integral_ += error * dt;
  if (integral_limit_ > 0.0f) {
    if (integral_ > integral_limit_) {
      integral_ = integral_limit_;
    } else if (integral_ < -integral_limit_) {
      integral_ = -integral_limit_;
    }
  }

  float derivative = 0.0f;
  if (has_prev_ && dt > 0.0f) {
    derivative = (error - prev_error_) / dt;
  }
  prev_error_ = error;
  has_prev_ = true;

  return kp_ * error + ki_ * integral_ + kd_ * derivative;
}
