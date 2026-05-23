#include "closed_loop_controller.h"

ClosedLoopController::ClosedLoopController(float kp, float ki, float kd, float slip_threshold,
                                           int slip_persistence_ticks)
    : pid_(kp, ki, kd), slip_(slip_threshold, slip_persistence_ticks) {}

float ClosedLoopController::update(float setpoint, float measurement, float dt) {
  const float correction = pid_.update(setpoint, measurement, dt);
  // The PID error (setpoint - measurement) doubles as the slip signal: a large,
  // persistent velocity disagreement is exactly what slip looks like.
  slip_.update(pid_.lastError());
  return setpoint + correction;
}

void ClosedLoopController::reset() {
  pid_.reset();
  slip_.reset();
}
