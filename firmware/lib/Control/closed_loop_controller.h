#pragma once

// Combines a PID regulator with a SlipDetector for one wheel. Pure logic
// (host-tested). Runs at 200 Hz on the ESP32. The PID output is a small
// correction to the commanded velocity; the slip flag is the safety-critical
// output (drives "ERR SLIP" up to the RPi).

#include "pid_controller.h"
#include "slip_detector.h"

class ClosedLoopController {
 public:
  ClosedLoopController(float kp, float ki, float kd, float slip_threshold,
                       int slip_persistence_ticks);

  // setpoint/measurement are wheel velocities (counts/s or steps/s — consistent
  // units). Returns the corrected velocity command; query slipping() afterward.
  float update(float setpoint, float measurement, float dt);

  bool slipping() const { return slip_.slipping(); }
  void reset();

 private:
  PidController pid_;
  SlipDetector slip_;
};
