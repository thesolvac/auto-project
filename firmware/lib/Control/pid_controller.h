#pragma once

// Standard discrete PID controller. Pure logic (host-tested).

class PidController {
 public:
  PidController(float kp, float ki, float kd);

  void setGains(float kp, float ki, float kd);
  void setIntegralLimit(float limit);  // |integral| clamp; <= 0 disables clamping
  void reset();

  // One update step. Returns the control output u = kp*e + ki*∫e + kd*de/dt.
  float update(float setpoint, float measurement, float dt);

  float lastError() const { return last_error_; }

 private:
  float kp_;
  float ki_;
  float kd_;
  float integral_ = 0.0f;
  float integral_limit_ = 0.0f;  // 0 => unbounded
  float prev_error_ = 0.0f;
  float last_error_ = 0.0f;
  bool has_prev_ = false;
};
