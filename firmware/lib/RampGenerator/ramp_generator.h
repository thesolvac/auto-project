#pragma once

// Trapezoidal velocity ramp: limits the rate of change of a commanded velocity
// to a maximum acceleration. Pure logic (host-tested); the hardware MotorDriver
// owns one of these and feeds its output to the STEP pulse generator.

class RampGenerator {
 public:
  explicit RampGenerator(float max_accel);  // units/s^2, must be > 0

  void setMaxAccel(float max_accel);
  void setTarget(float target_velocity);  // signed target [units/s]
  void reset(float velocity = 0.0f);

  // Advance the current velocity toward the target by at most max_accel*dt.
  // Returns the new current velocity.
  float update(float dt);

  float current() const { return current_; }
  float target() const { return target_; }

 private:
  float max_accel_;
  float target_ = 0.0f;
  float current_ = 0.0f;
};
