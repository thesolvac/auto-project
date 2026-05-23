#include "motor_driver.h"

#include <cmath>

namespace {
constexpr float kRmtTickNs = 1000.0f;       // 1 us per RMT tick
constexpr uint32_t kStepPulseTicks = 5;     // STEP high time: 5 us (TMC2209 min ~100 ns)
constexpr uint32_t kMaxDurationTicks = 32767;  // RMT duration field is 15-bit
constexpr float kMinStepRate = 1.0f;        // below this, treat as stopped [steps/s]
}  // namespace

MotorDriver::MotorDriver(uint8_t step_pin, uint8_t dir_pin, float max_accel_steps_s2)
    : step_pin_(step_pin), dir_pin_(dir_pin), ramp_(max_accel_steps_s2) {}

void MotorDriver::begin() {
  pinMode(dir_pin_, OUTPUT);
  digitalWrite(dir_pin_, LOW);
  rmt_ = rmtInit(step_pin_, true, RMT_MEM_64);  // tx mode, 64-word memory block
  if (rmt_ != nullptr) {
    rmtSetTick(rmt_, kRmtTickNs);
  }
}

void MotorDriver::setTargetSpeed(float steps_per_sec) { ramp_.setTarget(steps_per_sec); }

void MotorDriver::stop() {
  ramp_.setTarget(0.0f);
  applyFrequency(ramp_.update(0.0f));
}

void MotorDriver::update(float dt) { applyFrequency(ramp_.update(dt)); }

void MotorDriver::applyFrequency(float steps_per_sec) {
  if (rmt_ == nullptr) {
    return;
  }
  if (std::fabs(steps_per_sec) < kMinStepRate) {
    if (loop_active_) {
      rmtWrite(rmt_, nullptr, 0);  // stop the looping pulse train
      loop_active_ = false;
    }
    return;
  }

  digitalWrite(dir_pin_, steps_per_sec >= 0.0f ? HIGH : LOW);

  const float period_ticks = 1e6f / std::fabs(steps_per_sec);  // us == ticks (1 us tick)
  uint32_t low_ticks = (period_ticks > kStepPulseTicks)
                           ? static_cast<uint32_t>(period_ticks) - kStepPulseTicks
                           : 1;
  if (low_ticks > kMaxDurationTicks) {
    low_ticks = kMaxDurationTicks;
  }

  rmt_data_t pulse;
  pulse.duration0 = kStepPulseTicks;  // STEP high
  pulse.level0 = 1;
  pulse.duration1 = low_ticks;        // STEP low
  pulse.level1 = 0;
  rmtLoop(rmt_, &pulse, 1);  // hardware repeats the single-step pattern
  loop_active_ = true;
}
