#include "as5600_encoder.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t kRawAngleRegister = 0x0C;  // RAW_ANGLE high byte (low byte at 0x0D)
}

As5600Encoder::As5600Encoder(Tca9548a& mux, uint8_t channel, float ema_alpha)
    : mux_(mux), channel_(channel), velocity_filter_(ema_alpha) {}

bool As5600Encoder::readRaw(uint16_t& raw) {
  Wire.beginTransmission(address_);
  Wire.write(kRawAngleRegister);
  if (Wire.endTransmission(false) != 0) {  // repeated start, keep the bus
    return false;
  }
  if (Wire.requestFrom(address_, static_cast<uint8_t>(2)) != 2) {
    return false;
  }
  const uint8_t high = static_cast<uint8_t>(Wire.read());
  const uint8_t low = static_cast<uint8_t>(Wire.read());
  raw = static_cast<uint16_t>((static_cast<uint16_t>(high & 0x0F) << 8) | low);
  return true;
}

void As5600Encoder::update(float dt) {
  if (!mux_.selectChannel(channel_)) {
    return;
  }
  uint16_t raw = 0;
  if (!readRaw(raw)) {
    return;
  }
  if (has_last_) {
    const int delta = EncoderMath::wrappedDelta(last_raw_, raw);
    total_counts_ += delta;
    const float velocity = (dt > 0.0f) ? static_cast<float>(delta) / dt : 0.0f;
    velocity_filter_.update(velocity);
  } else {
    has_last_ = true;
  }
  last_raw_ = raw;
}
