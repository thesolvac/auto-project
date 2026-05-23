#include "tca9548a.h"

#include <Arduino.h>
#include <Wire.h>

Tca9548a::Tca9548a(uint8_t address) : address_(address) {}

bool Tca9548a::selectChannel(uint8_t channel) {
  if (channel > 7) {
    return false;
  }
  Wire.beginTransmission(address_);
  Wire.write(static_cast<uint8_t>(1u << channel));  // one-hot channel select
  return Wire.endTransmission() == 0;
}
