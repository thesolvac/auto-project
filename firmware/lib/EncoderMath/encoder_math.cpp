#include "encoder_math.h"

int EncoderMath::wrappedDelta(uint16_t prev, uint16_t curr) {
  int delta = static_cast<int>(curr) - static_cast<int>(prev);
  if (delta > kCounts / 2) {
    delta -= kCounts;  // wrapped downward through 0
  } else if (delta < -kCounts / 2) {
    delta += kCounts;  // wrapped upward through 4095
  }
  return delta;
}
