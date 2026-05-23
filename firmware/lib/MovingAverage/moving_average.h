#pragma once

// Fixed-window moving average over the last N samples. Header-only, pure logic,
// no dynamic allocation (datasheet-safe for ISR-adjacent use). Used by the
// ultrasonic sensor to smooth its 5-sample window.

#include <cstddef>

template <size_t N>
class MovingAverage {
 public:
  // Add a sample and return the current average over up to the last N samples.
  float push(float sample) {
    sum_ -= buffer_[index_];
    buffer_[index_] = sample;
    sum_ += sample;
    index_ = (index_ + 1) % N;
    if (count_ < N) {
      ++count_;
    }
    return average();
  }

  float average() const { return (count_ == 0) ? 0.0f : sum_ / static_cast<float>(count_); }

  void reset() {
    for (size_t i = 0; i < N; ++i) {
      buffer_[i] = 0.0f;
    }
    sum_ = 0.0f;
    index_ = 0;
    count_ = 0;
  }

 private:
  float buffer_[N] = {0.0f};
  float sum_ = 0.0f;
  size_t index_ = 0;
  size_t count_ = 0;
};
