#include "slip_detector.h"

#include <cmath>

SlipDetector::SlipDetector(float error_threshold, int persistence_ticks)
    : threshold_(std::fabs(error_threshold)), persistence_(persistence_ticks) {}

bool SlipDetector::update(float error) {
  if (std::fabs(error) > threshold_) {
    ++over_count_;
    if (over_count_ >= persistence_) {
      latched_ = true;
    }
  } else {
    over_count_ = 0;  // disagreement cleared before becoming persistent
  }
  return latched_;
}

void SlipDetector::reset() {
  over_count_ = 0;
  latched_ = false;
}
