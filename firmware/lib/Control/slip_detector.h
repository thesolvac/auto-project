#pragma once

// Slip detector. Pure logic (host-tested).
//
// Design note: the closed loop's PRIMARY job on this robot is not tight velocity
// regulation but *slip detection*. Steppers are commanded open-loop; the AS5600
// encoders tell us what actually happened. A large, *persistent* disagreement
// between commanded motion and measured motion means a wheel slipped (or
// stalled). Requiring persistence (rather than a single noisy sample) avoids
// false positives from transient encoder noise.

class SlipDetector {
 public:
  // `error_threshold`: magnitude above which a sample counts as "disagreeing".
  // `persistence_ticks`: consecutive disagreeing samples needed to latch slip.
  SlipDetector(float error_threshold, int persistence_ticks);

  // Feed one (commanded - measured) error sample. Returns true once slip is
  // latched (i.e. the error stayed over threshold for persistence_ticks).
  bool update(float error);

  void reset();
  bool slipping() const { return latched_; }

 private:
  float threshold_;
  int persistence_;
  int over_count_ = 0;
  bool latched_ = false;
};
