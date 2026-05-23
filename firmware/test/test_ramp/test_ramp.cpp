// Host-side Unity tests for the trapezoidal velocity ramp (native env).

#include <unity.h>

#include "ramp_generator.h"

void setUp(void) {}
void tearDown(void) {}

static void test_ramps_up_at_max_accel(void) {
  RampGenerator ramp(2.0f);  // 2 units/s^2
  ramp.setTarget(10.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, ramp.update(1.0f));   // +2 after 1 s
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f, ramp.update(1.0f));
}

static void test_reaches_and_holds_target(void) {
  RampGenerator ramp(2.0f);
  ramp.setTarget(10.0f);
  for (int i = 0; i < 5; ++i) {
    ramp.update(1.0f);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f, ramp.current());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f, ramp.update(1.0f));  // clamps, no overshoot
}

static void test_ramps_down_symmetrically(void) {
  RampGenerator ramp(2.0f);
  ramp.reset(10.0f);
  ramp.setTarget(0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 8.0f, ramp.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 6.0f, ramp.update(1.0f));
}

static void test_snaps_when_within_one_step(void) {
  RampGenerator ramp(100.0f);  // large accel: one step covers the gap
  ramp.setTarget(5.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, ramp.update(1.0f));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_ramps_up_at_max_accel);
  RUN_TEST(test_reaches_and_holds_target);
  RUN_TEST(test_ramps_down_symmetrically);
  RUN_TEST(test_snaps_when_within_one_step);
  return UNITY_END();
}
