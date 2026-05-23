// Host-side Unity tests for the PID controller, slip detector, and the combined
// closed-loop controller (native env).

#include <unity.h>

#include "closed_loop_controller.h"
#include "pid_controller.h"
#include "slip_detector.h"

void setUp(void) {}
void tearDown(void) {}

static void test_proportional_step(void) {
  PidController pid(2.0f, 0.0f, 0.0f);
  // error = 1 - 0 = 1, output = kp*e = 2.0
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, pid.update(1.0f, 0.0f, 0.1f));
}

static void test_integral_accumulates(void) {
  PidController pid(0.0f, 1.0f, 0.0f);
  pid.update(1.0f, 0.0f, 0.5f);  // integral = 0.5
  // integral = 1.0, output = ki*integral = 1.0
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, pid.update(1.0f, 0.0f, 0.5f));
}

static void test_derivative_on_error_change(void) {
  PidController pid(0.0f, 0.0f, 1.0f);
  pid.update(0.0f, 0.0f, 0.1f);  // first call: no derivative
  // error jumps to 1; de/dt = (1-0)/0.1 = 10, output = kd*10 = 10
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, pid.update(1.0f, 0.0f, 0.1f));
}

static void test_slip_needs_persistence(void) {
  SlipDetector slip(0.1f, 3);
  TEST_ASSERT_FALSE(slip.update(0.2f));
  TEST_ASSERT_FALSE(slip.update(0.2f));
  TEST_ASSERT_TRUE(slip.update(0.2f));  // 3rd consecutive over-threshold latches
}

static void test_slip_transient_does_not_latch(void) {
  SlipDetector slip(0.1f, 3);
  slip.update(0.2f);
  slip.update(0.2f);
  slip.update(0.0f);  // disagreement cleared -> counter resets
  TEST_ASSERT_FALSE(slip.update(0.2f));
  TEST_ASSERT_FALSE(slip.slipping());
}

static void test_slip_reset(void) {
  SlipDetector slip(0.1f, 1);
  TEST_ASSERT_TRUE(slip.update(0.5f));
  slip.reset();
  TEST_ASSERT_FALSE(slip.slipping());
}

static void test_closed_loop_reports_slip(void) {
  ClosedLoopController ctrl(1.0f, 0.0f, 0.0f, 0.1f, 2);
  // setpoint 1.0, measurement 0.0 -> persistent 1.0 error >> 0.1 threshold
  ctrl.update(1.0f, 0.0f, 0.01f);
  ctrl.update(1.0f, 0.0f, 0.01f);
  TEST_ASSERT_TRUE(ctrl.slipping());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_proportional_step);
  RUN_TEST(test_integral_accumulates);
  RUN_TEST(test_derivative_on_error_change);
  RUN_TEST(test_slip_needs_persistence);
  RUN_TEST(test_slip_transient_does_not_latch);
  RUN_TEST(test_slip_reset);
  RUN_TEST(test_closed_loop_reports_slip);
  return UNITY_END();
}
