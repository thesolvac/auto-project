// Bootstrap host-side unit test (Phase 0).
//
// Confirms the Unity test runner builds and executes on the `native` env.
// Real tests (UART round-trip, encoder wrap-around, PID step response,
// trapezoidal ramp) are added in Phase 1.

#include <unity.h>

void setUp(void) {}

void tearDown(void) {}

static void test_harness_runs(void) {
  TEST_ASSERT_EQUAL_INT(2, 1 + 1);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_harness_runs);
  return UNITY_END();
}
