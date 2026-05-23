// Host-side Unity tests for encoder wrap-around math and the EMA filter.

#include <unity.h>

#include "encoder_math.h"

void setUp(void) {}
void tearDown(void) {}

static void test_simple_delta(void) {
  TEST_ASSERT_EQUAL_INT(10, EncoderMath::wrappedDelta(100, 110));
  TEST_ASSERT_EQUAL_INT(-10, EncoderMath::wrappedDelta(110, 100));
}

static void test_wrap_forward_through_zero(void) {
  // 4090 -> 5 is +11 forward, not -4085.
  TEST_ASSERT_EQUAL_INT(11, EncoderMath::wrappedDelta(4090, 5));
}

static void test_wrap_backward_through_zero(void) {
  // 5 -> 4090 is -11 backward, not +4085.
  TEST_ASSERT_EQUAL_INT(-11, EncoderMath::wrappedDelta(5, 4090));
}

static void test_half_range_boundary(void) {
  TEST_ASSERT_EQUAL_INT(2048, EncoderMath::wrappedDelta(0, 2048));
  TEST_ASSERT_EQUAL_INT(-2047, EncoderMath::wrappedDelta(0, 2049));
}

static void test_ema_initializes_to_first_sample(void) {
  EmaFilter ema(0.2f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, ema.update(5.0f));  // no ramp-up from 0
}

static void test_ema_smooths_toward_input(void) {
  EmaFilter ema(0.2f);
  ema.update(0.0f);
  // value = 0.2*10 + 0.8*0 = 2.0
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, ema.update(10.0f));
  // value = 0.2*10 + 0.8*2 = 3.6
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 3.6f, ema.update(10.0f));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_simple_delta);
  RUN_TEST(test_wrap_forward_through_zero);
  RUN_TEST(test_wrap_backward_through_zero);
  RUN_TEST(test_half_range_boundary);
  RUN_TEST(test_ema_initializes_to_first_sample);
  RUN_TEST(test_ema_smooths_toward_input);
  return UNITY_END();
}
