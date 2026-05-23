// Host-side Unity tests for the UART protocol parser/formatters (native env).

#include <cstring>

#include <unity.h>

#include "uart_protocol.h"

void setUp(void) {}
void tearDown(void) {}

static void test_parse_move(void) {
  ParsedCommand cmd;
  TEST_ASSERT_TRUE(UartProtocol::parseCommand("MOVE 0.5 -0.3", cmd));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::kMove), static_cast<int>(cmd.type));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, cmd.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.3f, cmd.right);
}

static void test_parse_stop_exact(void) {
  ParsedCommand cmd;
  TEST_ASSERT_TRUE(UartProtocol::parseCommand("STOP", cmd));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::kStop), static_cast<int>(cmd.type));
}

static void test_parse_config(void) {
  ParsedCommand cmd;
  TEST_ASSERT_TRUE(UartProtocol::parseCommand("CFG kp 1.5", cmd));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::kConfig), static_cast<int>(cmd.type));
  TEST_ASSERT_EQUAL_STRING("kp", cmd.key);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.5f, cmd.value);
}

static void test_parse_leading_spaces(void) {
  ParsedCommand cmd;
  TEST_ASSERT_TRUE(UartProtocol::parseCommand("   MOVE 1 1", cmd));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::kMove), static_cast<int>(cmd.type));
}

static void test_reject_unknown_and_malformed(void) {
  ParsedCommand cmd;
  TEST_ASSERT_FALSE(UartProtocol::parseCommand("WALK 1 1", cmd));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandType::kUnknown), static_cast<int>(cmd.type));
  TEST_ASSERT_FALSE(UartProtocol::parseCommand("MOVE 1", cmd));  // missing arg
  TEST_ASSERT_FALSE(UartProtocol::parseCommand("STOPX", cmd));   // not a clean verb
}

static void test_format_telemetry_roundtrip(void) {
  char buf[64];
  const int n = UartProtocol::formatTelemetry(buf, sizeof(buf), 100, -50, 0.25f, 1.5f);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_EQUAL_STRING("TEL 100 -50 0.250 1.500", buf);
}

static void test_format_events(void) {
  char buf[40];
  TEST_ASSERT_TRUE(UartProtocol::formatSlip(buf, sizeof(buf)) > 0);
  TEST_ASSERT_EQUAL_STRING("ERR SLIP", buf);
  TEST_ASSERT_TRUE(UartProtocol::formatObstacle(buf, sizeof(buf), "front", 0.2f) > 0);
  TEST_ASSERT_EQUAL_STRING("EVT OBSTACLE front 0.200", buf);
  TEST_ASSERT_TRUE(UartProtocol::formatDone(buf, sizeof(buf)) > 0);
  TEST_ASSERT_EQUAL_STRING("EVT DONE", buf);
}

static void test_format_truncation_returns_zero(void) {
  char buf[4];
  TEST_ASSERT_EQUAL_INT(0, UartProtocol::formatSlip(buf, sizeof(buf)));  // "ERR SLIP" won't fit
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_parse_move);
  RUN_TEST(test_parse_stop_exact);
  RUN_TEST(test_parse_config);
  RUN_TEST(test_parse_leading_spaces);
  RUN_TEST(test_reject_unknown_and_malformed);
  RUN_TEST(test_format_telemetry_roundtrip);
  RUN_TEST(test_format_events);
  RUN_TEST(test_format_truncation_returns_zero);
  return UNITY_END();
}
