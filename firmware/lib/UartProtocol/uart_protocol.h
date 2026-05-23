#pragma once

// Line-based ASCII protocol between the RPi and the ESP32 (see
// docs/uart_protocol.md). This class is pure logic — no Arduino dependency — so
// the parser and formatters run under host-side Unity tests on the native env.

#include <cstddef>

// Command kinds the ESP32 accepts from the RPi.
enum class CommandType {
  kNone,     // empty / not yet parsed
  kMove,     // "MOVE <v_left> <v_right>"  wheel linear velocities [m/s]
  kStop,     // "STOP"
  kConfig,   // "CFG <key> <value>"
  kUnknown,  // recognized framing failed / unknown verb
};

// Result of parsing a single inbound line.
struct ParsedCommand {
  CommandType type = CommandType::kNone;
  float left = 0.0f;    // MOVE: left wheel velocity
  float right = 0.0f;   // MOVE: right wheel velocity
  char key[16] = {0};   // CFG: key
  float value = 0.0f;   // CFG: value
};

class UartProtocol {
 public:
  // Parse one line (trailing newline optional). Returns true if the line is a
  // valid, fully-formed command; false otherwise (out.type is then kUnknown).
  static bool parseCommand(const char* line, ParsedCommand& out);

  // Telemetry / event formatters. Each writes a NUL-terminated string into buf
  // and returns the character count (excluding the NUL), or 0 if it would not
  // fit. None of them append a newline; the caller frames lines.
  static int formatTelemetry(char* buf, size_t size, long enc_left, long enc_right,
                             float dist_front, float dist_rear);
  static int formatSlip(char* buf, size_t size);
  static int formatObstacle(char* buf, size_t size, const char* sensor, float dist);
  static int formatDone(char* buf, size_t size);
};
