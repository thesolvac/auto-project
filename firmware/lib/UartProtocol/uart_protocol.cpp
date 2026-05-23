#include "uart_protocol.h"

#include <cstdio>
#include <cstring>

namespace {

// Advance past leading spaces/tabs.
const char* skipBlanks(const char* s) {
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  return s;
}

// True if `line` starts with `verb` followed by a separator or end of string,
// so that "STOP" matches "STOP" but not "STOPXYZ".
bool startsWithVerb(const char* line, const char* verb) {
  const size_t len = std::strlen(verb);
  if (std::strncmp(line, verb, len) != 0) {
    return false;
  }
  const char next = line[len];
  return next == '\0' || next == ' ' || next == '\t' || next == '\r' || next == '\n';
}

bool fits(int written, size_t size) {
  return written >= 0 && static_cast<size_t>(written) < size;
}

}  // namespace

bool UartProtocol::parseCommand(const char* line, ParsedCommand& out) {
  out = ParsedCommand{};
  if (line == nullptr) {
    return false;
  }
  line = skipBlanks(line);

  if (startsWithVerb(line, "MOVE")) {
    float l = 0.0f;
    float r = 0.0f;
    if (std::sscanf(line + 4, " %f %f", &l, &r) == 2) {
      out.type = CommandType::kMove;
      out.left = l;
      out.right = r;
      return true;
    }
    out.type = CommandType::kUnknown;
    return false;
  }

  if (startsWithVerb(line, "STOP")) {
    out.type = CommandType::kStop;
    return true;
  }

  if (startsWithVerb(line, "CFG")) {
    char key[16] = {0};
    float v = 0.0f;
    if (std::sscanf(line + 3, " %15s %f", key, &v) == 2) {
      out.type = CommandType::kConfig;
      std::strncpy(out.key, key, sizeof(out.key) - 1);
      out.value = v;
      return true;
    }
    out.type = CommandType::kUnknown;
    return false;
  }

  out.type = CommandType::kUnknown;
  return false;
}

int UartProtocol::formatTelemetry(char* buf, size_t size, long enc_left, long enc_right,
                                  float dist_front, float dist_rear) {
  const int n = std::snprintf(buf, size, "TEL %ld %ld %.3f %.3f", enc_left, enc_right,
                              static_cast<double>(dist_front), static_cast<double>(dist_rear));
  return fits(n, size) ? n : 0;
}

int UartProtocol::formatSlip(char* buf, size_t size) {
  const int n = std::snprintf(buf, size, "ERR SLIP");
  return fits(n, size) ? n : 0;
}

int UartProtocol::formatObstacle(char* buf, size_t size, const char* sensor, float dist) {
  const int n =
      std::snprintf(buf, size, "EVT OBSTACLE %s %.3f", sensor, static_cast<double>(dist));
  return fits(n, size) ? n : 0;
}

int UartProtocol::formatDone(char* buf, size_t size) {
  const int n = std::snprintf(buf, size, "EVT DONE");
  return fits(n, size) ? n : 0;
}
