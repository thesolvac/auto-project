// ESP32 robot firmware — entry point and cooperative scheduler.
//
// Four cooperative tasks run off micros()/millis() with no delay() in the hot
// path (the only delayMicroseconds is the 10 us ultrasonic trigger pulse):
//   - UART parse  : every loop (~1 kHz effective)
//   - control loop: 200 Hz  (encoder update + PID + slip detection)
//   - sensor read : 50 Hz   (ultrasonic trigger + threshold check)
//   - telemetry   : 20 Hz   (TEL line to the RPi)
//
// Pin assignments mirror config/hardware_pins.yaml (the single source of truth);
// robot constants mirror config/robot_params.yaml.

#include <Arduino.h>
#include <Wire.h>

#include <cmath>

#include "as5600_encoder.h"
#include "closed_loop_controller.h"
#include "motor_driver.h"
#include "tca9548a.h"
#include "ultrasonic_sensor.h"
#include "uart_protocol.h"

namespace {

// ---- Pins (config/hardware_pins.yaml) ----
constexpr uint8_t kEnablePin = 25;  // TMC2209 EN, active low, shared
constexpr uint8_t kMs1Pin = 32;
constexpr uint8_t kMs2Pin = 33;
constexpr uint8_t kLeftStepPin = 26;
constexpr uint8_t kLeftDirPin = 27;
constexpr uint8_t kRightStepPin = 14;
constexpr uint8_t kRightDirPin = 12;
constexpr uint8_t kSdaPin = 21;
constexpr uint8_t kSclPin = 22;
constexpr uint8_t kLeftEncoderChannel = 0;
constexpr uint8_t kRightEncoderChannel = 1;
constexpr uint8_t kFrontTrigPin = 5;
constexpr uint8_t kFrontEchoPin = 34;
constexpr uint8_t kRearTrigPin = 4;
constexpr uint8_t kRearEchoPin = 35;

// ---- Robot constants (config/robot_params.yaml) ----
constexpr float kWheelRadiusM = 0.040f;
constexpr float kStepsPerRev = 200.0f;
constexpr float kMicrostepping = 16.0f;
constexpr int kEncoderCountsPerRev = 4096;
constexpr float kMaxAccelStepsS2 = 8000.0f;  // ~0.5 m/s^2 expressed in microsteps/s^2
constexpr float kObstacleThresholdM = 0.30f;

constexpr float kTwoPi = 6.28318530718f;
constexpr float kWheelCircumferenceM = kTwoPi * kWheelRadiusM;
// Microsteps per metre of wheel travel.
constexpr float kStepsPerMeter = (kStepsPerRev * kMicrostepping) / kWheelCircumferenceM;

// ---- Task periods [us] ----
constexpr uint32_t kControlPeriodUs = 5000;    // 200 Hz
constexpr uint32_t kSensorPeriodUs = 20000;    // 50 Hz
constexpr uint32_t kTelemetryPeriodUs = 50000; // 20 Hz

// ---- Hardware objects ----
MotorDriver g_left_motor(kLeftStepPin, kLeftDirPin, kMaxAccelStepsS2);
MotorDriver g_right_motor(kRightStepPin, kRightDirPin, kMaxAccelStepsS2);
Tca9548a g_mux(0x70);
As5600Encoder g_left_encoder(g_mux, kLeftEncoderChannel);
As5600Encoder g_right_encoder(g_mux, kRightEncoderChannel);
UltrasonicSensor g_front_sonar(kFrontTrigPin, kFrontEchoPin);
UltrasonicSensor g_rear_sonar(kRearTrigPin, kRearEchoPin);
// PID gains are placeholders to be tuned on hardware; slip = 0.10 m/s disagreement
// persisting for 40 ticks (~0.2 s at 200 Hz).
ClosedLoopController g_left_control(0.5f, 0.0f, 0.0f, 0.10f, 40);
ClosedLoopController g_right_control(0.5f, 0.0f, 0.0f, 0.10f, 40);

// ---- Commanded wheel velocities [m/s] ----
float g_cmd_left_mps = 0.0f;
float g_cmd_right_mps = 0.0f;
bool g_slip_reported = false;

// ---- UART line assembly ----
char g_line[64];
size_t g_line_len = 0;

float encoderVelocityMps(const As5600Encoder& enc) {
  return (enc.velocityCountsPerSec() / static_cast<float>(kEncoderCountsPerRev)) *
         kWheelCircumferenceM;
}

void handleLine(const char* line) {
  ParsedCommand cmd;
  if (!UartProtocol::parseCommand(line, cmd)) {
    return;
  }
  switch (cmd.type) {
    case CommandType::kMove:
      g_cmd_left_mps = cmd.left;
      g_cmd_right_mps = cmd.right;
      g_slip_reported = false;
      break;
    case CommandType::kStop:
      g_cmd_left_mps = 0.0f;
      g_cmd_right_mps = 0.0f;
      break;
    case CommandType::kConfig:
      // Reserved for runtime tuning (e.g. PID gains); no-op for now.
      break;
    default:
      break;
  }
}

void pollUart() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (g_line_len > 0) {
        g_line[g_line_len] = '\0';
        handleLine(g_line);
        g_line_len = 0;
      }
    } else if (g_line_len < sizeof(g_line) - 1) {
      g_line[g_line_len++] = c;
    }
  }
}

void runControl(float dt) {
  g_left_encoder.update(dt);
  g_right_encoder.update(dt);

  const float left_cmd =
      g_left_control.update(g_cmd_left_mps, encoderVelocityMps(g_left_encoder), dt);
  const float right_cmd =
      g_right_control.update(g_cmd_right_mps, encoderVelocityMps(g_right_encoder), dt);

  g_left_motor.setTargetSpeed(left_cmd * kStepsPerMeter);
  g_right_motor.setTargetSpeed(right_cmd * kStepsPerMeter);
  g_left_motor.update(dt);
  g_right_motor.update(dt);

  if ((g_left_control.slipping() || g_right_control.slipping()) && !g_slip_reported) {
    char buf[16];
    if (UartProtocol::formatSlip(buf, sizeof(buf)) > 0) {
      Serial.println(buf);
    }
    g_slip_reported = true;
  }
}

void runSensors() {
  g_front_sonar.trigger();
  g_rear_sonar.trigger();
  const float front = g_front_sonar.lastDistanceM();
  const float rear = g_rear_sonar.lastDistanceM();

  char buf[40];
  if (front < kObstacleThresholdM && UartProtocol::formatObstacle(buf, sizeof(buf), "front", front) > 0) {
    Serial.println(buf);
  }
  if (rear < kObstacleThresholdM && UartProtocol::formatObstacle(buf, sizeof(buf), "rear", rear) > 0) {
    Serial.println(buf);
  }
}

void runTelemetry() {
  char buf[64];
  const int n = UartProtocol::formatTelemetry(buf, sizeof(buf), g_left_encoder.totalCounts(),
                                              g_right_encoder.totalCounts(),
                                              g_front_sonar.lastDistanceM(),
                                              g_rear_sonar.lastDistanceM());
  if (n > 0) {
    Serial.println(buf);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin(kSdaPin, kSclPin);

  pinMode(kEnablePin, OUTPUT);
  digitalWrite(kEnablePin, LOW);  // EN active low: enable both drivers
  pinMode(kMs1Pin, OUTPUT);
  pinMode(kMs2Pin, OUTPUT);
  digitalWrite(kMs1Pin, HIGH);  // TMC2209: MS1=MS2=high -> 1/16 microstepping
  digitalWrite(kMs2Pin, HIGH);

  g_left_motor.begin();
  g_right_motor.begin();
  g_front_sonar.begin();
  g_rear_sonar.begin();
}

void loop() {
  static uint32_t last_control_us = 0;
  static uint32_t last_sensor_us = 0;
  static uint32_t last_telemetry_us = 0;
  const uint32_t now = micros();

  pollUart();

  if (now - last_control_us >= kControlPeriodUs) {
    last_control_us += kControlPeriodUs;
    runControl(static_cast<float>(kControlPeriodUs) * 1e-6f);
  }
  if (now - last_sensor_us >= kSensorPeriodUs) {
    last_sensor_us += kSensorPeriodUs;
    runSensors();
  }
  if (now - last_telemetry_us >= kTelemetryPeriodUs) {
    last_telemetry_us += kTelemetryPeriodUs;
    runTelemetry();
  }
}
