// ESP32 robot firmware — entry point.
//
// Phase 0: empty skeleton that compiles cleanly for esp32dev. The cooperative
// scheduler (UART 1kHz, control 200Hz, sensors 50Hz, telemetry 20Hz) and the
// driver classes (MotorDriver, AS5600Encoder, UltrasonicSensor, ...) are added
// in Phase 1. No delay() is used anywhere — timing is millis()/timer based.

#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Phase 1 cooperative scheduler goes here.
}
