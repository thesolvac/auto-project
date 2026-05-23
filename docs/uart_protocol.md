# UART Protocol — ESP32 ↔ Raspberry Pi

Line-based ASCII protocol over USB serial at **115200 baud, 8N1**. Each message
is a single `\n`-terminated line: a whitespace-separated command/telemetry token
followed by arguments.

> Skeleton — the full grammar, argument units, and the `UartProtocol` parser are
> specified and host-tested in Phase 1. The shape below establishes intent.

## Direction

- **RPi → ESP32**: motion commands and configuration.
- **ESP32 → RPi**: telemetry and asynchronous events.

## Commands (RPi → ESP32) — provisional

| Line | Meaning |
|---|---|
| `MOVE <v_left> <v_right>` | set wheel linear velocities [m/s] |
| `STOP` | halt both motors |
| `CFG <key> <value>` | runtime configuration |

## Telemetry & events (ESP32 → RPi) — provisional

| Line | Meaning |
|---|---|
| `TEL <enc_l> <enc_r> <dist_front> <dist_rear>` | periodic telemetry @ 20 Hz |
| `ERR SLIP` | closed-loop controller detected wheel slip |
| `EVT OBSTACLE <sensor> <dist>` | ultrasonic threshold tripped |
| `EVT DONE` | commanded move-steps completed |

Exact field order, units, framing, and error handling are finalized in Phase 1.
