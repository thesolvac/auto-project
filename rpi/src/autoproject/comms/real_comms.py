"""Real robot communications over UART to the ESP32 (Phase 7 hardware).

Written and type-checked now; executed only on the physical robot (``mode: real``).
A background thread reads newline-framed telemetry/event lines (see
docs/uart_protocol.md), parses them, and dispatches base-class callbacks. The
serial link auto-reconnects if the device disappears.
"""

from __future__ import annotations

import logging
import threading
import time

import serial  # pyserial; only imported when this module is used (real mode)

from autoproject.comms.interfaces import IRobotComms, Telemetry

logger = logging.getLogger(__name__)

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
_RECONNECT_DELAY_S = 1.0


class RealRobotComms(IRobotComms):
    """``IRobotComms`` over a pyserial UART link to the ESP32."""

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD) -> None:
        super().__init__()
        self._port = port
        self._baud = baud
        self._serial: serial.Serial | None = None
        self._telemetry: Telemetry | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None

    def start(self) -> None:
        """Open the port and spin up the background reader thread."""
        self._stop.clear()
        self._reader = threading.Thread(
            target=self._read_loop, name="robot-comms", daemon=True
        )
        self._reader.start()

    def close(self) -> None:
        """Stop the reader thread and close the serial port."""
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=2.0)
        if self._serial is not None and self._serial.is_open:
            self._serial.close()

    # --- IRobotComms commands ---
    def move(self, left_mps: float, right_mps: float) -> None:
        self._send(f"MOVE {left_mps:.3f} {right_mps:.3f}")

    def stop(self) -> None:
        self._send("STOP")

    def get_telemetry(self) -> Telemetry | None:
        with self._lock:
            return self._telemetry

    # --- internals ---
    def _send(self, line: str) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.write((line + "\n").encode("ascii"))

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                if self._serial is None or not self._serial.is_open:
                    self._serial = serial.Serial(self._port, self._baud, timeout=1.0)
                    logger.info("Opened serial port %s @ %d", self._port, self._baud)
                raw = self._serial.readline()
                if raw:
                    self._handle_line(raw.decode("ascii", errors="ignore").strip())
            except (serial.SerialException, OSError) as exc:
                logger.warning(
                    "Serial error (%s); reconnecting in %.1fs", exc, _RECONNECT_DELAY_S
                )
                self._serial = None
                time.sleep(_RECONNECT_DELAY_S)

    def _handle_line(self, line: str) -> None:
        parts = line.split()
        if not parts:
            return
        if parts[0] == "TEL" and len(parts) == 5:
            self._parse_telemetry(parts)
        elif parts[0] == "ERR" and len(parts) >= 2 and parts[1] == "SLIP":
            self._emit_slip()
        elif parts[0] == "EVT" and len(parts) >= 2:
            if parts[1] == "OBSTACLE" and len(parts) == 4:
                self._emit_obstacle(parts[2], float(parts[3]))
            elif parts[1] == "DONE":
                self._emit_done()

    def _parse_telemetry(self, parts: list[str]) -> None:
        try:
            telemetry = Telemetry(
                enc_left_counts=int(parts[1]),
                enc_right_counts=int(parts[2]),
                dist_front_m=float(parts[3]),
                dist_rear_m=float(parts[4]),
                timestamp_s=time.monotonic(),
            )
        except ValueError:
            logger.debug("Malformed telemetry line: %r", parts)
            return
        with self._lock:
            self._telemetry = telemetry
