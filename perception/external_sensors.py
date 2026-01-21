"""
External Sensors Module

Reads sensor data from ESP32 sensor pod via serial.
Provides distance sensing (ToF + ultrasonics) and IMU data.
"""
import asyncio
import json
import logging
import threading
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional, Callable

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False

logger = logging.getLogger(__name__)


@dataclass
class ExternalSensorReading:
    """Single reading from ESP32 sensor pod."""
    ts_ms: int = 0
    tof_mm: int = -1
    ultra_l_mm: int = -1
    ultra_c_mm: int = -1
    ultra_r_mm: int = -1
    ax_g: float = 0.0
    ay_g: float = 0.0
    az_g: float = 0.0
    gx_dps: float = 0.0
    gy_dps: float = 0.0
    gz_dps: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    local_time: datetime = field(default_factory=datetime.now)

    @classmethod
    def from_json(cls, data: dict) -> "ExternalSensorReading":
        """Parse JSON from ESP32."""
        mpu = data.get("mpu", {})
        return cls(
            ts_ms=data.get("ts_ms", 0),
            tof_mm=data.get("tof_mm", -1),
            ultra_l_mm=data.get("ultra_l_mm", -1),
            ultra_c_mm=data.get("ultra_c_mm", -1),
            ultra_r_mm=data.get("ultra_r_mm", -1),
            ax_g=mpu.get("ax_g", 0.0),
            ay_g=mpu.get("ay_g", 0.0),
            az_g=mpu.get("az_g", 0.0),
            gx_dps=mpu.get("gx_dps", 0.0),
            gy_dps=mpu.get("gy_dps", 0.0),
            gz_dps=mpu.get("gz_dps", 0.0),
            pitch=mpu.get("pitch", 0.0),
            roll=mpu.get("roll", 0.0),
            yaw=mpu.get("yaw", 0.0),
        )

    def get_min_front_distance(self) -> int:
        """Get minimum distance from forward-facing sensors."""
        distances = [d for d in [self.tof_mm, self.ultra_c_mm] if d > 0]
        return min(distances) if distances else -1

    def get_min_distance(self) -> int:
        """Get minimum distance from all sensors."""
        distances = [d for d in [self.tof_mm, self.ultra_l_mm, self.ultra_c_mm, self.ultra_r_mm] if d > 0]
        return min(distances) if distances else -1

    def is_valid(self) -> bool:
        """Check if reading has valid data."""
        return self.ts_ms > 0 and any(
            d > 0 for d in [self.tof_mm, self.ultra_l_mm, self.ultra_c_mm, self.ultra_r_mm]
        )

    def is_tilted(self, threshold: float = 15.0) -> bool:
        """Check if robot is tilted beyond threshold (degrees)."""
        return abs(self.pitch) > threshold or abs(self.roll) > threshold


class ExternalSensorReader:
    """
    Reads and parses JSON sensor data from ESP32 via serial.
    Runs in background thread, provides latest readings on demand.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 115200,
        buffer_size: int = 100
    ):
        self.port = port
        self.baud = baud
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._buffer: deque = deque(maxlen=buffer_size)
        self._latest: Optional[ExternalSensorReading] = None
        self._on_reading: Optional[Callable[[ExternalSensorReading], None]] = None
        self._error_count = 0
        self._read_count = 0

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def latest(self) -> Optional[ExternalSensorReading]:
        return self._latest

    def set_callback(self, callback: Callable[[ExternalSensorReading], None]):
        """Set callback for each new reading."""
        self._on_reading = callback

    def connect(self) -> bool:
        """Connect to ESP32 serial port."""
        if not HAS_SERIAL:
            logger.warning("pyserial not installed - external sensors disabled")
            return False

        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=1.0)
            import time
            time.sleep(2.0)  # Wait for ESP32 reset
            self._serial.reset_input_buffer()
            logger.info(f"Connected to external sensors on {self.port}")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to connect to external sensors: {e}")
            return False

    def start(self) -> bool:
        """Start background reading thread."""
        if not self.is_connected:
            if not self.connect():
                return False

        if self._running:
            return True

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("External sensor reader started")
        return True

    def stop(self):
        """Stop reading and close connection."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._serial:
            self._serial.close()
            self._serial = None
        logger.info(f"External sensor reader stopped. Reads: {self._read_count}, Errors: {self._error_count}")

    def _read_loop(self):
        """Background thread: read and parse serial data."""
        while self._running and self._serial:
            try:
                if self._serial.in_waiting:
                    line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                    if line.startswith("{"):
                        self._parse_line(line)
            except serial.SerialException as e:
                logger.error(f"Serial read error: {e}")
                self._error_count += 1
                import time
                time.sleep(0.5)
            except Exception as e:
                logger.debug(f"Read loop error: {e}")
                self._error_count += 1

    def _parse_line(self, line: str):
        """Parse JSON line into reading."""
        try:
            data = json.loads(line)
            reading = ExternalSensorReading.from_json(data)

            if reading.is_valid():
                self._latest = reading
                self._buffer.append(reading)
                self._read_count += 1

                if self._on_reading:
                    self._on_reading(reading)

        except json.JSONDecodeError:
            self._error_count += 1
        except Exception as e:
            logger.debug(f"Parse error: {e}")
            self._error_count += 1

    def get_latest(self) -> Optional[ExternalSensorReading]:
        """Get most recent valid reading."""
        return self._latest

    def get_recent(self, n: int = 10) -> list:
        """Get last N readings."""
        return list(self._buffer)[-n:]

    def get_averaged_distances(self, n: int = 5) -> dict:
        """Get averaged distances from recent readings."""
        recent = self.get_recent(n)
        if not recent:
            return {"tof": -1, "left": -1, "center": -1, "right": -1}

        def avg_valid(values):
            valid = [v for v in values if v > 0]
            return int(sum(valid) / len(valid)) if valid else -1

        return {
            "tof": avg_valid([r.tof_mm for r in recent]),
            "left": avg_valid([r.ultra_l_mm for r in recent]),
            "center": avg_valid([r.ultra_c_mm for r in recent]),
            "right": avg_valid([r.ultra_r_mm for r in recent]),
        }

    def get_status(self) -> dict:
        """Get reader status for debugging."""
        return {
            "connected": self.is_connected,
            "running": self._running,
            "read_count": self._read_count,
            "error_count": self._error_count,
            "buffer_size": len(self._buffer),
            "latest_ts": self._latest.ts_ms if self._latest else None,
        }


# Standalone test
async def test_external_sensors():
    """Test external sensor reading."""
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

    reader = ExternalSensorReader(port=port)

    def on_reading(r: ExternalSensorReading):
        print(f"\r[{r.ts_ms:>8}ms] "
              f"ToF:{r.tof_mm:>4} "
              f"L:{r.ultra_l_mm:>4} C:{r.ultra_c_mm:>4} R:{r.ultra_r_mm:>4} | "
              f"Pitch:{r.pitch:>6.1f}° Roll:{r.roll:>6.1f}° Yaw:{r.yaw:>6.1f}°",
              end="", flush=True)

    reader.set_callback(on_reading)

    if reader.start():
        print(f"Reading from {port}... (Ctrl+C to stop)\n")
        try:
            while True:
                await asyncio.sleep(1.0)
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            reader.stop()
            print(f"\nStatus: {reader.get_status()}")
    else:
        print("Failed to connect")


if __name__ == "__main__":
    asyncio.run(test_external_sensors())
