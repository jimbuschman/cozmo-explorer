# ESP32 Sensor Pod Integration for Cozmo Explorer

## Overview
Integrate an ESP32-based sensor pod (ToF + 3x Ultrasonic + MPU6050 IMU) with the existing Cozmo Explorer project. This gives Cozmo forward-facing distance sensors for proactive obstacle avoidance.

## ESP32 Output Format
The ESP32 sends JSON over serial at 115200 baud:
```json
{"ts_ms":12345,"tof_mm":41,"mpu":{"ax_g":0.012,"ay_g":-0.034,"az_g":1.002,"gx_dps":0.15,"gy_dps":-0.22,"gz_dps":0.08,"pitch":1.23,"roll":-0.45,"yaw":12.5},"ultra_l_mm":85,"ultra_c_mm":116,"ultra_r_mm":72}
```

## Files to Create

### 1. CREATE: `perception/external_sensors.py`

```python
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
```

## Files to Modify

### 2. MODIFY: `perception/__init__.py`

Replace contents with:
```python
"""Perception module - camera and sensor processing"""
from .external_sensors import ExternalSensorReader, ExternalSensorReading

__all__ = ['ExternalSensorReader', 'ExternalSensorReading']
```

### 3. MODIFY: `cozmo_interface/robot.py`

In the `SensorData` dataclass, add these fields after the existing ones:

```python
@dataclass
class SensorData:
    """Current sensor readings from the robot"""
    cliff_detected: bool = False
    collision_detected: bool = False
    is_picked_up: bool = False
    is_on_charger: bool = False
    battery_voltage: float = 0.0
    head_angle: float = 0.0
    lift_height: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_angle: float = 0.0
    
    # === ADD THESE NEW FIELDS ===
    # External sensor pod (ESP32)
    ext_tof_mm: int = -1
    ext_ultra_l_mm: int = -1
    ext_ultra_c_mm: int = -1
    ext_ultra_r_mm: int = -1
    ext_pitch: float = 0.0
    ext_roll: float = 0.0
    ext_yaw: float = 0.0
    ext_connected: bool = False

    def get_front_obstacle_distance(self) -> int:
        """Get closest obstacle distance from forward-facing sensors."""
        distances = [d for d in [self.ext_tof_mm, self.ext_ultra_c_mm] if d > 0]
        return min(distances) if distances else -1

    def get_obstacle_distances(self) -> dict:
        """Get all obstacle distances."""
        return {
            "front": self.get_front_obstacle_distance(),
            "left": self.ext_ultra_l_mm,
            "right": self.ext_ultra_r_mm,
        }
```

### 4. MODIFY: `config.py`

Add at the end:
```python
# External sensor settings (ESP32 pod)
EXT_SENSOR_PORT = "/dev/ttyUSB0"  # Linux default, use "COM3" etc on Windows
EXT_SENSOR_BAUD = 115200
```

### 5. MODIFY: `requirements.txt`

Add:
```
pyserial>=3.5
```

### 6. MODIFY: `brain/behaviors.py`

Replace the entire `WanderBehavior` class with this version that uses distance sensors for proactive avoidance:

```python
class WanderBehavior(Behavior):
    """
    Random wandering exploration with proactive obstacle avoidance.
    
    Uses external distance sensors (if available) for proactive avoidance,
    falls back to reactive collision detection if not.
    """

    # Distance thresholds (mm)
    DANGER_DISTANCE = 80      # Emergency stop
    SLOW_DISTANCE = 150       # Slow down
    CAUTION_DISTANCE = 250    # Start looking for alternatives
    
    def __init__(
        self,
        robot: CozmoRobot,
        duration: float = 30.0,
        speed: float = 50.0,
        turn_probability: float = 0.1
    ):
        super().__init__(robot)
        self.duration = duration
        self.speed = speed
        self.turn_probability = turn_probability

    async def run(self) -> BehaviorResult:
        elapsed = 0.0
        check_interval = 0.15  # Faster checks for obstacle response
        distance_traveled = 0.0
        turns_made = 0
        stall_check_count = 0
        last_pose_x = self.robot.pose.x
        last_pose_y = self.robot.pose.y

        logger.info(f"Starting wander for {self.duration}s")
        has_distance_sensors = self.robot.sensors.ext_connected

        while elapsed < self.duration and not self.is_cancelled:
            sensors = self.robot.sensors
            
            # === SAFETY CHECKS ===
            if sensors.cliff_detected:
                logger.info("Cliff detected, backing up and turning")
                await self._escape_cliff()
                turns_made += 1
                elapsed += 1.5
                last_pose_x, last_pose_y = self.robot.pose.x, self.robot.pose.y
                continue

            if sensors.is_picked_up:
                await self.robot.stop()
                return BehaviorResult(BehaviorStatus.INTERRUPTED, "Robot picked up")

            # Check tilt from external IMU
            if has_distance_sensors and (abs(sensors.ext_pitch) > 20 or abs(sensors.ext_roll) > 20):
                logger.warning(f"Tilt detected: pitch={sensors.ext_pitch:.1f}° roll={sensors.ext_roll:.1f}°")
                await self.robot.stop()
                await asyncio.sleep(0.5)
                continue

            # === PROACTIVE OBSTACLE AVOIDANCE ===
            if has_distance_sensors:
                front_dist = sensors.get_front_obstacle_distance()
                left_dist = sensors.ext_ultra_l_mm
                right_dist = sensors.ext_ultra_r_mm
                
                # Emergency stop
                if 0 < front_dist < self.DANGER_DISTANCE:
                    logger.warning(f"Obstacle at {front_dist}mm, emergency reverse")
                    await self.robot.stop()
                    await self.robot.drive(-self.speed, duration=0.5)
                    await asyncio.sleep(0.5)
                    turn_angle = self._pick_turn_direction(left_dist, right_dist, 90)
                    await self.robot.turn(turn_angle)
                    turns_made += 1
                    elapsed += 1.5
                    last_pose_x, last_pose_y = self.robot.pose.x, self.robot.pose.y
                    continue
                
                # Caution zone - turn away
                if 0 < front_dist < self.CAUTION_DISTANCE:
                    turn_angle = self._pick_turn_direction(left_dist, right_dist, 45)
                    logger.info(f"Obstacle at {front_dist}mm, turning {turn_angle}°")
                    await self.robot.stop()
                    await self.robot.turn(turn_angle)
                    turns_made += 1
                    elapsed += abs(turn_angle) / 45
                    continue
                
                # Speed adjustment
                current_speed = self.speed * 0.5 if 0 < front_dist < self.SLOW_DISTANCE else self.speed
            else:
                current_speed = self.speed

            # === REACTIVE COLLISION DETECTION (fallback) ===
            current_x, current_y = self.robot.pose.x, self.robot.pose.y
            movement = math.sqrt((current_x - last_pose_x)**2 + (current_y - last_pose_y)**2)

            is_stalled = False
            if sensors.collision_detected:
                is_stalled = True
                logger.info("Collision detected")
                sensors.collision_detected = False
            elif stall_check_count > 5 and movement < 5.0:
                is_stalled = True
                logger.info(f"Stall detected, movement={movement:.1f}mm")

            if is_stalled:
                await self._escape_stall()
                turns_made += 1
                elapsed += 1.5
                stall_check_count = 0
                last_pose_x, last_pose_y = self.robot.pose.x, self.robot.pose.y
                continue

            stall_check_count += 1
            if stall_check_count > 8:
                last_pose_x, last_pose_y = current_x, current_y
                stall_check_count = 0

            # === RANDOM EXPLORATION ===
            if random.random() < self.turn_probability:
                await self.robot.stop()
                turn_angle = random.uniform(-60, 60)
                await self.robot.turn(turn_angle)
                turns_made += 1
                elapsed += abs(turn_angle) / 45
                last_pose_x, last_pose_y = self.robot.pose.x, self.robot.pose.y
                stall_check_count = 0

            # === DRIVE ===
            await self.robot.drive(current_speed)
            await asyncio.sleep(check_interval)
            distance_traveled += current_speed * check_interval
            elapsed += check_interval

        await self.robot.stop()

        return BehaviorResult(
            BehaviorStatus.COMPLETED if not self.is_cancelled else BehaviorStatus.INTERRUPTED,
            f"Wandered for {elapsed:.1f}s",
            {"duration": elapsed, "distance_estimate": distance_traveled, "turns": turns_made}
        )

    def _pick_turn_direction(self, left_dist: int, right_dist: int, magnitude: int = 90) -> int:
        """Pick turn direction based on clearance."""
        if left_dist <= 0 and right_dist <= 0:
            return random.choice([-magnitude, magnitude])
        if left_dist <= 0:
            return -magnitude
        if right_dist <= 0:
            return magnitude
        return magnitude if left_dist > right_dist else -magnitude

    async def _escape_cliff(self):
        await self.robot.stop()
        await self.robot.drive(-self.speed, duration=0.5)
        await asyncio.sleep(0.5)
        await self.robot.turn(random.choice([-90, -135, 90, 135]))

    async def _escape_stall(self):
        await self.robot.stop()
        await self.robot.drive(-self.speed, duration=0.7)
        await asyncio.sleep(0.7)
        await self.robot.turn(random.choice([-120, -90, 90, 120]))
```

### 7. MODIFY: `main.py`

Add import at top with other imports:
```python
import os
from perception.external_sensors import ExternalSensorReader
from typing import Optional
```

In `CozmoExplorer.__init__`, add after existing attributes:
```python
self.external_sensors: Optional[ExternalSensorReader] = None
```

In `CozmoExplorer.initialize()`, add this block AFTER robot connection succeeds but BEFORE state machine init:
```python
# Connect external sensors (ESP32 pod)
ext_port = os.environ.get("EXT_SENSOR_PORT", config.EXT_SENSOR_PORT)
logger.info(f"Connecting external sensors on {ext_port}...")
self.external_sensors = ExternalSensorReader(port=ext_port, baud=config.EXT_SENSOR_BAUD)

if self.external_sensors.start():
    logger.info("External sensors connected!")
    self.robot._sensors.ext_connected = True
    
    def on_ext_reading(reading):
        s = self.robot._sensors
        s.ext_tof_mm = reading.tof_mm
        s.ext_ultra_l_mm = reading.ultra_l_mm
        s.ext_ultra_c_mm = reading.ultra_c_mm
        s.ext_ultra_r_mm = reading.ultra_r_mm
        s.ext_pitch = reading.pitch
        s.ext_roll = reading.roll
        s.ext_yaw = reading.yaw
    
    self.external_sensors.set_callback(on_ext_reading)
else:
    logger.warning("External sensors not available - reactive collision detection only")
```

In `CozmoExplorer._shutdown()`, add before robot disconnect:
```python
if self.external_sensors:
    self.external_sensors.stop()
```

In `CozmoExplorer._print_status()`, add after existing logger.info calls:
```python
if self.robot.sensors.ext_connected:
    s = self.robot.sensors
    dists = s.get_obstacle_distances()
    logger.info(f"Obstacles: F={dists['front']}mm L={dists['left']}mm R={dists['right']}mm")
    logger.info(f"IMU: pitch={s.ext_pitch:.1f}° roll={s.ext_roll:.1f}° yaw={s.ext_yaw:.1f}°")
```

## Testing

### Test ESP32 sensors standalone:
```bash
python -m perception.external_sensors /dev/ttyUSB0
# Or on Windows:
python -m perception.external_sensors COM3
```

### Run full system with external sensors:
```bash
# Linux
EXT_SENSOR_PORT=/dev/ttyUSB0 python main.py

# Windows (set env var first or edit config.py)
python main.py
```

### Run without external sensors (falls back to reactive mode):
```bash
python main.py
# Will show warning but continue working
```

## Summary of Changes

| File | Action | Purpose |
|------|--------|---------|
| `perception/external_sensors.py` | CREATE | ESP32 serial reader |
| `perception/__init__.py` | MODIFY | Export new classes |
| `cozmo_interface/robot.py` | MODIFY | Add ext sensor fields to SensorData |
| `config.py` | MODIFY | Add sensor port/baud settings |
| `requirements.txt` | MODIFY | Add pyserial |
| `brain/behaviors.py` | MODIFY | Proactive obstacle avoidance |
| `main.py` | MODIFY | Wire up external sensors |