# ESP32 Sensor Pod Integration

## Overview

The ESP32 sensor pod provides forward-facing distance sensors and an IMU for the Cozmo robot. It mounts on the trailer and communicates with the PC via WiFi UDP.

## Sensors

| Sensor | Type | Field | Notes |
|--------|------|-------|-------|
| VL53L0X | Time-of-Flight | `tof_mm` | Front distance, center |
| HC-SR04 L | Ultrasonic | `ultra_l_mm` | Left distance, 15 deg offset |
| HC-SR04 C | Ultrasonic | `ultra_c_mm` | Center distance |
| HC-SR04 R | Ultrasonic | `ultra_r_mm` | Right distance, -15 deg offset |
| MPU6050 | IMU (accel) | `ax_g`, `ay_g`, `az_g` | Acceleration in g's |
| MPU6050 | IMU (gyro) | `gx_dps`, `gy_dps`, `gz_dps` | Rotation in deg/s |
| MPU6050 | IMU (angles) | `pitch`, `roll`, `yaw` | Derived orientation |

## Data Format

The ESP32 sends JSON over UDP (port 5000) at ~20Hz:

```json
{
  "ts_ms": 12345,
  "tof_mm": 150,
  "ultra_l_mm": 200,
  "ultra_c_mm": 180,
  "ultra_r_mm": 220,
  "mpu": {
    "ax_g": 0.01,
    "ay_g": -0.02,
    "az_g": 1.0,
    "gx_dps": 0.15,
    "gy_dps": -0.22,
    "gz_dps": 0.08,
    "pitch": 1.23,
    "roll": -0.45,
    "yaw": 12.5
  }
}
```

## Physical Mounting

```
           (front)
      UL ------- UR       Ultrasonics L/R angled 15 deg outward
          ToF              ToF center, pointing straight ahead
          UC               Ultrasonic center, straight ahead
         [COZMO]
          MPU              MPU6050 on Cozmo's back
        [TRAILER]
           (rear)
```

Heights and angles are defined in `config.py` under `SENSOR_GEOMETRY`.

## IMU Mounting

The MPU6050 is mounted component-side-down. Axis signs are corrected in software:
- `IMU_ACCEL_X_SIGN = 1.0`
- `IMU_ACCEL_Y_SIGN = -1.0` (inverted)
- `IMU_ACCEL_Z_SIGN = -1.0` (inverted)

Resting pitch is ~17.5 degrees due to mounting angle. Tilt detection uses a baseline calibrated from the first real sensor reading.

## Connection

### WiFi UDP (default)

```
config.py:
  EXT_SENSOR_MODE = "udp"
  EXT_SENSOR_UDP_PORT = 5000
```

The ESP32 connects to WiFi and broadcasts UDP packets to port 5000. The `ExternalSensorReader` in `perception/external_sensors.py` listens on this port.

### Serial USB (alternative)

```
config.py:
  EXT_SENSOR_MODE = "serial"
  EXT_SENSOR_PORT = "COM3"       # Windows
  EXT_SENSOR_PORT = "/dev/ttyUSB0"  # Linux
  EXT_SENSOR_BAUD = 115200
```

## Software Integration

### Reader: `perception/external_sensors.py`

`ExternalSensorReader` runs a background thread (serial) or asyncio task (UDP) that:
1. Receives JSON packets from ESP32
2. Parses into `ExternalSensorReading` dataclass
3. Calls a callback to update `robot.sensors` fields

### Sensor Fields on Robot

The callback in `main.py` maps ESP32 readings to `robot.sensors`:

| ESP32 Field | Robot Sensor Field | Notes |
|-------------|--------------------|-------|
| `tof_mm` | `ext_tof_mm` | Front ToF distance |
| `ultra_l_mm` | `ext_ultra_l_mm` | Left ultrasonic |
| `ultra_c_mm` | `ext_ultra_c_mm` | Center ultrasonic |
| `ultra_r_mm` | `ext_ultra_r_mm` | Right ultrasonic |
| `ax_g` | `ext_ax_g` | Accel X (with sign correction) |
| `ay_g` | `ext_ay_g` | Accel Y (with sign correction) |
| `az_g` | `ext_az_g` | Accel Z (with sign correction) |
| derived | `ext_pitch` | Computed from corrected accel |
| derived | `ext_roll` | Computed from corrected accel |
| `yaw` | `ext_yaw` | Raw yaw from ESP32 |

### How the Navigator Uses Sensors

`FrontierNavigator` uses the ESP32 sensors for:
1. **Map updates**: Ray-traces all 4 distance sensors into the occupancy grid each tick
2. **Obstacle detection**: Front distance < 200mm triggers escape
3. **Stall detection**: IMU yaw change + distance traveled to detect stalls
4. **Tilt detection**: Pitch/roll exceeding 25 degrees (relative to baseline) stops the robot
5. **Escape direction**: Compares left vs right distance to pick turn direction

### Testing

```bash
# Test ESP32 sensors standalone
python -m perception.external_sensors

# With specific port (serial mode)
python -m perception.external_sensors COM3
```

## Power

The ESP32 is powered through an Arduino Nano relay. The Nano must be plugged into USB for the ESP32 to receive power. A buck converter for external battery power is planned to reduce trailer drag from the USB cable.
