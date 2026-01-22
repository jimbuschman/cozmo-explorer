# Nano + ESP32 Sensor Integration (Ultrasonic + VL53L0X + MPU6050)

## Overview
This document describes the wiring and code for connecting:
- **Arduino Nano** (3x ultrasonic sensors)
- **ESP32-S3** (VL53L0X ToF sensor + MPU6050 IMU)
- **Serial communication between Nano and ESP32**

The ESP32 collects data from its ToF sensor, IMU, and the Nano, then outputs a single JSON line per reading with a timestamp.

---

## Wiring

### 1. Nano (3x Ultrasonic)

#### Ultrasonic sensor pin mapping
| Sensor | TRIG | ECHO |
|--------|------|------|
| Left   | D2   | D3   |
| Center | D4   | D5   |
| Right  | D6   | D7   |

#### Nano → ESP32 serial link
| Nano Pin | ESP32 Pin | Purpose |
|----------|-----------|---------|
| D11 (TX) | GPIO16 (RX) | Data to ESP32 |
| D10 (RX) | GPIO17 (TX) | Not used (optional) |
| GND      | GND        | Common ground |

---

### 2. ESP32 (VL53L0X ToF + MPU6050 IMU)

Both sensors use I2C and share the same bus.

#### I2C device addresses
| Device   | I2C Address |
|----------|-------------|
| VL53L0X  | 0x29        |
| MPU6050  | 0x68        |

#### VL53L0X pin mapping
| VL53L0X Pin | ESP32 Pin | Notes |
|-------------|-----------|-------|
| VIN         | 3.3V      | **Do not use 5V** |
| GND         | GND       | |
| SDA         | SDA (GPIO21) | I2C data (shared) |
| SCL         | SCL (GPIO22) | I2C clock (shared) |
| XSHUT       | (optional) | Only needed for power-cycle control |

#### MPU6050 pin mapping
| MPU6050 Pin | ESP32 Pin | Notes |
|-------------|-----------|-------|
| VCC         | 3.3V      | Can also use 5V if module has regulator |
| GND         | GND       | |
| SDA         | SDA (GPIO21) | I2C data (shared with VL53L0X) |
| SCL         | SCL (GPIO22) | I2C clock (shared with VL53L0X) |
| INT         | (optional) | For interrupt-driven reads |

> **Important:** Use **3.3V** on the VL53L0X to avoid damaging it. MPU6050 modules typically have onboard regulators and can accept 5V, but check your specific board.

---

## Output Format (for system ingestion)

The ESP32 outputs a single JSON object per reading:

```json
{"ts_ms":12345,"tof_mm":41,"mpu":{"ax_g":0.012,"ay_g":-0.034,"az_g":1.002,"gx_dps":0.15,"gy_dps":-0.22,"gz_dps":0.08,"pitch":1.23,"roll":-0.45,"yaw":12.5},"ultra_l_mm":85,"ultra_c_mm":116,"ultra_r_mm":72}
```

| Field | Description |
|-------|-------------|
| `ts_ms` | Timestamp in milliseconds (relative to ESP32 boot) |
| `tof_mm` | ToF distance (mm), `-1` indicates invalid reading |
| `mpu.ax_g`, `ay_g`, `az_g` | Accelerometer (g) |
| `mpu.gx_dps`, `gy_dps`, `gz_dps` | Gyroscope (degrees/sec), bias-corrected |
| `mpu.pitch`, `roll`, `yaw` | Orientation angles (degrees), complementary filtered |
| `ultra_*_mm` | Ultrasonic distances (mm), `-1` indicates invalid reading |

---

## Nano Code (Ultrasonic + JSON Output)

```cpp
// Arduino Nano - 3x Ultrasonic Sensor Reader
// Sends JSON to ESP32 via SoftwareSerial on D10/D11

#include <SoftwareSerial.h>

const int TRIG_L = 2, ECHO_L = 3;
const int TRIG_C = 4, ECHO_C = 5;
const int TRIG_R = 6, ECHO_R = 7;

// SoftwareSerial: RX=D10, TX=D11
SoftwareSerial espSerial(10, 11);

void setup() {
  Serial.begin(9600);       // USB debug
  espSerial.begin(9600);    // To ESP32

  pinMode(TRIG_L, OUTPUT);
  pinMode(TRIG_C, OUTPUT);
  pinMode(TRIG_R, OUTPUT);

  pinMode(ECHO_L, INPUT);
  pinMode(ECHO_C, INPUT);
  pinMode(ECHO_R, INPUT);

  digitalWrite(TRIG_L, LOW);
  digitalWrite(TRIG_C, LOW);
  digitalWrite(TRIG_R, LOW);

  Serial.println("Nano ready");
  delay(500);
}

long readUS(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH, 25000);
  if (dur == 0) return -1;
  return (dur * 343) / 2000;
}

void loop() {
  long L = readUS(TRIG_L, ECHO_L);
  delay(30);
  long C = readUS(TRIG_C, ECHO_C);
  delay(30);
  long R = readUS(TRIG_R, ECHO_R);

  // Send JSON to ESP32
  espSerial.print("{\"ultra_l_mm\":");
  espSerial.print(L);
  espSerial.print(",\"ultra_c_mm\":");
  espSerial.print(C);
  espSerial.print(",\"ultra_r_mm\":");
  espSerial.print(R);
  espSerial.println("}");

  // Debug to USB
  Serial.print("L:");
  Serial.print(L);
  Serial.print(" C:");
  Serial.print(C);
  Serial.print(" R:");
  Serial.println(R);

  delay(100);
}
```

---

## ESP32 Code (ToF + MPU6050 + Nano + Combined JSON Output)

```cpp
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>

Adafruit_VL53L0X lox;
MPU6050 mpu;
HardwareSerial NanoSerial(1);

// --- Calibration & filtering ---
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

float pitch = 0, roll = 0, yaw = 0;
unsigned long lastTime = 0;

const float alpha = 0.98;  // complementary filter constant

void setup() {
  Serial.begin(115200);
  delay(100);

  // VL53L0X
  Wire.begin();
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1);
  }

  // MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Failed to connect to MPU6050");
    while (1);
  }

  // UART to Nano
  NanoSerial.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println("ESP32 Ready");

  // --- Calibrate gyro bias (simple average) ---
  const int samples = 200;
  long gxSum = 0, gySum = 0, gzSum = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gxSum += gx;
    gySum += gy;
    gzSum += gz;
    delay(5);
  }

  gyroBiasX = gxSum / (float)samples;
  gyroBiasY = gySum / (float)samples;
  gyroBiasZ = gzSum / (float)samples;

  lastTime = millis();
}

void loop() {
  // Read VL53L0X
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  long tofDist = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;

  // Read MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to real units
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float gx_dps = gx / 131.0;
  float gy_dps = gy / 131.0;
  float gz_dps = gz / 131.0;

  // Apply gyro bias correction
  gx_dps -= gyroBiasX / 131.0;
  gy_dps -= gyroBiasY / 131.0;
  gz_dps -= gyroBiasZ / 131.0;

  // --- Compute pitch/roll/yaw ---
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Accelerometer angles (degrees)
  float accPitch = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
  float accRoll  = atan2(-ax_g, az_g) * 180.0 / PI;

  // Complementary filter for pitch & roll
  pitch = alpha * (pitch + gy_dps * dt) + (1 - alpha) * accPitch;
  roll  = alpha * (roll + gx_dps * dt) + (1 - alpha) * accRoll;

  // Yaw integration (drifts over time)
  yaw += gz_dps * dt;

  // Read Nano serial data
  long L = -1, C = -1, R = -1;
  if (NanoSerial.available()) {
    String line = NanoSerial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      int idxL = line.indexOf("ultra_l_mm");
      int idxC = line.indexOf("ultra_c_mm");
      int idxR = line.indexOf("ultra_r_mm");

      if (idxL >= 0) {
        int start = line.indexOf(":", idxL) + 1;
        int end = line.indexOf(",", idxL);
        L = line.substring(start, end).toInt();
      }
      if (idxC >= 0) {
        int start = line.indexOf(":", idxC) + 1;
        int end = line.indexOf(",", idxC);
        C = line.substring(start, end).toInt();
      }
      if (idxR >= 0) {
        int start = line.indexOf(":", idxR) + 1;
        int end = line.indexOf("}", idxR);
        R = line.substring(start, end).toInt();
      }
    }
  }

  // Build final JSON with timestamp
  unsigned long ts = millis();

  Serial.print("{\"ts_ms\":");
  Serial.print(ts);
  Serial.print(",\"tof_mm\":");
  Serial.print(tofDist);

  Serial.print(",\"mpu\":{");
  Serial.print("\"ax_g\":"); Serial.print(ax_g, 3);
  Serial.print(",\"ay_g\":"); Serial.print(ay_g, 3);
  Serial.print(",\"az_g\":"); Serial.print(az_g, 3);
  Serial.print(",\"gx_dps\":"); Serial.print(gx_dps, 3);
  Serial.print(",\"gy_dps\":"); Serial.print(gy_dps, 3);
  Serial.print(",\"gz_dps\":"); Serial.print(gz_dps, 3);
  Serial.print(",\"pitch\":"); Serial.print(pitch, 3);
  Serial.print(",\"roll\":"); Serial.print(roll, 3);
  Serial.print(",\"yaw\":"); Serial.print(yaw, 3);
  Serial.print("}");

  Serial.print(",\"ultra_l_mm\":");
  Serial.print(L);
  Serial.print(",\"ultra_c_mm\":");
  Serial.print(C);
  Serial.print(",\"ultra_r_mm\":");
  Serial.print(R);

  Serial.println("}");

  delay(50);
}
```

---

## Notes

- **Gyro calibration:** Keep the device stationary during startup (~1 second) for accurate bias calibration.
- **Yaw drift:** The yaw angle will drift over time since there's no magnetometer for absolute heading. Consider adding a magnetometer (e.g., HMC5883L) if absolute heading is needed.
- **Complementary filter:** The `alpha` constant (0.98) balances gyro responsiveness vs. accelerometer noise rejection. Adjust if needed.
- If you want **real-world timestamps**, replace `millis()` with a real-time clock (RTC) or NTP sync (ESP32 WiFi).
- If you change baud rates, make sure **Nano and ESP32 match**.
- If you want faster readings, reduce delays but ensure sensors are not interfering.

---

## Required Libraries

| Library | Install via |
|---------|-------------|
| Adafruit_VL53L0X | Arduino Library Manager |
| MPU6050 | Arduino Library Manager (by Electronic Cats or jrowberg/i2cdevlib) |

---

## Quick Verification

When working correctly, the ESP32 serial output should look like:

```
{"ts_ms":1000,"tof_mm":41,"mpu":{"ax_g":0.012,"ay_g":-0.034,"az_g":1.002,"gx_dps":0.15,"gy_dps":-0.22,"gz_dps":0.08,"pitch":1.23,"roll":-0.45,"yaw":12.5},"ultra_l_mm":85,"ultra_c_mm":116,"ultra_r_mm":72}
{"ts_ms":1050,"tof_mm":37,"mpu":{"ax_g":0.010,"ay_g":-0.032,"az_g":1.001,"gx_dps":0.12,"gy_dps":-0.20,"gz_dps":0.06,"pitch":1.21,"roll":-0.44,"yaw":12.6},"ultra_l_mm":84,"ultra_c_mm":115,"ultra_r_mm":71}
```