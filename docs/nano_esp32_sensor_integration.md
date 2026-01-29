# Nano + ESP32 Sensor Integration (Ultrasonic + VL53L0X + MPU6050)

## Overview
This document describes the wiring and code for connecting:
- **Arduino Nano** (3x ultrasonic sensors)
- **ESP32-S3** (VL53L0X ToF sensor + MPU6050 IMU)
- **Serial communication between Nano and ESP32**
- **WiFi UDP communication from ESP32 to PC** (wireless mode)

The ESP32 collects data from its ToF sensor, IMU, and the Nano, then outputs a single JSON line per reading via:
1. USB Serial (for debugging/tethered mode)
2. WiFi UDP (for wireless operation)

---

## Network Topology

### Tethered Mode (Serial)
```
┌─────────────┐  USB   ┌─────────────┐  UART  ┌─────────────┐
│     PC      │◄──────►│    ESP32    │◄──────►│    Nano     │
│  (Python)   │ Serial │  ToF + IMU  │        │ Ultrasonics │
└─────────────┘        └─────────────┘        └─────────────┘
```

### Wireless Mode (WiFi UDP)
```
                         Home WiFi Network
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     │                     │
         ▼                     ▼                     ▼
┌─────────────────┐    ┌─────────────┐       ┌─────────────┐
│       PC        │    │    ESP32    │ UART  │    Nano     │
│  Built-in WiFi  │    │  ToF + IMU  │◄─────►│ Ultrasonics │
│  (Python UDP)   │    │    WiFi     │       │             │
└────────┬────────┘    └──────┬──────┘       └─────────────┘
         │                    │
         │    USB WiFi        │
         ▼    Adapter         │
┌─────────────────┐           │
│  Cozmo's WiFi   │◄──────────┘ (ESP32 sends UDP to PC)
│  192.168.1.1    │
└─────────────────┘
```

**Note:** In wireless mode, your PC needs two WiFi connections:
- Built-in WiFi → Home network (for ESP32 data + internet)
- USB WiFi adapter → Cozmo's network (for robot control)

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
| VL53L0X Pin | ESP32-S3 Pin | Notes |
|-------------|--------------|-------|
| VIN         | 3.3V         | **Do not use 5V** |
| GND         | GND          | |
| SDA         | GPIO8        | I2C data (shared) |
| SCL         | GPIO9        | I2C clock (shared) |
| XSHUT       | (optional)   | Only needed for power-cycle control |

#### MPU6050 pin mapping
| MPU6050 Pin | ESP32-S3 Pin | Notes |
|-------------|--------------|-------|
| VCC         | 3.3V         | Can also use 5V if module has regulator |
| GND         | GND          | |
| SDA         | GPIO8        | I2C data (shared with VL53L0X) |
| SCL         | GPIO9        | I2C clock (shared with VL53L0X) |
| INT         | (optional)   | For interrupt-driven reads |

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

## Nano Code

File: `sensors/NanoSensorsCozmo.cpp`

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

## ESP32 Code

File: `sensors/esp32Cozmo.cpp`

```cpp
// ESP32 Sensor Pod - VL53L0X ToF + MPU6050 IMU + Nano Ultrasonics
// Outputs JSON via:
//   1. USB Serial (for debugging/tethered mode)
//   2. WiFi UDP (for wireless operation)

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>

// =============================================================================
// CONFIGURATION - Edit these for your setup
// =============================================================================
const char* WIFI_SSID = "YOUR_WIFI_SSID";      // Your home WiFi network name
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";  // Your home WiFi password
const char* PC_IP = "192.168.1.50";            // Your PC's IP on the network
const int UDP_PORT = 5000;                      // UDP port to send to

const bool ENABLE_WIFI = true;   // Set false to disable WiFi (serial only)
const bool ENABLE_SERIAL = true; // Set false to disable serial output
// =============================================================================

Adafruit_VL53L0X lox;
MPU6050 mpu;
HardwareSerial NanoSerial(1);
WiFiUDP udp;

// --- Calibration & filtering ---
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float pitch = 0, roll = 0, yaw = 0;
unsigned long lastTime = 0;
const float alpha = 0.98;  // complementary filter constant

bool wifiConnected = false;

void setup() {
  Serial.begin(115200);
  delay(100);

  // VL53L0X (I2C on GPIO8=SDA, GPIO9=SCL for ESP32-S3)
  Wire.begin(8, 9);
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

  // WiFi setup
  if (ENABLE_WIFI) {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.println();
      Serial.print("WiFi connected! IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Sending UDP to: ");
      Serial.print(PC_IP);
      Serial.print(":");
      Serial.println(UDP_PORT);
    } else {
      Serial.println();
      Serial.println("WiFi connection failed - continuing with serial only");
    }
  }

  Serial.println("ESP32 Ready");

  // --- Calibrate gyro bias (simple average) ---
  Serial.println("Calibrating gyro - keep device still...");
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
  Serial.println("Calibration complete");

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

  // Build JSON string
  String json = "{\"ts_ms\":";
  json += millis();
  json += ",\"tof_mm\":";
  json += tofDist;

  json += ",\"mpu\":{";
  json += "\"ax_g\":"; json += String(ax_g, 3);
  json += ",\"ay_g\":"; json += String(ay_g, 3);
  json += ",\"az_g\":"; json += String(az_g, 3);
  json += ",\"gx_dps\":"; json += String(gx_dps, 3);
  json += ",\"gy_dps\":"; json += String(gy_dps, 3);
  json += ",\"gz_dps\":"; json += String(gz_dps, 3);
  json += ",\"pitch\":"; json += String(pitch, 3);
  json += ",\"roll\":"; json += String(roll, 3);
  json += ",\"yaw\":"; json += String(yaw, 3);
  json += "}";

  json += ",\"ultra_l_mm\":";
  json += L;
  json += ",\"ultra_c_mm\":";
  json += C;
  json += ",\"ultra_r_mm\":";
  json += R;
  json += "}";

  // Output via Serial (USB)
  if (ENABLE_SERIAL) {
    Serial.println(json);
  }

  // Output via WiFi UDP
  if (ENABLE_WIFI && wifiConnected) {
    udp.beginPacket(PC_IP, UDP_PORT);
    udp.print(json);
    udp.endPacket();
  }

  delay(50);
}
```

---

## Python Configuration

### config.py settings

```python
# External sensor settings (ESP32 pod)
# Mode: "serial" (USB tethered) or "udp" (WiFi wireless)
EXT_SENSOR_MODE = "serial"  # Change to "udp" for wireless operation

# Serial mode settings
EXT_SENSOR_PORT = "/dev/ttyUSB0"  # Linux default, use "COM3" etc on Windows
EXT_SENSOR_BAUD = 115200

# UDP mode settings (for WiFi)
EXT_SENSOR_UDP_PORT = 5000  # Port to listen on for ESP32 UDP packets
```

### Environment variable overrides

```bash
# Serial mode
EXT_SENSOR_MODE=serial EXT_SENSOR_PORT=COM3 python main.py

# UDP mode
EXT_SENSOR_MODE=udp EXT_SENSOR_UDP_PORT=5000 python main.py
```

---

## Setup Guide

### Tethered Mode (Serial)

1. Upload `NanoSensorsCozmo.cpp` to Arduino Nano
2. Upload `esp32Cozmo.cpp` to ESP32 (with `ENABLE_WIFI = false` or leave as-is)
3. Connect ESP32 to PC via USB
4. Set `EXT_SENSOR_MODE = "serial"` in `config.py`
5. Run `python main.py`

### Wireless Mode (WiFi UDP)

1. **Hardware setup:**
   - Get a USB WiFi adapter for your PC
   - Connect PC's built-in WiFi to your home network
   - Connect USB WiFi adapter to Cozmo's network

2. **Find your PC's IP address:**
   ```bash
   # Windows
   ipconfig

   # Linux/Mac
   ip addr
   ```
   Look for your home network adapter's IP (e.g., `192.168.1.50`)

3. **Configure ESP32:**
   Edit `sensors/esp32Cozmo.cpp`:
   ```cpp
   const char* WIFI_SSID = "YourHomeWiFi";
   const char* WIFI_PASS = "YourWiFiPassword";
   const char* PC_IP = "192.168.1.50";  // Your PC's IP
   const int UDP_PORT = 5000;
   const bool ENABLE_WIFI = true;
   ```

4. **Upload firmware:**
   - Upload `NanoSensorsCozmo.cpp` to Arduino Nano
   - Upload `esp32Cozmo.cpp` to ESP32

5. **Configure Python:**
   Set `EXT_SENSOR_MODE = "udp"` in `config.py`

6. **Run:**
   ```bash
   python main.py
   ```

---

## Testing

### Test Nano standalone
Connect Nano via USB and open Serial Monitor (9600 baud):
```
L:150 C:200 R:180
L:152 C:198 R:182
```

### Test ESP32 standalone (serial mode)
Connect ESP32 via USB and open Serial Monitor (115200 baud):
```
{"ts_ms":1000,"tof_mm":41,"mpu":{...},"ultra_l_mm":85,"ultra_c_mm":116,"ultra_r_mm":72}
```

### Test Python receiver (serial)
```bash
python -m perception.external_sensors COM3
# or
python -m perception.external_sensors /dev/ttyUSB0
```

### Test Python receiver (UDP)
```bash
python -m perception.external_sensors udp 5000
```

---

## Troubleshooting

### ESP32 won't connect to WiFi
- Check SSID and password are correct
- Make sure your router allows new devices
- Try moving closer to the router
- Check Serial Monitor for error messages

### PC not receiving UDP packets
- Verify PC IP address is correct in ESP32 code
- Check firewall isn't blocking UDP port 5000
- Make sure PC is on the same network as ESP32
- Try `netcat -ul 5000` to test UDP reception

### Ultrasonic readings are -1
- Check Nano → ESP32 wiring (D11 → GPIO16)
- Verify Nano is running and sending data
- Check baud rate matches (9600)

### Sensor readings are noisy
- Add capacitors near sensor power pins
- Use shorter wires
- Increase delay between readings

---

## Required Libraries

| Library | Platform | Install via |
|---------|----------|-------------|
| Adafruit_VL53L0X | ESP32 | Arduino Library Manager |
| MPU6050 | ESP32 | Arduino Library Manager (Electronic Cats or jrowberg/i2cdevlib) |
| SoftwareSerial | Nano | Built-in |
| WiFi | ESP32 | Built-in |
| WiFiUdp | ESP32 | Built-in |

---

## Notes

- **Gyro calibration:** Keep the device stationary during startup (~1 second) for accurate bias calibration.
- **Yaw drift:** The yaw angle will drift over time since there's no magnetometer for absolute heading.
- **Complementary filter:** The `alpha` constant (0.98) balances gyro responsiveness vs. accelerometer noise rejection.
- If you want faster readings, reduce delays but ensure sensors are not interfering.
- The ESP32 outputs at ~20Hz (50ms delay). Adjust if needed.
