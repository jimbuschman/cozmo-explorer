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
