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
