#include <Wire.h>

#define I2C_ADDR 0x12   // Nano I2C slave address

// Define pins for 3 SR04 sensors
#define TRIG1 2
#define ECHO1 3
#define TRIG2 4
#define ECHO2 5
#define TRIG3 6
#define ECHO3 7

long distances[3];  // store distances in cm

void setup() {
  Wire.begin(I2C_ADDR); // Nano as I2C slave
  Wire.onRequest(requestEvent); // callback when master requests data
  
  // SR04 pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  
  Serial.begin(9600);
  Serial.println("Nano with 3 SR04 sensors ready");
}

long readSR04(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  long distance = duration * 0.034 / 2;          // convert to cm
  return distance;
}

void loop() {
  // Read all 3 sensors
  distances[0] = readSR04(TRIG1, ECHO1);
  distances[1] = readSR04(TRIG2, ECHO2);
  distances[2] = readSR04(TRIG3, ECHO3);
  
  // Optional: print locally for debugging
  Serial.print("D1: "); Serial.print(distances[0]);
  Serial.print("  D2: "); Serial.print(distances[1]);
  Serial.print("  D3: "); Serial.println(distances[2]);
  
  delay(100); // small delay
}

// Callback when ESP32 requests data
void requestEvent() {
  for (int i = 0; i < 3; i++) {
    Wire.write((byte)distances[i]); // send each distance as a byte
  }
}
