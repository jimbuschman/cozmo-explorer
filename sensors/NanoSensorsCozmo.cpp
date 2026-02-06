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

  // Reduced delay - the 30ms between sensor reads + serial transmit time
  // already creates ~150ms cycle. Keep short to feed ESP32 more often.
  delay(20);
}
