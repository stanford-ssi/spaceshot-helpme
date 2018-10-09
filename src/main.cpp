#include "ADXL375.h"

#define LED_PIN SCL

ADXL375 accel;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  SerialUSB.begin(9600);
  accel.init();
}

void loop() {

  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);
  delay(100);

  AccelData xyz = accel.getXYZ();
  SerialUSB.println("X, Y, Z:");
  SerialUSB.println(xyz.x);
  SerialUSB.println(xyz.y);
  SerialUSB.println(xyz.z);
}
