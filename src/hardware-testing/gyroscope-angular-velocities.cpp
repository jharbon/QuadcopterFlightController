#include <Arduino.h>
#include <Wire.h>
#include "Gyroscope.hpp"

Gyroscope gyroscope;
float rollRate, pitchRate, yawRate;

void setup() {
  Serial.begin(38400);
  Wire.setClock(400000);  // Set I2C clock speed to 400kHz
  Wire.begin();
  gyroscope.init();
}

void loop() {
  gyroscope.updateAngularVels();
  rollRate = gyroscope.getRollRate();
  pitchRate = gyroscope.getPitchRate();
  yawRate = gyroscope.getYawRate();

  Serial.print("Roll rate: ");
  Serial.print(rollRate);
  Serial.println("deg/s");

  Serial.print("Pitch rate: ");
  Serial.print(pitchRate);
  Serial.println("deg/s");

  Serial.print("Yaw rate: ");
  Serial.print(yawRate);
  Serial.println("deg/s");

  delay(200);
}