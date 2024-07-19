#include <Arduino.h>
#include <Wire.h>
#include "Accelerometer.hpp"

Accelerometer accelerometer;
float velZ;

void setup() {
    Serial.begin(38400);
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    Wire.begin();
    accelerometer.init();
}

void loop() {
    accelerometer.update();
    velZ = accelerometer.getVelZ();

    Serial.print("Velocity Z: ");
    Serial.print(velZ*100);
    Serial.println("cm/s");

    delay(100);
}