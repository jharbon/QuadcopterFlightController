#include "Gyroscope.hpp"
#include <Wire.h>

const int MPU6050_ADDRESS = 0x68;
const int MPU6050_POWER_MANAGEMENT_REGISTER = 0x6B;
const int MPU6050_POWER_MODE = 0x00;
const int MPU6050_CONFIG_REGISTER = 0x1A;
const int MPU6050_CONFIG_MODE = 0x05;  // Low-pass filter selected with cutoff frequency of 10Hz to avoid interference from motor vibrations

const int GYROSCOPE_CONFIG_REGISTER = 0x1B;
const int GYROSCOPE_CONFIG_MODE = 0x08;  // Set sensitivity scale factor to 65.5 LSB/deg/s
const float SENSITIVITY_SCALE_FACTOR = 65.5;
const int GYROSCOPE_MEASUREMENTS_FIRST_REGISTER = 0x43;

const int CALIBRATION_DURATION = 2000;  // Milliseconds

void Gyroscope::init() {
  startPowerMode();
  rollRateCalibration = pitchRateCalibration = yawRateCalibration = 0;
  measureCalibration();
}

void Gyroscope::startPowerMode() {
  // Start the MPU6050 in power mode
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_POWER_MANAGEMENT_REGISTER);
  Wire.write(MPU6050_POWER_MODE);
  Wire.endTransmission();
  delay(250);
}

void Gyroscope::measureCalibration() {
  // Determine calibration values for each of the roll, pitch and yaw rates 
  float rollRateSum = 0;
  float pitchRateSum = 0;
  float yawRateSum = 0;
  for (int i = 0; i < CALIBRATION_DURATION; i++) {
    updateAngularVels();  // Calibration values are zero whilst using this method here

    rollRateSum += rollRate;
    pitchRateSum += pitchRate;
    yawRateSum += yawRate;

    delay(1);
  }

  rollRateCalibration = rollRateSum/CALIBRATION_DURATION;
  pitchRateCalibration = pitchRateSum/CALIBRATION_DURATION;
  yawRateCalibration = yawRateSum/CALIBRATION_DURATION;
}

void Gyroscope::updateAngularVels() {
  configSignals();
  Wire.requestFrom(MPU6050_ADDRESS, 6); // Get data from the 6 registers corresponding to the gyroscope measurements about the x, y and z axes
  // Read data twice for each of x, y and z since each full measurement about a given axis is spread across two registers
  // For each measurement shift the first of two bytes 8 bits to the left and add the second byte to the right with | 
  int16_t lsbX = Wire.read() << 8 | Wire.read();
  int16_t lsbY = Wire.read() << 8 | Wire.read();
  int16_t lsbZ = Wire.read() << 8 | Wire.read();

  // Convert from LSB to deg/s and calibrate 
  rollRate = (float)lsbX/SENSITIVITY_SCALE_FACTOR - rollRateCalibration;
  pitchRate = (float)lsbY/SENSITIVITY_SCALE_FACTOR - pitchRateCalibration;
  yawRate = (float)lsbZ/SENSITIVITY_SCALE_FACTOR - yawRateCalibration;
}

void Gyroscope::configSignals(void) {
  // Turn on low-pass filter
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_CONFIG_REGISTER);
  Wire.write(MPU6050_CONFIG_MODE); 
  Wire.endTransmission();

  // Set sensitivity scale factor
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYROSCOPE_CONFIG_REGISTER);
  Wire.write(GYROSCOPE_CONFIG_MODE);  
  Wire.endTransmission();

  // Select the first register we will use for pulling gyroscope measurements from the MPU6050
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYROSCOPE_MEASUREMENTS_FIRST_REGISTER);
  Wire.endTransmission();
}

float Gyroscope::getRollRate() {
  return rollRate;
}

float Gyroscope::getPitchRate() {
  return pitchRate;
}

float Gyroscope::getYawRate() {
  return yawRate;
}
