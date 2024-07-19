#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <chrono>

class Accelerometer
{
  private: 
  std::chrono::steady_clock::time_point prevTimeLogged;
  float accX, accY, accZ;  // g's
  float velZ;  // m/s
  float roll, pitch;  // Degrees

  void startPowerMode();
  void configSignals();
  void updateAccs();
  void updateVelocity(float deltaT);
  void updateRollPitch();

  public:
  Accelerometer() {};
  void init();
  void update();
  float calculateInertialAccZ();
  float getAccX();
  float getAccY();
  float getAccZ();
  float getVelZ();
  float getRoll();
  float getPitch();
};

#endif