#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <chrono>

class Accelerometer
{
  private: 
  std::chrono::steady_clock::time_point prevTimeLogged;
  float accX, accY, accZ;
  float velZ;
  float roll, pitch;

  void startPowerMode();
  void configSignals();
  void updateAccs();
  void updateVelocity(float deltaT);
  void updateRollPitch();

  public:
  Accelerometer() {};
  void init();
  void update();
  float getAccX();
  float getAccY();
  float getAccZ();
  float getVelZ();
  float getRoll();
  float getPitch();
};

#endif