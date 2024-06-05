#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

class Accelerometer
{
  private: 
  float accX, accY, accZ;
  float roll, pitch;

  void startPowerMode();
  void configSignals();
  void updateAccs();

  public:
  Accelerometer() {};
  void init();
  void updateRollPitch();
  float getAccX();
  float getAccY();
  float getAccZ();
  float getRoll();
  float getPitch();
};

#endif