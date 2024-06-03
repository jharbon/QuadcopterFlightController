#ifndef GYROSCOPE_H
#define GYROSCOPE_H

class Gyroscope 
{
  private: 
  float rollRate, pitchRate, yawRate;
  float rollRateCalibration, pitchRateCalibration, yawRateCalibration;

  void startPowerMode();
  void measureCalibration();
  void configSignals();

  public:
  Gyroscope() {};
  void init();
  void updateAngularVels();
  float getRollRate();
  float getPitchRate();
  float getYawRate();
};

#endif
