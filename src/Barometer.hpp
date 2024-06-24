#ifndef BAROMETER_H
#define BAROMETER_H

struct RawMeasurements {
    signed long adcT;
    signed long adcP;
};

class Barometer {
  private: 
  // Temperature compensation parameters
  unsigned short digT1;
  signed short digT2;
  signed short digT3;
  signed long fineT;
  // Pressure compensation parameters
  unsigned short digP1;
  signed short digP2; 
  signed short digP3; 
  signed short digP4; 
  signed short digP5; 
  signed short digP6; 
  signed short digP7; 
  signed short digP8;
  signed short digP9;

  double temperature;  // K
  double pressureInit;  // Pa
  double pressure;  // Pa
  double altitude;  // m

  void configSignals();
  RawMeasurements measureRawValues();
  double compensateTemperature(signed long adcT);
  double compensatePressure(signed long adcP);
  void updateAltitude();

  public:
  Barometer() {};
  void init();

  void update();
  double getTemperature();
  double getPressure();
  double getAltitude();
};

#endif
