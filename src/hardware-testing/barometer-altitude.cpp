#include <Arduino.h>
#include <Wire.h>
#include "Barometer.hpp"
#include <vector>
#include <numeric>

Barometer barometer;
double temperature;
double pressure;
double altitude;
int i = 0;
std::vector<double> altitudeValues;
double altitudeRollingAvg;

void setup() {
    Serial.begin(38400);
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    Wire.begin();
    barometer.init();
}

void loop() {
    barometer.update();
    temperature = barometer.getTemperature();
    pressure = barometer.getPressure();
    altitude = barometer.getAltitude();
    if (altitudeValues.size() < 5) {
        // We want 5 values before we start updating the rolling average
        altitudeValues.push_back(altitude);
    }
    else {
        // Get rolling average for the previous 5 altitude values and then update the vector with the latest value
        altitudeRollingAvg = std::accumulate(altitudeValues.begin(), altitudeValues.end(), 0.0) / altitudeValues.size();
        altitudeValues[i % 5] = altitude;
        i++;
    }
    
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("K");

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println("Pa");

    Serial.print("Altitude: ");
    Serial.print(altitude*100);
    Serial.println("cm");

    /* The altitude measurement is noisy so the rolling average is helpful to check if we are getting the right
    'average behaviour' i.e. altitude rolling average increasing when the barometer moves vertically up and 
    decreasing when the barometer moves vertically down */
    Serial.print("Altitude rolling average: ");
    Serial.print(altitudeRollingAvg*100);
    Serial.println("cm");
    
    delay(200);
}
