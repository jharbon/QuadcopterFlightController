#include <Arduino.h>
#include <Wire.h>
#include "Accelerometer.hpp"
#include "Barometer.hpp"
#include <vector>
#include <numeric>
#include <cmath>

int n = 0;
bool finished = false;
const float TIME_STEP = 0.1;  // Seconds

Accelerometer accelerometer;
float velZ;

Barometer barometer;
double altitude;
std::vector<double> altitudesVec;

void setup() {
    Serial.begin(38400);
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    Wire.begin();
    accelerometer.init();
    barometer.init();
}

void loop() {
    if (!finished) {
        accelerometer.update();
        velZ = accelerometer.getVelZ();

        barometer.update();
        altitude = barometer.getAltitude();
        altitudesVec.push_back(altitude);

        n++;

        if (n == 600) {
            // Determine how much the velocity drifts per unit time on average whilst the accelerometer remains stationary
            float driftNoise = velZ / (n*TIME_STEP);

            Serial.print("Velocity drift noise: ");
            Serial.print(driftNoise*100);
            Serial.println("cm/s/s");

            // Sample mean of altitudes
            float altitudesSum = 0;
            for (auto& a : altitudesVec) {
                altitudesSum += a;
            }
            float altitudesMean = altitudesSum / n;

            // Sample variance of altitudes
            float sumSquareDiff = 0;
            for (auto& a : altitudesVec) {
                sumSquareDiff += pow((a - altitudesMean), 2);
            }
            float altitudesVariance = sumSquareDiff / (n - 1);

            Serial.print("Altitude mean: ");
            Serial.print(altitudesMean*100);
            Serial.println("cm");

            Serial.print("Altitude variance: ");
            Serial.print(altitudesVariance*pow(100, 2));
            Serial.println("cm^2");

            finished = true;
        }

        delay(TIME_STEP*1000);
    }
}
