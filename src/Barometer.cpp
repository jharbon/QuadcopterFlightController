#include "Barometer.hpp"
#include <Wire.h>
#include <cmath>

const int BMP280_ADDRESS = 0x76;
const int BMP280_CONTROL_MEASUREMENT_REGISTER = 0xF4;
const int BMP280_CONTROL_MEASUREMENT_MODE = 0x57;  // Normal power mode, x16 pressure oversampling and x2 temperature oversampling
const int BMP280_CONFIG_REGISTER = 0xF5;
const int BMP280_CONFIG_MODE = 0x10;  // SPI config set to 0 (irrelevant because we are using I2C), Infinite Impulse Response (IIR) coefficient = 16 and t-standby = 0.5ms
const int BMP280_COMPENSATION_PARAMETERS_FIRST_REGISTER = 0x88;
const int BMP280_MEASUREMENTS_FIRST_REGISTER = 0xF7;

const float CELSIUS_TO_KELVIN = 273.15;
const float AIR_AVERAGE_MOLECULAR_MASS = 0.02896;  // kg/mol
const float GAS_CONSTANT = 8.3144598;  // kg⋅(m^2)/((s^2)⋅K⋅mol)
const float GRAVITATIONAL_ACC = 9.81;  // m/s^2

void Barometer::init() {
    configSignals();

    // Estimate initial pressure value (at zero altitude) to be used in the barometric formula
    RawMeasurements rawMeasurements = measureRawValues();
    // Compensate temperature first so that the fine temperature value is initialised
    temperature = compensateTemperature(rawMeasurements.adcT);
    pressure = pressureInit = compensatePressure(rawMeasurements.adcP);
    altitude = 0.0;
}

void Barometer::configSignals() {
    // Set values for power mode, oversampling, IIR coefficient etc...
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_CONTROL_MEASUREMENT_REGISTER);
    Wire.write(BMP280_CONTROL_MEASUREMENT_MODE);
    Wire.endTransmission();
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_CONFIG_REGISTER);
    Wire.write(BMP280_CONFIG_MODE);
    Wire.endTransmission();

    delay(250);

    // Get compensation parameters for the calibration of temperature and pressure measurements
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_COMPENSATION_PARAMETERS_FIRST_REGISTER);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS, 24);
    digT1 = Wire.read() | Wire.read() << 8;
    digT2 = Wire.read() | Wire.read() << 8;
    digT3 = Wire.read() | Wire.read() << 8;
    digP1 = Wire.read() | Wire.read() << 8;
    digP2 = Wire.read() | Wire.read() << 8;
    digP3 = Wire.read() | Wire.read() << 8;
    digP4 = Wire.read() | Wire.read() << 8;
    digP5 = Wire.read() | Wire.read() << 8;
    digP6 = Wire.read() | Wire.read() << 8;
    digP7 = Wire.read() | Wire.read() << 8;
    digP8 = Wire.read() | Wire.read() << 8;
    digP9 = Wire.read() | Wire.read() << 8;
}

RawMeasurements Barometer::measureRawValues() {
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_MEASUREMENTS_FIRST_REGISTER);
    Wire.endTransmission();

    Wire.requestFrom(BMP280_ADDRESS, 6);
    uint8_t msbPress = Wire.read();
    uint8_t lsbPress = Wire.read();
    uint8_t xlsbPress = Wire.read();
    uint8_t msbTemp = Wire.read();
    uint8_t lsbTemp = Wire.read();
    uint8_t xlsbTemp = Wire.read();

    RawMeasurements rawMeasurements;
    // We need bits 7, 6, 5 and 4 (see the BMP280 data sheet) of each xlsb byte to specify bits 3, 2, 1 and 0 of the corresponding adc value
    rawMeasurements.adcT = msbTemp << 12 | lsbTemp << 4 | xlsbTemp >> 4;
    rawMeasurements.adcP = msbPress << 12 | lsbPress << 4 | xlsbPress >> 4;

    return rawMeasurements;
}

/* This method is almost identical to a function provided by Bosch Sensortec in the BMP280 data sheet. Temperature is first
compensated to degree Celsius with a resolution of 0.01, where an output value of e.g. 5123 equals 51.23 degree Celsius.
Temperature is returned in degree Kelvin */
double Barometer::compensateTemperature(signed long adcT) {
    signed long var1, var2, T;
    var1 = ((((adcT >> 3) - ((signed long)digT1 << 1))) * ((signed long)digT2)) >> 11;
    var2 = (((((adcT >> 4) - ((signed long)digT1)) * ((adcT >> 4) - ((signed long)digT1))) >> 12) * ((signed long)digT3)) >> 14;
    fineT = var1 + var2;
    T = (fineT * 5 + 128) >> 8;
    return (double)T/100 + CELSIUS_TO_KELVIN;  // Conversion to Celsius and then to Kelvin 
}

/* This method is almost identical to a function provided by Bosch Sensortec in the BMP280 data sheet
Pressure is first compensated to Pa as an unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits),
where an output value of e.g. 24674867 represents 24674867/256 = 96386.2 Pa = 963.862 hPa
*/
double Barometer::compensatePressure(signed long adcP) {
    signed long long var1, var2, P;
    var1 = ((signed long long)fineT) - 128000;
    var2 = var1 * var1 * (signed long long)digP6;
    var2 = var2 + ((var1 * (signed long long)digP5) << 17);
    var2 = var2 + (((signed long long)digP4) << 35);
    var1 = ((var1 * var1 * (signed long long)digP3) >> 8) + ((var1 * (signed long long)digP2) << 12);
    var1 = (((((signed long long)1) << 47) + var1)) * ((signed long long)digP1) >> 33;
    if (var1 == 0) {
        return 0;  // Avoid exception caused by division by zero
    }
    P = 1048576 - adcP;
    P = (((P<<31)-var2) * 3125) / var1;
    var1 = (((signed long long)digP9) * (P>>13) * (P >> 13)) >> 25;
    var2 = (((signed long long)digP8) * P) >> 19;
    P = ((P + var1 + var2) >> 8) + (((signed long long)digP7) << 4);
    return (double)P/256;  // Conversion to Pa
}

void Barometer::update() {
    RawMeasurements rawMeasurements = measureRawValues();
    // Compensate temperature first so that the fine temperature value is updated
    temperature = compensateTemperature(rawMeasurements.adcT);
    pressure = compensatePressure(rawMeasurements.adcP);

    updateAltitude();
}

void Barometer::updateAltitude() {
    // Rearranged the barometric formula to calculate altitude
    altitude = -(GAS_CONSTANT*temperature/(AIR_AVERAGE_MOLECULAR_MASS*GRAVITATIONAL_ACC)) * log(pressure/pressureInit);
}

double Barometer::getTemperature() {
    return temperature;
}

double Barometer::getPressure() {
    return pressure;
}

double Barometer::getAltitude() {
    return altitude;
}
