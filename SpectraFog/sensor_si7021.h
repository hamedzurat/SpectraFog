// ============================================================
// SI7021 Temperature/Humidity Sensor Module
// ============================================================

#ifndef SENSOR_SI7021_H
#define SENSOR_SI7021_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

struct SI7021Data {
  float temperature_celsius;
  float temperature_fahrenheit;
  float humidity_percent;
  bool valid;
  unsigned long timestamp;
};

class SI7021Sensor {
private:
  TwoWire* wire;
  SI7021Data currentData;
  unsigned long lastUpdate;
  
  float readHumidity();
  float readTemperatureCelsius();
  float celsiusToFahrenheit(float celsius);
  bool checkI2CConnection();

public:
  SI7021Sensor(TwoWire* i2cBus);
  
  bool begin();
  void update();
  SI7021Data getData();
  bool isDataValid();
};

#endif // SENSOR_SI7021_H