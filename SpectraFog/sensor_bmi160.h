// ============================================================
// BMI160 Accelerometer & Gyroscope Sensor Module
// ============================================================

#ifndef SENSOR_BMI160_H
#define SENSOR_BMI160_H

#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_BMI160.h"
#include "config.h"

struct BMI160Data {
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_radps;
  float gyro_y_radps;
  float gyro_z_radps;
  bool valid;
  unsigned long timestamp;
};

class BMI160Sensor {
private:
  DFRobot_BMI160* bmi160;
  uint8_t i2c_address;
  BMI160Data currentData;
  unsigned long lastUpdate;
  
  float accel_lsb_per_g;
  float gyro_lsb_per_dps;
  
  bool i2cRead8(uint8_t reg, uint8_t &value);
  bool getAccelLsbPerG();
  bool getGyroLsbPerDPS();
  float generateFakeAccelValue(float base, float range);
  float generateFakeGyroValue(float range);

public:
  BMI160Sensor();
  
  bool begin();
  void update();
  BMI160Data getData();
  bool isDataValid();
};

#endif // SENSOR_BMI160_H