// ============================================================
// HC-SR04 Ultrasonic Distance Sensor Module
// ============================================================

#ifndef SENSOR_ULTRASONIC_H
#define SENSOR_ULTRASONIC_H

#include <Arduino.h>
#include "config.h"

struct UltrasonicData {
  float distance_cm;
  unsigned long raw_us;
  bool valid;
  unsigned long timestamp;
};

class UltrasonicSensor {
private:
  UltrasonicData currentData;
  unsigned long lastUpdate;
  
  unsigned long measureEchoOnce();
  float microsecondsToCm(unsigned long us);
  float aggregateDistanceCm(int samples, unsigned long &lastRawUs, bool &anyValid);

public:
  UltrasonicSensor();
  
  bool begin();
  void update();
  UltrasonicData getData();
  bool isDataValid();
};

#endif // SENSOR_ULTRASONIC_H