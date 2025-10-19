// ============================================================
// PIR Motion Sensor Module
// ============================================================

#ifndef SENSOR_PIR_H
#define SENSOR_PIR_H

#include <Arduino.h>
#include "config.h"

struct PIRData {
  bool motion_detected;
  unsigned long motion_start_time;
  unsigned long motion_duration_ms;
  unsigned long timestamp;
};

class PIRSensor {
private:
  PIRData currentData;
  int lastState;
  unsigned long lastChangeTime;
  unsigned long lastUpdate;
  unsigned long fakeMotionTimer;

public:
  PIRSensor();
  
  bool begin();
  void update();
  PIRData getData();
  bool isMotionDetected();
};

#endif // SENSOR_PIR_H