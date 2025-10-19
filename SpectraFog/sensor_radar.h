// ============================================================
// HLK-LD2450 Radar Sensor Module
// ============================================================

#ifndef SENSOR_RADAR_H
#define SENSOR_RADAR_H

#include <Arduino.h>
#include "config.h"

struct RadarTarget {
  bool valid;
  int16_t x_mm;
  int16_t y_mm;
  int16_t velocity_cmps;
  unsigned long age_ms;
};

struct RadarData {
  RadarTarget targets[RADAR_MAX_TARGETS];
  unsigned long frames_received;
  unsigned long frames_valid;
  unsigned long timestamp;
};

class RadarSensor {
private:
  RadarData currentData;
  unsigned long lastUpdate;
  uint8_t buffer[128];
  int bufferLength;
  unsigned long lastSeenTime[RADAR_MAX_TARGETS];
  
  void pushBytes();
  void parseFrames();
  uint16_t readLittleEndianU16(const uint8_t* p);

public:
  RadarSensor();
  
  bool begin();
  void update();
  RadarData getData();
  int getActiveTargetCount();
};

#endif // SENSOR_RADAR_H