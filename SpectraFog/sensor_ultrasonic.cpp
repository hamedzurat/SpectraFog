// ============================================================
// HC-SR04 Ultrasonic Distance Sensor Implementation
// ============================================================

#include "sensor_ultrasonic.h"

UltrasonicSensor::UltrasonicSensor() {
  currentData = {0, 0, false, 0};
  lastUpdate = 0;
}

bool UltrasonicSensor::begin() {
  #if !USE_FAKE_ULTRASONIC_DATA
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  Serial.println("Ultrasonic: Initialized successfully");
  #else
  Serial.println("Ultrasonic: Using FAKE DATA mode");
  #endif
  
  return true;
}

unsigned long UltrasonicSensor::measureEchoOnce() {
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  return pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
}

float UltrasonicSensor::microsecondsToCm(unsigned long us) {
  if (us == 0) return NAN;
  
  const float cm_per_us = (ULTRASONIC_SPEED_OF_SOUND_MS * 0.0001);
  float d = (us * cm_per_us) / 2.0;
  
  if (d < ULTRASONIC_MIN_DISTANCE_CM) d = ULTRASONIC_MIN_DISTANCE_CM;
  if (d > ULTRASONIC_MAX_DISTANCE_CM) d = ULTRASONIC_MAX_DISTANCE_CM;
  
  return d;
}

float UltrasonicSensor::aggregateDistanceCm(int n, unsigned long &lastRawUs, bool &anyValid) {
  if (n <= 1) {
    lastRawUs = measureEchoOnce();
    anyValid = (lastRawUs > 0);
    return microsecondsToCm(lastRawUs);
  }
  
  const int MAX_N = 50;
  n = (n > MAX_N) ? MAX_N : n;
  float vals[MAX_N];
  int validCount = 0;
  
  for (int i = 0; i < n; ++i) {
    unsigned long us = measureEchoOnce();
    float d = microsecondsToCm(us);
    if (!isnan(d)) {
      vals[validCount++] = d;
      lastRawUs = us;
    }
    delay(ULTRASONIC_SAMPLE_GAP_MS);
  }
  
  anyValid = (validCount > 0);
  if (!anyValid) {
    lastRawUs = 0;
    return NAN;
  }
  
  if (!ULTRASONIC_USE_MEDIAN) {
    float sum = 0.0;
    for (int i = 0; i < validCount; ++i) sum += vals[i];
    return sum / validCount;
  }
  
  // Median - simple insertion sort
  for (int i = 1; i < validCount; ++i) {
    float key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      --j;
    }
    vals[j + 1] = key;
  }
  
  if (validCount % 2 == 1) {
    return vals[validCount / 2];
  } else {
    int k = validCount / 2;
    return (vals[k - 1] + vals[k]) * 0.5;
  }
}

void UltrasonicSensor::update() {
  unsigned long now = millis();
  if (now - lastUpdate < ULTRASONIC_UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = now;
  
  #if USE_FAKE_ULTRASONIC_DATA
  currentData.distance_cm = 10.0 + (random(0, 2000) / 10.0);
  currentData.raw_us = currentData.distance_cm * 58;
  currentData.valid = true;
  currentData.timestamp = now;
  #else
  unsigned long raw_us = 0;
  bool valid = false;
  float dist_cm = aggregateDistanceCm(ULTRASONIC_AGGREGATE_SAMPLES, raw_us, valid);
  
  currentData.distance_cm = dist_cm;
  currentData.raw_us = raw_us;
  currentData.valid = valid;
  currentData.timestamp = now;
  #endif
  
  #if DEBUG_PRINT_SENSOR_DATA
  if (currentData.valid) {
    Serial.printf("Ultrasonic: Distance=%.1f cm\n", currentData.distance_cm);
  }
  #endif
}

UltrasonicData UltrasonicSensor::getData() {
  return currentData;
}

bool UltrasonicSensor::isDataValid() {
  return currentData.valid;
}