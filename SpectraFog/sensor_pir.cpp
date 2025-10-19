// ============================================================
// PIR Motion Sensor Implementation
// ============================================================

#include "sensor_pir.h"

PIRSensor::PIRSensor() {
  currentData = {false, 0, 0, 0};
  lastState = LOW;
  lastChangeTime = 0;
  lastUpdate = 0;
  fakeMotionTimer = 0;
}

bool PIRSensor::begin() {
  #if !USE_FAKE_PIR_DATA
  pinMode(PIR_MOTION_PIN, INPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.println("PIR: Initialized successfully");
  #else
  Serial.println("PIR: Using FAKE DATA mode");
  #endif
  
  return true;
}

void PIRSensor::update() {
  unsigned long now = millis();
  if (now - lastUpdate < PIR_UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = now;
  
  #if USE_FAKE_PIR_DATA
  // Generate fake motion events every 5-10 seconds
  if (now - fakeMotionTimer > random(5000, 10000)) {
    currentData.motion_detected = !currentData.motion_detected;
    fakeMotionTimer = now;
    
    if (currentData.motion_detected) {
      currentData.motion_start_time = now;
      currentData.motion_duration_ms = 0;
    } else {
      currentData.motion_duration_ms = now - currentData.motion_start_time;
    }
  } else if (currentData.motion_detected) {
    currentData.motion_duration_ms = now - currentData.motion_start_time;
  }
  currentData.timestamp = now;
  #else
  int raw = digitalRead(PIR_MOTION_PIN);
  
  // Debounce
  if (raw != lastState && (now - lastChangeTime) > PIR_DEBOUNCE_TIME_MS) {
    lastState = raw;
    lastChangeTime = now;
    
    if (raw == HIGH) {
      currentData.motion_detected = true;
      currentData.motion_start_time = now;
      currentData.motion_duration_ms = 0;
      digitalWrite(STATUS_LED_PIN, HIGH);
      
      #if DEBUG_PRINT_SENSOR_DATA
      Serial.println("PIR: Motion DETECTED");
      #endif
    } else {
      currentData.motion_detected = false;
      currentData.motion_duration_ms = now - currentData.motion_start_time;
      digitalWrite(STATUS_LED_PIN, LOW);
      
      #if DEBUG_PRINT_SENSOR_DATA
      Serial.printf("PIR: Motion ended (duration: %lu ms)\n", currentData.motion_duration_ms);
      #endif
    }
  }
  
  currentData.timestamp = now;
  #endif
}

PIRData PIRSensor::getData() {
  return currentData;
}

bool PIRSensor::isMotionDetected() {
  return currentData.motion_detected;
}