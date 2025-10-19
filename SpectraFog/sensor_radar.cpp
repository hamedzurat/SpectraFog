// ============================================================
// HLK-LD2450 Radar Sensor Implementation
// ============================================================

#include "sensor_radar.h"

const uint8_t FRAME_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t FRAME_TAIL[2] = {0x55, 0xCC};

RadarSensor::RadarSensor() {
  bufferLength = 0;
  lastUpdate = 0;
  currentData.frames_received = 0;
  currentData.frames_valid = 0;
  currentData.timestamp = 0;
  
  for (int i = 0; i < RADAR_MAX_TARGETS; i++) {
    currentData.targets[i].valid = false;
    currentData.targets[i].x_mm = 0;
    currentData.targets[i].y_mm = 0;
    currentData.targets[i].velocity_cmps = 0;
    currentData.targets[i].age_ms = 0;
    lastSeenTime[i] = 0;
  }
}

bool RadarSensor::begin() {
  #if !USE_FAKE_RADAR_DATA
  Serial2.begin(RADAR_BAUD_RATE, SERIAL_8N1, RADAR_UART_RX_PIN, RADAR_UART_TX_PIN);
  Serial.println("Radar: Initialized successfully");
  #else
  Serial.println("Radar: Using FAKE DATA mode");
  #endif
  
  return true;
}

uint16_t RadarSensor::readLittleEndianU16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

void RadarSensor::pushBytes() {
  while (Serial2.available() && bufferLength < (int)sizeof(buffer)) {
    buffer[bufferLength++] = (uint8_t)Serial2.read();
  }
}

void RadarSensor::parseFrames() {
  while (true) {
    if (bufferLength < RADAR_FRAME_SIZE) return;
    
    // Find header
    int headerPos = -1;
    for (int i = 0; i <= bufferLength - RADAR_FRAME_SIZE; i++) {
      if (buffer[i] == FRAME_HEADER[0] && 
          buffer[i+1] == FRAME_HEADER[1] && 
          buffer[i+2] == FRAME_HEADER[2] && 
          buffer[i+3] == FRAME_HEADER[3]) {
        headerPos = i;
        break;
      }
    }
    
    if (headerPos < 0) {
      int drop = bufferLength - 29;
      if (drop > 0) {
        memmove(buffer, buffer + drop, bufferLength - drop);
        bufferLength -= drop;
      }
      return;
    }
    
    if (headerPos + RADAR_FRAME_SIZE > bufferLength) {
      if (headerPos > 0) {
        memmove(buffer, buffer + headerPos, bufferLength - headerPos);
        bufferLength -= headerPos;
      }
      return;
    }
    
    uint8_t frame[RADAR_FRAME_SIZE];
    memcpy(frame, buffer + headerPos, RADAR_FRAME_SIZE);
    int take = headerPos + RADAR_FRAME_SIZE;
    memmove(buffer, buffer + take, bufferLength - take);
    bufferLength -= take;
    
    currentData.frames_received++;
    
    // Check tail
    if (frame[28] != FRAME_TAIL[0] || frame[29] != FRAME_TAIL[1]) {
      continue;
    }
    
    unsigned long now = millis();
    
    // Parse 3 targets
    for (int k = 0; k < RADAR_MAX_TARGETS; k++) {
      int base = 4 + k * 8;
      uint16_t xr = readLittleEndianU16(&frame[base + 0]);
      uint16_t yr = readLittleEndianU16(&frame[base + 2]);
      uint16_t vr = readLittleEndianU16(&frame[base + 4]);
      
      // Decode per LD2450 protocol
      int16_t x = (xr & 0x8000) ? (int16_t)(xr - 32768) : (int16_t)(-(int32_t)xr);
      int16_t y = (int16_t)(yr - 32768);
      int16_t v = (vr & 0x8000) ? (int16_t)(-(int32_t)(vr - 32768)) : (int16_t)vr;
      
      bool nz = !(xr == 0 && yr == 0 && vr == 0);
      currentData.targets[k].valid = nz;
      currentData.targets[k].x_mm = x;
      currentData.targets[k].y_mm = y;
      currentData.targets[k].velocity_cmps = v;
      lastSeenTime[k] = now;
    }
    
    currentData.frames_valid++;
  }
}

void RadarSensor::update() {
  unsigned long now = millis();
  if (now - lastUpdate < RADAR_UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = now;
  
  #if USE_FAKE_RADAR_DATA
  // Generate fake radar targets
  for (int i = 0; i < RADAR_MAX_TARGETS; i++) {
    if (random(0, 100) < 30) {  // 30% chance of target
      currentData.targets[i].valid = true;
      currentData.targets[i].x_mm = random(-3000, 3000);
      currentData.targets[i].y_mm = random(500, 4000);
      currentData.targets[i].velocity_cmps = random(-100, 100);
      currentData.targets[i].age_ms = 0;
    } else {
      currentData.targets[i].valid = false;
    }
  }
  currentData.timestamp = now;
  #else
  pushBytes();
  parseFrames();
  
  // Update age for all targets
  for (int i = 0; i < RADAR_MAX_TARGETS; i++) {
    currentData.targets[i].age_ms = now - lastSeenTime[i];
  }
  
  currentData.timestamp = now;
  #endif
  
  #if DEBUG_PRINT_SENSOR_DATA
  int count = getActiveTargetCount();
  if (count > 0) {
    Serial.printf("Radar: %d active target(s)\n", count);
  }
  #endif
}

RadarData RadarSensor::getData() {
  return currentData;
}

int RadarSensor::getActiveTargetCount() {
  int count = 0;
  for (int i = 0; i < RADAR_MAX_TARGETS; i++) {
    if (currentData.targets[i].valid) count++;
  }
  return count;
}