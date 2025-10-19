// ============================================================
// ESP32 Multi-Sensor WebSocket Server
// Main Arduino Sketch
// ============================================================

#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#include "config.h"
#include "sensor_bmi160.h"
#include "sensor_si7021.h"
#include "sensor_pir.h"
#include "sensor_ultrasonic.h"
#include "sensor_radar.h"
#include "buzzer.h"
#include "wifi_manager.h"
#include "web_interface.h"

// ============================================================
// Global Objects
// ============================================================
TwoWire I2C_Bus2 = TwoWire(1);

BMI160Sensor* bmi160 = nullptr;
SI7021Sensor* si7021 = nullptr;
PIRSensor* pirSensor = nullptr;
UltrasonicSensor* ultrasonic = nullptr;
RadarSensor* radar = nullptr;
Buzzer* buzzer = nullptr;
WiFiManager* wifiManager = nullptr;
WebServer* webServer = nullptr;

unsigned long lastSensorUpdate = 0;

// ============================================================
// Function Prototypes
// ============================================================
void handleRoot();
void broadcastSensorData();

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(2000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   ESP32 Multi-Sensor System v1.0      ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize WiFi and WebSocket
  wifiManager = new WiFiManager();
  if (!wifiManager->begin()) {
    Serial.println("ERROR: Failed to start WiFi!");
    while (1) delay(1000);
  }
  
  // Initialize HTTP Web Server for serving HTML page
  webServer = new WebServer(HTTP_SERVER_PORT);
  webServer->on("/", HTTP_GET, handleRoot);
  webServer->begin();
  Serial.println("HTTP Server: Started on port " + String(HTTP_SERVER_PORT));
  
  // Initialize sensors based on config
  Serial.println("\n--- Initializing Sensors ---");
  
  #if ENABLE_BMI160_ACCELEROMETER
  bmi160 = new BMI160Sensor();
  bmi160->begin();
  #endif
  
  #if ENABLE_SI7021_TEMP_HUMIDITY
  si7021 = new SI7021Sensor(&I2C_Bus2);
  si7021->begin();
  #endif
  
  #if ENABLE_PIR_MOTION
  pirSensor = new PIRSensor();
  pirSensor->begin();
  #endif
  
  #if ENABLE_ULTRASONIC_DISTANCE
  ultrasonic = new UltrasonicSensor();
  ultrasonic->begin();
  #endif
  
  #if ENABLE_RADAR_LD2450
  radar = new RadarSensor();
  radar->begin();
  #endif
  
  #if ENABLE_BUZZER
  buzzer = new Buzzer();
  buzzer->begin();
  #endif
  
  Serial.println("\n--- Initialization Complete ---");
  Serial.println("System ready!\n");
  
  // Play startup sound
  #if ENABLE_BUZZER
  buzzer->enable();
  buzzer->playSuccess();
  delay(500);
  buzzer->disable();
  #endif
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
  // Update WiFi and WebSocket
  wifiManager->update();
  webServer->handleClient();
  
  // Update all sensors
  #if ENABLE_BMI160_ACCELEROMETER
  if (bmi160) bmi160->update();
  #endif
  
  #if ENABLE_SI7021_TEMP_HUMIDITY
  if (si7021) si7021->update();
  #endif
  
  #if ENABLE_PIR_MOTION
  if (pirSensor) pirSensor->update();
  #endif
  
  #if ENABLE_ULTRASONIC_DISTANCE
  if (ultrasonic) ultrasonic->update();
  #endif
  
  #if ENABLE_RADAR_LD2450
  if (radar) radar->update();
  #endif
  
  #if ENABLE_BUZZER
  if (buzzer) buzzer->update();
  #endif
  
  // Broadcast sensor data via WebSocket
  unsigned long now = millis();
  if (now - lastSensorUpdate >= WEBSOCKET_BROADCAST_INTERVAL_MS) {
    lastSensorUpdate = now;
    broadcastSensorData();
  }
}

// ============================================================
// HTTP Handler - Serve Web Page
// ============================================================
void handleRoot() {
  webServer->send_P(200, "text/html", WEB_PAGE);
}

// ============================================================
// Broadcast Sensor Data as JSON
// ============================================================
void broadcastSensorData() {
  if (!wifiManager->isClientConnected()) {
    return;
  }
  
  // Create JSON document
  StaticJsonDocument<2048> doc;
  
  // BMI160 Accelerometer & Gyroscope
  #if ENABLE_BMI160_ACCELEROMETER
  if (bmi160) {
    BMI160Data accelData = bmi160->getData();
    JsonObject bmi = doc.createNestedObject("bmi160");
    bmi["ax"] = accelData.accel_x_mps2;
    bmi["ay"] = accelData.accel_y_mps2;
    bmi["az"] = accelData.accel_z_mps2;
    bmi["gx"] = accelData.gyro_x_radps;
    bmi["gy"] = accelData.gyro_y_radps;
    bmi["gz"] = accelData.gyro_z_radps;
    bmi["valid"] = accelData.valid;
    bmi["timestamp"] = accelData.timestamp;
  }
  #endif
  
  // SI7021 Temperature/Humidity
  #if ENABLE_SI7021_TEMP_HUMIDITY
  if (si7021) {
    SI7021Data tempData = si7021->getData();
    JsonObject si = doc.createNestedObject("si7021");
    si["temp_c"] = tempData.temperature_celsius;
    si["temp_f"] = tempData.temperature_fahrenheit;
    si["humidity"] = tempData.humidity_percent;
    si["valid"] = tempData.valid;
    si["timestamp"] = tempData.timestamp;
  }
  #endif
  
  // PIR Motion Sensor
  #if ENABLE_PIR_MOTION
  if (pirSensor) {
    PIRData pirData = pirSensor->getData();
    JsonObject pir = doc.createNestedObject("pir");
    pir["motion"] = pirData.motion_detected;
    pir["start_time"] = pirData.motion_start_time;
    pir["duration"] = pirData.motion_duration_ms;
    pir["timestamp"] = pirData.timestamp;
  }
  #endif
  
  // Ultrasonic Distance Sensor
  #if ENABLE_ULTRASONIC_DISTANCE
  if (ultrasonic) {
    UltrasonicData distData = ultrasonic->getData();
    JsonObject ultra = doc.createNestedObject("ultrasonic");
    ultra["distance"] = distData.distance_cm;
    ultra["raw_us"] = distData.raw_us;
    ultra["valid"] = distData.valid;
    ultra["timestamp"] = distData.timestamp;
  }
  #endif
  
  // Radar Sensor
  #if ENABLE_RADAR_LD2450
  if (radar) {
    RadarData radarData = radar->getData();
    JsonObject radarObj = doc.createNestedObject("radar");
    radarObj["frames_total"] = radarData.frames_received;
    radarObj["frames_valid"] = radarData.frames_valid;
    radarObj["timestamp"] = radarData.timestamp;
    
    JsonArray targets = radarObj.createNestedArray("targets");
    for (int i = 0; i < RADAR_MAX_TARGETS; i++) {
      JsonObject t = targets.createNestedObject();
      t["valid"] = radarData.targets[i].valid;
      t["x"] = radarData.targets[i].x_mm;
      t["y"] = radarData.targets[i].y_mm;
      t["v"] = radarData.targets[i].velocity_cmps;
      t["age"] = radarData.targets[i].age_ms;
    }
  }
  #endif
  
  // System info
  JsonObject sys = doc.createNestedObject("system");
  sys["uptime"] = millis();
  sys["free_heap"] = ESP.getFreeHeap();
  sys["clients"] = wifiManager->getConnectedClients();
  
  // Serialize and broadcast
  String json;
  serializeJson(doc, json);
  wifiManager->broadcastJSON(json);
}