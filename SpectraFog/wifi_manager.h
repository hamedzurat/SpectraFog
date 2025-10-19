// ============================================================
// WiFi and WebSocket Manager
// ============================================================

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "config.h"

class WiFiManager {
private:
  WebSocketsServer* webSocket;
  unsigned long lastBroadcast;
  bool apStarted;
  
  static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
  static WiFiManager* instance;

public:
  WiFiManager();
  
  bool begin();
  void update();
  void broadcastJSON(String json);
  bool isClientConnected();
  int getConnectedClients();
  
  void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
};

#endif // WIFI_MANAGER_H