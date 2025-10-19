// ============================================================
// WiFi and WebSocket Manager Implementation
// ============================================================

#include "wifi_manager.h"

WiFiManager* WiFiManager::instance = nullptr;

WiFiManager::WiFiManager() {
  webSocket = nullptr;
  lastBroadcast = 0;
  apStarted = false;
  instance = this;
}

bool WiFiManager::begin() {
  Serial.println("WiFi: Starting Access Point...");
  
  // Configure and start AP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ACCESS_POINT_IP, ACCESS_POINT_GATEWAY, ACCESS_POINT_SUBNET);
  
  bool success = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, false, MAX_WIFI_CONNECTIONS);
  
  if (!success) {
    Serial.println("WiFi: Failed to start Access Point");
    return false;
  }
  
  apStarted = true;
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("WiFi: AP started successfully\n");
  Serial.print("WiFi: SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("WiFi: IP Address: ");
  Serial.println(IP);
  Serial.print("WiFi: Web Interface: http://");
  Serial.println(IP);
  
  // Start WebSocket server
  webSocket = new WebSocketsServer(WEBSOCKET_PORT);
  webSocket->begin();
  webSocket->onEvent(webSocketEvent);
  
  Serial.println("WebSocket: Server started on port " + String(WEBSOCKET_PORT));
  
  return true;
}

void WiFiManager::webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (instance) {
    instance->handleWebSocketEvent(num, type, payload, length);
  }
}

void WiFiManager::handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      #if DEBUG_PRINT_WEBSOCKET_MESSAGES
      Serial.printf("WebSocket: Client #%u disconnected\n", num);
      #endif
      break;
      
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket->remoteIP(num);
        Serial.printf("WebSocket: Client #%u connected from %s\n", num, ip.toString().c_str());
      }
      break;
      
    case WStype_TEXT:
      #if DEBUG_PRINT_WEBSOCKET_MESSAGES
      Serial.printf("WebSocket: Received message from #%u: %s\n", num, payload);
      #endif
      
      // Handle commands from client
      String message = String((char*)payload);
      if (message == "ping") {
        webSocket->sendTXT(num, "pong");
      }
      break;
  }
}

void WiFiManager::update() {
  if (webSocket) {
    webSocket->loop();
  }
}

void WiFiManager::broadcastJSON(String json) {
  if (webSocket && isClientConnected()) {
    unsigned long now = millis();
    if (now - lastBroadcast >= WEBSOCKET_BROADCAST_INTERVAL_MS) {
      webSocket->broadcastTXT(json);
      lastBroadcast = now;
    }
  }
}

bool WiFiManager::isClientConnected() {
  return (webSocket && webSocket->connectedClients() > 0);
}

int WiFiManager::getConnectedClients() {
  return webSocket ? webSocket->connectedClients() : 0;
}