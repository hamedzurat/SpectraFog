// ESP32 Ultrasonic Proximity Alert System with Web Server
// HC-SR04: TRIG->GPIO5, ECHO->GPIO18 (via voltage divider), VCC->5V, GND->GND
// Active Buzzer: I/O->GPIO25, VCC->3V3, GND->GND
// LED: GPIO2 (built-in on most ESP32 boards)

#include <WiFi.h>
#include <WebServer.h>

// ===================== USER CONFIGURATION =====================
// WiFi Access Point
const char* AP_SSID = "ESP32-AP";
const char* AP_PASS = "esp32pass";

// Pin assignments
const int PIN_TRIG = 5;
const int PIN_ECHO = 18;
const int BUZZER_PIN = 25;
const int LED_PIN = 2;

// Sensor physics
const unsigned long TIMEOUT_US = 30000;
const float SPEED_OF_SOUND_MS = 343.0f;  // m/s at ~20°C
const float SENSOR_MIN_CM = 2.0f;
const float SENSOR_MAX_CM = 400.0f;
const bool CLAMP_TO_SPEC = true;

// Measurement settings
const int AGG_SAMPLES = 3;               // samples to average per reading
const unsigned int AGG_GAP_MS = 10;      // delay between samples
const unsigned long UPDATE_INTERVAL_MS = 150;  // how often to measure distance

// Alert behavior: beep interval based on distance
const float DISTANCE_MIN_CM = 10.0f;     // closest distance (continuous beeping)
const float DISTANCE_MAX_CM = 200.0f;    // farthest alert distance (slowest beeping)
const unsigned long BEEP_INTERVAL_MIN_MS = 50;    // continuous beeping at min distance
const unsigned long BEEP_INTERVAL_MAX_MS = 1000;  // 1 second beep at max distance
const bool USE_LINEAR_INTERPOLATION = true;  // false = exponential curve

// Startup beep pattern
const int STARTUP_SHORT_BEEPS = 3;
const unsigned int STARTUP_SHORT_ON_MS = 100;
const unsigned int STARTUP_SHORT_OFF_MS = 100;
const unsigned int STARTUP_LONG_BEEP_MS = 500;
const unsigned int STARTUP_PAUSE_MS = 300;

// Serial output
const bool ENABLE_SERIAL_OUTPUT = true;
const int SERIAL_BAUD = 115200;
const int DISTANCE_DECIMALS = 1;  // decimal places for distance display

// Web server
const bool ENABLE_WEB_SERVER = true;
const int WEB_SERVER_PORT = 80;

// =============================================================

WebServer server(WEB_SERVER_PORT);

// Shared state
volatile float currentDistance = 0.0f;
volatile bool distanceValid = false;
volatile unsigned long currentBeepInterval = 0;
volatile unsigned long totalMeasurements = 0;
volatile unsigned long validMeasurements = 0;
volatile unsigned long webHits = 0;

unsigned long lastUpdateTime = 0;
unsigned long lastBeepTime = 0;
bool beepState = false;

// Distance history for graph (circular buffer)
const int HISTORY_SIZE = 50;
float distanceHistory[HISTORY_SIZE];
int historyIndex = 0;
int historyCount = 0;

void addToHistory(float dist) {
  distanceHistory[historyIndex] = dist;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

// Measure echo time
unsigned long measureEchoOnce() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, TIMEOUT_US);
}

// Convert microseconds to centimeters
float microsecondsToCm(unsigned long us) {
  if (us == 0) return NAN;
  const float cm_per_us = (SPEED_OF_SOUND_MS * 0.0001f);
  float d = (us * cm_per_us) / 2.0f;
  if (CLAMP_TO_SPEC) {
    if (d < SENSOR_MIN_CM) d = SENSOR_MIN_CM;
    if (d > SENSOR_MAX_CM) d = SENSOR_MAX_CM;
  }
  return d;
}

// Get average distance from multiple samples
float getAverageDistance() {
  float sum = 0.0f;
  int validCount = 0;

  for (int i = 0; i < AGG_SAMPLES; ++i) {
    unsigned long us = measureEchoOnce();
    float d = microsecondsToCm(us);
    if (!isnan(d)) {
      sum += d;
      validCount++;
    }
    if (i < AGG_SAMPLES - 1) delay(AGG_GAP_MS);
  }

  if (validCount == 0) return NAN;
  return sum / validCount;
}

// Calculate beep interval based on distance (interpolated)
unsigned long calculateBeepInterval(float distance) {
  if (isnan(distance)) return 0;  // no beep if invalid
  
  // Outside alert range
  if (distance > DISTANCE_MAX_CM) return 0;
  if (distance < DISTANCE_MIN_CM) distance = DISTANCE_MIN_CM;
  
  // Interpolate beep interval between min and max
  float t = (distance - DISTANCE_MIN_CM) / (DISTANCE_MAX_CM - DISTANCE_MIN_CM);
  t = constrain(t, 0.0f, 1.0f);
  
  unsigned long interval;
  if (USE_LINEAR_INTERPOLATION) {
    interval = BEEP_INTERVAL_MIN_MS + (unsigned long)(t * (BEEP_INTERVAL_MAX_MS - BEEP_INTERVAL_MIN_MS));
  } else {
    float exp_t = t * t;
    interval = BEEP_INTERVAL_MIN_MS + (unsigned long)(exp_t * (BEEP_INTERVAL_MAX_MS - BEEP_INTERVAL_MIN_MS));
  }
  
  return interval;
}

// Handle beep timing (non-blocking)
void handleBeeping() {
  if (currentBeepInterval == 0) {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    beepState = false;
    return;
  }

  unsigned long currentTime = millis();
  if (currentTime - lastBeepTime >= currentBeepInterval) {
    beepState = !beepState;
    digitalWrite(BUZZER_PIN, beepState ? HIGH : LOW);
    digitalWrite(LED_PIN, beepState ? HIGH : LOW);
    lastBeepTime = currentTime;
  }
}

// Startup beep sequence
void startupBeeps() {
  for (int i = 0; i < STARTUP_SHORT_BEEPS; ++i) {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    delay(STARTUP_SHORT_ON_MS);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    if (i < STARTUP_SHORT_BEEPS - 1) delay(STARTUP_SHORT_OFF_MS);
  }
  delay(STARTUP_PAUSE_MS);
  
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  delay(STARTUP_LONG_BEEP_MS);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  delay(STARTUP_PAUSE_MS);
}

// Web server handler - auto-refreshing HTML page with graph
void handleRoot() {
  webHits++;
  unsigned long up_ms = millis();
  unsigned long up_s = up_ms / 1000UL;

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>ESP32 Ultrasonic</title>";
  html += "<meta http-equiv='refresh' content='1'>";
  html += "<style>";
  html += "body{font-family:monospace;background:#1a1a1a;color:#0f0;padding:20px;margin:0}";
  html += "h1{color:#0f0;border-bottom:2px solid #0f0;padding-bottom:10px}";
  html += "h2{color:#0f0;margin-top:25px}";
  html += ".status{font-size:1.2em;line-height:1.8}";
  html += ".distance{font-size:2em;color:#00ff00;font-weight:bold}";
  html += ".error{color:#ff3333}";
  html += ".on{color:#00ff00}";
  html += ".off{color:#666}";
  html += "hr{border:1px solid #0f0;margin:20px 0}";
  html += "canvas{border:1px solid #0f0;background:#0a0a0a;display:block;margin:20px 0;max-width:100%}";
  html += "</style></head><body>";
  
  html += "<h1>ESP32 Ultrasonic Proximity Alert</h1>";
  
  html += "<h2>CURRENT STATUS</h2>";
  html += "<div class='status'>";
  if (distanceValid) {
    html += "Distance: <span class='distance'>" + String(currentDistance, DISTANCE_DECIMALS) + " cm</span><br>";
    html += "Beep Interval: ";
    if (currentBeepInterval == 0) {
      html += "<span class='off'>OFF (out of range)</span><br>";
    } else {
      html += "<span class='on'>" + String(currentBeepInterval) + " ms</span><br>";
    }
  } else {
    html += "Distance: <span class='error'>ERROR - No valid reading</span><br>";
    html += "Beep Interval: <span class='off'>OFF</span><br>";
  }
  html += "</div>";
  
  // Distance history graph
  html += "<h2>DISTANCE HISTORY</h2>";
  html += "<canvas id='graph' width='800' height='300'></canvas>";
  html += "<script>";
  html += "const canvas=document.getElementById('graph');";
  html += "const ctx=canvas.getContext('2d');";
  html += "const w=canvas.width;const h=canvas.height;";
  html += "const data=[";
  
  // Add history data
  if (historyCount > 0) {
    int startIdx = (historyIndex - historyCount + HISTORY_SIZE) % HISTORY_SIZE;
    for (int i = 0; i < historyCount; i++) {
      int idx = (startIdx + i) % HISTORY_SIZE;
      if (i > 0) html += ",";
      html += String(distanceHistory[idx], 1);
    }
  }
  
  html += "];";
  html += "const maxDist=" + String(DISTANCE_MAX_CM, 0) + ";";
  html += "const minDist=" + String(DISTANCE_MIN_CM, 0) + ";";
  
  // Draw graph
  html += "ctx.fillStyle='#0a0a0a';ctx.fillRect(0,0,w,h);";
  
  // Draw grid lines
  html += "ctx.strokeStyle='#1a3a1a';ctx.lineWidth=1;";
  html += "for(let i=0;i<=4;i++){";
  html += "let y=h/4*i;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}";
  
  // Draw distance zones as background
  html += "const zoneY1=h*(1-" + String(DISTANCE_MIN_CM) + "/maxDist);";
  html += "const zoneY2=h*(1-50/maxDist);";
  html += "const zoneY3=h*(1-100/maxDist);";
  html += "ctx.fillStyle='rgba(255,0,0,0.1)';ctx.fillRect(0,zoneY1,w,h-zoneY1);";
  html += "ctx.fillStyle='rgba(255,255,0,0.1)';ctx.fillRect(0,zoneY2,w,zoneY1-zoneY2);";
  html += "ctx.fillStyle='rgba(0,255,0,0.1)';ctx.fillRect(0,0,w,zoneY3);";
  
  // Draw data line
  html += "if(data.length>1){";
  html += "ctx.strokeStyle='#0f0';ctx.lineWidth=2;ctx.beginPath();";
  html += "for(let i=0;i<data.length;i++){";
  html += "let x=w/(data.length-1)*i;";
  html += "let y=h*(1-data[i]/maxDist);";
  html += "if(i==0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}";
  html += "ctx.stroke();}";
  
  // Draw current point
  html += "if(data.length>0){";
  html += "let x=w;let y=h*(1-data[data.length-1]/maxDist);";
  html += "ctx.fillStyle='#0f0';ctx.beginPath();ctx.arc(x,y,4,0,2*Math.PI);ctx.fill();}";
  
  // Labels
  html += "ctx.fillStyle='#0f0';ctx.font='12px monospace';";
  html += "ctx.fillText(maxDist+'cm',5,15);";
  html += "ctx.fillText(minDist+'cm',5,h-5);";
  html += "ctx.fillText('History: '+data.length+' samples',w-150,15);";
  
  html += "</script>";
  
  html += "<hr>";
  html += "<h2>CONFIGURATION</h2>";
  html += "Alert Range: " + String(DISTANCE_MIN_CM, 0) + " - " + String(DISTANCE_MAX_CM, 0) + " cm<br>";
  html += "Beep Range: " + String(BEEP_INTERVAL_MIN_MS) + " - " + String(BEEP_INTERVAL_MAX_MS) + " ms<br>";
  html += "Interpolation: " + String(USE_LINEAR_INTERPOLATION ? "Linear" : "Exponential") + "<br>";
  html += "Update Rate: " + String(UPDATE_INTERVAL_MS) + " ms<br>";
  html += "Samples/Reading: " + String(AGG_SAMPLES) + "<br>";
  
  html += "<hr>";
  html += "<h2>STATISTICS</h2>";
  html += "Total Measurements: " + String(totalMeasurements) + "<br>";
  html += "Valid Measurements: " + String(validMeasurements) + "<br>";
  if (totalMeasurements > 0) {
    float success_rate = (validMeasurements * 100.0f) / totalMeasurements;
    html += "Success Rate: " + String(success_rate, 1) + "%<br>";
  }
  html += "Web Hits: " + String(webHits) + "<br>";
  html += "Uptime: " + String(up_s) + " s<br>";
  
  html += "<hr>";
  html += "<h2>SYSTEM INFO</h2>";
  html += "AP SSID: " + String(AP_SSID) + "<br>";
  html += "AP IP: " + WiFi.softAPIP().toString() + "<br>";
  html += "Chip: " + String(ESP.getChipModel()) + "<br>";
  html += "Cores: " + String(ESP.getChipCores()) + "<br>";
  html += "CPU Freq: " + String(getCpuFrequencyMhz()) + " MHz<br>";
  html += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes<br>";
  
  html += "<hr>";
  html += "<p style='color:#666;font-size:0.9em'>Auto-refreshing every 1 second...</p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  if (ENABLE_SERIAL_OUTPUT) {
    Serial.print("AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP PASS: ");
    Serial.println(AP_PASS);
    Serial.print("AP IP:   ");
    Serial.println(WiFi.softAPIP());
  }
}

void setup() {
  if (ENABLE_SERIAL_OUTPUT) {
    Serial.begin(SERIAL_BAUD);
  }
  
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  delay(1000);
  
  if (ENABLE_SERIAL_OUTPUT) {
    Serial.println("\n=== Ultrasonic Proximity Alert System ===");
    Serial.print("Alert range: ");
    Serial.print(DISTANCE_MIN_CM, 0);
    Serial.print(" - ");
    Serial.print(DISTANCE_MAX_CM, 0);
    Serial.println(" cm");
    Serial.print("Beep interval: ");
    Serial.print(BEEP_INTERVAL_MIN_MS);
    Serial.print(" - ");
    Serial.print(BEEP_INTERVAL_MAX_MS);
    Serial.println(" ms");
    Serial.print("Interpolation: ");
    Serial.println(USE_LINEAR_INTERPOLATION ? "Linear" : "Exponential");
  }
  
  if (ENABLE_WEB_SERVER) {
    startAP();
    server.on("/", handleRoot);
    server.begin();
    if (ENABLE_SERIAL_OUTPUT) {
      Serial.println("\nWeb server started:");
      Serial.print("  http://");
      Serial.println(WiFi.softAPIP());
    }
  }
  
  if (ENABLE_SERIAL_OUTPUT) {
    Serial.println("\nStarting...\n");
  }
  
  delay(1000);
  startupBeeps();
  
  lastUpdateTime = millis();
  lastBeepTime = millis();
}

void loop() {
  // Handle web server requests
  if (ENABLE_WEB_SERVER) {
    server.handleClient();
  }
  
  unsigned long currentTime = millis();
  
  // Update distance reading at set interval
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    totalMeasurements++;
    float distance = getAverageDistance();
    
    if (!isnan(distance)) {
      validMeasurements++;
      currentDistance = distance;
      distanceValid = true;
      currentBeepInterval = calculateBeepInterval(distance);
      
      // Add to history
      addToHistory(distance);
      
      if (ENABLE_SERIAL_OUTPUT) {
        Serial.print("Distance: ");
        Serial.print(distance, DISTANCE_DECIMALS);
        Serial.print(" cm | Beep: ");
        if (currentBeepInterval == 0) {
          Serial.println("OFF");
        } else {
          Serial.print(currentBeepInterval);
          Serial.println(" ms");
        }
      }
    } else {
      distanceValid = false;
      currentBeepInterval = 0;
      if (ENABLE_SERIAL_OUTPUT) {
        Serial.println("Distance: ERROR");
      }
    }
    
    lastUpdateTime = currentTime;
  }
  
  // Handle buzzer/LED timing
  handleBeeping();
}
