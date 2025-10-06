
// ESP32 Ultrasonic Proximity Alert System with Interpolated Beep Intervals
// HC-SR04: TRIG->GPIO5, ECHO->GPIO18 (via voltage divider), VCC->5V, GND->GND
// Active Buzzer: I/O->GPIO25, VCC->3V3, GND->GND
// LED: GPIO2 (built-in on most ESP32 boards)

// ===================== USER CONFIGURATION =====================
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

// =============================================================

unsigned long lastUpdateTime = 0;
unsigned long lastBeepTime = 0;
unsigned long beepInterval = 0;
bool beepState = false;

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
  // Closer = shorter interval (faster beeps)
  // Farther = longer interval (slower beeps)
  
  float t = (distance - DISTANCE_MIN_CM) / (DISTANCE_MAX_CM - DISTANCE_MIN_CM);
  t = constrain(t, 0.0f, 1.0f);
  
  unsigned long interval;
  if (USE_LINEAR_INTERPOLATION) {
    // Linear interpolation
    interval = BEEP_INTERVAL_MIN_MS + (unsigned long)(t * (BEEP_INTERVAL_MAX_MS - BEEP_INTERVAL_MIN_MS));
  } else {
    // Exponential interpolation (more dramatic changes when close)
    float exp_t = t * t;  // square for exponential feel
    interval = BEEP_INTERVAL_MIN_MS + (unsigned long)(exp_t * (BEEP_INTERVAL_MAX_MS - BEEP_INTERVAL_MIN_MS));
  }
  
  return interval;
}

// Handle beep timing (non-blocking)
void handleBeeping() {
  if (beepInterval == 0) {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    beepState = false;
    return;
  }

  unsigned long currentTime = millis();
  if (currentTime - lastBeepTime >= beepInterval) {
    beepState = !beepState;
    digitalWrite(BUZZER_PIN, beepState ? HIGH : LOW);
    digitalWrite(LED_PIN, beepState ? HIGH : LOW);
    lastBeepTime = currentTime;
  }
}

// Startup beep sequence
void startupBeeps() {
  // Short beeps
  for (int i = 0; i < STARTUP_SHORT_BEEPS; ++i) {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    delay(STARTUP_SHORT_ON_MS);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    if (i < STARTUP_SHORT_BEEPS - 1) delay(STARTUP_SHORT_OFF_MS);
  }
  delay(STARTUP_PAUSE_MS);
  
  // Long beep
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  delay(STARTUP_LONG_BEEP_MS);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  delay(STARTUP_PAUSE_MS);
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
    Serial.println("\nStarting...\n");
  }
  
  delay(1000);
  startupBeeps();
  
  lastUpdateTime = millis();
  lastBeepTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update distance reading at set interval
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    float distance = getAverageDistance();
    
    if (!isnan(distance)) {
      beepInterval = calculateBeepInterval(distance);
      
      if (ENABLE_SERIAL_OUTPUT) {
        Serial.print("Distance: ");
        Serial.print(distance, DISTANCE_DECIMALS);
        Serial.print(" cm | Beep interval: ");
        if (beepInterval == 0) {
          Serial.println("OFF (out of range)");
        } else {
          Serial.print(beepInterval);
          Serial.println(" ms");
        }
      }
    } else {
      beepInterval = 0;
      if (ENABLE_SERIAL_OUTPUT) {
        Serial.println("Distance: ERROR - No valid reading");
      }
    }
    
    lastUpdateTime = currentTime;
  }
  
  // Handle buzzer/LED timing
  handleBeeping();
}
