// HC-SR04 Ultrasonic Sensor + ESP32 (CSV streamer, no time column)
// Wiring: TRIG->GPIO5, ECHO->GPIO18 (via voltage divider), VCC->5V, GND->GND
// LED indicator on GPIO2 (optional)

// ===================== User-Tunable Parameters =====================
// 1) Measurement & aggregation
const int PIN_TRIG = 5;
const int PIN_ECHO = 18;
const int LED_PIN = 2;  // onboard LED on many ESP32 dev boards

const unsigned long TIMEOUT_US = 30000;  // pulseIn timeout (us)
const unsigned long INTERVAL_MS = 200;   // period between outputs
const int AGG_SAMPLES = 5;               // how many readings to aggregate per row
const unsigned int AGG_GAP_MS = 10;      // delay between samples when aggregating
const bool USE_MEDIAN = false;           // true: median, false: mean of valid samples

// 2) Physics/precision knobs
const float SPEED_OF_SOUND_MS = 343.0f;  // m/s at ~20°C (raise slightly if hotter)
const int CSV_DECIMALS = 2;              // decimal places for distance_cm in CSV
const bool CLAMP_TO_SPEC = true;         // clamp distances to the spec range
const float SENSOR_MIN_CM = 2.0f;        // HC-SR04 spec
const float SENSOR_MAX_CM = 400.0f;      // HC-SR04 spec

// 3) LED thresholds (purely cosmetic)
const float CLOSE_THRESHOLD_CM = 20.0f;
const float MEDIUM_THRESHOLD_CM = 50.0f;
// ==================================================================

void printCSVRow(float dist_cm, unsigned long raw_us, bool valid) {
  // distance_cm
  if (isnan(dist_cm)) Serial.print("");
  else Serial.print(dist_cm, CSV_DECIMALS);
  Serial.print(",");
  // raw_us (empty if invalid)
  if (!valid) Serial.print("");
  else Serial.print(raw_us);
  Serial.print(",");
  Serial.println(valid ? 1 : 0);
}

// ---- Ultrasonic primitives ----
unsigned long measureEchoOnce() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, TIMEOUT_US);
}

float microsecondsToCm(unsigned long us) {
  if (us == 0) return NAN;
  // distance(cm) = (us * speed_of_sound_cm_per_us) / 2
  // speed_of_sound_cm_per_us = (SPEED_OF_SOUND_MS * 100.0) / 1e6
  const float cm_per_us = (SPEED_OF_SOUND_MS * 0.0001f);  // 343 m/s -> 0.0343 cm/us
  float d = (us * cm_per_us) / 2.0f;
  if (CLAMP_TO_SPEC) {
    if (d < SENSOR_MIN_CM) d = SENSOR_MIN_CM;
    if (d > SENSOR_MAX_CM) d = SENSOR_MAX_CM;
  }
  return d;
}

// ---- Aggregation (mean or median of valid samples) ----
float aggregateDistanceCm(int n, unsigned long &lastRawUs, bool &anyValid) {
  if (n <= 1) {
    lastRawUs = measureEchoOnce();
    anyValid = (lastRawUs > 0);
    return microsecondsToCm(lastRawUs);
  }

  // Collect samples
  const int MAX_N = 50;  // safety cap
  n = (n > MAX_N) ? MAX_N : n;
  float vals[MAX_N];
  unsigned long raws[MAX_N];
  int validCount = 0;

  for (int i = 0; i < n; ++i) {
    unsigned long us = measureEchoOnce();
    raws[i] = us;
    float d = microsecondsToCm(us);
    if (!isnan(d)) {
      vals[validCount++] = d;
      lastRawUs = us;  // keep last valid raw
    }
    delay(AGG_GAP_MS);
  }

  anyValid = (validCount > 0);
  if (!anyValid) {
    lastRawUs = 0;  // signify invalid
    return NAN;
  }

  if (!USE_MEDIAN) {
    // mean
    float sum = 0.0f;
    for (int i = 0; i < validCount; ++i) sum += vals[i];
    return sum / validCount;
  }

  // median (simple insertion sort due to small N)
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
    return (vals[k - 1] + vals[k]) * 0.5f;
  }
}

// ---- LED feedback ----
void updateLED(float distance) {
  if (isnan(distance)) {
    digitalWrite(LED_PIN, LOW);
    return;
  }
  if (distance < CLOSE_THRESHOLD_CM) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  } else if (distance < MEDIUM_THRESHOLD_CM) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

// ---- Setup / Loop ----
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(LED_PIN, LOW);

  delay(2000);
  Serial.println("distance_cm,raw_us,valid");
}

void loop() {
  unsigned long raw_us = 0;
  bool valid = false;
  float dist_cm = aggregateDistanceCm(AGG_SAMPLES, raw_us, valid);

  updateLED(dist_cm);
  printCSVRow(dist_cm, raw_us, valid);

  delay(INTERVAL_MS);
}
