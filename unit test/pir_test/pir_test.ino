// PIR Motion Sensor + ESP32 Debug Version
// Prints everything to Serial for debugging

const int PIR_PIN = 27;
const int LED_PIN = 2;

const unsigned long DEBOUNCE_MS = 50;

int lastState = LOW;
unsigned long lastChangeMs = 0;
unsigned long motionStartMs = 0;

void setup() {
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(2000); // give time for serial to attach
  Serial.println("\n=== PIR Motion Detector Debug Mode ===");
}

void loop() {
  int raw = digitalRead(PIR_PIN);
  unsigned long now = millis();

  // // Always print raw readings
  // Serial.print("[");
  // Serial.print(now);
  // Serial.print(" ms] RAW PIR = ");
  // Serial.println(raw);

  // Detect changes with debounce
  if (raw != lastState && (now - lastChangeMs) > DEBOUNCE_MS) {
    lastState = raw;
    lastChangeMs = now;

    if (raw == HIGH) {
      motionStartMs = now;
      Serial.print("[");
      Serial.print(now);
      Serial.println(" ms] >>> MOTION DETECTED (PIR=HIGH)");

      // LED ON (longer this time so you can see it)
      digitalWrite(LED_PIN, HIGH);
    } else {
      unsigned long duration = now - motionStartMs;
      Serial.print("[");
      Serial.print(now);
      Serial.print(" ms] <<< Motion ended (PIR=LOW), duration = ");
      Serial.print(duration);
      Serial.println(" ms");

      // LED OFF
      digitalWrite(LED_PIN, LOW);
    }
  }

  delay(200); // slower updates so logs are readable
}


// const int PIR_PIN = 27;
// const int LED_PIN = 2;

// void setup() {
//   pinMode(PIR_PIN, INPUT);
//   pinMode(LED_PIN, OUTPUT);
//   Serial.begin(115200);
//   delay(2000);
//   Serial.println("HC-SR501 Test: waiting for warm-up...");
// }

// void loop() {
//   int val = digitalRead(PIR_PIN);
//   digitalWrite(LED_PIN, val); // mirror PIR state
//   Serial.print("PIR=");
//   Serial.println(val);
//   delay(50);
// }
