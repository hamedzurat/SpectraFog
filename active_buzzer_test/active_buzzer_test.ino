// ESP32 + 3-wire Active Buzzer Module (GND, I/O, VCC)
// Wiring: VCC->3V3, GND->GND, I/O->GPIO 25
// Built-in LED on GPIO 2 blinks with buzzer

const int BUZZER_PIN = 25;
const int LED_PIN = 2;

void beepBurst(unsigned onMs, unsigned offMs, int times) {
  for (int i = 0; i < times; ++i) {
    // ON
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);

    delay(onMs);

    // OFF
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    if (i < times - 1) delay(offMs);
  }
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  // Startup: three quick beeps
  beepBurst(120, 120, 3);
  delay(300);
  beepBurst(500, 0, 1);
}

void loop() {
  // Pattern: 3 short beeps, pause, 1 long beep, long pause
  beepBurst(100, 100, 3);
  delay(400);
  beepBurst(400, 0, 1);
  delay(1200);
}
