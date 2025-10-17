// ESP32 + passive piezo using tone
// Wiring: Buzzer + -> GPIO 25, Buzzer - -> GND
// Open Serial Monitor @ 115200

const int BUZZER_PIN = 25;

// Common note frequencies (A4 = 440 Hz)
#define C4 261
#define D4 294
#define E4 329
#define F4 349
#define G4 392
#define A4 440
#define B4 494
#define C5 523

struct Note {
  int freq;
  int dur_ms;
};

// Public-domain example: "Twinkle, Twinkle, Little Star"
Note melody[] = {
  { C4, 400 }, { C4, 400 }, { G4, 400 }, { G4, 400 }, { A4, 400 }, { A4, 400 }, { G4, 800 }, { F4, 400 }, { F4, 400 }, { E4, 400 }, { E4, 400 }, { D4, 400 }, { D4, 400 }, { C4, 800 }, { G4, 400 }, { G4, 400 }, { F4, 400 }, { F4, 400 }, { E4, 400 }, { E4, 400 }, { D4, 800 }, { G4, 400 }, { G4, 400 }, { F4, 400 }, { F4, 400 }, { E4, 400 }, { E4, 400 }, { D4, 800 }, { C4, 400 }, { C4, 400 }, { G4, 400 }, { G4, 400 }, { A4, 400 }, { A4, 400 }, { G4, 800 }, { F4, 400 }, { F4, 400 }, { E4, 400 }, { E4, 400 }, { D4, 400 }, { D4, 400 }, { C4, 800 }
};
const int N = sizeof(melody) / sizeof(melody[0]);

void setup() {
  Serial.begin(115200);
  Serial.println("Passive piezo + tone(): public-domain demo");
}

void loop() {
  for (int i = 0; i < N; i++) {
    int f = melody[i].freq;
    int d = melody[i].dur_ms;
    if (f > 0) {
      Serial.print("Playing ");
      Serial.print(f);
      Serial.print(" Hz for ");
      Serial.print(d);
      Serial.println(" ms");
      tone(BUZZER_PIN, f, d);
    }
    delay(d + 50);  // small gap
  }
  Serial.println("Loop again after 1 s...");
  delay(1000);
}
