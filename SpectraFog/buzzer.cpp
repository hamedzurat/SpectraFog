// ============================================================
// Buzzer Control Implementation
// ============================================================

#include "buzzer.h"

Buzzer::Buzzer() {
  enabled = BUZZER_DEFAULT_ENABLED;
  isPlaying = false;
  playStartTime = 0;
  currentNoteDuration = 0;
}

bool Buzzer::begin() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Buzzer: Initialized successfully");
  return true;
}

void Buzzer::enable() {
  enabled = true;
  Serial.println("Buzzer: Enabled");
}

void Buzzer::disable() {
  enabled = false;
  noTone(BUZZER_PIN);
  Serial.println("Buzzer: Disabled");
}

bool Buzzer::isEnabled() {
  return enabled;
}

void Buzzer::playTone(int frequency, int duration_ms) {
  if (!enabled) return;
  
  tone(BUZZER_PIN, frequency, duration_ms);
  isPlaying = true;
  playStartTime = millis();
  currentNoteDuration = duration_ms;
}

void Buzzer::playBeep() {
  playTone(NOTE_A4, 100);
}

void Buzzer::playSuccess() {
  if (!enabled) return;
  tone(BUZZER_PIN, NOTE_C4, 100);
  delay(120);
  tone(BUZZER_PIN, NOTE_E4, 100);
  delay(120);
  tone(BUZZER_PIN, NOTE_G4, 200);
}

void Buzzer::playError() {
  if (!enabled) return;
  tone(BUZZER_PIN, NOTE_G4, 100);
  delay(120);
  tone(BUZZER_PIN, NOTE_E4, 100);
  delay(120);
  tone(BUZZER_PIN, NOTE_C4, 200);
}

void Buzzer::update() {
  if (isPlaying && (millis() - playStartTime > currentNoteDuration)) {
    isPlaying = false;
  }
}