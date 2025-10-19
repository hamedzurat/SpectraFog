// ============================================================
// Buzzer Control Module
// ============================================================

#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include "config.h"

// Note frequencies
#define NOTE_C4 261
#define NOTE_D4 294
#define NOTE_E4 329
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523

struct BuzzerNote {
  int frequency;
  int duration_ms;
};

class Buzzer {
private:
  bool enabled;
  bool isPlaying;
  unsigned long playStartTime;
  int currentNoteDuration;

public:
  Buzzer();
  
  bool begin();
  void enable();
  void disable();
  bool isEnabled();
  void playTone(int frequency, int duration_ms);
  void playBeep();
  void playSuccess();
  void playError();
  void update();
};

#endif // BUZZER_H