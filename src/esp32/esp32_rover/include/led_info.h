#ifndef _LED_INFO_H_
#define _LED_INFO_H_

#include <Arduino.h>

#include <Arduino.h>
#include <Ticker.h>

#define BLINK_DURATION 50  // 50ms ON/OFF Dauer

enum LED_INFO_PATTERN : uint8_t {
    LED_IDLE = 0b00010000,

    // Dyna-Fehler
    LED_DYNA_NOSERVOS = 0b00010001,

    // I2C-Fehler
    LED_I2C_ERR = 0b00010011,

};


struct LedPattern {
  uint8_t pin;
  int blinks;
  uint8_t count;
  bool state;
  long offDelay;
  Ticker ticker;
  bool active;
};

// Zwei LEDs
LedPattern ledPatterns[2];

void toggleLEDPattern(int index) {
  LedPattern &p = ledPatterns[index];

  if (p.count < p.blinks * 2) {
    // Blinkphase: 50ms toggeln
    digitalWrite(p.pin, p.state ? LOW : HIGH);
    p.state = !p.state;
    p.count++;
    p.ticker.once_ms(BLINK_DURATION, toggleLEDPattern, index);
  } else {
    // Pausephase
    p.state = false;
    digitalWrite(p.pin, LOW);
    p.count = 0;
    p.ticker.once_ms(p.offDelay, toggleLEDPattern, index);
  }
}

void setLEDPattern(uint8_t pin1, uint8_t pin2, uint8_t pattern, long off_ms=1000) {
  // LED 1 konfigurieren
  pinMode(pin1, OUTPUT);
  ledPatterns[0] = {
    .pin = pin1,
    .blinks = __builtin_popcount(pattern & 0x0F),
    .count = 0,
    .state = false,
    .offDelay = off_ms,
    .ticker = Ticker(),
    .active = true
  };

  // LED 2 konfigurieren
  pinMode(pin2, OUTPUT);
  ledPatterns[1] = {
    .pin = pin2,
    .blinks = __builtin_popcount((pattern >> 4) & 0x0F),
    .count = 0,
    .state = false,
    .offDelay = off_ms,
    .ticker = Ticker(),
    .active = true
  };

  // Starte die Timer für beide LEDs
  for (int i = 0; i < 2; i++) {
    if (ledPatterns[i].blinks > 0) {
      ledPatterns[i].ticker.once_ms(0, toggleLEDPattern, i);  // sofort starten
    }
  }
}



#endif
