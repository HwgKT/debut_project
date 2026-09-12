#include <Arduino.h>

#define LED_PIN PC13
#define BIN_PIN PA0
  bool ledState = false;
  bool lastBinState = HIGH;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BIN_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  bool binState = digitalRead(BIN_PIN);
  if (lastBinState == HIGH && binState == LOW) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LOW : HIGH);
      delay (100);
    }
    lastBinState = binState;
}