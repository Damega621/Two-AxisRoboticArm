
#ifdef TEST_POWER

#include <Arduino.h>

#define LED_BUILTIN 2

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);  // Onboard LED
}

void loop() {
  Serial.println("µC ACTIVE");
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
}

#endif
