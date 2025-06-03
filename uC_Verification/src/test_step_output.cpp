#include <Arduino.h>

#ifdef TEST_STEP_OUTPUT

#define STEP_PIN0 14
#define STEP_PIN1 27
#define STEP_PIN2 26
#define STEP_PIN3 25

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN0, OUTPUT);
}

void loop() {
  static unsigned long lastPulseTime = 0;
  static int pulseCount = 0;
  unsigned long currentTime = micros();

  if (currentTime - lastPulseTime >= 1000) { // 1 kHz
    digitalWrite(STEP_PIN0, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN0, LOW);
    lastPulseTime = currentTime;
    pulseCount++;
    Serial.print("Pulse Count: ");
    Serial.println(pulseCount);
  }
}

#endif
