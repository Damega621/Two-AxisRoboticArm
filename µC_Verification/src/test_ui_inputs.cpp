#ifdef TEST_UI_INPUTS

#include <Arduino.h>

#define BUTTON1_PIN 14
#define ESTOP_PIN 27
#define LED_PIN 2

bool prev_button_state = HIGH;
bool prev_estop_state = HIGH;
volatile bool estop_triggered = false;

void IRAM_ATTR handleEstop() {
  estop_triggered = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(ESTOP_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), handleEstop, FALLING);
  Serial.println("UI_µC_dsig Input Monitoring Initialized");
}

void loop() {
  // === Live Logic Level Reading ===
  int button_state = digitalRead(BUTTON1_PIN);
  int estop_state = digitalRead(ESTOP_PIN);

  if (button_state != prev_button_state) {
    Serial.print("BUTTON1_PIN is now: ");
    Serial.println(button_state == HIGH ? "HIGH" : "LOW");
    prev_button_state = button_state;
  }

  if (estop_state != prev_estop_state) {
    Serial.print("ESTOP_PIN is now: ");
    Serial.println(estop_state == HIGH ? "HIGH" : "LOW");
    prev_estop_state = estop_state;
  }

  // === Confirm ISR handling separately ===
  if (estop_triggered) {
    Serial.println("[E-STOP] ISR Activated!");
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    estop_triggered = false;
  }

  delay(100);  // Sampling delay for stability
}

#endif