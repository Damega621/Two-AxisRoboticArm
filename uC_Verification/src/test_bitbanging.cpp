#ifdef TEST_BITBANGING

#include <Arduino.h>

const int in1 = 25;
const int in2 = 26;
const int in3 = 27;
const int in4 = 14;

void setup() {
  Serial.begin(115200);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}



void step1() {
  Serial.println("Step 1");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void step2() {
  Serial.println("Step 2");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void step3() {
  Serial.println("Step 3");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void step4() {
  Serial.println("Step 4");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void loop() {
  step1(); delay(100);
  step2(); delay(100);
  step3(); delay(100);
  step4(); delay(100);
}

#endif