#include "IRsetup.h"

void motorTest() {
             digitalWrite(PIN_Motor_AIN_1, LOW);     // LOW is forward direction for motor A
             analogWrite(PIN_Motor_PWMA, 240);       // HIGH is forward direction for motor B
             digitalWrite(PIN_Motor_BIN_1, HIGH);
             analogWrite(PIN_Motor_PWMB, 180);
}

void motorPin() {
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
}
