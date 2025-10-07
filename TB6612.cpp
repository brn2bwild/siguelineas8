#include "Arduino.h"
#include "TB6612.h"

void TB6612::begin() {
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(pwmB, OUTPUT);

  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);

  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
}

void TB6612::motores(int velIzq, int velDer) {
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, HIGH);
  analogWrite(pwmA, velIzq);
  analogWrite(pwmB, velDer);
}

void TB6612::freno(bool motorIzq, bool motorDer) {
  if (motorIzq) {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, LOW);
    analogWrite(pwmA, 0);
  }
  if (motorDer) {
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, LOW);
    analogWrite(pwmB, 0);
  }
}