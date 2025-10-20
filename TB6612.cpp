#include "Arduino.h"
#include "TB6612.h"

void TB6612::begin() {
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);

  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
}

void TB6612::motorIzq(int velocidad) {
  if (velocidad >= 0) {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
    analogWrite(pwmA, velocidad);
  } else {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
    analogWrite(pwmA, (-1) * velocidad);
  }
}

void TB6612::motorDer(int velocidad) {
  if (velocidad >= 0) {
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
    analogWrite(pwmB, velocidad);
  } else {
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
    analogWrite(pwmB, (-1) * velocidad);
  }
}

void TB6612::motores(int velIzq, int velDer) {
  motorIzq(velIzq);
  motorDer(velDer);
}

void TB6612::freno(bool mIzq, bool mDer, int velocidad) {
  if (mIzq) {
    motorIzq((-1) * velocidad);
    motorDer(velocidad);
  }
  if (mDer) {
    motorIzq(velocidad);
    motorDer((-1) * velocidad);
  }
}