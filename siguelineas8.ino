#include <QTRSensors.h>
#include "TB6612.h"

#define leds 13

#define qtrLeds 11
#define pwmA 5
#define ain1 9  // 5 motor derecho conector B
#define ain2 4  // 4 motor derecho conector A

#define pwmB 6
#define bin1 7  // 7
#define bin2 6  // 8

#define boton 12

const uint8_t cantidad_sensores = 8;
uint16_t SenIR[cantidad_sensores];

QTRSensors qtr;
TB6612 puenteh;

void setup() {
  pinMode(boton, INPUT);
  pinMode(qtrLeds, OUTPUT);
  pinMode(leds, OUTPUT);

  digitalWrite(qtrLeds, HIGH);

  puenteh.begin();

  Serial.begin(9600);
  delay(100);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A7, A0, A1, A2, A3, A4, A5, A6 }, cantidad_sensores);
  delay(500);

  while (digitalRead(boton)) {
    digitalWrite(leds, !digitalRead(leds));
    delay(500);
  }

  for (uint16_t i = 0; i < 120; i++) {
    digitalWrite(leds, HIGH);
    delay(20);
    qtr.calibrate();
    digitalWrite(leds, LOW);
    delay(20);
  }

  digitalWrite(leds, HIGH);

  while (digitalRead(boton)) {};

  delay(1000);
}

// void motorIzquierdo(int valor) {
//   if (valor >= 0) {
//     digitalWrite(ain1, LOW);
//     digitalWrite(ain2, HIGH);
//   } else {
//     digitalWrite(ain1, HIGH);
//     digitalWrite(ain2, LOW);
//     valor *= -1;
//   }
//   analogWrite(pwmA, valor);
// }

// void motorDerecho(int valor) {
//   if (valor >= 0) {
//     digitalWrite(bin1, LOW);
//     digitalWrite(bin2, HIGH);
//   } else {
//     digitalWrite(bin1, HIGH);
//     digitalWrite(bin2, LOW);
//     valor *= -1;
//   }
//   analogWrite(pwmB, valor);
// }

// void freno(bool izquierdo, bool derecho, int valor) {
//   if (izquierdo) {
//     digitalWrite(ain1, HIGH);
//     digitalWrite(ain2, HIGH);
//     analogWrite(pwmA, valor);
//   }
//   if (derecho) {
//     digitalWrite(bin1, HIGH);
//     digitalWrite(bin2, HIGH);
//     analogWrite(pwmB, valor);
//   }
// }

// void motor(int izquierda, int derecha) {
//   motorIzquierdo(izquierda);
//   motorDerecho(derecha);
// }

//con ésta velocidad los valores son p = 0.025, i = 0.0, d = 0.385
//int velocidad_maxima = 255;

//---------- estas constantes son por si la pista tiene muchas curvas
// const float kp = 0.137;  //0.047 proporcional ----> 0.67 jalo chido   0.107 jalo mas chido
// const float kp = 0.127;
// // const float ki = 0.02;    //integral ------> 0.0 jalo chido
// const float ki = 0.0;
// const float kd = 0.645;  //0.645 derivativo -------0.645 jalo chido
const float kp = 0.07;  // 0.07 con velocidad de 200
// const float ki = 0.02;    //integral ------> 0.0 jalo chido
const float ki = 0.000;  //0 con velocidad de 200
const float kd = 0.645;  //0.645 con velocidad de 200


// velocidad del sigue líneas
//int velocidad_maxima = 180;
// const float kp = 0.047; //0.047 proporcional
// const float ki = 0.0;    //integral
// const float kd = 0.800;//0.645 derivativo

// velocidad del sigue líneas
const int velocidad_maxima = 60;  // 230 de velocidad funcional

int proporcional = 0;
int integral = 0;
int derivativo = 0;

long error;
int ultimo_proporcional;
int objetivo = 3500;

int vel_motor_izquierdo;
int vel_motor_derecho;

void loop() {
  unsigned int position = qtr.readLineBlack(SenIR);

  // // Esta parte sirve para comprobar que los sensores están leyendo correctamente
  // qtr.read(SenIR);

  // for(int i = 0; i < 8; i++){
  //   Serial.print(SenIR[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  proporcional = ((int)position) - 3500;

  // if (proporcional <= -objetivo) {
  //   //  motorIzquierdo(0);
  //   //  freno(true, false, 200);
  //   puenteh.motores(0, 255);
  // } else if (proporcional >= objetivo) {
  //   //  motorDerecho(0);
  //   //  freno(false, true, 200);
  //   puenteh.motores(255, 0);
  // }

  integral += ultimo_proporcional;

  derivativo = proporcional - ultimo_proporcional;

  ultimo_proporcional = proporcional;

  error = (proporcional * kp) + (integral * ki) + (derivativo * kd);

  vel_motor_izquierdo = velocidad_maxima - error;
  vel_motor_derecho = velocidad_maxima + error;

  if (vel_motor_izquierdo < 0) vel_motor_izquierdo = 0;
  else if (vel_motor_izquierdo > velocidad_maxima) vel_motor_izquierdo = velocidad_maxima;

  if (vel_motor_derecho < 0) vel_motor_derecho = 0;
  else if (vel_motor_derecho > velocidad_maxima) vel_motor_derecho = velocidad_maxima;

  Serial.print("vel izq: ");
  Serial.print(vel_motor_izquierdo);
  Serial.print(", vel der: ");
  Serial.print(vel_motor_derecho);
  Serial.print(", proportional: ");
  Serial.println(proporcional);

  // puenteh.motores(0, 0);
  puenteh.motores(vel_motor_izquierdo, vel_motor_derecho);
}