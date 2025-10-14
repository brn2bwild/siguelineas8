#include <QTRSensors.h>
#include "TB6612.h"

#define leds 13
#define qtrLeds 11
#define boton 12

/* Descomentar esta línea para debuggear el código*/
// #define DEBUG

QTRSensors qtr;
TB6612 puenteh;

const uint8_t cantidad_sensores = 8;
uint16_t SenIR[cantidad_sensores];

/* Estas constantes son por si la pista tiene muchas curvas */
const float kp = 0.1;   // 0.07 con velocidad de 200
const float ki = 0.0;  //0 con velocidad de 200
const float kd = 0.0;  //0.645 con velocidad de 200

/* Valores máximos para el diferencial */
const int diferencial_maximo = 255;

/* Velocidad del sigue líneas */
const int velocidad_maxima = 20;  // 230 de velocidad funcional

int proporcional = 0;
int integral = 0;
int derivativo = 0;

long diferencial;
int ultimo_proporcional;
int objetivo = 3500;

int vel_motor_izquierdo;
int vel_motor_derecho;

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

void loop() {
  unsigned int position = qtr.readLineBlack(SenIR);

  /* Esta parte sirve para comprobar que los sensores están leyendo correctamente */
  // qtr.read(SenIR);
  // for(int i = 0; i < 8; i++){
  //   Serial.print(SenIR[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  proporcional = ((int)position) - 3500;

  /* Estas líneas hacen falta para implementar el freno en las curvas más complicadas*/
  // if (proporcional <= -objetivo) {
  //   //  motorIzquierdo(0);
  //   //  freno(true, false, 200);
  //   puenteh.motores(0, 255);
  // } else if (proporcional >= objetivo) {
  //   //  motorDerecho(0);
  //   //  freno(false, true, 200);
  //   puenteh.motores(255, 0);
  // }

  derivativo = proporcional - ultimo_proporcional;

  integral += ultimo_proporcional;

  ultimo_proporcional = proporcional;

  diferencial = (proporcional * kp) + (integral * ki) + (derivativo * kd);

  diferencial = constrain(diferencial, -diferencial_maximo, diferencial_maximo);

  vel_motor_izquierdo = constrain(velocidad_maxima - diferencial, -velocidad_maxima, velocidad_maxima);
  vel_motor_derecho = constrain(velocidad_maxima + diferencial, -velocidad_maxima, velocidad_maxima);

#ifndef DEBUG
  puenteh.motores(vel_motor_izquierdo, vel_motor_derecho);
#endif

#ifdef DEBUG
  /* Esta parte sirve para comprobar los valores de las velocidades */
  Serial.print("vel izq: ");
  Serial.print(vel_motor_izquierdo);
  Serial.print(", vel der: ");
  Serial.print(vel_motor_derecho);
  Serial.print(", proportional: ");
  Serial.print(proporcional);
  Serial.print(", diferencial: ");
  Serial.println(diferencial);
#endif
}