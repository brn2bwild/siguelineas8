#include <QTRSensors.h>
#include "TB6612.h"

/* Constantes para el control de la placa*/
#define LEDS 13
#define QTR_LEDS 11
#define BUTTON 12

/* Descomentar esta línea para debuggear el código*/
// #define DEBUG

/* Estas constantes para el control PID */
#define KP 0.1  // 0.07 con velocidad de 200
#define KD 0.0  //0.645 con velocidad de 200
#define KI 0.0  //0 con velocidad de 200
#define SETPOINT 3500

/* Velocidades del siguelíneas */
#define MIN_SPEED 30
#define MAX_SPEED 60  // 230 de velocidad funcional

/* Valor para el adjust */
#define MAX_ADJUST 255

/* Cantidad de sensores de la tarjeta */
#define SENSORS_NUM 8

QTRSensors qtr;
TB6612 puenteh;

/* Se declara el arreglo para los sensores */
uint16_t SenIR[SENSORS_NUM];

int error, last_error, integral, derivative, left_motor_speed, right_motor_speed;

void setup() {
  pinMode(BUTTON, INPUT);
  pinMode(QTR_LEDS, OUTPUT);
  pinMode(LEDS, OUTPUT);

  digitalWrite(QTR_LEDS, HIGH);

#ifndef DEBUG
  puenteh.begin();
#endif

#ifdef DEBUG
  Serial.begin(9600);
  delay(10);
#endif

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A7, A0, A1, A2, A3, A4, A5, A6 }, SENSORS_NUM);
  delay(500);

  while (digitalRead(BUTTON)) {
    digitalWrite(LEDS, !digitalRead(LEDS));
    delay(500);
  }

  for (uint16_t i = 0; i < 120; i++) {
    digitalWrite(LEDS, HIGH);
    delay(20);
    qtr.calibrate();
    digitalWrite(LEDS, LOW);
    delay(20);
  }

  digitalWrite(LEDS, HIGH);

  while (digitalRead(BUTTON)) {};

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


  error = SETPOINT - position;
  /* Estas líneas hacen falta para implementar el freno en las curvas más complicadas*/
  // if (error <= -objetivo) {
  //   //  motorIzquierdo(0);
  //   //  freno(true, false, 200);
  //   puenteh.motores(0, 255);
  // } else if (error >= objetivo) {
  //   //  motorDerecho(0);
  //   //  freno(false, true, 200);
  //   puenteh.motores(255, 0);
  // }

  derivative = error - last_error;

  integral += error;

  int adjust = (KP * error) + (KI * integral) + (KD * derivative);

  // adjust = constrain(adjust, -MAX_ADJUST, MAX_ADJUST);

  left_motor_speed = constrain(MAX_SPEED - adjust, MIN_SPEED, MAX_SPEED);
  right_motor_speed = constrain(MAX_SPEED + adjust, MIN_SPEED, MAX_SPEED);

  last_error = error;

#ifndef DEBUG
  puenteh.motores(left_motor_speed, right_motor_speed);
#endif

#ifdef DEBUG
  /* Esta parte sirve para comprobar los valores de las velocidades */
  Serial.print("left speed: ");
  Serial.print(left_motor_speed);
  Serial.print(", right speef: ");
  Serial.print(right_motor_speed);
  Serial.print(", proportional: ");
  Serial.print(error);
  Serial.print(", adjust: ");
  Serial.println(adjust);
#endif
}