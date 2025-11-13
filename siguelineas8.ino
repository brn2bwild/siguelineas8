#include <QTRSensors.h>
#include "TB6612.h"

/* Constantes para el control de la placa*/
#define LEDS 13
#define QTR_LEDS 11
#define BUTTON 12

/* Descomentar esta línea para debuggear el código*/
// #define DEBUG

/* Estas constantes para el control PID */
const float KP = 0.3;  // 0.07 con velocidad de 200
const float KI = 0.0;   //0 con velocidad de 200
const float KD = 0.4;   //0.645 con velocidad de 200
const int SETPOINT = 3500;
const int OBJECTIVE = 3450;

/* Velocidades del siguelíneas */
const int MAX_SPEED = 140;  // 230 de velocidad funcional
const int MIN_SPEED = MAX_SPEED * -1;
const int BRAKE_SPEED = 140;

/* Valor para el diff */
const int MAX_PWM = 255;

/* Cantidad de sensores de la tarjeta */
const int SENSORS_NUM = 8;

QTRSensors qtr;
TB6612 puenteh;

/* Se declara el arreglo para los sensores */
uint16_t SenIR[SENSORS_NUM];

int error, last_position, integral, derivative, left_motor_speed, right_motor_speed;

void setup() {
  pinMode(BUTTON, INPUT);
  pinMode(QTR_LEDS, OUTPUT);
  pinMode(LEDS, OUTPUT);

  digitalWrite(QTR_LEDS, HIGH);

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


  error = position - SETPOINT;

/* Estas líneas implementan el freno en las curvas más complicadas*/
#ifndef DEBUG
  if (error <= -OBJECTIVE) {
    puenteh.motores(-BRAKE_SPEED, BRAKE_SPEED);
  } else if (error >= OBJECTIVE) {
    puenteh.motores(BRAKE_SPEED, -BRAKE_SPEED);
  }
#endif

  derivative = position - last_position;

  // integral += error;

  int diff = (KP * error) + (KD * derivative);

  last_position = position;

  left_motor_speed = constrain(MAX_SPEED - diff, MIN_SPEED, MAX_SPEED);
  right_motor_speed = constrain(MAX_SPEED + diff, MIN_SPEED, MAX_SPEED);

#ifndef DEBUG
  puenteh.motores(left_motor_speed, right_motor_speed);
#endif

  /* Esta parte sirve para comprobar los valores de las velocidades */
#ifdef DEBUG
  Serial.print("error: ");
  Serial.print(error);
  Serial.print(", diff: ");
  Serial.print(diff);
  Serial.print(", left speed: ");
  Serial.print(left_motor_speed);
  Serial.print(", right speed: ");
  Serial.println(right_motor_speed);
#endif
}