#include <QTRSensors.h>
#include "TB6612.h"

/* Constantes para el control de la placa*/
#define LEDS 13
#define QTR_LEDS 11
#define BUTTON 12

/* Descomentar esta línea para debuggear el código*/
// #define DEBUG

/* Estas constantes para el control PID */
const float KP = 0.2;  // 0.07 con velocidad de 200
const float KI = 0.0;  //0 con velocidad de 200
const float KD = 0.5;  //0.645 con velocidad de 200
const int SETPOINT = 3500;

/* Velocidades del siguelíneas */
const int MAX_SPEED = 70;  // 230 de velocidad funcional
const int MIN_SPEED = MAX_SPEED * (-1);
const int BRAKE_SPEED = 255;

/* Cantidad de sensores de la tarjeta */
const int SENSORS_NUM = 8;

QTRSensors qtr;
TB6612 puenteh;

/* Se declara el arreglo para los sensores */
uint16_t SenIR[SENSORS_NUM];

uint16_t position;
float error = 0.0, last_error = 0.0;
float error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;
float integral = 0.0;
float derivative = 0.0;
float pid_output = 0.0;

int left_motor_speed, right_motor_speed;

void setup() {
  pinMode(BUTTON, INPUT);
  pinMode(QTR_LEDS, OUTPUT);
  pinMode(LEDS, OUTPUT);

  // digitalWrite(QTR_LEDS, HIGH);

#ifdef DEBUG
  Serial.begin(9600);
  delay(10);
#endif

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A7, A0, A1, A2, A3, A4, A5, A6 }, SENSORS_NUM);
  qtr.setEmitterPin(QTR_LEDS);
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

#ifdef DEBUG
void printQtrData() {
  /* Esta parte sirve para comprobar que los sensores están leyendo correctamente */
  qtr.read(SenIR);
  for (int i = 0; i < 8; i++) {
    Serial.print(SenIR[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}
#endif

void computePID() {
  position = qtr.readLineBlack(SenIR);

  error = position - SETPOINT;

  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = error;
  integral = KI * (error6 + error5 + error4 + error3 + error2 + error1 + error);

  integral = constrain(integral, MIN_SPEED, MAX_SPEED);

  derivative = KD * (error - last_error);
  last_error = error;

  pid_output = (KP * error) + integral + derivative;
}

void driveMotors() {
  left_motor_speed = constrain(MAX_SPEED - pid_output, MIN_SPEED, MAX_SPEED);
  right_motor_speed = constrain(MAX_SPEED + pid_output, MIN_SPEED, MAX_SPEED);

  (pid_output > 0) ? puenteh.motores(left_motor_speed, MAX_SPEED) : puenteh.motores(MAX_SPEED, right_motor_speed);
}

#ifdef DEBUG
void printPIDDAta() {
  Serial.print("error: ");
  Serial.print(error);
  Serial.print(", pid_output: ");
  Serial.print(pid_output);
  Serial.print(", left speed: ");
  Serial.print(MAX_SPEED - pid_output);
  Serial.print(", right speed: ");
  Serial.println(MAX_SPEED + pid_output);
}
#endif

void loop() {
  computePID();

  driveMotors();
}