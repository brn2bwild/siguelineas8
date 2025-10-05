#include <QTRSensors.h>

#define leds 13

#define qtrLeds 11
#define pwmA 5
#define ain1 9  // 5 motor derecho conector B
#define ain2 4  // 4 motor derecho conector A

#define pwmB 6
#define bin1 7  // 7
#define bin2 6  // 8

#define boton 12

// int vbase = 80;
// int p, u, d;
// float i = 0;
// float p_anterior = 0;

QTRSensors qtr;

const uint8_t cantidad_sensores = 8;
uint16_t SenIR[cantidad_sensores];

void setup() {
  pinMode(boton, INPUT);
  pinMode(qtrLeds, OUTPUT);
  pinMode(leds, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(pwmB, OUTPUT);

  digitalWrite(qtrLeds, HIGH);

  motor(0, 0);

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

  while (digitalRead(boton))
    ;

  delay(1000);
}

void motorIzquierdo(int valor) {
  if (valor >= 0) {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  } else {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
    valor *= -1;
  }
  analogWrite(pwmA, valor);
}

void motorDerecho(int valor) {
  if (valor >= 0) {
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
  } else {
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
    valor *= -1;
  }
  analogWrite(pwmB, valor);
}

void freno(bool izquierdo, bool derecho, int valor) {
  if (izquierdo) {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, HIGH);
    analogWrite(pwmA, valor);
  }
  if (derecho) {
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, HIGH);
    analogWrite(pwmB, valor);
  }
}

void motor(int izquierda, int derecha) {
  motorIzquierdo(izquierda);
  motorDerecho(derecha);
}

//con ésta velocidad los valores son p = 0.025, i = 0.0, d = 0.385
//int vMax = 255;

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
//int vMax = 180;
// const float kp = 0.047; //0.047 proporcional
// const float ki = 0.0;    //integral
// const float kd = 0.800;//0.645 derivativo

// velocidad del sigue líneas
int vMax = 30;  // 230 de velocidad funcional
int vCurva = 80;

int proporcional = 0;
int integral = 0;
int derivativo = 0;

long error;
int ultimo_proporcional;
int objetivo = 3500;

int vel_motor_izquierdo;
int vel_motor_derecho;

void loop() {
  // while (digitalRead(boton)) {
  unsigned int position = qtr.readLineBlack(SenIR);

  // qtr.read(SenIR);

  // for(int i = 0; i < 8; i++){
  //   Serial.print(SenIR[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  proporcional = ((int)position) - 3500;


   if (proporcional <= -objetivo) {
     motorIzquierdo(0);
     freno(true, false, 200);
   } else if (proporcional >= objetivo) {
     motorDerecho(0);
     freno(false, true, 200);
   }

  integral += ultimo_proporcional;

  // if(integral > 1000) integral = 1000;
  // if(integral < -1000) integral = -1000;

  derivativo = proporcional - ultimo_proporcional;

  ultimo_proporcional = proporcional;

  error = (proporcional * kp) + (integral * ki) + (derivativo * kd);

  //Serial.print("error: ");
  //Serial.println(error);
  // Serial.print(";");

  //------ estas líneas son por si la pista tiene muchas curvas difíciles----------//
  vel_motor_izquierdo = vMax + error;
  vel_motor_derecho = vMax - error;


  if (vel_motor_izquierdo < -vMax) vel_motor_izquierdo = -vMax;
  else if (vel_motor_izquierdo > vMax) vel_motor_izquierdo = vMax;

  if (vel_motor_derecho < -vMax) vel_motor_derecho = -vMax;
  else if (vel_motor_derecho > vMax) vel_motor_derecho = vMax;


  // if (position <1500 || position >4500) vel_motor_izquierdo = vCurva;
  // // else if (vel_motor_izquierdo > vMax) vel_motor_izquierdo = vMax;
  //   Serial.println(vel_motor_izquierdo);

  // if (position <1500 || position >4500) vel_motor_derecho = vCurva;
  // // else if (vel_motor_derecho > vMax) vel_motor_derecho = vMax;
  // Serial.println(vel_motor_derecho);



  //   motor(vel_motor_izquierdo, vel_motor_derecho);
  // }

  motor(0, 0);
}