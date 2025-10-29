#include "Arduino.h"
#include <stdint.h>
#ifndef TB6612_h
#define TB6612_h

class TB6612 {
public:
  const uint8_t ain1 = 9;
  const uint8_t ain2 = 4;
  const uint8_t pwmA = 5;
  const uint8_t bin1 = 7;
  const uint8_t bin2 = 8;
  const uint8_t pwmB = 6;

  void begin();
  void motorIzq(uint8_t velocidad);
  void motorDer(uint8_t velocidad);
  void motores(uint8_t velIzq, uint8_t velDer);
  void freno(bool sentido, uint8_t velocidad);
private:
};

#endif