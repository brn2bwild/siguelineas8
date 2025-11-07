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

  // void begin();
  TB6612();
  void motorIzq(int velocidad);
  void motorDer(int velocidad);
  void motores(int velIzq, int velDer);
  void freno(bool sentido, int velocidad);
private:
};

#endif