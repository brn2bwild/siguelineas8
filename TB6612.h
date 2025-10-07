#include "Arduino.h"
#ifndef TB6612_h
#define TB6612_h

class TB6612 {
public:
  const int ain1 = 9;
  const int ain2 = 4;
  const int pwmA = 5;
  const int bin1 = 7;
  const int bin2 = 8;
  const int pwmB = 6;
  void begin();
  void motores(int velIzq, int velDer);
  void freno(bool motorIzq, bool motorDer);
private:
};

#endif