#ifndef Wheel_h
#define Wheel_H

#include "Arduino.h"
#include <ESP32Servo.h>

class Wheel
{
  public:
    Wheel(int orientation, int servoPin);
    void setZero(int zeropos);
    void setMaxRange(int maxRange);
    void steer(int input);
    void setOffset(int offset);

  private:
    int _orientation;
    int _zero;
    int _maxRange;
    int _offset;
    Servo _servo;
};

#endif
