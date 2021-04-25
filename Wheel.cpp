#include "Arduino.h"
#include "Wheel.h"
#include <ESP32Servo.h>

Wheel::Wheel(int orientation, int servoPin)
{
  _orientation = orientation;
  ESP32PWM::allocateTimer(orientation);
  int minPeriod = 500; // 1000us
  int maxPeriod = 2400; // 2000us
  _offset = 0;

  _servo.setPeriodHertz(50);
  _servo.attach(servoPin, minPeriod, maxPeriod);
}

void Wheel::setZero(int zeropos)
{
  _zero = zeropos;
}
void Wheel::setMaxRange(int maxRange)
{
  _maxRange = maxRange;
}
void Wheel::setOffset(int offset)
{
  _offset = offset;
}
void Wheel::steer(int input)
{
  //FRONT
  if (_orientation == 0 || _orientation == 1)
  {
    
    int angle = map(input, -127, 127, _zero - _maxRange, _zero + _maxRange);
    _servo.write(angle + _offset);
  }
  //BACK
  else if (_orientation == 2 || _orientation == 3)
  {
    int angle = map(input, -127, 127, _zero + _maxRange, _zero - _maxRange);
    _servo.write(angle - _offset);
  }
}
