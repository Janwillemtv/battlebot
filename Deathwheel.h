#ifndef Deathwheel_h
#define Deathwheel_h

#include "Arduino.h"
#include <ESP32Servo.h>

class Deathwheel
{
  public:
  Deathwheel(int sawpin);
  void raiseSpeed(int spd);
  void startKilling();
  void stop();
  bool isLive();
  
  private:
  Servo _sawblade;
  bool _sawing;
  int _spd;
};

#endif
