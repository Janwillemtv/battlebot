#ifndef Deathwheel_h
#define Deathwheel_h

#include "Arduino.h"
#include <ESP32Servo.h>

class Deathwheel
{
  public:
  Deathwheel(int sawpin, int armpin);
  void raiseSpeed(int spd);
  void startKilling();
  void stop();
  void moveArm(int armpos);
  void setupSaw();
  bool isLive();
  
  private:
  Servo _sawblade;
  Servo _arm;
  int _armpos;
  bool _sawing;
  int _spd;
};

#endif
