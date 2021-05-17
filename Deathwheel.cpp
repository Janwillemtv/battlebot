#include "Deathwheel.h"
#include "Arduino.h"

Deathwheel::Deathwheel(int sawpin, int armpin, int zero, int maxpos)
{
  ESP32PWM::allocateTimer(4);
  _sawblade.setPeriodHertz(50);
  _sawblade.attach(sawpin, 1000, 2000);

  ESP32PWM::allocateTimer(5);
  _arm.setPeriodHertz(50);
  _arm.attach(armpin, 1000, 3000);

  _zero = zero;
  _max = maxpos;

  _arm.write(zero);

  //_arm.write(180);

  this->setupSaw();

  this->stop();
}

void Deathwheel::setupSaw()
{
  _sawblade.write(180);
  delay(1000);
  _sawblade.write(0);
}
void Deathwheel::moveArm(int armpos)
{
  _arm.write(map(armpos, 0, 127, _zero, _max));
}

void Deathwheel::wiggleArm(int wiggle, int bias)
{
  _arm.write(map(wiggle, bias, 127, _max, _zero));
}
void Deathwheel::startKilling()
{
  _sawblade.write(_spd);
  _sawing = true;
}

void Deathwheel::stop()
{
  _sawblade.write(0);
  _sawing = false;
}

void Deathwheel::raiseSpeed(int amount)
{
  _spd += amount;
  if (_spd < 0) _spd = 0;
  else if (_spd > 255) _spd = 255;
  if (this->isLive()) startKilling();
}

bool Deathwheel::isLive()
{
  return _sawing;
}
