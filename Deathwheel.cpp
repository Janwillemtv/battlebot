#include "Deathwheel.h"
#include "Arduino.h"

Deathwheel::Deathwheel(int sawpin)
{
  ESP32PWM::allocateTimer(4);
  _sawblade.setPeriodHertz(50);
  _sawblade.attach(sawpin, 1000, 2000);
  this->stop();
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
