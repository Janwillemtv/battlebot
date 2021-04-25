#include "ButtonMaker.h"
#include "Arduino.h"

ButtonMaker::ButtonMaker() {
  _debounce = false;
}

bool ButtonMaker::isPress(bool input)
{
  if (input)
  {
    if (_debounce == false)
    {
      _debounce = true;
      return true;
    }
  }
  else _debounce = false;
  return false;
}
