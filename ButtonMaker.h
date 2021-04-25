#ifndef ButtonMaker_h
#define ButtonMaker_h

#include "Arduino.h"

class ButtonMaker
{
  public:
    ButtonMaker();
    bool isPress(bool input);

  private:
    bool _debounce;
};

#endif
