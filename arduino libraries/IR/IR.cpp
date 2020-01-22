#include "IR.h"

//****** IR ******//
IR::IR() {
}

IR::IR(int pinTrigger) {
  IR::init(pinTrigger);
}

void IR::init(int pinTrigger)
{
  _pinTrigger = pinTrigger;
  pinMode( _pinTrigger , INPUT);
}

uint8_t IR::read(){
  return digitalRead(_pinTrigger);

}
