#include "ServoEncoder.h"

//****** ServoEncoder ******//
ServoEncoder::ServoEncoder() {
}

ServoEncoder::ServoEncoder(int pinEncoder, int position) {
  ServoEncoder::init(pinEncoder, position);
}

void ServoEncoder::init(int pinEncoder, int position)
{
  _pinEncoder = pinEncoder;
  pinMode(_pinEncoder, INPUT);
  lap = 1;
  donelap = true;
  _pos = position;
}

int ServoEncoder::getLap() {
  return lap;
}

int ServoEncoder::read() {
  int val;

  val = analogRead(_pinEncoder);
  if (_pos == LEFT_POS) {
     if (donelap == false) {
       if (val >= 0 && val < 50) {
          lap = lap + 1;
          donelap = true;
       }
     }

     if (val > 750 && val < 800) {
          donelap = false;
     }
  } else {
     if (donelap == false) {
       if (val >= 750 && val < 800) {
          lap = lap + 1;
          donelap = true;
       }
     }

     if (val > 0 && val < 50) {
          donelap = false;
     }
  }

  return val;
}
