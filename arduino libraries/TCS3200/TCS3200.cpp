#include "TCS3200.h"

#define LED_RGB               A2
#define S2_PIN_RGB            A1
#define S3_PIN_RGB            A0
#define OUT_PIN_RGB           2

#define TIME_CHECK            50000

//****** TCS3200 ******//
TCS3200::TCS3200() {
  TCS3200::init();
}

bool TCS3200::callback()
{
    _RGBstatus = TCS3200_DETECT;
   return true;
}

void TCS3200::init()
{
  _g_count = 0;
  _RGBstatus = TCS3200_DETACHED;

  pinMode(LED_RGB, OUTPUT);
  pinMode(S2_PIN_RGB, OUTPUT);
  pinMode(S3_PIN_RGB, OUTPUT);
  pinMode(OUT_PIN_RGB, INPUT);
  _calibration = 0;

  digitalWrite(LED_RGB, HIGH);   // Turn off LEDs
}

void TCS3200::filterColor(uint8_t S2, uint8_t S3)
{
  if(S2 != 0) {
    S2 = HIGH;
  }

  if(S3 != 0) {
    S2 = HIGH;
  }

  digitalWrite(S2_PIN_RGB, S2);
  digitalWrite(S3_PIN_RGB, S3);
}

void TCS3200::WB(uint8_t S2, uint8_t S3)
{
  _g_count = 0;
  _g_flag = _g_flag + 1;
  filterColor(S2, S3);
  start = millis();
}

void TCS3200::getFreq()
{
  switch(_g_flag)
  {
    case 0:
#ifdef DEBUG
      Serial.println("Read color:");
#endif //DEBUG
      WB(LOW, LOW);                 // Filter without Red
      break;
    case 1:
      _freqValues[0] = _g_count;
#ifdef DEBUG
      Serial.print("Frequency R=");
      Serial.println(_freqValues[0]);
#endif //DEBUG
      WB(HIGH, HIGH);               // Filter without Green
      break;
    case 2:
      _freqValues[1] = _g_count;
#ifdef DEBUG
      Serial.print("Frequency G=");
      Serial.println(_freqValues[1]);
#endif //DEBUG
      WB(LOW, HIGH);                // Filter without Blue
      break;
    case 3:
      _freqValues[2] = _g_count;
#ifdef DEBUG
      Serial.print("Frequency B=");
      Serial.println(_freqValues[2]);
#endif //DEBUG
      WB(HIGH, LOW);                // Clear (no filter)
      _RGBstatus = TCS3200_READY;
      break;
    default:
      _g_count = 0;
      break;
  }
}

void TCS3200::count()
{
  _g_count = _g_count + 1;
}

void TCS3200::attach()
{
  if (_RGBstatus == TCS3200_DETACHED) {
    digitalWrite(LED_RGB, LOW);    // Turn on the LEDs

    _freqValues[3] = {};
    _scaleFactor[3] = {};
    _g_flag = 0;
    _g_count = 0;

    start = millis();
    attachInterrupt(digitalPinToInterrupt(OUT_PIN_RGB), count, RISING);
    _RGBstatus = TCS3200_ATTACHED;
  }
}

void TCS3200::detach()
{
   detachInterrupt(digitalPinToInterrupt(OUT_PIN_RGB));
   _g_flag = 0;
   _g_count = 0;
   _RGBstatus = TCS3200_DETACHED;
}

bool TCS3200::read(int *RGBValues)
{
  if (millis() - start > 50)
    callback();

  if (_RGBstatus == TCS3200_DETECT) {
    getFreq();
    if (_g_flag > 3) {
      if (_calibration == 0) {
        for(int i = 0; i < 3; i++) {
          _scaleFactor[i] = 255.0 / _freqValues[i];
#ifdef DEBUG
          Serial.print("Scale factor: ");
          Serial.println(_scaleFactor[i]);
#endif
        }
      }
    } else {
        _RGBstatus = TCS3200_WAIT;
    }
  }

  if (_RGBstatus == TCS3200_READY) {
    _calibration = 1;
    for(int j = 0; j < 3; j++) {
      RGBValues[j] = _freqValues[j] * _scaleFactor[j];
      if (RGBValues[j] > 255)
        RGBValues[j] = 255;      // RGB correction
#ifdef DEBUG
      Serial.print("RGB Values: ");
      Serial.println(RGBValues[j]);
#endif
    }
    return true;
  } else {
    return false;
  }
}
