#ifndef STEPMOTORS_h
#define STEPMOTORS_h
#include "Arduino.h"

#define CARD_RPM            25

typedef enum
{
  STEP_FORWARD = 0,
  STEP_BACKWARDS,
  STEP_LEFT,
  STEP_RIGHT,
  STEP_LEFT_ORDER,
  STEP_RIGHT_ORDER,
  STEP_LAST_VAL
} Direction;

class StepMotors
{
public:
	StepMotors();
	void init(int dirPin1, int dirPin2);
	StepMotors(int dirPin1, int dirPin2);
	uint8_t moveAcc(Direction movem, uint16_t milimeters, uint16_t rpm);
private:
	int _dirPin1;
	int _dirPin2;
};

#endif //STEPMOTORS_h
