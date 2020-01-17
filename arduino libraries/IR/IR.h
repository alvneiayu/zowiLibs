#ifndef IR_h
#define IR_h
#include "Arduino.h"

class IR
{
public:
	IR();
	void init(int pinTrigger);
	IR(int pinTrigger);
	float read();

private:
	int _pinTrigger;

};

#endif //IR_h
