#ifndef IR_h
#define IR_h
#include "Arduino.h"

class IR
{
public:
	IR();
	void init(int pinTrigger);
	IR(int pinTrigger);
	uint8_t read();

private:
	int _pinTrigger;

};

#endif //IR_h
