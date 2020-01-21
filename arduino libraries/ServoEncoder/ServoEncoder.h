#ifndef ServoEncoder_h
#define ServoEncoder_h
#include "Arduino.h"

#define LEFT_POS        1
#define RIGHT_POS       -1

class ServoEncoder
{
public:
	ServoEncoder();
	void init(int pinEncoder, int position);
	ServoEncoder(int pinTrigger, int position);
	int getLap();
	int read();
private:
	int _pinEncoder;
        int lap;
        int donelap;
	int _pos;
};

#endif //ServoEncoder_h
