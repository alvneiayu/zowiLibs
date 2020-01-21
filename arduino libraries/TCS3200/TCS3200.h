#ifndef TCS3200_h
#define TCS3200_h
#include "Arduino.h"
#include "TimerOne.h"

#define TCS3200_DETACHED 0
#define TCS3200_ATTACHED 1
#define TCS3200_READY    2
#define TCS3200_DETECT   3
#define TCS3200_WAIT     4

static int _g_count;
static int _g_flag;
static float _scaleFactor[3];
static int _freqValues[3];
static int _RGBstatus;
static int _calibration;
static int _pinS2;
static int _pinS3;
static int _pinOut;
static int _pinLRGB;

class TCS3200
{
public:
	void init();
	TCS3200();
	void attach();
	void detach();
	bool read(int *RGBValues);
private:
	static void filterColor(uint8_t S2, uint8_t S3);
	static void WB(uint8_t S2, uint8_t S3);
	static void getFreq();
	static void count();
	static void callback();
};

#endif //TCS3200_h
