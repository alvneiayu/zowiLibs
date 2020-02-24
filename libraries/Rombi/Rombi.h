#ifndef Rombi_h
#define Rombi_h

#include <EEPROM.h>

#include <US.h>
#include <LedMatrix.h>
#include <BatReader.h>
#include <IR.h>
#include <TCS3200.h>
#include <StepMotors.h>

#include "Rombi_mouths.h"
#include "Rombi_sounds.h"
#include "Rombi_gestures.h"


//-- Constants
#define FORWARD     1
#define BACKWARD    -1
#define LEFT        1
#define RIGHT       -1
#define SMALL       5
#define MEDIUM      15
#define BIG         30

#define PIN_Buzzer  10
#define PIN_Trigger 8
#define PIN_Echo    9
#define PIN_NoiseSensor A6

#define IR_LEFT_PIN           3
#define IR_RIGHT_PIN          A3

class Rombi
{
  public:

    //-- Rombi initialization
    void init(int RL, int RR, bool load_calibration=true, int NoiseSensor=PIN_NoiseSensor, int Buzzer=PIN_Buzzer, int USTrigger=PIN_Trigger, int USEcho=PIN_Echo, int IRLeft=IR_LEFT_PIN, int IRRight=IR_RIGHT_PIN);

    //-- HOME = Rombi at rest position
    void home();
    bool getRestState();
    void setRestState(bool state);
    
    //-- Predetermined Motion Functions
    void left(uint16_t milimeters = 1);
    void right(uint16_t milimeters = 1);
    void back(uint16_t milimeters = 1);
    void forward(uint16_t milimeters = 1);

    void left_order();
    void right_order();

    //-- Sensors functions
    float getDistance(); //US sensor
    int getNoise();      //Noise Sensor
    uint8_t getIR(int side);
    int getRGB(int *RGBValues);
    int getEncLap(int side);
    int getEncVal(int side);

    //-- Battery
    double getBatteryLevel();
    double getBatteryVoltage();
    
    //-- Mouth & Animations
    void putMouth(unsigned long int mouth, bool predefined = true);
    void putAnimationMouth(unsigned long int anim, int index);
    void clearMouth();

    //-- Sounds
    void _tone (float noteFrequency, long noteDuration, int silentDuration);
    void bendTones (float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration);
    void sing(int songName);

    //-- Gestures
    void playGesture(int gesture);

 
  private:
    
    LedMatrix ledmatrix;
    BatReader battery;
    US us;
    IR ir_left;
    IR ir_right;
    TCS3200 rgb_detector;
    StepMotors motors;

    int pinBuzzer;
    int pinNoiseSensor;
    
    unsigned long final_time;
    unsigned long partial_time;
    float increment[4];

    bool isRombiResting;

    unsigned long int getMouthShape(int number);
    unsigned long int getAnimShape(int anim, int index);
    void _execute(int A[4], int O[4], int T, double phase_diff[4], float steps);

};

#endif


