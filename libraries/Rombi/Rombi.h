#ifndef Rombi_h
#define Rombi_h

#include <Servo.h>
#include <Oscillator.h>
#include <EEPROM.h>

#include <US.h>
#include <LedMatrix.h>
#include <BatReader.h>
#include <IR.h>
#include <TCS3200.h>
#include <ServoEncoder.h>

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

#define SERVO_ENC_LEFT_PIN    A4
#define SERVO_ENC_RIGHT_PIN   A5

class Rombi
{
  public:

    //-- Rombi initialization
    void init(int RL, int RR, bool load_calibration=true, int NoiseSensor=PIN_NoiseSensor, int Buzzer=PIN_Buzzer, int USTrigger=PIN_Trigger, int USEcho=PIN_Echo, int IRLeft=IR_LEFT_PIN, int IRRight=IR_RIGHT_PIN, int LeftEncoder=SERVO_ENC_LEFT_PIN, int RightEncoder=SERVO_ENC_RIGHT_PIN);

    //-- Attach & detach functions
    void attachServos();
    void detachServos();

    //-- Oscillator Trims
    void setTrims(int YL, int YR, int RL, int RR);
    void saveTrimsOnEEPROM();

    //-- Predetermined Motion Functions
    void _moveServos(int time, int  servo_target[]);
    void oscillateServos(int A[4], int O[4], int T, double phase_diff[4], float cycle);

    //-- HOME = Rombi at rest position
    void home();
    bool getRestState();
    void setRestState(bool state);
    
    //-- Predetermined Motion Functions
    void left(int T = 1000);
    void right(int T = 1000);
    void back(int T = 1000);
    void forward(int T = 1000);
    void stop(int T = 1000);

    void left_order(int T = 1000);
    void right_order(int T = 1000);

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
    Oscillator servo[2];
    US us;
    IR ir_left;
    IR ir_right;
    TCS3200 rgb_detector;
    ServoEncoder left_encoder;
    ServoEncoder right_encoder;

    int servo_pins[2];
    int servo_trim[2];
    int servo_position[2];

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


