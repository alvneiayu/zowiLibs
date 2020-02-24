
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif


#include "Rombi.h"
#include <US.h>
#include <IR.h>
#include <TCS3200.h>


void Rombi::init(int RL, int RR, bool load_calibration, int NoiseSensor, int Buzzer, int USTrigger, int USEcho, int IRLeft, int IRRight) {
  
  motors.init(RL, RR);

  isRombiResting=false;

  // IR init
  ir_left.init(IRLeft);
  ir_right.init(IRRight);

  // RGB Init
  rgb_detector.init();

  //US sensor init with the pins:
  us.init(USTrigger, USEcho);

  //Buzzer & noise sensor pins:
  pinBuzzer = Buzzer;
  pinNoiseSensor = NoiseSensor;

  pinMode(Buzzer,OUTPUT);
  pinMode(NoiseSensor,INPUT);
}

///////////////////////////////////////////////////////////////////
//-- BASIC MOTION FUNCTIONS -------------------------------------//
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
//-- HOME = Rombi at rest position -------------------------------//
///////////////////////////////////////////////////////////////////
void Rombi::home(){

  if(isRombiResting==false){ //Go to rest position only if necessary

    int homes[2]={90, 90}; //All the servos at rest position
   // _moveServos(500,homes);   //Move the servos in half a second

    isRombiResting=true;
  }
}

bool Rombi::getRestState(){

    return isRombiResting;
}

void Rombi::setRestState(bool state){

    isRombiResting = state;
}


///////////////////////////////////////////////////////////////////
//-- PREDETERMINED MOTION SEQUENCES -----------------------------//
///////////////////////////////////////////////////////////////////

//---------------------------------------------------------
//-- Rombi movement: left
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Rombi::left(uint16_t milimeters) {
  motors.moveAcc(STEP_LEFT, milimeters, CARD_RPM);
}

//---------------------------------------------------------
//-- Rombi movement: right
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Rombi::right(uint16_t milimeters) {
  motors.moveAcc(STEP_RIGHT, milimeters, CARD_RPM);
}

//---------------------------------------------------------
//-- Rombi movement: forward
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Rombi::forward(uint16_t milimeters) {
  motors.moveAcc(STEP_FORWARD, milimeters, CARD_RPM);
}

//---------------------------------------------------------
//-- Rombi movement: back
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Rombi::back(uint16_t milimeters) {
  motors.moveAcc(STEP_BACKWARDS, milimeters, CARD_RPM);
}

//---------------------------------------------------------
//-- Rombi movement: left_order
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Rombi::left_order() {
  motors.moveAcc(STEP_LEFT_ORDER, 101, CARD_RPM); //101 = 90º
}

//---------------------------------------------------------
//-- Rombi movement: right_order
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Rombi::right_order() {
  motors.moveAcc(STEP_RIGHT_ORDER, 101, CARD_RPM); //101 = 90º
}

///////////////////////////////////////////////////////////////////
//-- SENSORS FUNCTIONS  -----------------------------------------//
///////////////////////////////////////////////////////////////////

//---------------------------------------------------------
//-- Rombi getDistance: return rombi's ultrasonic sensor measure
//---------------------------------------------------------
float Rombi::getDistance(){

  return us.read();
}


//---------------------------------------------------------
//-- Rombi getNoise: return rombi's noise sensor measure
//---------------------------------------------------------
int Rombi::getNoise(){

  int noiseLevel = 0;
  int noiseReadings = 0;
  int numReadings = 2;  

    noiseLevel = analogRead(pinNoiseSensor);

    for(int i=0; i<numReadings; i++){
        noiseReadings += analogRead(pinNoiseSensor);
        delay(4); // delay in between reads for stability
    }

    noiseLevel = noiseReadings / numReadings;

    return noiseLevel;
}

//---------------------------------------------------------
//-- Rombi getIR: return rombi's IR sensor val
//---------------------------------------------------------
uint8_t Rombi::getIR(int side) {
    if (side == LEFT) {
        return ir_left.read();
    } else {
        return ir_right.read();
    }
}

//---------------------------------------------------------
//-- Rombi getRGB: return rombi's RGB sensor val
//---------------------------------------------------------
int Rombi::getRGB(int *RGBValues) {
    rgb_detector.attach();
    if (rgb_detector.read(RGBValues)) {
        rgb_detector.detach();
        return true;
    }

    return false;
}

//---------------------------------------------------------
//-- Rombi getBatteryLevel: return battery voltage percent
//---------------------------------------------------------
double Rombi::getBatteryLevel(){

  //The first read of the batery is often a wrong reading, so we will discard this value. 
    double batteryLevel = battery.readBatPercent();
    double batteryReadings = 0;
    int numReadings = 10;

    for(int i=0; i<numReadings; i++){
        batteryReadings += battery.readBatPercent();
        delay(1); // delay in between reads for stability
    }

    batteryLevel = batteryReadings / numReadings;

    return batteryLevel;
}


double Rombi::getBatteryVoltage(){

  //The first read of the batery is often a wrong reading, so we will discard this value. 
    double batteryLevel = battery.readBatVoltage();
    double batteryReadings = 0;
    int numReadings = 10;

    for(int i=0; i<numReadings; i++){
        batteryReadings += battery.readBatVoltage();
        delay(1); // delay in between reads for stability
    }

    batteryLevel = batteryReadings / numReadings;

    return batteryLevel;
}


///////////////////////////////////////////////////////////////////
//-- MOUTHS & ANIMATIONS ----------------------------------------//
///////////////////////////////////////////////////////////////////

unsigned long int Rombi::getMouthShape(int number){
  unsigned long int types []={zero_code,one_code,two_code,three_code,four_code,five_code,six_code,seven_code,eight_code,
  nine_code,smile_code,happyOpen_code,happyClosed_code,heart_code,bigSurprise_code,smallSurprise_code,tongueOut_code,
  vamp1_code,vamp2_code,lineMouth_code,confused_code,diagonal_code,sad_code,sadOpen_code,sadClosed_code,
  okMouth_code, xMouth_code,interrogation_code,thunder_code,culito_code,angry_code};

  return types[number];
}


unsigned long int Rombi::getAnimShape(int anim, int index){

  unsigned long int littleUuh_code[]={
     0b00000000000000001100001100000000,
     0b00000000000000000110000110000000,
     0b00000000000000000011000011000000,
     0b00000000000000000110000110000000,
     0b00000000000000001100001100000000,
     0b00000000000000011000011000000000,
     0b00000000000000110000110000000000,
     0b00000000000000011000011000000000  
  };

  unsigned long int dreamMouth_code[]={
     0b00000000000000000000110000110000,
     0b00000000000000010000101000010000,  
     0b00000000011000100100100100011000,
     0b00000000000000010000101000010000           
  };

  unsigned long int adivinawi_code[]={
     0b00100001000000000000000000100001,
     0b00010010100001000000100001010010,
     0b00001100010010100001010010001100,
     0b00000000001100010010001100000000,
     0b00000000000000001100000000000000,
     0b00000000000000000000000000000000
  };

  unsigned long int wave_code[]={
     0b00001100010010100001000000000000,
     0b00000110001001010000100000000000,
     0b00000011000100001000010000100000,
     0b00000001000010000100001000110000,
     0b00000000000001000010100100011000,
     0b00000000000000100001010010001100,
     0b00000000100000010000001001000110,
     0b00100000010000001000000100000011,
     0b00110000001000000100000010000001,
     0b00011000100100000010000001000000    
  };

  switch  (anim){

    case littleUuh:
        return littleUuh_code[index];
        break;
    case dreamMouth:
        return dreamMouth_code[index];
        break;
    case adivinawi:
        return adivinawi_code[index];
        break;
    case wave:
        return wave_code[index];
        break;    
  }   
}


void Rombi::putAnimationMouth(unsigned long int aniMouth, int index){

      ledmatrix.writeFull(getAnimShape(aniMouth,index));
}


void Rombi::putMouth(unsigned long int mouth, bool predefined){

  if (predefined){
    ledmatrix.writeFull(getMouthShape(mouth));
  }
  else{
    ledmatrix.writeFull(mouth);
  }
}


void Rombi::clearMouth(){

  ledmatrix.clearMatrix();
}


///////////////////////////////////////////////////////////////////
//-- SOUNDS -----------------------------------------------------//
///////////////////////////////////////////////////////////////////

void Rombi::_tone (float noteFrequency, long noteDuration, int silentDuration){

    // tone(10,261,500);
    // delay(500);

      if(silentDuration==0){silentDuration=1;}

      tone(Rombi::pinBuzzer, noteFrequency, noteDuration);
      delay(noteDuration);       //milliseconds to microseconds
      //noTone(PIN_Buzzer);
      delay(silentDuration);     
}


void Rombi::bendTones (float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration){

  //Examples:
  //  bendTones (880, 2093, 1.02, 18, 1);
  //  bendTones (note_A5, note_C7, 1.02, 18, 0);

  if(silentDuration==0){silentDuration=1;}

  if(initFrequency < finalFrequency)
  {
      for (int i=initFrequency; i<finalFrequency; i=i*prop) {
          _tone(i, noteDuration, silentDuration);
      }

  } else{

      for (int i=initFrequency; i>finalFrequency; i=i/prop) {
          _tone(i, noteDuration, silentDuration);
      }
  }
}


void Rombi::sing(int songName){
  switch(songName){

    case S_connection:
      _tone(note_E5,50,30);
      _tone(note_E6,55,25);
      _tone(note_A6,60,10);
    break;

    case S_disconnection:
      _tone(note_E5,50,30);
      _tone(note_A6,55,25);
      _tone(note_E6,50,10);
    break;

    case S_buttonPushed:
      bendTones (note_E6, note_G6, 1.03, 20, 2);
      delay(30);
      bendTones (note_E6, note_D7, 1.04, 10, 2);
    break;

    case S_mode1:
      bendTones (note_E6, note_A6, 1.02, 30, 10);  //1318.51 to 1760
    break;

    case S_mode2:
      bendTones (note_G6, note_D7, 1.03, 30, 10);  //1567.98 to 2349.32
    break;

    case S_mode3:
      _tone(note_E6,50,100); //D6
      _tone(note_G6,50,80);  //E6
      _tone(note_D7,300,0);  //G6
    break;

    case S_surprise:
      bendTones(800, 2150, 1.02, 10, 1);
      bendTones(2149, 800, 1.03, 7, 1);
    break;

    case S_OhOoh:
      bendTones(880, 2000, 1.04, 8, 3); //A5 = 880
      delay(200);

      for (int i=880; i<2000; i=i*1.04) {
           _tone(note_B5,5,10);
      }
    break;

    case S_OhOoh2:
      bendTones(1880, 3000, 1.03, 8, 3);
      delay(200);

      for (int i=1880; i<3000; i=i*1.03) {
          _tone(note_C6,10,10);
      }
    break;

    case S_cuddly:
      bendTones(700, 900, 1.03, 16, 4);
      bendTones(899, 650, 1.01, 18, 7);
    break;

    case S_sleeping:
      bendTones(100, 500, 1.04, 10, 10);
      delay(500);
      bendTones(400, 100, 1.04, 10, 1);
    break;

    case S_happy:
      bendTones(1500, 2500, 1.05, 20, 8);
      bendTones(2499, 1500, 1.05, 25, 8);
    break;

    case S_superHappy:
      bendTones(2000, 6000, 1.05, 8, 3);
      delay(50);
      bendTones(5999, 2000, 1.05, 13, 2);
    break;

    case S_happy_short:
      bendTones(1500, 2000, 1.05, 15, 8);
      delay(100);
      bendTones(1900, 2500, 1.05, 10, 8);
    break;

    case S_sad:
      bendTones(880, 669, 1.02, 20, 200);
    break;

    case S_confused:
      bendTones(1000, 1700, 1.03, 8, 2); 
      bendTones(1699, 500, 1.04, 8, 3);
      bendTones(1000, 1700, 1.05, 9, 10);
    break;

    case S_fart1:
      bendTones(1600, 3000, 1.02, 2, 15);
    break;

    case S_fart2:
      bendTones(2000, 6000, 1.02, 2, 20);
    break;

    case S_fart3:
      bendTones(1600, 4000, 1.02, 2, 20);
      bendTones(4000, 3000, 1.02, 2, 20);
    break;

  }
}



///////////////////////////////////////////////////////////////////
//-- GESTURES ---------------------------------------------------//
///////////////////////////////////////////////////////////////////

void Rombi::playGesture(int gesture){

  int sadPos[4]=      {110, 70, 20, 160};
  int bedPos[4]=      {100, 80, 60, 120};
  int fartPos_1[4]=   {90, 90, 145, 122}; //rightBend
  int fartPos_2[4]=   {90, 90, 80, 122};
  int fartPos_3[4]=   {90, 90, 145, 80};
  int confusedPos[4]= {110, 70, 90, 90};
  int angryPos[4]=    {90, 90, 70, 110};
  int headLeft[4]=    {110, 110, 90, 90};
  int headRight[4]=   {70, 70, 90, 90};
  int fretfulPos[4]=  {90, 90, 90, 110};
  int bendPos_1[4]=   {90, 90, 70, 35};
  int bendPos_2[4]=   {90, 90, 55, 35};
  int bendPos_3[4]=   {90, 90, 42, 35};
  int bendPos_4[4]=   {90, 90, 34, 35};
  
  switch(gesture){

    case RombiHappy: 
        _tone(note_E5,50,30);
        putMouth(smile);
        sing(S_happy_short);
        sing(S_happy_short);

        home();
        putMouth(happyOpen);
    break;


    case RombiSuperHappy:
        putMouth(happyOpen);
        sing(S_happy);
        putMouth(happyClosed);
        putMouth(happyOpen);
        sing(S_superHappy);
        putMouth(happyClosed);

        home();  
        putMouth(happyOpen);
    break;


    case RombiSad: 
        putMouth(sad);
        bendTones(880, 830, 1.02, 20, 200);
        putMouth(sadClosed);
        bendTones(830, 790, 1.02, 20, 200);  
        putMouth(sadOpen);
        bendTones(790, 740, 1.02, 20, 200);
        putMouth(sadClosed);
        bendTones(740, 700, 1.02, 20, 200);
        putMouth(sadOpen);
        bendTones(700, 669, 1.02, 20, 200);
        putMouth(sad);
        delay(500);

        home();
        delay(300);
        putMouth(happyOpen);
    break;


    case RombiSleeping:

        for(int i=0; i<4;i++){
          putAnimationMouth(dreamMouth,0);
          bendTones (100, 200, 1.04, 10, 10);
          putAnimationMouth(dreamMouth,1);
          bendTones (200, 300, 1.04, 10, 10);  
          putAnimationMouth(dreamMouth,2);
          bendTones (300, 500, 1.04, 10, 10);   
          delay(500);
          putAnimationMouth(dreamMouth,1);
          bendTones (400, 250, 1.04, 10, 1); 
          putAnimationMouth(dreamMouth,0);
          bendTones (250, 100, 1.04, 10, 1); 
          delay(500);
        } 

        putMouth(lineMouth);
        sing(S_cuddly);

        home();  
        putMouth(happyOpen);
    break;


    case RombiFart:
        delay(300);     
        putMouth(lineMouth);
        sing(S_fart1);  
        putMouth(tongueOut);
        delay(250);
        delay(300);
        putMouth(lineMouth);
        sing(S_fart2); 
        putMouth(tongueOut);
        delay(250);
        delay(300);
        putMouth(lineMouth);
        sing(S_fart3);
        putMouth(tongueOut);    
        delay(300);

        home(); 
        delay(500); 
        putMouth(happyOpen);
    break;


    case RombiConfused:
        putMouth(confused);
        sing(S_confused);
        delay(500);

        home();  
        putMouth(happyOpen);
    break;


    case RombiLove:
        putMouth(heart);
        sing(S_cuddly);

        home(); 
        sing(S_happy_short);  
        putMouth(happyOpen);
    break;


    case RombiAngry: 
        putMouth(angry);

        _tone(note_A5,100,30);
        bendTones(note_A5, note_D6, 1.02, 7, 4);
        bendTones(note_D6, note_G6, 1.02, 10, 1);
        bendTones(note_G6, note_A5, 1.02, 10, 1);
        delay(15);
        bendTones(note_A5, note_E5, 1.02, 20, 4);
        delay(400);
        bendTones(note_A5, note_D6, 1.02, 20, 4);
        bendTones(note_A5, note_E5, 1.02, 20, 4);

        home();  
        putMouth(happyOpen);
    break;


    case RombiFretful: 
        putMouth(angry);
        bendTones(note_A5, note_D6, 1.02, 20, 4);
        bendTones(note_A5, note_E5, 1.02, 20, 4);
        delay(300);
        putMouth(lineMouth);

        for(int i=0; i<4; i++){
          home();
        }

        putMouth(angry);
        delay(500);

        home();  
        putMouth(happyOpen);
    break;


    case RombiMagic:

        //Initial note frecuency = 400
        //Final note frecuency = 1000
        
        // Reproduce the animation four times
        for(int i = 0; i<4; i++){ 

          int noteM = 400; 

            for(int index = 0; index<6; index++){
              putAnimationMouth(adivinawi,index);
              bendTones(noteM, noteM+100, 1.04, 10, 10);    //400 -> 1000 
              noteM+=100;
            }

            clearMouth();
            bendTones(noteM-100, noteM+100, 1.04, 10, 10);  //900 -> 1100

            for(int index = 0; index<6; index++){
              putAnimationMouth(adivinawi,index);
              bendTones(noteM, noteM+100, 1.04, 10, 10);    //1000 -> 400 
              noteM-=100;
            }
        } 
 
        delay(300);
        putMouth(happyOpen);
    break;


    case RombiWave:
        
        // Reproduce the animation four times
        for(int i = 0; i<2; i++){ 

            int noteW = 500; 

            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW+100, 1.02, 10, 10); 
              noteW+=101;
            }
            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW+100, 1.02, 10, 10); 
              noteW+=101;
            }
            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW-100, 1.02, 10, 10); 
              noteW-=101;
            }
            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW-100, 1.02, 10, 10); 
              noteW-=101;
            }
        }    

        clearMouth();
        delay(100);
        putMouth(happyOpen);
    break;

    case RombiVictory:
        
        putMouth(smallSurprise);
        //final pos   = {90,90,150,30}
        for (int i = 0; i < 60; ++i){
          int pos[]={90,90,90+i,90-i};  
          _tone(1600+i*20,15,1);
        }

        putMouth(bigSurprise);
        //final pos   = {90,90,90,90}
        for (int i = 0; i < 60; ++i){
          int pos[]={90,90,150-i,30+i};  
          _tone(2800+i*20,15,1);
        }

        putMouth(happyOpen);
        //SUPER HAPPY
        //-----
        sing(S_superHappy);
        putMouth(happyClosed);
        //-----

        home();
        clearMouth();
        putMouth(happyOpen);

    break;

    case RombiFail:

        putMouth(sadOpen);
        _tone(900,200,1);
        putMouth(sadClosed);
        _tone(600,200,1);
        putMouth(confused);
        _tone(300,200,1);
        putMouth(xMouth);

        _tone(150,2200,1);
        
        delay(600);
        clearMouth();
        putMouth(happyOpen);
        home();

    break;

  }
}    
