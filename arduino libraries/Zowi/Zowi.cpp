
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif


#include "Zowi.h"
#include <Oscillator.h>
#include <US.h>



void Zowi::init(int RL, int RR, bool load_calibration, int NoiseSensor, int Buzzer, int USTrigger, int USEcho) {
  
  servo_pins[0] = RL;
  servo_pins[1] = RR;

  attachServos();
  isZowiResting=false;

  if (load_calibration) {
    for (int i = 0; i < 2; i++) {
      int servo_trim = EEPROM.read(i);
      if (servo_trim > 128) servo_trim -= 256;
      servo[i].SetTrim(servo_trim);
    }
  }
  
  for (int i = 0; i < 2; i++) servo_position[i] = 90;

  //US sensor init with the pins:
  us.init(USTrigger, USEcho);

  //Buzzer & noise sensor pins:
  pinBuzzer = Buzzer;
  pinNoiseSensor = NoiseSensor;

  pinMode(Buzzer,OUTPUT);
  pinMode(NoiseSensor,INPUT);
}

///////////////////////////////////////////////////////////////////
//-- ATTACH & DETACH FUNCTIONS ----------------------------------//
///////////////////////////////////////////////////////////////////
void Zowi::attachServos(){
    servo[0].attach(servo_pins[0]);
    servo[1].attach(servo_pins[1]);
}

void Zowi::detachServos(){
    servo[0].detach();
    servo[1].detach();
}

///////////////////////////////////////////////////////////////////
//-- OSCILLATORS TRIMS ------------------------------------------//
///////////////////////////////////////////////////////////////////
void Zowi::setTrims(int YL, int YR, int RL, int RR) {
  servo[0].SetTrim(YL);
  servo[1].SetTrim(YR);
}

void Zowi::saveTrimsOnEEPROM() {
  
  for (int i = 0; i < 4; i++){ 
      EEPROM.write(i, servo[i].getTrim());
  } 
      
}


///////////////////////////////////////////////////////////////////
//-- BASIC MOTION FUNCTIONS -------------------------------------//
///////////////////////////////////////////////////////////////////
void Zowi::_moveServos(int time, int  servo_target[]) {

  attachServos();
  if(getRestState()==true){
        setRestState(false);
  }

  if(time>10){
    for (int i = 0; i < 2; i++) increment[i] = ((servo_target[i]) - servo_position[i]) / (time / 10.0);
    final_time =  millis() + time;

    for (int iteration = 1; millis() < final_time; iteration++) {
      partial_time = millis() + 10;
      for (int i = 0; i < 2; i++) servo[i].SetPosition(servo_position[i] + (iteration * increment[i]));
      while (millis() < partial_time); //pause
    }
  }
  else{
    for (int i = 0; i < 2; i++) servo[i].SetPosition(servo_target[i]);
  }
  for (int i = 0; i < 2; i++) servo_position[i] = servo_target[i];
}


///////////////////////////////////////////////////////////////////
//-- HOME = Zowi at rest position -------------------------------//
///////////////////////////////////////////////////////////////////
void Zowi::home(){

  if(isZowiResting==false){ //Go to rest position only if necessary

    int homes[2]={90, 90}; //All the servos at rest position
    _moveServos(500,homes);   //Move the servos in half a second

    detachServos();
    isZowiResting=true;
  }
}

bool Zowi::getRestState(){

    return isZowiResting;
}

void Zowi::setRestState(bool state){

    isZowiResting = state;
}


///////////////////////////////////////////////////////////////////
//-- PREDETERMINED MOTION SEQUENCES -----------------------------//
///////////////////////////////////////////////////////////////////

//---------------------------------------------------------
//-- Zowi movement: left
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Zowi::left(int T) {
  int left[]={85, 85};
  _moveServos(T, left);
}

//---------------------------------------------------------
//-- Zowi movement: right
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Zowi::right(int T) {
  int right[]={100, 100};
  _moveServos(T, right);
}

//---------------------------------------------------------
//-- Zowi movement: forward
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Zowi::forward(int T) {
  int forward[]={100, 85};
  _moveServos(T, forward);
}

//---------------------------------------------------------
//-- Zowi movement: back
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Zowi::back(int T) {
  int back[]={85, 100};
  _moveServos(T, back);
}

//---------------------------------------------------------
//-- Zowi movement: stop
//--  Parameters:
//--    T: Period
//---------------------------------------------------------
void Zowi::stop(int T) {
  int stop[]={90, 90};
  _moveServos(T, stop);
}

///////////////////////////////////////////////////////////////////
//-- SENSORS FUNCTIONS  -----------------------------------------//
///////////////////////////////////////////////////////////////////

//---------------------------------------------------------
//-- Zowi getDistance: return zowi's ultrasonic sensor measure
//---------------------------------------------------------
float Zowi::getDistance(){

  return us.read();
}


//---------------------------------------------------------
//-- Zowi getNoise: return zowi's noise sensor measure
//---------------------------------------------------------
int Zowi::getNoise(){

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
//-- Zowi getBatteryLevel: return battery voltage percent
//---------------------------------------------------------
double Zowi::getBatteryLevel(){

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


double Zowi::getBatteryVoltage(){

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

unsigned long int Zowi::getMouthShape(int number){
  unsigned long int types []={zero_code,one_code,two_code,three_code,four_code,five_code,six_code,seven_code,eight_code,
  nine_code,smile_code,happyOpen_code,happyClosed_code,heart_code,bigSurprise_code,smallSurprise_code,tongueOut_code,
  vamp1_code,vamp2_code,lineMouth_code,confused_code,diagonal_code,sad_code,sadOpen_code,sadClosed_code,
  okMouth_code, xMouth_code,interrogation_code,thunder_code,culito_code,angry_code};

  return types[number];
}


unsigned long int Zowi::getAnimShape(int anim, int index){

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


void Zowi::putAnimationMouth(unsigned long int aniMouth, int index){

      ledmatrix.writeFull(getAnimShape(aniMouth,index));
}


void Zowi::putMouth(unsigned long int mouth, bool predefined){

  if (predefined){
    ledmatrix.writeFull(getMouthShape(mouth));
  }
  else{
    ledmatrix.writeFull(mouth);
  }
}


void Zowi::clearMouth(){

  ledmatrix.clearMatrix();
}


///////////////////////////////////////////////////////////////////
//-- SOUNDS -----------------------------------------------------//
///////////////////////////////////////////////////////////////////

void Zowi::_tone (float noteFrequency, long noteDuration, int silentDuration){

    // tone(10,261,500);
    // delay(500);

      if(silentDuration==0){silentDuration=1;}

      tone(Zowi::pinBuzzer, noteFrequency, noteDuration);
      delay(noteDuration);       //milliseconds to microseconds
      //noTone(PIN_Buzzer);
      delay(silentDuration);     
}


void Zowi::bendTones (float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration){

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


void Zowi::sing(int songName){
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

void Zowi::playGesture(int gesture){

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

    case ZowiHappy: 
        _tone(note_E5,50,30);
        putMouth(smile);
        sing(S_happy_short);
        sing(S_happy_short);

        home();
        putMouth(happyOpen);
    break;


    case ZowiSuperHappy:
        putMouth(happyOpen);
        sing(S_happy);
        putMouth(happyClosed);
        putMouth(happyOpen);
        sing(S_superHappy);
        putMouth(happyClosed);

        home();  
        putMouth(happyOpen);
    break;


    case ZowiSad: 
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


    case ZowiSleeping:

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


    case ZowiFart:
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


    case ZowiConfused:
        putMouth(confused);
        sing(S_confused);
        delay(500);

        home();  
        putMouth(happyOpen);
    break;


    case ZowiLove:
        putMouth(heart);
        sing(S_cuddly);

        home(); 
        sing(S_happy_short);  
        putMouth(happyOpen);
    break;


    case ZowiAngry: 
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


    case ZowiFretful: 
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


    case ZowiMagic:

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


    case ZowiWave:
        
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

    case ZowiVictory:
        
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

    case ZowiFail:

        putMouth(sadOpen);
        _tone(900,200,1);
        putMouth(sadClosed);
        _tone(600,200,1);
        putMouth(confused);
        _tone(300,200,1);
        putMouth(xMouth);

        detachServos();
        _tone(150,2200,1);
        
        delay(600);
        clearMouth();
        putMouth(happyOpen);
        home();

    break;

  }
}    
