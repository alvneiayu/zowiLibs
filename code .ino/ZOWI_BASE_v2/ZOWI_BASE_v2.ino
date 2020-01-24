
//----------------------------------------------------------------
//-- Zowi basic firmware v2
//-- (c) BQ. Released under a GPL licencse
//-- 04 December 2015
//-- Authors:  Anita de Prado: ana.deprado@bq.com
//--           Jose Alberca:   jose.alberca@bq.com
//--           Javier Isabel:  javier.isabel@bq.com
//--           Juan Gonzalez (obijuan): juan.gonzalez@bq.com
//--           Irene Sanz : irene.sanz@bq.com
//--           Alvaro Neira : alvaro.neira@bq.com
//-----------------------------------------------------------------
//-- Experiment with all the features that Zowi has!
//-----------------------------------------------------------------

#include <Servo.h> 
#include <Oscillator.h>
#include <EEPROM.h>
#include <BatReader.h>
#include <US.h>
#include <LedMatrix.h>

//-- Library to manage serial commands
#include <ZowiSerialCommand.h>
ZowiSerialCommand SCmd;  //The SerialCommand object

//-- Zowi Library
#include <Zowi.h>
Zowi zowi;  //This is Zowi!!
 
//---------------------------------------------------------
//-- Configuration of pins where the servos are attached
/*
         --------------- 
        |               |
        |     O   O     |
      | |               | |
RR => |-|               |-| <= RL
      |  ---------------  |

*/

  #define PIN_RL 4 //servo[2]
  #define PIN_RR 5 //servo[3]
//---------------------------------------------------------

//---Zowi Buttons
#define PIN_SecondButton 6
#define PIN_ThirdButton 7

#define MAX_LOOPS             1000

///////////////////////////////////////////////////////////////////
//-- Global Variables -------------------------------------------//
///////////////////////////////////////////////////////////////////

const char programID[]="ZOWI_BASE_v2"; //Each program will have a ID

const char name_fac='$'; //Factory name
const char name_fir='#'; //First name

//-- Movement parameters
int T=1000;              //Initial duration of movement
int moveId=0;            //Number of movement
int moveSize=15;         //Asociated with the height of some movements

typedef enum
{
  STOP = 0,
  MOVESTRAIGHT,
  MOVEBACK,
  MOVELEFT,
  MOVERIGHT
} State;

//---------------------------------------------------------
//-- Zowi has 5 modes:
//--    * MODE = 0: Zowi is awaiting  
//--    * MODE = 1: Dancing mode!  
//--    * MODE = 2: Obstacle detector mode  
//--    * MODE = 3: Noise detector mode   
//--    * MODE = 4: ZowiPAD or any Teleoperation mode (listening SerialPort). 
//---------------------------------------------------------
volatile int MODE=1; //State of zowi in the principal state machine. 

volatile bool buttonPushed=false;  //Variable to remember when a button has been pushed
volatile bool buttonAPushed=false; //Variable to remember when A button has been pushed
volatile bool buttonBPushed=false; //Variable to remember when B button has been pushed

unsigned long previousMillis=0;

bool obstacleDetected = false;

typedef enum
{
  GREEN = 0,
  RED,
  BLUE,
  WHITE,
  BLACK,
  YELLOW,
} Color;

int green[4] = {GREEN, 73, 95, 94};
int blue[4] = {BLUE, 78, 112, 110};
int red[4] = {RED, 173, 73, 73};
int black[4] = {BLACK, 15, 15, 15};
int black_2[4] = {BLACK, 43, 34, 35};
int yellow[4] = {YELLOW, 237, 180, 177};

#define NUMBER_OF_COLORS 6

int *colors[NUMBER_OF_COLORS] = {green, blue, red, black, black_2, yellow};

int color_index = 0;
int color_orders[15] = {};
bool valid_color = false;

int forward[2] = { RED, MOVESTRAIGHT };
int left[2] = { GREEN, MOVELEFT };
int right[2] = { BLUE, MOVERIGHT };
int stop[2] = { BLACK, STOP };
int back[2] = { YELLOW, MOVEBACK };

#define NUMBER_OF_ORDERS 5
int *orders_color[NUMBER_OF_ORDERS] = {forward, left, right, back, stop};

int returnColor(int *RGBval) {
  bool found;

  for (int i = 0; i < NUMBER_OF_COLORS; i++) {
    found = true;
    for (int j = 1; j < 4; j++) {
      if (colors[i][j] - 15 > RGBval[j - 1] || colors[i][j] + 15 < RGBval[j - 1]) {
        found = false;
        break;
      }
    }

    if (found == true)
      return colors[i][0];
  }

  return WHITE;
}

int executeOrder(int order) {
  bool found;

  for (int i = 0; i < NUMBER_OF_ORDERS; i++) {
    if (orders_color[i][0] != order)
      continue;

    switch (orders_color[i][1]) {
    case MOVESTRAIGHT:
      zowi.forward(2000);
      break;
    case MOVELEFT:
      zowi.left_order(1010);
      break;
    case MOVERIGHT:
      zowi.right_order(850);
      break;
    case STOP:
      zowi.stop(100);
      break;
    case MOVEBACK:
      zowi.back(2500);
      zowi.stop(100);
      break;
    default:
      break;
    }
  }
}

///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup(){

  //Serial communication initialization
  Serial.begin(115200);  

  pinMode(PIN_SecondButton,INPUT);
  pinMode(PIN_ThirdButton,INPUT);
  
  //Set the servo pins
  zowi.init(PIN_RL,PIN_RR,false);
 
  //Uncomment this to set the servo trims manually and save on EEPROM 
    //zowi.setTrims(TRIM_YL, TRIM_YR, TRIM_RL, TRIM_RR);
    //zowi.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.

  //Set a random seed
  randomSeed(analogRead(A6));

  //Setup callbacks for SerialCommand commands 
  SCmd.addCommand("S", receiveStop);      //  sendAck & sendFinalAck
  SCmd.addCommand("L", receiveLED);       //  sendAck & sendFinalAck
  SCmd.addCommand("T", recieveBuzzer);    //  sendAck & sendFinalAck
  SCmd.addCommand("M", receiveMovement);  //  sendAck & sendFinalAck
  SCmd.addCommand("H", receiveGesture);   //  sendAck & sendFinalAck
  SCmd.addCommand("K", receiveSing);      //  sendAck & sendFinalAck
  SCmd.addCommand("C", receiveTrims);     //  sendAck & sendFinalAck
  SCmd.addCommand("G", receiveServo);     //  sendAck & sendFinalAck
  SCmd.addCommand("R", receiveName);      //  sendAck & sendFinalAck
  SCmd.addCommand("E", requestName);
  SCmd.addCommand("D", requestDistance);
  SCmd.addCommand("N", requestNoise);
  SCmd.addCommand("B", requestBattery);
  SCmd.addCommand("I", requestProgramId);
  SCmd.addDefaultHandler(receiveStop);



  //Zowi wake up!
  //zowi.sing(S_connection);
  zowi.home();


  //If Zowi's name is '&' (factory name) means that is the first time this program is executed.
  //This first time, Zowi mustn't do anything. Just born at the factory!
  //5 = EEPROM address that contains first name character
  if (EEPROM.read(5)==name_fac){ 

    EEPROM.put(5, name_fir); //From now, the name is '#'
    EEPROM.put(6, '\0'); 
    zowi.putMouth(culito);

    while(true){    
       delay(1000);
    }
  }  


  //Send Zowi name, programID & battery level.
  requestName();
  delay(50);
  requestProgramId();
  delay(50);
  requestBattery();
  
  //Checking battery
  ZowiLowBatteryAlarm();


 // Animation Uuuuuh - A little moment of initial surprise
 //-----
  for(int i=0; i<2; i++){
      for (int i=0;i<8;i++){
        if(buttonPushed){break;}  
        zowi.putAnimationMouth(littleUuh,i);
        delay(150);
      }
  }
 //-----


  //Smile for a happy Zowi :)
  if(!buttonPushed){ 
    zowi.putMouth(smile);
    //zowi.sing(S_happy);
    delay(200);
  }


  //If Zowi's name is '#' means that Zowi hasn't been baptized
  //In this case, Zowi does a longer greeting
  //5 = EEPROM address that contains first name character
  if (EEPROM.read(5)==name_fir){ 

    if(!buttonPushed){  
        zowi.back(0.5);
        zowi.forward(0.5);
        delay(200); 
    }

    if(!buttonPushed){ 
        zowi.putMouth(smallSurprise);
        zowi.home();
    }  
  }


  if(!buttonPushed){ 
    zowi.putMouth(happyOpen);
  }

  previousMillis = millis();

}

///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() {
  int RGBValues[3] = {};
  int col;

  if (Serial.available()>0 && MODE!=3){

    MODE=1;
    //zowi.putMouth(happyOpen);

    buttonPushed=false;
  }

  //First attemp to initial software
  if (buttonPushed){  

    zowi.home();

    delay(100); //Wait for all buttons 
    //zowi.sing(S_buttonPushed);
    delay(200); //Wait for all buttons 

    zowi.putMouth(MODE);
 
    int showTime = 2000;
    while((showTime>0)){ //Wait to show the MODE number 
        
        showTime-=10;
        delay(10);
    }
     
    //zowi.putMouth(happyOpen);

    buttonPushed=false;

  }else{

    switch (MODE) {

      //-- MODE 0 - Zowi is awaiting
      //---------------------------------------------------------
      case 0:
      
        //Every 80 seconds in this mode, Zowi falls asleep 
        if (millis()-previousMillis>=80000){
            ZowiSleeping_withInterrupts(); //ZZzzzzz...
            previousMillis=millis();         
        }

        break;
        

      //-- MODE 1 - Noise detector mode
      //---------------------------------------------------------  
      case 1:
      
          static uint8_t sensorLeft = 0;
          static uint8_t sensorRight = 0;
          static State state = STOP;
          static int loops = 0;

         if (buttonBPushed == false)
           buttonBPushed = digitalRead(PIN_ThirdButton);

         if (buttonAPushed == false) {
           buttonAPushed = digitalRead(PIN_SecondButton);
           if (buttonAPushed == true && color_index != 0) {
             color_index = 0;
             memset(color_orders, 0, sizeof(color_orders));            
           }
         }

         if (buttonBPushed == true) {
           if (color_index != 0 && color_orders[color_index - 1] == BLACK) {
             zowi.putMouth(smile);

             for (int i = 0; i < color_index; i++) {
               executeOrder(color_orders[i]);
             }

             color_index = 0;
             memset(color_orders, 0, sizeof(color_orders));
             buttonBPushed=false;
           } else {
             zowi.putMouth(interrogation);
             buttonBPushed=false;
             return;
           }
         } else if (buttonAPushed == true) {
           if (zowi.getRGB(RGBValues)) {
             col = returnColor(RGBValues);
             if (col >= 0) {
               if (col == WHITE) {
                 valid_color = false;
               }

               if (valid_color == false && col != WHITE) {
                 valid_color = true;
                 color_orders[color_index] = col;
                 color_index = color_index + 1;
               }
             }
           }

           sensorLeft = zowi.getIR(LEFT);
           sensorRight = zowi.getIR(RIGHT);
           if ((sensorLeft == LOW) && (sensorRight == LOW)) {
             zowi.forward(10);
             loops = 0;
             state = MOVESTRAIGHT;
             zowi.putMouth(smile);
           } else if ((sensorLeft == LOW) && (sensorRight == HIGH)) {
             zowi.left(10);
             loops = 0;
             state = MOVELEFT;
             zowi.putMouth(smile);
           } else if ((sensorLeft == HIGH) && (sensorRight == LOW)) {
             zowi.right(10);
             loops = 0;
             state = MOVERIGHT;
             zowi.putMouth(smile);
           } else if ((sensorLeft == HIGH) && (sensorRight == HIGH)) {
             if (loops <= MAX_LOOPS) {
               if (state == MOVERIGHT) {
                 zowi.right(10);
                 loops++;
                 zowi.putMouth(smile);
               } else if (state == MOVELEFT) {
                 zowi.left(10);
                 loops++;
                 zowi.putMouth(smile);
               } else {
                 zowi.stop(10);
                 loops = 0;
                 state = STOP;
                 zowi.putMouth(sad);
               }
             } else {
               zowi.stop(10);
               loops = 0;
               state = STOP;
               zowi.putMouth(sad);
             }
           }
           if (color_index != 0 && color_orders[color_index - 1] == BLACK) {
             //zowi.forward(3000);
             buttonAPushed = false;
           }
         } else {
           if (color_index != 0 && color_orders[color_index - 1] == BLACK) {
             zowi.stop(10);
             zowi.putMouth(smile);
           } else {
             zowi.stop(10);
             zowi.putMouth(sad);
           }
         }

        break;
        
      case 2: //Calibration RGB
        char buf[200];
        
        if (zowi.getRGB(RGBValues)) {
          col = returnColor(RGBValues);
          for (int i = 0; i < 3; i++) {
            sprintf(buf, "RGB[%d] = %d", i, RGBValues[i]);
            Serial.println(buf);
          }
        }
      break;

      //-- MODE 2 - ZowiPAD or any Teleoperation mode (listening SerialPort) 
      //---------------------------------------------------------
      case 3:

        SCmd.readSerial();
        
        //If Zowi is moving yet
        if (zowi.getRestState()==false){  
          move(moveId);
        }
      
        break;   

           
      default:
          MODE=1;
          break;
    }

  } 

}  



///////////////////////////////////////////////////////////////////
//-- Functions --------------------------------------------------//
//////////////////////////////////////////////////////////////////        /

//-- Function to read distance sensor & to actualize obstacleDetected variable
void obstacleDetector(){

   int distance = zowi.getDistance();

        if(distance<15){
          obstacleDetected = true;
        }else{
          obstacleDetected = false;
        }
}


//-- Function to receive Stop command.
void receiveStop(){

    sendAck();
    zowi.home();
    sendFinalAck();

}


//-- Function to receive LED commands
void receiveLED(){  

    //sendAck & stop if necessary
    sendAck();
    zowi.home();

    //Examples of receiveLED Bluetooth commands
    //L 000000001000010100100011000000000
    //L 001111111111111111111111111111111 (todos los LED encendidos)
    unsigned long int matrix;
    char *arg;
    char *endstr;
    arg=SCmd.next();
    //Serial.println (arg);
    if (arg != NULL) {
      matrix=strtoul(arg,&endstr,2);    // Converts a char string to unsigned long integer
      zowi.putMouth(matrix,false);
    }else{
      zowi.putMouth(xMouth);
      delay(2000);
      zowi.clearMouth();
    }

    sendFinalAck();

}


//-- Function to receive buzzer commands
void recieveBuzzer(){
  
    //sendAck & stop if necessary
    sendAck();
    zowi.home(); 

    bool error = false; 
    int frec;
    int duration; 
    char *arg; 
    
    arg = SCmd.next(); 
    if (arg != NULL) { frec=atoi(arg); }    // Converts a char string to an integer   
    else {error=true;}
    
    arg = SCmd.next(); 
    if (arg != NULL) { duration=atoi(arg); } // Converts a char string to an integer  
    else {error=true;}

    if(error==true){

      zowi.putMouth(xMouth);
      delay(2000);
      zowi.clearMouth();

    }else{ 

      zowi._tone(frec, duration, 1);   
    }

    sendFinalAck();

}


//-- Function to receive TRims commands
void receiveTrims(){  

    //sendAck & stop if necessary
    sendAck();
    zowi.home(); 

    int trim_YL,trim_YR,trim_RL,trim_RR;

    //Definition of Servo Bluetooth command
    //C trim_YL trim_YR trim_RL trim_RR
    //Examples of receiveTrims Bluetooth commands
    //C 20 0 -8 3
    bool error = false;
    char *arg;
    arg=SCmd.next();
    if (arg != NULL) { trim_YL=atoi(arg); }    // Converts a char string to an integer   
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { trim_YR=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { trim_RL=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { trim_RR=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}
    
    if(error==true){

      zowi.putMouth(xMouth);
      delay(2000);
      zowi.clearMouth();

    }else{ //Save it on EEPROM
      zowi.setTrims(trim_YL, trim_YR, trim_RL, trim_RR);
      zowi.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.
    } 

    sendFinalAck();

}


//-- Function to receive Servo commands
void receiveServo(){  

    sendAck(); 
    moveId = 30;

    //Definition of Servo Bluetooth command
    //G  servo_YL servo_YR servo_RL servo_RR 
    //Example of receiveServo Bluetooth commands
    //G 90 85 96 78 
    bool error = false;
    char *arg;
    int servo_YL,servo_YR,servo_RL,servo_RR;

    arg=SCmd.next();
    if (arg != NULL) { servo_YL=atoi(arg); }    // Converts a char string to an integer   
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { servo_YR=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { servo_RL=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}

    arg = SCmd.next(); 
    if (arg != NULL) { servo_RR=atoi(arg); }    // Converts a char string to an integer  
    else {error=true;}
    
    if(error==true){

      zowi.putMouth(xMouth);
      delay(2000);
      zowi.clearMouth();

    }else{ //Update Servo:

      int servoPos[4]={servo_YL, servo_YR, servo_RL, servo_RR}; 
      zowi._moveServos(200, servoPos);   //Move 200ms
      
    }

    sendFinalAck();

}


//-- Function to receive movement commands
void receiveMovement(){

    sendAck();

    if (zowi.getRestState()==true){
        zowi.setRestState(false);
    }

    //Definition of Movement Bluetooth commands
    //M  MoveID  T   MoveSize  
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {moveId=atoi(arg);}
    else{
      zowi.putMouth(xMouth);
      delay(2000);
      zowi.clearMouth();
      moveId=0; //stop
    }
    
    arg = SCmd.next(); 
    if (arg != NULL) {T=atoi(arg);}
    else{
      T=1000;
    }

    arg = SCmd.next(); 
    if (arg != NULL) {moveSize=atoi(arg);}
    else{
      moveSize =15;
    }
}


//-- Function to execute the right movement according the movement command received.
void move(int moveId){

  bool manualMode = false;

  switch (moveId) {
    case 0:
      zowi.home();
      break;
    case 1: //M 1 1000 
      zowi.left(T);
      break;
    case 2: //M 2 1000 
      zowi.right(T);
      break;
    case 3: //M 3 1000 
      zowi.forward(T);
      break;
    case 4: //M 4 1000 
      zowi.back(T);
      break;
    default:
        manualMode = true;
      break;
  }

  if(!manualMode){
    sendFinalAck();
  }
       
}


//-- Function to receive gesture commands
void receiveGesture(){

    //sendAck & stop if necessary
    sendAck();
    zowi.home(); 

    //Definition of Gesture Bluetooth commands
    //H  GestureID  
    int gesture = 0;
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {gesture=atoi(arg);}
    else 
    {
      zowi.putMouth(xMouth);
      delay(2000);
      zowi.clearMouth();
    }

    switch (gesture) {
      case 1: //H 1 
        zowi.playGesture(ZowiHappy);
        break;
      case 2: //H 2 
        zowi.playGesture(ZowiSuperHappy);
        break;
      case 3: //H 3 
        zowi.playGesture(ZowiSad);
        break;
      case 4: //H 4 
        zowi.playGesture(ZowiSleeping);
        break;
      case 5: //H 5  
        zowi.playGesture(ZowiFart);
        break;
      case 6: //H 6 
        zowi.playGesture(ZowiConfused);
        break;
      case 7: //H 7 
        zowi.playGesture(ZowiLove);
        break;
      case 8: //H 8 
        zowi.playGesture(ZowiAngry);
        break;
      case 9: //H 9  
        zowi.playGesture(ZowiFretful);
        break;
      case 10: //H 10
        zowi.playGesture(ZowiMagic);
        break;  
      case 11: //H 11
        zowi.playGesture(ZowiWave);
        break;   
      case 12: //H 12
        zowi.playGesture(ZowiVictory);
        break; 
      case 13: //H 13
        zowi.playGesture(ZowiFail);
        break;         
      default:
        break;
    }

    sendFinalAck();
}

//-- Function to receive sing commands
void receiveSing(){

    //sendAck & stop if necessary
    sendAck();
    zowi.home(); 

    //Definition of Sing Bluetooth commands
    //K  SingID    
    int sing = 0;
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {sing=atoi(arg);}
    else 
    {
      zowi.putMouth(xMouth);
      delay(2000);
      zowi.clearMouth();
    }

    switch (sing) {
      case 1: //K 1 
        zowi.sing(S_connection);
        break;
      case 2: //K 2 
        zowi.sing(S_disconnection);
        break;
      case 3: //K 3 
        zowi.sing(S_surprise);
        break;
      case 4: //K 4 
        zowi.sing(S_OhOoh);
        break;
      case 5: //K 5  
        zowi.sing(S_OhOoh2);
        break;
      case 6: //K 6 
        zowi.sing(S_cuddly);
        break;
      case 7: //K 7 
        zowi.sing(S_sleeping);
        break;
      case 8: //K 8 
        zowi.sing(S_happy);
        break;
      case 9: //K 9  
        zowi.sing(S_superHappy);
        break;
      case 10: //K 10
        zowi.sing(S_happy_short);
        break;  
      case 11: //K 11
        zowi.sing(S_sad);
        break;   
      case 12: //K 12
        zowi.sing(S_confused);
        break; 
      case 13: //K 13
        zowi.sing(S_fart1);
        break;
      case 14: //K 14
        zowi.sing(S_fart2);
        break;
      case 15: //K 15
        zowi.sing(S_fart3);
        break;    
      case 16: //K 16
        zowi.sing(S_mode1);
        break; 
      case 17: //K 17
        zowi.sing(S_mode2);
        break; 
      case 18: //K 18
        zowi.sing(S_mode3);
        break;   
      case 19: //K 19
        zowi.sing(S_buttonPushed);
        break;                      
      default:
        break;
    }

    sendFinalAck();
}


//-- Function to receive Name command
void receiveName(){

    //sendAck & stop if necessary
    sendAck();
    zowi.home(); 

    char newZowiName[11] = "";  //Variable to store data read from Serial.
    int eeAddress = 5;          //Location we want the data to be in EEPROM.
    char *arg; 
    arg = SCmd.next(); 
    
    if (arg != NULL) {

      //Complete newZowiName char string
      int k = 0;
      while((*arg) && (k<11)){ 
          newZowiName[k]=*arg++;
          k++;
      }
      
      EEPROM.put(eeAddress, newZowiName); 
    }
    else 
    {
      zowi.putMouth(xMouth);
      delay(2000);
      zowi.clearMouth();
    }

    sendFinalAck();

}


//-- Function to send Zowi's name
void requestName(){

    zowi.home(); //stop if necessary

    char actualZowiName[11]= "";  //Variable to store data read from EEPROM.
    int eeAddress = 5;            //EEPROM address to start reading from

    //Get the float data from the EEPROM at position 'eeAddress'
    EEPROM.get(eeAddress, actualZowiName);

    Serial.print(F("&&"));
    Serial.print(F("E "));
    Serial.print(actualZowiName);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send ultrasonic sensor measure (distance in "cm")
void requestDistance(){

    zowi.home();  //stop if necessary  

    int distance = zowi.getDistance();
    Serial.print(F("&&"));
    Serial.print(F("D "));
    Serial.print(distance);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send noise sensor measure
void requestNoise(){

    zowi.home();  //stop if necessary

    int microphone= zowi.getNoise(); //analogRead(PIN_NoiseSensor);
    Serial.print(F("&&"));
    Serial.print(F("N "));
    Serial.print(microphone);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send battery voltage percent
void requestBattery(){

    zowi.home();  //stop if necessary

    //The first read of the batery is often a wrong reading, so we will discard this value. 
    double batteryLevel = zowi.getBatteryLevel();

    Serial.print(F("&&"));
    Serial.print(F("B "));
    Serial.print(batteryLevel);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send program ID
void requestProgramId(){

    zowi.home();   //stop if necessary

    Serial.print(F("&&"));
    Serial.print(F("I "));
    Serial.print(programID);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send Ack comand (A)
void sendAck(){

  delay(30);

  Serial.print(F("&&"));
  Serial.print(F("A"));
  Serial.println(F("%%"));
  Serial.flush();
}


//-- Function to send final Ack comand (F)
void sendFinalAck(){

  delay(30);

  Serial.print(F("&&"));
  Serial.print(F("F"));
  Serial.println(F("%%"));
  Serial.flush();
}



//-- Functions with animatics
//--------------------------------------------------------

void ZowiLowBatteryAlarm(){

    double batteryLevel = zowi.getBatteryLevel();

    if(batteryLevel<45){
        
      while(!buttonPushed){

          zowi.putMouth(thunder);
          zowi.bendTones (880, 2000, 1.04, 8, 3);  //A5 = 880
          
          delay(30);

          zowi.bendTones (2000, 880, 1.02, 8, 3);  //A5 = 880
          zowi.clearMouth();
          delay(500);
      } 
    }
}

void ZowiSleeping_withInterrupts(){

  int bedPos_0[4]={90, 90}; 

  if(!buttonPushed){
    zowi._moveServos(700, bedPos_0);  
  }

  for(int i=0; i<4;i++){

    if(buttonPushed){break;}
      zowi.putAnimationMouth(dreamMouth,0);
      zowi.bendTones (100, 200, 1.04, 10, 10);
    
    if(buttonPushed){break;}
      zowi.putAnimationMouth(dreamMouth,1);
      zowi.bendTones (200, 300, 1.04, 10, 10);  

    if(buttonPushed){break;}
      zowi.putAnimationMouth(dreamMouth,2);
      zowi.bendTones (300, 500, 1.04, 10, 10);   

    delay(500);
    
    if(buttonPushed){break;}
      zowi.putAnimationMouth(dreamMouth,1);
      zowi.bendTones (400, 250, 1.04, 10, 1); 

    if(buttonPushed){break;}
      zowi.putAnimationMouth(dreamMouth,0);
      zowi.bendTones (250, 100, 1.04, 10, 1); 
    
    delay(500);
  } 

  if(!buttonPushed){
    zowi.putMouth(lineMouth);
    zowi.sing(S_cuddly);
  }

  zowi.home();
  if(!buttonPushed){zowi.putMouth(happyOpen);}  

}
