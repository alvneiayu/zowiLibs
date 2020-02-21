
//----------------------------------------------------------------
//-- Rombi basic firmware v2
//-- (c) BQ. Released under a GPL licencse
//-- 04 December 2015
//-- Authors:  Anita de Prado: ana.deprado@bq.com
//--           Jose Alberca:   jose.alberca@bq.com
//--           Javier Isabel:  javier.isabel@bq.com
//--           Juan Gonzalez (obijuan): juan.gonzalez@bq.com
//--           Irene Sanz : irene.sanz@bq.com
//--           Alvaro Neira : alvaro.neira@bq.com
//-----------------------------------------------------------------
//-- Experiment with all the features that Rombi has!
//-----------------------------------------------------------------

#include <Servo.h> 
#include <Oscillator.h>
#include <EEPROM.h>
#include <BatReader.h>
#include <US.h>
#include <LedMatrix.h>

//-- Library to manage serial commands
#include <RombiSerialCommand.h>
RombiSerialCommand SCmd;  //The SerialCommand object

//-- Rombi Library
#include <Rombi.h>
Rombi rombi;  //This is Rombi!!
 
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

//---Rombi Buttons
#define PIN_SecondButton 6
#define PIN_ThirdButton 7

#define MAX_LOOPS             3000

///////////////////////////////////////////////////////////////////
//-- Global Variables -------------------------------------------//
///////////////////////////////////////////////////////////////////

const char programID[]="ROMBI_BASE_v2"; //Each program will have a ID

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
//-- Rombi has 5 modes:
//--    * MODE = 0: Rombi is awaiting  
//--    * MODE = 1: Dancing mode!  
//--    * MODE = 2: Obstacle detector mode  
//--    * MODE = 3: Noise detector mode   
//--    * MODE = 4: RombiPAD or any Teleoperation mode (listening SerialPort). 
//---------------------------------------------------------
volatile int MODE=1; //State of rombi in the principal state machine. 

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

typedef enum
{
  COLOR_DETCT = 0,
  COLOR_VERIFICATION,
  COLOR_VERIFIED,
} RGBState;

int yellow[4] = {YELLOW, 230, 200, 115};

#define NUMBER_OF_COLORS_SPECIALS 1

int *colors[NUMBER_OF_COLORS_SPECIALS] = {yellow};

int color_index = 0;
int color_orders[50] = {};
int rgb_state = COLOR_DETCT;

int forward[2] = { RED, MOVESTRAIGHT };
int left[2] = { GREEN, MOVELEFT };
int right[2] = { BLUE, MOVERIGHT };
int stop[2] = { BLACK, STOP };
int back[2] = { YELLOW, MOVEBACK };

#define NUMBER_OF_ORDERS 5
#define THRESHOLD 30
int *orders_color[NUMBER_OF_ORDERS] = {forward, left, right, back, stop};

int test_sequence[9] = {YELLOW, RED, GREEN, BLUE, RED, GREEN, YELLOW, BLUE, GREEN};

#define NUMBER_OF_VERIFICATIONS 2
int num_ver = 0;

int returnColor(int *RGBval) {
  bool found;

  if (RGBval[0] > 200 && RGBval[1] > 200 && RGBval[2] > 200)
    return WHITE;

  if (RGBval[0] < 80 && RGBval[1] < 80 && RGBval[2] < 80)
    return BLACK;

  for (int i = 0; i < NUMBER_OF_COLORS_SPECIALS; i++) {
    found = true;
    for (int j = 1; j < 4; j++) {
      if (colors[i][j] - THRESHOLD > RGBval[j - 1] || colors[i][j] + THRESHOLD < RGBval[j - 1]) {
        found = false;
        break;
      }
    }

    if (found == true)
      return colors[i][0];
  }

  if (RGBval[0] > RGBval[1] && RGBval[0] > RGBval[2])
    return RED;

  if (RGBval[1] > RGBval[0] && RGBval[1] > RGBval[2])
    return GREEN;

  if (RGBval[2] > RGBval[0] && RGBval[2] > RGBval[1])
    return BLUE;
}

int executeOrder(int order) {
  bool found;

  for (int i = 0; i < NUMBER_OF_ORDERS; i++) {
    if (orders_color[i][0] != order)
      continue;

    switch (orders_color[i][1]) {
    case MOVESTRAIGHT:
      rombi.forward(2000);
      break;
    case MOVELEFT:
      rombi.left_order(1010);
      break;
    case MOVERIGHT:
      rombi.right_order(850);
      break;
    case STOP:
      rombi.stop(100);
      break;
    case MOVEBACK:
      rombi.back(2500);
      rombi.stop(100);
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
  rombi.init(PIN_RL,PIN_RR,false);
 
  //Uncomment this to set the servo trims manually and save on EEPROM 
    //rombi.setTrims(TRIM_YL, TRIM_YR, TRIM_RL, TRIM_RR);
    //rombi.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.

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



  //Rombi wake up!
  //rombi.sing(S_connection);
  rombi.home();


  //If Rombi's name is '&' (factory name) means that is the first time this program is executed.
  //This first time, Rombi mustn't do anything. Just born at the factory!
  //5 = EEPROM address that contains first name character
  if (EEPROM.read(5)==name_fac){ 

    EEPROM.put(5, name_fir); //From now, the name is '#'
    EEPROM.put(6, '\0'); 
    rombi.putMouth(culito);

    while(true){    
       delay(1000);
    }
  }  


  //Send Rombi name, programID & battery level.
  requestName();
  delay(50);
  requestProgramId();
  delay(50);
  requestBattery();
  
  //Checking battery
  RombiLowBatteryAlarm();


 // Animation Uuuuuh - A little moment of initial surprise
 //-----
  for(int i=0; i<2; i++){
      for (int i=0;i<8;i++){
        if(buttonPushed){break;}  
        rombi.putAnimationMouth(littleUuh,i);
        delay(150);
      }
  }
 //-----


  //Smile for a happy Rombi :)
  if(!buttonPushed){ 
    rombi.putMouth(smile);
    //rombi.sing(S_happy);
    delay(200);
  }


  //If Rombi's name is '#' means that Rombi hasn't been baptized
  //In this case, Rombi does a longer greeting
  //5 = EEPROM address that contains first name character
  if (EEPROM.read(5)==name_fir){ 

    if(!buttonPushed){  
        rombi.back(0.5);
        rombi.forward(0.5);
        delay(200); 
    }

    if(!buttonPushed){ 
        rombi.putMouth(smallSurprise);
        rombi.home();
    }  
  }


  if(!buttonPushed){ 
    rombi.putMouth(happyOpen);
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
    //rombi.putMouth(happyOpen);

    buttonPushed=false;
  }

  //First attemp to initial software
  if (buttonPushed){  

    rombi.home();

    delay(100); //Wait for all buttons 
    //rombi.sing(S_buttonPushed);
    delay(200); //Wait for all buttons 

    rombi.putMouth(MODE);
 
    int showTime = 2000;
    while((showTime>0)){ //Wait to show the MODE number 
        
        showTime-=10;
        delay(10);
    }
     
    //rombi.putMouth(happyOpen);

    buttonPushed=false;

  }else{

    switch (MODE) {

      //-- MODE 0 - Rombi is awaiting
      //---------------------------------------------------------
      case 0:
      
        //Every 80 seconds in this mode, Rombi falls asleep 
        if (millis()-previousMillis>=80000){
            RombiSleeping_withInterrupts(); //ZZzzzzz...
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
             bool test = true;

             //rombi.putMouth(smile);

             /*for (int i = 0; i < color_index; i++) {
               executeOrder(color_orders[i]);
             }*/

             for (int i = 0; i < 5; i ++) {
               for (int j = 0; j < 9; j++) {
                 if (color_orders[j + (i * 9)] != test_sequence[j]) {
                   test = false;
                   break;
                 }
               }

               if (test == false)
                 break;
             }

             if (test)
               rombi.putMouth(smile);
             else
               rombi.putMouth(sad);

             buttonBPushed=false;
           } else {
             rombi.putMouth(interrogation);
             buttonBPushed=false;
             return;
           }
         } else if (buttonAPushed == true) {
           if (rombi.getRGB(RGBValues)) {
             col = returnColor(RGBValues);
             if (col >= 0 && col != WHITE) {
               if (color_index > 0) {
                 if (col != color_orders[color_index - 1]) {
                   if (rgb_state == COLOR_VERIFICATION) {
                     if (col == color_orders[color_index]) {
                       if (num_ver >= COLOR_VERIFICATION) {
                         rgb_state = COLOR_VERIFIED;
                       } else {
                         rgb_state = COLOR_VERIFICATION;
                         num_ver = num_ver + 1;
                       }
                     } else {
                       color_orders[color_index] = col;
                     }
                   } else {
                     color_orders[color_index] = col;
                     if (NUMBER_OF_VERIFICATIONS > 0) {
                       rgb_state = COLOR_VERIFICATION;
                     } else {
                       rgb_state = COLOR_VERIFIED;
                     }
                   }
                 }
               } else {
                 if (rgb_state == COLOR_VERIFICATION) {
                   if (col == color_orders[color_index]) {
                     if (num_ver >= COLOR_VERIFICATION) {
                       rgb_state = COLOR_VERIFIED;
                     } else {
                       rgb_state = COLOR_VERIFICATION;
                       num_ver = num_ver + 1;
                     }
                   } else {
                     color_orders[color_index] = col;
                   }
                 } else {
                   color_orders[color_index] = col;
                   if (NUMBER_OF_VERIFICATIONS > 0) {
                     rgb_state = COLOR_VERIFICATION;
                   } else {
                     rgb_state = COLOR_VERIFIED;
                   }
                 }
               }

               if (rgb_state == COLOR_VERIFIED) {
                 rgb_state = COLOR_DETCT;
                 color_index = color_index + 1;
                 num_ver = 0;

                 if (col == RED) {
                   rombi.putMouth(one);
                 } else if (col == GREEN) {
                   rombi.putMouth(two);
                 } else if (col == YELLOW) {
                   rombi.putMouth(three);
                 } else if (col == BLUE) {
                   rombi.putMouth(four);
                 } else if (col == BLACK) {
                   rombi.putMouth(five);
                 }
               }
             }
           }

           sensorLeft = rombi.getIR(LEFT);
           sensorRight = rombi.getIR(RIGHT);
           if ((sensorLeft == LOW) && (sensorRight == LOW)) {
             rombi.forward(5);
             loops = 0;
             state = MOVESTRAIGHT;
             //rombi.putMouth(smile);
           } else if ((sensorLeft == LOW) && (sensorRight == HIGH)) {
             rombi.left(5);
             loops = 0;
             state = MOVELEFT;
             //rombi.putMouth(smile);
           } else if ((sensorLeft == HIGH) && (sensorRight == LOW)) {
             rombi.right(5);
             loops = 0;
             state = MOVERIGHT;
             //rombi.putMouth(smile);
           } else if ((sensorLeft == HIGH) && (sensorRight == HIGH)) {
             if (loops <= MAX_LOOPS) {
               if (state == MOVERIGHT) {
                 rombi.right(5);
                 loops++;
                 //rombi.putMouth(smile);
               } else if (state == MOVELEFT) {
                 rombi.left(5);
                 loops++;
                 //rombi.putMouth(smile);
               } else {
                 rombi.stop(5);
                 loops = 0;
                 state = STOP;
                 rombi.putMouth(sad);
               }
             } else {
               rombi.stop(5);
               loops = 0;
               state = STOP;
               rombi.putMouth(sad);
               color_orders[color_index] = BLACK;
               color_index = color_index + 1;
             }
           }
           if (color_index != 0 && color_orders[color_index - 1] == BLACK) {
             buttonAPushed = false;
           }
         } else {
           if (color_index != 0 && color_orders[color_index - 1] == BLACK) {
             rombi.stop(10);
             rombi.putMouth(smile);
           } else {
             rombi.stop(10);
             rombi.putMouth(sad);
           }
         }

        break;
        
      case 2: //Calibration RGB
        char buf[200];
        
        if (rombi.getRGB(RGBValues)) {
          col = returnColor(RGBValues);
          for (int i = 0; i < 3; i++) {
            sprintf(buf, "RGB[%d] = %d", i, RGBValues[i]);
            Serial.println(buf);
          }

          if (col == RED) {
            rombi.putMouth(one);
          } else if (col == GREEN) {
            rombi.putMouth(two);
          } else if (col == YELLOW) {
            rombi.putMouth(three);
          } else if (col == BLUE) {
            rombi.putMouth(four);
          } else if (col == BLACK) {
            rombi.putMouth(five);
          } else if (col == WHITE) {
            rombi.putMouth(six);
          }
        }
      break;

      //-- MODE 2 - RombiPAD or any Teleoperation mode (listening SerialPort) 
      //---------------------------------------------------------
      case 3:

        SCmd.readSerial();
        
        //If Rombi is moving yet
        if (rombi.getRestState()==false){  
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

   int distance = rombi.getDistance();

        if(distance<15){
          obstacleDetected = true;
        }else{
          obstacleDetected = false;
        }
}


//-- Function to receive Stop command.
void receiveStop(){

    sendAck();
    rombi.home();
    sendFinalAck();

}


//-- Function to receive LED commands
void receiveLED(){  

    //sendAck & stop if necessary
    sendAck();
    rombi.home();

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
      rombi.putMouth(matrix,false);
    }else{
      rombi.putMouth(xMouth);
      delay(2000);
      rombi.clearMouth();
    }

    sendFinalAck();

}


//-- Function to receive buzzer commands
void recieveBuzzer(){
  
    //sendAck & stop if necessary
    sendAck();
    rombi.home(); 

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

      rombi.putMouth(xMouth);
      delay(2000);
      rombi.clearMouth();

    }else{ 

      rombi._tone(frec, duration, 1);   
    }

    sendFinalAck();

}


//-- Function to receive TRims commands
void receiveTrims(){  

    //sendAck & stop if necessary
    sendAck();
    rombi.home(); 

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

      rombi.putMouth(xMouth);
      delay(2000);
      rombi.clearMouth();

    }else{ //Save it on EEPROM
      rombi.setTrims(trim_YL, trim_YR, trim_RL, trim_RR);
      rombi.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.
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

      rombi.putMouth(xMouth);
      delay(2000);
      rombi.clearMouth();

    }else{ //Update Servo:

      int servoPos[4]={servo_YL, servo_YR, servo_RL, servo_RR}; 
      rombi._moveServos(200, servoPos);   //Move 200ms
      
    }

    sendFinalAck();

}


//-- Function to receive movement commands
void receiveMovement(){

    sendAck();

    if (rombi.getRestState()==true){
        rombi.setRestState(false);
    }

    //Definition of Movement Bluetooth commands
    //M  MoveID  T   MoveSize  
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {moveId=atoi(arg);}
    else{
      rombi.putMouth(xMouth);
      delay(2000);
      rombi.clearMouth();
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
      rombi.home();
      break;
    case 1: //M 1 1000 
      rombi.left(T);
      break;
    case 2: //M 2 1000 
      rombi.right(T);
      break;
    case 3: //M 3 1000 
      rombi.forward(T);
      break;
    case 4: //M 4 1000 
      rombi.back(T);
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
    rombi.home(); 

    //Definition of Gesture Bluetooth commands
    //H  GestureID  
    int gesture = 0;
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {gesture=atoi(arg);}
    else 
    {
      rombi.putMouth(xMouth);
      delay(2000);
      rombi.clearMouth();
    }

    switch (gesture) {
      case 1: //H 1 
        rombi.playGesture(RombiHappy);
        break;
      case 2: //H 2 
        rombi.playGesture(RombiSuperHappy);
        break;
      case 3: //H 3 
        rombi.playGesture(RombiSad);
        break;
      case 4: //H 4 
        rombi.playGesture(RombiSleeping);
        break;
      case 5: //H 5  
        rombi.playGesture(RombiFart);
        break;
      case 6: //H 6 
        rombi.playGesture(RombiConfused);
        break;
      case 7: //H 7 
        rombi.playGesture(RombiLove);
        break;
      case 8: //H 8 
        rombi.playGesture(RombiAngry);
        break;
      case 9: //H 9  
        rombi.playGesture(RombiFretful);
        break;
      case 10: //H 10
        rombi.playGesture(RombiMagic);
        break;  
      case 11: //H 11
        rombi.playGesture(RombiWave);
        break;   
      case 12: //H 12
        rombi.playGesture(RombiVictory);
        break; 
      case 13: //H 13
        rombi.playGesture(RombiFail);
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
    rombi.home(); 

    //Definition of Sing Bluetooth commands
    //K  SingID    
    int sing = 0;
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {sing=atoi(arg);}
    else 
    {
      rombi.putMouth(xMouth);
      delay(2000);
      rombi.clearMouth();
    }

    switch (sing) {
      case 1: //K 1 
        rombi.sing(S_connection);
        break;
      case 2: //K 2 
        rombi.sing(S_disconnection);
        break;
      case 3: //K 3 
        rombi.sing(S_surprise);
        break;
      case 4: //K 4 
        rombi.sing(S_OhOoh);
        break;
      case 5: //K 5  
        rombi.sing(S_OhOoh2);
        break;
      case 6: //K 6 
        rombi.sing(S_cuddly);
        break;
      case 7: //K 7 
        rombi.sing(S_sleeping);
        break;
      case 8: //K 8 
        rombi.sing(S_happy);
        break;
      case 9: //K 9  
        rombi.sing(S_superHappy);
        break;
      case 10: //K 10
        rombi.sing(S_happy_short);
        break;  
      case 11: //K 11
        rombi.sing(S_sad);
        break;   
      case 12: //K 12
        rombi.sing(S_confused);
        break; 
      case 13: //K 13
        rombi.sing(S_fart1);
        break;
      case 14: //K 14
        rombi.sing(S_fart2);
        break;
      case 15: //K 15
        rombi.sing(S_fart3);
        break;    
      case 16: //K 16
        rombi.sing(S_mode1);
        break; 
      case 17: //K 17
        rombi.sing(S_mode2);
        break; 
      case 18: //K 18
        rombi.sing(S_mode3);
        break;   
      case 19: //K 19
        rombi.sing(S_buttonPushed);
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
    rombi.home(); 

    char newRombiName[11] = "";  //Variable to store data read from Serial.
    int eeAddress = 5;          //Location we want the data to be in EEPROM.
    char *arg; 
    arg = SCmd.next(); 
    
    if (arg != NULL) {

      //Complete newRombiName char string
      int k = 0;
      while((*arg) && (k<11)){ 
          newRombiName[k]=*arg++;
          k++;
      }
      
      EEPROM.put(eeAddress, newRombiName); 
    }
    else 
    {
      rombi.putMouth(xMouth);
      delay(2000);
      rombi.clearMouth();
    }

    sendFinalAck();

}


//-- Function to send Rombi's name
void requestName(){

    rombi.home(); //stop if necessary

    char actualRombiName[11]= "";  //Variable to store data read from EEPROM.
    int eeAddress = 5;            //EEPROM address to start reading from

    //Get the float data from the EEPROM at position 'eeAddress'
    EEPROM.get(eeAddress, actualRombiName);

    Serial.print(F("&&"));
    Serial.print(F("E "));
    Serial.print(actualRombiName);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send ultrasonic sensor measure (distance in "cm")
void requestDistance(){

    rombi.home();  //stop if necessary  

    int distance = rombi.getDistance();
    Serial.print(F("&&"));
    Serial.print(F("D "));
    Serial.print(distance);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send noise sensor measure
void requestNoise(){

    rombi.home();  //stop if necessary

    int microphone= rombi.getNoise(); //analogRead(PIN_NoiseSensor);
    Serial.print(F("&&"));
    Serial.print(F("N "));
    Serial.print(microphone);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send battery voltage percent
void requestBattery(){

    rombi.home();  //stop if necessary

    //The first read of the batery is often a wrong reading, so we will discard this value. 
    double batteryLevel = rombi.getBatteryLevel();

    Serial.print(F("&&"));
    Serial.print(F("B "));
    Serial.print(batteryLevel);
    Serial.println(F("%%"));
    Serial.flush();
}


//-- Function to send program ID
void requestProgramId(){

    rombi.home();   //stop if necessary

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

void RombiLowBatteryAlarm(){

    double batteryLevel = rombi.getBatteryLevel();

    if(batteryLevel<45){
        
      while(!buttonPushed){

          rombi.putMouth(thunder);
          rombi.bendTones (880, 2000, 1.04, 8, 3);  //A5 = 880
          
          delay(30);

          rombi.bendTones (2000, 880, 1.02, 8, 3);  //A5 = 880
          rombi.clearMouth();
          delay(500);
      } 
    }
}

void RombiSleeping_withInterrupts(){

  int bedPos_0[4]={90, 90}; 

  if(!buttonPushed){
    rombi._moveServos(700, bedPos_0);  
  }

  for(int i=0; i<4;i++){

    if(buttonPushed){break;}
      rombi.putAnimationMouth(dreamMouth,0);
      rombi.bendTones (100, 200, 1.04, 10, 10);
    
    if(buttonPushed){break;}
      rombi.putAnimationMouth(dreamMouth,1);
      rombi.bendTones (200, 300, 1.04, 10, 10);  

    if(buttonPushed){break;}
      rombi.putAnimationMouth(dreamMouth,2);
      rombi.bendTones (300, 500, 1.04, 10, 10);   

    delay(500);
    
    if(buttonPushed){break;}
      rombi.putAnimationMouth(dreamMouth,1);
      rombi.bendTones (400, 250, 1.04, 10, 1); 

    if(buttonPushed){break;}
      rombi.putAnimationMouth(dreamMouth,0);
      rombi.bendTones (250, 100, 1.04, 10, 1); 
    
    delay(500);
  } 

  if(!buttonPushed){
    rombi.putMouth(lineMouth);
    rombi.sing(S_cuddly);
  }

  rombi.home();
  if(!buttonPushed){rombi.putMouth(happyOpen);}  

}
