#include "StepMotors.h"

// Driver parameters
#define MICROSTEPS          16

// Stepper motor parameters
#define DEG_STEP            7.5
#define GEAR_RATIO          5
#define STEPS_REVOLUTION    (360*GEAR_RATIO/DEG_STEP)

// Acceleration profile
#define ACC_PERCENTAGE      20    // Percentage of the time accelerating
#define INITIAL_RPM_W       15    // Wheel rpm
#define INITIAL_DELAY       ((((60000000*DEG_STEP)/(INITIAL_RPM_W*GEAR_RATIO*360.0*MICROSTEPS))-0.8)/2.0)

// Deceleration profile
#define DEC_PERCENTAGE      20    // Percentage of the time decelerating
#define FINAL_RPM_W         15    // Wheel rpm
#define FINAL_DELAY         ((((60000000*DEG_STEP)/(FINAL_RPM_W*GEAR_RATIO*360.0*MICROSTEPS))-0.8)/2.0)

// Rombi parameters
#define ROMBI_WIDTH         109   // 109 mm
#define WHEEL_WIDTH         8     // 8 mm
#define WHEEL_RADIO         30.2    // 30.2 mm

//****** StepMotors ******//
StepMotors::StepMotors() {
}

StepMotors::StepMotors(int dirPin1, int dirPin2) {
  StepMotors::init(dirPin1, dirPin2);
}

void StepMotors::init(int dirPin1, int dirPin2)
{
  _dirPin1 = dirPin1;
  _dirPin2 = dirPin2;

  pinMode(_dirPin1, OUTPUT);
  pinMode(_dirPin2, OUTPUT);
}

uint8_t StepMotors::moveAcc(Direction movem, uint16_t milimeters, uint16_t rpm) {
  uint8_t acceleration = 0;
  uint8_t deceleration = 0;
  uint16_t steps;
  uint32_t i = 0;
  uint32_t usteps = 0;
  float del = INITIAL_DELAY;
  uint16_t req_del = 0;
  float acc_ramp = 0.0;
  uint32_t acc_usteps = 0;
  float dec_ramp = 0.0;
  uint32_t dec_usteps = 0;

  if (movem == STEP_LEFT_ORDER || movem == STEP_RIGHT_ORDER) {
    steps  = milimeters;
  } else {
    steps = ((STEPS_REVOLUTION*milimeters)/(2*PI*WHEEL_WIDTH));
  }

  if ((movem >= STEP_LAST_VAL) || (steps == 0) || (rpm == 0)) {
    return 1;
  }

  if ((ACC_PERCENTAGE + DEC_PERCENTAGE) > 100)    return 2;

  req_del = (uint16_t) (((60000000*DEG_STEP)/(rpm*GEAR_RATIO*360.0*MICROSTEPS))-0.8)/2.0;
  usteps = (uint32_t) steps*MICROSTEPS;

  acc_usteps = (uint32_t) (ACC_PERCENTAGE * usteps)/100;
  dec_usteps = (uint32_t) ((100 - DEC_PERCENTAGE)*usteps)/100;

  if (acc_usteps > 0) {
    acc_ramp = (INITIAL_DELAY - req_del)/acc_usteps;
  } else {
    acc_ramp = 0;
  }

  dec_ramp = (FINAL_DELAY - req_del)/(usteps - dec_usteps);

  if ((rpm > INITIAL_RPM_W) && (acc_ramp > 0)) {
    acceleration = 1;
  } else {
    acceleration = 0;
  }

  if ((rpm > FINAL_RPM_W) && (dec_ramp > 0)) {
    deceleration = 1;
  } else {
    deceleration = 0;
  }

  switch (movem) {
    case STEP_FORWARD:
      digitalWrite(_dirPin1, STEP_BACKWARDS);
      digitalWrite(_dirPin2, STEP_FORWARD);
      break;
    case STEP_BACKWARDS:
      digitalWrite(_dirPin1, STEP_FORWARD);
      digitalWrite(_dirPin2, STEP_BACKWARDS);
      break;
    case STEP_LEFT:
      digitalWrite(_dirPin1, STEP_FORWARD);
      digitalWrite(_dirPin2, STEP_FORWARD);
      break;
    case STEP_RIGHT:
      digitalWrite(_dirPin1, STEP_BACKWARDS);
      digitalWrite(_dirPin2, STEP_BACKWARDS);
      break;
    default:
      break;
  }

  if (acceleration == 1) {
    while (i < acc_usteps) {
      if (del > req_del) {
        del -= acc_ramp;
      }

      PORTD |= (B110000);  //190 ns    Set PORTD4 and PORTD5 high
      delayMicroseconds((uint16_t) del);
      PORTD &= ~(B110000);  //190 ns    Set PORTD4 and PORTD5 low
      delayMicroseconds((uint16_t) del);
      i++;  //325 ns
    }
  } else {
    del = req_del;
    while (i < acc_usteps) {
      PORTD |= (B110000);  //190 ns    Set PORTD4 and PORTD5 high
      delayMicroseconds(req_del);
      PORTD &= ~(B110000);  //190 ns    Set PORTD4 and PORTD5 low
      delayMicroseconds(req_del);
      i++;  //325 ns
    }
  }

  while (i < dec_usteps) {
    PORTD |= (B110000);  //190 ns    Set PORTD4 and PORTD5 high
    delayMicroseconds(req_del);
    PORTD &= ~(B110000);  //190 ns    Set PORTD4 and PORTD5 low
    delayMicroseconds(req_del);
    i++;  //325 ns
  }

  if (deceleration == 1) {
    while (i < usteps) {
      if (del < FINAL_DELAY) {
        del += dec_ramp;
      }

      PORTD |= (B110000);  //190 ns    Set PORTD4 and PORTD5 high
      delayMicroseconds((uint16_t) del);
      PORTD &= ~(B110000);  //190 ns    Set PORTD4 and PORTD5 low
      delayMicroseconds((uint16_t) del);
      i++;  //325 ns
    }
  } else {
    while (i < usteps) {
      PORTD |= (B110000);  //190 ns    Set PORTD4 and PORTD5 high
      delayMicroseconds(req_del);
      PORTD &= ~(B110000);  //190 ns    Set PORTD4 and PORTD5 low
      delayMicroseconds(req_del);
      i++;  //325 ns
    }
  }
  i = 0;
  return 0;
}
