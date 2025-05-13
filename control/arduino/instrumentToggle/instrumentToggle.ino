
// Staticboard CNC Shield - https://github.com/staticboards/sb-cnc-shield
#include <AccelStepper.h>
#include <MultiStepper.h>



// X = 9, Y = 10, and Z = 11 (SpinDir) on SB CNC board
int limitPin_RHS_FWD = 9;  // X limit on board
int limitPin_RHS_BWD = 10; // Y limit on board
// Pins are free from unused Z and Spin drivers so use these for LHS
int limitPin_LHS_FWD = 7; // Z direction on board
int limitPin_LHS_BWD = 4; // Z step on board
// pin 12 also free
int pinForward_RHS =  13; // spin direction
int pinBackward_RHS = 12; // spin enable on board
int pinForward_LHS =  11; // z limit on board
int pinBackward_LHS = 3;

// Shield RHS motor:
const byte enablePin = 8;
const byte direction_RHS = 5;
const byte stepPin_RHS = 2;
// Shield LHS motor:
// const byte enablePin2 = 8;
const byte direction_LHS = 6;
const byte stepPin_LHS = 3;


// Motion variables / parameters
float RHS_SPEED = 5.0;
float LHS_SPEED = RHS_SPEED;
float speedRHS = 0.0; // mm/s 
float speedLHS = speedRHS;
float PULLEY_RADIUS = 6.0; //mm
float omegaRHS = speedRHS/PULLEY_RADIUS;
float omegaLHS = speedLHS/PULLEY_RADIUS;

int MICROSTEPS_RHS = 4;
int STEPS_PER_REV_RHS = 200* MICROSTEPS_RHS;
int MICROSTEPS_LHS = 4;
int STEPS_PER_REV_LHS = 200* MICROSTEPS_LHS;

float DIST_AWAY_SWITCH = 10.0; //mm
float RADS_AWAY_SWITCH = DIST_AWAY_SWITCH/PULLEY_RADIUS;
float REVS_AWAY_SWITCH = RADS_AWAY_SWITCH/(2*PI);
long STEPS_AWAY_SWITCH = int(REVS_AWAY_SWITCH*STEPS_PER_REV_RHS);

float stepRateRHS = STEPS_PER_REV_RHS*omegaRHS;
float stepRateLHS = STEPS_PER_REV_LHS*omegaLHS;


long posRHS = 0;
long posLHS = 0;
long MOVE_STEPS = 10000;



// The states will determine whether the motors run and their direction
// 0    ->  no motion
// 1    ->  forwards
// 2    ->  backwards
#define STOP 0
#define FORWARDS 1
#define BACKWARDS 2
int motorState_RHS = STOP;
int motorState_LHS = STOP;

int prevMotorState_RHS = motorState_RHS;
int prevMotorState_LHS = motorState_LHS;


AccelStepper stepperRHS(AccelStepper::DRIVER, stepPin_RHS, direction_RHS);
AccelStepper stepperLHS(AccelStepper::DRIVER, stepPin_LHS, direction_LHS);
MultiStepper steppers;



void setup() {

  Serial.begin(115200);

  // Set pins as outputs to driver
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);


  // Set max speeds
  stepperLHS.setMaxSpeed(2000);
  stepperRHS.setMaxSpeed(2000);


  // Use multistepper to handle both motors
  // steppers.addStepper(stepperLHS);
  // steppers.addStepper(stepperRHS);


  // Setup the initialise, RHS, and LHS limit switches
  pinMode(limitPin_RHS_FWD, INPUT_PULLUP);
  pinMode(limitPin_RHS_BWD, INPUT_PULLUP);
  pinMode(limitPin_LHS_FWD, INPUT_PULLUP);
  pinMode(limitPin_LHS_BWD, INPUT_PULLUP);

  pinMode(pinForward_RHS,  INPUT_PULLUP);
  pinMode(pinBackward_RHS, INPUT_PULLUP);
  pinMode(pinForward_LHS,  INPUT_PULLUP);
  pinMode(pinBackward_LHS, INPUT_PULLUP);


}





void checkEndStops(){
  int frontHit_RHS = digitalRead(limitPin_RHS_FWD); // X limit on board
  int backHit_RHS  = digitalRead(limitPin_RHS_BWD); // Y limit on board  

  if (frontHit_RHS == false){
    Serial.println("RHS Front endstop");
    digitalWrite(enablePin, LOW);
    // stepperRHS.moveTo(stepperRHS.currentPosition() - STEPS_AWAY_SWITCH);

    speedRHS = -RHS_SPEED;
    omegaRHS = speedRHS/PULLEY_RADIUS;
    stepRateRHS = STEPS_PER_REV_RHS*omegaRHS;

    stepperRHS.setSpeed(stepRateRHS);

    stepperRHS.moveTo(stepperRHS.currentPosition() + STEPS_AWAY_SWITCH);

    stepperRHS.runSpeedToPosition();
    
    digitalWrite(enablePin, HIGH);
    speedRHS = 0;

  }
  else if(backHit_RHS == false){
    Serial.println("RHS Back endstop");
    digitalWrite(enablePin, LOW);

    speedRHS = RHS_SPEED;
    omegaRHS = speedRHS/PULLEY_RADIUS;
    stepRateRHS = STEPS_PER_REV_RHS*omegaRHS;
    
    stepperRHS.setSpeed(stepRateRHS);

    stepperRHS.moveTo(stepperRHS.currentPosition() - STEPS_AWAY_SWITCH);

    stepperRHS.runSpeedToPosition();

    digitalWrite(enablePin, HIGH);
    speedRHS = 0;

  }

  int frontHit_LHS = digitalRead(limitPin_LHS_FWD); // Z direction on board
  int backHit_LHS  = digitalRead(limitPin_LHS_BWD); // Z steo on board  

  if (frontHit_LHS == false){
    Serial.println("LHS Front endstop");
    digitalWrite(enablePin, LOW);

    speedLHS = -LHS_SPEED;
    omegaLHS = speedLHS/PULLEY_RADIUS;
    stepRateLHS = STEPS_PER_REV_LHS*omegaLHS;

    stepperLHS.setSpeed(stepRateLHS);

    stepperLHS.moveTo(stepperLHS.currentPosition() + STEPS_AWAY_SWITCH);

    stepperLHS.runSpeedToPosition();

    digitalWrite(enablePin, HIGH);
    speedLHS = 0;
  }
  else if(backHit_LHS == false){
    Serial.println("LHS Back endstop");
    digitalWrite(enablePin, LOW);

    speedLHS = LHS_SPEED;
    omegaLHS = speedLHS/PULLEY_RADIUS;
    stepRateLHS = STEPS_PER_REV_LHS*omegaLHS;

    stepperLHS.setSpeed(stepRateLHS);

    stepperLHS.moveTo(stepperLHS.currentPosition() - STEPS_AWAY_SWITCH);

    stepperLHS.runSpeedToPosition();

    digitalWrite(enablePin, HIGH);
    speedLHS = 0;
  }
}





void checkEndStops2(){
  int frontHit_RHS = digitalRead(limitPin_RHS_FWD); // X limit on board
  int backHit_RHS  = digitalRead(limitPin_RHS_BWD); // Y limit on board  

  if (frontHit_RHS == false){
    Serial.println("RHS Front endstop");
    digitalWrite(enablePin, LOW);
    // stepperRHS.moveTo(stepperRHS.currentPosition() - STEPS_AWAY_SWITCH);

    speedRHS = -RHS_SPEED;
    omegaRHS = speedRHS/PULLEY_RADIUS;
    stepRateRHS = STEPS_PER_REV_RHS*omegaRHS;

    stepperRHS.setSpeed(stepRateRHS);

    stepperRHS.moveTo(stepperRHS.currentPosition() + STEPS_AWAY_SWITCH);

  }
  else if(backHit_RHS == false){
    Serial.println("RHS Back endstop");
    digitalWrite(enablePin, LOW);

    speedRHS = RHS_SPEED;
    omegaRHS = speedRHS/PULLEY_RADIUS;
    stepRateRHS = STEPS_PER_REV_RHS*omegaRHS;
    
    stepperRHS.setSpeed(stepRateRHS);

    stepperRHS.moveTo(stepperRHS.currentPosition() - STEPS_AWAY_SWITCH);

    stepperRHS.runSpeedToPosition();

    digitalWrite(enablePin, HIGH);
    speedRHS = 0;

  }

  int frontHit_LHS = digitalRead(limitPin_LHS_FWD); // Z direction on board
  int backHit_LHS  = digitalRead(limitPin_LHS_BWD); // Z steo on board  

  if (frontHit_LHS == false){
    Serial.println("LHS Front endstop");
    digitalWrite(enablePin, LOW);

    speedLHS = -LHS_SPEED;
    omegaLHS = speedLHS/PULLEY_RADIUS;
    stepRateLHS = STEPS_PER_REV_LHS*omegaLHS;

    stepperLHS.setSpeed(stepRateLHS);

    stepperLHS.moveTo(stepperLHS.currentPosition() + STEPS_AWAY_SWITCH);

    stepperLHS.runSpeedToPosition();

    digitalWrite(enablePin, HIGH);
    speedLHS = 0;
  }
  else if(backHit_LHS == false){
    Serial.println("LHS Back endstop");
    digitalWrite(enablePin, LOW);

    speedLHS = LHS_SPEED;
    omegaLHS = speedLHS/PULLEY_RADIUS;
    stepRateLHS = STEPS_PER_REV_LHS*omegaLHS;

    stepperLHS.setSpeed(stepRateLHS);

    stepperLHS.moveTo(stepperLHS.currentPosition() - STEPS_AWAY_SWITCH);

    stepperLHS.runSpeedToPosition();

    digitalWrite(enablePin, HIGH);
    speedLHS = 0;
  }
}





// Set speeds given a desired linear speed and angular speed
void setSpeedsV(){

  switch(motorState_RHS){
    case STOP:
      speedRHS = 0.0;
      break;

    case FORWARDS:
      speedRHS = RHS_SPEED;
      break;

    case BACKWARDS:
      speedRHS = -RHS_SPEED;
      break;
  }

  switch(motorState_LHS){
    case STOP:
      speedLHS = 0.0;
      break;

    case FORWARDS:
      speedLHS = RHS_SPEED;
      break;

    case BACKWARDS:
      speedLHS = -RHS_SPEED;
      break;
  }

  // Checking endstops will set zero speed if either one is hit
  checkEndStops();
  
  omegaRHS = speedRHS/PULLEY_RADIUS;
  omegaLHS = speedLHS/PULLEY_RADIUS;

  stepRateRHS = STEPS_PER_REV_RHS*omegaRHS; // [steps/2pi]*[2*pi/s]
  stepRateLHS = STEPS_PER_REV_LHS*omegaLHS; // [steps/2pi]*[2*pi/s]
  
  stepperRHS.setSpeed(stepRateRHS);
  stepperLHS.setSpeed(stepRateLHS); 

  // Serial.println(stepRateRHS);
  // Serial.println(stepRateLHS);
}




void setPositions(){

  switch(motorState_RHS){
    case STOP:
      speedRHS = 0.0;
      posRHS = stepperRHS.currentPosition();
      break;

    case FORWARDS:
      speedRHS = RHS_SPEED;
      posRHS = stepperRHS.currentPosition() + MOVE_STEPS;
      break;

    case BACKWARDS:
      speedRHS = -RHS_SPEED;
      posRHS = stepperRHS.currentPosition() - MOVE_STEPS;
      break;
  }

  switch(motorState_LHS){
    case STOP:
      speedLHS = 0.0;
      posLHS = stepperLHS.currentPosition();
      break;

    case FORWARDS:
      speedLHS = RHS_SPEED;
      posLHS = stepperLHS.currentPosition() + MOVE_STEPS;
      break;

    case BACKWARDS:
      speedLHS = -RHS_SPEED;
      posLHS = stepperLHS.currentPosition() - MOVE_STEPS;
      break;
  }

  omegaRHS = speedRHS/PULLEY_RADIUS;
  omegaLHS = speedLHS/PULLEY_RADIUS;

  stepRateRHS = STEPS_PER_REV_RHS*omegaRHS; // [steps/2pi]*[2*pi/s]
  stepRateLHS = STEPS_PER_REV_LHS*omegaLHS; // [steps/2pi]*[2*pi/s]

  long positions[2];
  positions[0] = posRHS;
  positions[1] = posLHS;  
  steppers.moveTo(positions);
  
  stepperRHS.setSpeed(stepRateRHS);
  stepperLHS.setSpeed(stepRateLHS); 

  // Serial.println(stepRateRHS);
  // Serial.println(stepRateLHS);
}






void setStates(){
  int rhs_fwd = digitalRead(pinForward_RHS);
  int rhs_bwd = digitalRead(pinBackward_RHS);

  prevMotorState_RHS = motorState_RHS;
  // Here 0 means that the button on haptic has been pressed
  // Doing XOR operation on buttons
  if((rhs_fwd == 1) && (rhs_bwd == 1)) {
    motorState_RHS = STOP; // 
  }
  else if ((rhs_fwd == 0) && (rhs_bwd == 1)){
    motorState_RHS = FORWARDS; // Forwards
  }
  else if ((rhs_fwd == 1) && (rhs_bwd == 0)){
    motorState_RHS = BACKWARDS; // Backwards
  }
  else if ((rhs_fwd == 0) && (rhs_bwd == 0)){
    motorState_RHS = STOP; // No motion
  }

  // Enable motor and set direction based on state
  switch(motorState_RHS){
    case STOP:
      digitalWrite(enablePin, HIGH);
      // speedRHS = 0.0;
      break;

    case FORWARDS:
      digitalWrite(enablePin, LOW);
      // speedRHS = RHS_SPEED;
      break;

    case BACKWARDS:
      digitalWrite(enablePin, LOW);
      // speedRHS = -RHS_SPEED;
      break;
  }


  // Same for left side
  int lhs_fwd = digitalRead(pinForward_LHS);
  int lhs_bwd = digitalRead(pinBackward_LHS);

  prevMotorState_LHS = motorState_LHS;
  // Here 0 means that the button on haptic has been pressed
  // Doing XOR operation on buttons
  if((lhs_fwd == 1) && (lhs_bwd == 1)) {
    motorState_LHS = STOP; // 
  }
  else if ((lhs_fwd == 0) && (lhs_bwd == 1)){
    motorState_LHS = FORWARDS; // Forwards
  }
  else if ((lhs_fwd == 1) && (lhs_bwd == 0)){
    motorState_LHS = BACKWARDS; // Backwards
  }
  else if ((lhs_fwd == 0) && (lhs_bwd == 0)){
    motorState_LHS = STOP; // No motion
  }


  // Enable motor and set direction based on state
  // This won't work right because enable is controlled by RHS, only one enable
  // switch(motorState_LHS){
  //   case STOP:
  //     digitalWrite(enablePin, HIGH);
  //     // speedLHS = 0.0;
  //     break;

  //   case FORWARDS:
  //     digitalWrite(enablePin, LOW);
  //     // speedLHS = LHS_SPEED;
  //     break;

  //   case BACKWARDS:
  //     digitalWrite(enablePin, LOW);
  //     // speedLHS = -LHS_SPEED;
  //     break;
  // }

}







void loop() {

  // Sit idle until one of the digital inputs 
  // setStates();
  // setSpeedsV();


  // stepperLHS.runSpeed();
  // stepperRHS.runSpeed();


  // ALternatively, set desired position depending on buttons 
  //(very far for movement states to effectively have constant speed and 
  // currentPosition for no movement) and use steppers.run() in loop. 
  // Then set desired position on endstop press and a flag. 
  // Only reset flag to allow movement when button status changes.
  setStates(); // This disables or enables motors
  setPositions(); // This sets desired postions
  checkEndStops2(); // Re-enable motors if necessary and set position. Disable, set zero speed, and set desPos to currPos if position reached.
  steppers.run();
}
