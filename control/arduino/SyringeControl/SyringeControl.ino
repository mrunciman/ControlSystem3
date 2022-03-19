
/*Pressure control sketch for syringe pump.

*/

#include <math.h>
#include <Wire.h>
//Sensor library - https://github.com/sparkfun/MS5803-14BA_Breakout/
#include <SparkFun_MS5803_I2C.h>
#include <uStepperSLite.h>
// #include <digitalWriteFast.h>
#define PI 3.1415926535897932384626433832795


////////////////////////////////////////////////////////
// Handshake variables
bool shakeFlag = false;
String shakeInput; // 3 bit password to assign pump name/position
char shakeKey[5] = "LHS";
// TOP = 4, RHS = 7, LHS = 8
// PRI = 9


////////////////////////////////////////////////////////
//uStepper S Lite Setup
#define MAXACCELERATION 10000       //Max acceleration in steps/s^2 (2000 = 5 mm/s^2)
#define MAXVELOCITY 3000           //Max velocity in steps/s (2000 is 5 mm/s)
float SPEEDP_HIGH = 1000.0;
float SPEEDP_LOW = 500.0;
float STEPS_PER_REV = 3200.0;
float DEG_PER_REV = 360.0;
float angPos = 0.0; //DEG_PER_REV * stepIn/STEPS_PER_REV = 360 * stepIn/STEPS_PER_REV;
float angMeas = 0.0; // stepCount = angMeas*STEPS_PER_REV/DEG_PER_REV
uStepperSLite stepper(MAXACCELERATION, MAXVELOCITY);

// USEFUL FUNCTIONS:
// stepper.isStalled()
// disableMotor();
// setMaxVelocity(float vel)
// runContinous(CCW)
// softStop(SOFT)
// getCurrentDirection()
// getMotorState()
// RPMToStepDelay
// currentPidSpeed
// moveToAngle()
// moveSteps(int32_t steps, bool dir, bool holdMode=BRAKEON)
// getStepsSinceReset() Steps applied in the clockwise direction is added and steps applied in the counter clockwise direction is subtracted.
// CCW moves piston forwards


/////////////////////////////////////////////////////
// Flag setup
volatile int pumpState;
bool disconFlag = false;
volatile bool serFlag = false;

////////////////////////////////////////////////////////
// Setting Serial input variables
char firstDigit = 0;  // For checking incoming communications
String flushInputBuffer;

////////////////////////////////////////////////////////
// Pressure sensor variables
MS5803 sensor(ADDRESS_LOW);//CSB pin pulled low, so address low
double pressureAbs = 1000.00; // Initial value
int pressureX10 = int(pressureAbs*10);
int PRESS_THRESH = 10;//mbar
int PRESS_MAX = 3500;
int PRESS_MIN = 400;
int PRESS_FINE = 50; // When within this threshold, use finer movements
double PRESS_LOW = 1000.00;
double pressSetpoint = 850.00;//mbar
bool pressFlag = true;
int pressureError;
int pressSteps = 64; // Num steps to move when calibrating
int SAMP_DIV = 6; // Factor to divide Timer2 frequency by
volatile int sampCount = 0;
volatile bool sampFlag = false;
// int OCR_2p5mmps = 7999;
// Calibration
int stateCount = 0;
int startTime;
int STABLE_TIME = 4000; // time in milliseconds reqd for pressure to be at calibration setpoint
bool lowFlag = false;
bool highFlag = true;
bool posPressFlag = false;
double PRESS_FORCE_TEST = 2000.00 + PRESS_THRESH;

////////////////////////////////////////////////////////
// Stepper variables

//For step count:
//Steps per revolution motor = 200 steps
//16 microsteps per step, so 3200 steps per revolution
//Lead = start*pitch      Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
//Steps/mm = (StepPerRev*Microsteps)/(Lead) = 400 steps/mm
int STEPS_PER_MM = 400; // Higher steps per mm value for ustepper s lite than old setup (old = 100 steps/mm)
int limitSteps = STEPS_PER_MM*2; // number of pulses for 2 mm
int prevMotorState = 0;
int motorState = 3;
volatile int stepCount = 0;
String stepRecv;
int stepIn;
int stepError = 0;

int fSamp = 21;
int stepsPerLoop = 2000/fSamp; // number of steps at max step frequency in fSamp Hz timestep
unsigned long tSampu = 1000000/fSamp; // (1/fSamp)e6 Time between samples in microseconds
unsigned long oneHour = 3600000000;
unsigned long tStep = oneHour;
unsigned long timeSinceStep = 0;
unsigned long timeNow;
unsigned long timeAtStep;
unsigned long writeTime;
unsigned long tStep2k = 1; // When tStep equals 500 us, 2 kHz pulse achieved - 666 us for 1500 Hz maximum - 

////////////////////////////////////////////////////////
// Messages
char data[64]; // Char array to write stepNo, pressure and time into
char endByte[3] = "E";
char disableMsg[3] = "D ";
char limitHit[3] = "L ";

////////////////////////////////////////////////////////
// Actuator geometry
float STROKE = 10.0; //Eq. triangle length in mm
float L0 = 50; //STROKE/(1.0 - 2.0/PI); // flat length for contraction = STROKE
float ACT_WIDTH = 18.0; // width of muscle in mm
float NUM_L = 4; // number of subdivisions in muscle
float A_SYRINGE = PI*pow(13.25, 2.0); // piston area in mm^2
float FACT_V = (ACT_WIDTH*pow(L0 , 2.0))/(2.0*NUM_L);
float MAX_V = FACT_V*(2.0/PI); // volume in mm^3 when fully actuated
// steps to fill actuator rounded down, minus some fraction of a timestep's worth
int maxSteps = ((MAX_V/A_SYRINGE)*STEPS_PER_MM - (3*stepsPerLoop/4)); 
int minSteps = 10;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Wire.begin();
    //Sensor startup - see https://github.com/sparkfun/MS5803-14BA_Breakout/
  sensor.reset();
  sensor.begin();
  delay(1000);

  pressureAbs = sensor.getPressure(ADC_4096);
  //Serial configuration
  Serial.begin(115200);
  // Wait here until serial port is opened
  while (!Serial){
    ;
  }

  // Disable all interrupts
  noInterrupts();
  
  //Initialise timer 2 - prescaler = 1024 gives ~61 - 15625 Hz range
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM21);   //Set CTC mode
  // Set OCR2A to 124 for 125 Hz timer, where serial
  // can be read and pressure taken every 6th cycle.
  // desired frequency = 16Mhz/(prescaler*(1+OCR)) 
    // 16e6/(1024*(255+1))= 61.035 Hz
    // 16e6/(1024*(124+1)) = 125 Hz
  OCR2A = 124;//124 for 125 Hz and 156 for 100 Hz
  // Set CS22, CS21 and CS20 bits for 1024 prescaler
  // TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   // Turn on
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts


  // uStepper S Lite setup - 
  //http://ustepper.com/docs/ustepperslite/html/classuStepperSLite.html#a9522d6afb14f71c6034ece6537180e00
  // stepper.setup();
  stepper.setup(PID, STEPS_PER_REV, 20.0, 0.1, 0.0, true);
  stepper.softStop(SOFT);
}


// Internal interrupt service routine, timer 2 overflow
ISR(TIMER2_COMPA_vect){
  interrupts(); //re-enable interrupts
  // This will be called at 125 Hz, so every SAMP_DIV times set sampling flag, reducing the frequency.
  if (sampCount == SAMP_DIV){
    sampFlag = true;
    serFlag = true;
    sampCount = 1;
  }
  else {
    sampCount += 1;
  }
}


void pressureRead() {
  pressureAbs = sensor.getPressure(ADC_4096);
  // Filter out false readings
  // Stop motor and wait if pressure exceeds maximum
  if (pressureAbs > PRESS_MAX){
    // extInterrupt = true;
    stepper.softStop(HARD); // Stop motor
  }
  else if (pressureAbs < 0){
    pressureAbs = 0.00;
  }
}


void pressInitZeroVol() {
  //Set state for motor motion based on comparison of pressure signal with setpoint
  pressureRead();
  pressureError = pressureAbs - pressSetpoint;
  prevMotorState = motorState;
  // Assign motor state based on pressure error
  if (pressureAbs < pressSetpoint - PRESS_THRESH){ // The pressure is less than setpoint minus threshold
    lowFlag = true;
    if (pressureAbs > pressSetpoint - 2*PRESS_THRESH){
      // Pressure is at setpoint minus between 1 and 2 times threshold
      motorState = 0;
      // Increment counter if previous state was also zero
      // Pressure is stable if counter reaches some limit
      if (prevMotorState == 0){
        stateCount = millis() - startTime;
      }
      // Set back to zero if not
      else{
        stateCount = 0;
        startTime = millis();
      }
    }
    else{
      // Pressure too low, move plunger forward
      motorState = 1;
    }
  }
  else { // Pressure is over lower bound
    if (lowFlag == true){
      if (pressureAbs <= pressSetpoint + PRESS_THRESH){
        // If we previously reached lower bound and within threshold, stop
        motorState = 0;
        // Increment counter if previous state was also zero
        // Pressure is stable if counter reaches some limit
        if (prevMotorState == 0){
          stateCount = millis() - startTime;
        }
        // Set back to zero if not
        else{
          stateCount = 0;
          startTime = millis();
        }
      }
      else{
        // If we previously reached lower bound but outside of threshold, move back
        motorState = 2;
        // Reset lower bound flag, to reach it again.
        lowFlag = false;
      }
    }
    else{ 
      // If haven't yet reached lower bound, move plunger back
      motorState = 2;
    }
  }

  // If close to target pressure, use finer movements
  if (motorState == 1 || motorState == 2){
    // if within 50 mbar of target pressure go slower
    if (abs(pressureError) < PRESS_FINE){ 
      pressSteps = 10;
      // stepper.setMaxVelocity(SPEEDP_LOW);
    }
    else{
      pressSteps = 50;
      // stepper.setMaxVelocity(SPEEDP_HIGH);
    }
  }

  switch (motorState) {
    case 0:
      // Stop motor
      // Serial.println("Stop");
      stepper.softStop(HARD);
      break;
    case 1:
      //Move motor forwards
      // if not moving or it is moving clockwise:
      // if (!stepper.getMotorState() || !stepper.getCurrentDirection()){ 
      stepper.moveSteps(pressSteps, CW, HARD);
      // }
      //Serial.println("INCREASE PRESSURE");
      break;
    case 2:
      //Move motor backwards
      // if not moving or it is moving anticlockwise
      // if (!stepper.getMotorState() || stepper.getCurrentDirection() ){ 
      stepper.moveSteps(pressSteps, CCW, HARD);
      // }
      //Serial.println("DECREASE PRESSURE");
      break;
    default:
      //Just in case nothing matches, stop motor
      // Serial.println("Default");
      stepper.softStop(HARD);
      break;
  }
}


void pressControl() {
  //Set state for motor motion based on comparison of pressure signal with setpoint
  //If within pressThresh mbar, don't move motor

  pressureRead();
  // pressureError = pressureAbs - pressSetpoint;
  prevMotorState = motorState;
  // Assign motor state based on pressure
  if (pressureAbs < pressSetpoint + PRESS_THRESH){
    if (pressureAbs > pressSetpoint - PRESS_THRESH){
      highFlag = true;
      // Pressure is at setpoint plus or minus threshold
      motorState = 0; // Stationary
      // Increment counter if previous state was also zero
      // Pressure is stable if counter reaches some limit
      if (prevMotorState == 0){
        stateCount = millis() - startTime;
      }
      // Set back to zero if pressure previously out of threshold
      else{
        stateCount = 0;
        startTime = millis();
      }
    }
    else{
      // Pressure too low, move plunger forward
      motorState = 1;
    }
  }
  else { // Pressure is higher than upper bound
    motorState = 2; // Move backwards
  }

  // If close to target pressure, use finer movements
  if (motorState == 1 || motorState == 2){
    // if within 50 mbar of target pressure go slower
    if (abs(pressureError) < PRESS_FINE){ 
      pressSteps = 10;
      // stepper.setMaxVelocity(SPEEDP_LOW);
    }
    else{
      pressSteps = 50;
      // stepper.setMaxVelocity(SPEEDP_HIGH);
    }
  }

  switch (motorState) {
    case 0:
      // Stop
      stepper.softStop(HARD);
      break;
    case 1:
      //Move motor forwards
      stepper.moveSteps(pressSteps, CW, HARD);
      break;
    case 2:
      //Move motor back
      stepper.moveSteps(pressSteps, CCW, HARD);
      break;
    default:
      //Stop just in case nothing matches
      stepper.softStop(HARD);
      break;
  }
}


//Function to read input from python script and confirm pump position (TOP, LHS, RHS).
void handShake() {
  while (Serial.available() > 0) {
    // Read input from serial and reply with pump name
    shakeInput = Serial.readStringUntil('\n');
    if (shakeInput != ""){
      sprintf(data, "%s\n", shakeKey);
      Serial.write(data);

      // Compare pump name with value read from serial
      // If they match, handshake is successful
      if (shakeInput == shakeKey){
        // Say handshake is complete for switch in loop()
        shakeFlag = true;

        // Enable the motor after handshaking
        stepper.enableMotor();
        shakeInput = "";

        // Start timer for pressure readings
        TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

        flushInputBuffer = Serial.readStringUntil('\n');
      }
    }
  }
}


//Function to read input in serial monitor and set the new desired pressure.
void readSerial() {
  firstDigit = Serial.read();
  // Control code sends capital S to receive stepCount
  // Capital S in ASCII is 83, so check for that:
  if (firstDigit == 83) {
    stepRecv = Serial.readStringUntil('\n');
    if (stepRecv == "Closed"){
      // Disable the motor
      disconFlag = true;
      writeSerial('D'); //Send disable message
      stepper.setup(PID, STEPS_PER_REV, 20.0, 0.1, 0.0, true);
      stepper.disableMotor();   
    }
    else{
      stepIn = stepRecv.toInt();
      if (stepIn > maxSteps){
        stepIn = maxSteps;
      }
      if (stepIn < minSteps){
        stepIn = minSteps;
      }
    }
  }
  else {
    flushInputBuffer = Serial.readStringUntil('\n');
  }
}


void writeSerial(char msg){
  writeTime = millis();
  pressureX10 = int(pressureAbs*10);

  if (msg == 'S'){ // Normal operation, send stepCount etc
    sprintf(data, "%06d,%d,%lu%s", stepCount, pressureX10, writeTime, endByte);
  }
  else if (msg == 'D'){ // Python cut off comms, acknowledge this
    sprintf(data, "%s%s,%d,%lu%s", disableMsg, shakeKey, pressureX10, writeTime, endByte);
  }
  else if (msg == 'L'){ // Limit switch hit, advise Python
    sprintf(data, "%s%s,%d,%lu%s", limitHit, shakeKey, pressureX10, writeTime, endByte);
  }
  else if (msg == 'p'){ // Calibrating
    sprintf(data, "%04d%s,%d,%lu%s", STABLE_TIME-stateCount, shakeKey, pressureX10, writeTime, endByte);
  }
  else if(msg = 'P'){ // Calibration finished
    sprintf(data, "%06d%s,%d,%lu%s", stepCount, shakeKey, pressureX10, writeTime, endByte);
  }
  Serial.write(data);
}


void loop() {
  if (0 == true){    // Use stepper.isStalled()  ?
    pumpState = 0;//Limits hit
  }
  else if(shakeFlag == false){
    pumpState = 1;//Handshake
  }
  else if(disconFlag == true){
    pumpState = 2;//Disconnection
  }
  else if(pressFlag == false){//CHANGE TO TRUE TO ACTIVATE
    pumpState = 3;//Calibration
  }
  else{
    pumpState = 4;//Active
  }

  switch(pumpState){
    //////////////////////////////////////////////////////////////////////////////////////////
    //Limits hit
    case 0:
      stepper.softStop(SOFT);
      //Make sure motor is disabled 
      stepper.disableMotor(); // digitalWrite(enablePin, HIGH);
      // Turn off timers for interrupts
      // TCCR2B = 0; 
      stateCount = 0;
      startTime = millis();
      if (sampFlag == true) {
        pressureRead();
        sampFlag = false;
      }
      flushInputBuffer = Serial.readStringUntil('\n');
      // Notify that limit hit
      writeSerial('L');
      stepper.setup(PID, STEPS_PER_REV, 20.0, 0.1, 0.0, true);
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Handshake
    case 1:
      handShake();
      break;
    
    //////////////////////////////////////////////////////////////////////////////////////////
    //Disconnection
    case 2:
      // Stop motor
      stepper.softStop(SOFT);
      // Turn off timer for interrupts
      TCCR2B = 0;
      // Move back into handshaking mode
      disconFlag = false;
      shakeFlag = false;
      pressFlag = true;

      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Calibration
    case 3:
      if (sampFlag == true) {
        sampFlag = false;

        if (posPressFlag == true){ // Positive pressure test
          // Toggle between high and low setpoints
          if (pressSetpoint == PRESS_FORCE_TEST){
            pressControl();
            if (pressureAbs >= PRESS_FORCE_TEST){
              pressSetpoint = PRESS_LOW;
            }
          }
          else if (pressSetpoint == PRESS_LOW){
            pressInitZeroVol();
            if (pressureAbs <= PRESS_LOW){
              pressSetpoint = PRESS_FORCE_TEST;
            }
          }
          angMeas = stepper.encoder.getAngleMoved();
          stepCount = int(angMeas*STEPS_PER_REV/DEG_PER_REV);
          writeSerial('S');
        }
        else{
          // CALIBRATE
          pressInitZeroVol();
          // If enough time has passed, say volume is 0, tell python and move on
          if (stateCount >= STABLE_TIME){
            // Step count should now be zero - muscle empty.
            stepCount = 0;
            stepper.encoder.setHome();
            // Notify that calibration is done
            writeSerial('P');
            stepper.hardStop(HARD);
            // posPressFlag = true; // TRANSITION HERE TO POSITIVE PRESSURE CONTROL
            // pressSetpoint = PRESS_FORCE_TEST;
            stateCount = 0;
            pressFlag = false; // TRANSITION TO ACTIVE STATE (OVERRIDES POSITIVE PRESSURE FLAG)
          }
        }

        // Check for disconnection
        if (Serial.available() > 0) {
          firstDigit = Serial.read();
          if (firstDigit == 83) {  // Check for 'S'
            stepRecv = Serial.readStringUntil('\n');
            if (stepRecv == "Closed"){
              disconFlag = true;
              writeSerial('D');
              // Reset using the setup function
              stepper.setup(PID, STEPS_PER_REV, 20.0, 0.1, 0.0, true);
              
            }
          }
        }
        // If no reply, say calibration in progress
        else{
          writeSerial('p');
        }
        
      }
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Active
    case 4:
      // Call overpressure protection every 6th Timer2 interrupt
      if (sampFlag == true) {
        pressureRead();
        sampFlag = false;
      }

      if (serFlag == true) { //Changed from serFlag to see if I can make things faster   if (Serial.available() > 0) 
        if (Serial.available() > 0){ 
          readSerial(); // Read stepIn
          angPos = DEG_PER_REV*(float(stepIn)/STEPS_PER_REV);
          angMeas = stepper.encoder.getAngleMoved();
          stepCount = int(angMeas*STEPS_PER_REV/DEG_PER_REV);
          //Send stepCount
          writeSerial('S');
          serFlag = false;
        }
      }
      else if (!Serial) { //if serial disconnected, go to disconected state
        disconFlag = true;
        writeSerial('D');
        stepper.setup(PID, STEPS_PER_REV, 20.0, 0.1, 0.0, true);
      }
      // Move to the desired position
      stepper.moveToAngle(angPos, HARD);
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    // Unrecognised state, act as if limit hit
    default:
      stepper.softStop(SOFT);
      //Make sure motor is disabled 
      stepper.disableMotor(); // digitalWrite(enablePin, HIGH);
      // Turn off timers for interrupts
      TCCR2B = 0;
      stateCount = 0;
      startTime = millis();
      pressureRead();
      flushInputBuffer = Serial.readStringUntil('\n');
      // Notify Python
      writeSerial('L');
      break;
  }
}
