
/*
Pressure control sketch for syringe pumps
*/

#include <math.h>
#include <Wire1.h>
#include <SparkFun_MS5803_TwoI2C.h>
#include <uStepperS.h>
#define PI 3.1415926535897932384626433832795

////////////////////////////////////////////////////////
// Handshake variables
String shakeInput; // 3 bit password to assign pump name/position
char shakeKey[5] = "LHS"; //

////////////////////////////////////////////////////////
//uStepper S Lite Setup
#define MAXACCELERATION 2000       //Max acceleration in steps/s^2 (2000 = 5 mm/s^2)
#define MAXVELOCITY 2000           //Max velocity in steps/s (2000 is 5 mm/s)
float SPEEDP_HIGH = 1000.0;
float SPEEDP_LOW = 500.0;
float STEPS_PER_REV = 3200.0;
float DEG_PER_REV = 360.0;
float angPos = 0.0; //DEG_PER_REV * stepIn/STEPS_PER_REV = 360 * stepIn/STEPS_PER_REV;
float angMeas = 0.0; // stepCount = angMeas*STEPS_PER_REV/DEG_PER_REV
uStepperS stepper;

/////////////////////////////////////////////////////
// Flag setup
int pumpState;
bool shakeFlag = false;
bool disconFlag = false;
bool pressFlag = true;

////////////////////////////////////////////////////////
// Setting Serial input variables
char firstDigit = 0;  // For checking incoming communications
String flushInputBuffer;
bool serErrorFlag = false;

////////////////////////////////////////////////////////
// Pressure sensor variables
MS5803 sensor(ADDRESS_LOW);//CSB pin pulled low, so address low
double pressureAbs = 1000.00; // Initial value
int PRESS_THRESH = 10;//mbar
int PRESS_MAX = 2500;
int PRESS_MIN = 400;
int PRESS_FINE = 50; // When within this threshold, use finer movements
double pressSetpoint = 850.00;//mbar
int pressureError;
int pressSteps = 64; // Num steps to move when calibrating
int PRESS_STEPS_FINE = 20;
int PRESS_STEPS_LG = 40;

////////////////////////////////////////////////////////
// Calibration
int stateCount = 0;
int startTime;
int STABLE_TIME = 4000; // time in milliseconds reqd for pressure to be at calibration setpoint
bool lowFlag = false;
bool highFlag = true;

////////////////////////////////////////////////////////
// Stepper variables

//For step count:
//Steps per revolution motor = 200 steps
//16 microsteps per step, so 3200 steps per revolution
//Lead = start*pitch      Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
//Steps/mm = (StepPerRev*Microsteps)/(Lead) = 400 steps/mm
int STEPS_PER_MM = 400; // Higher steps per mm value for ustepper s lite than old setup (old = 100 steps/mm)
int prevMotorState = 0;
int motorState = 0;
int stepCount = 0;
// String stepRecv;
int prevStepIn = 10;
char stepRecv[7] = "00000";
char byteRead = 0;
int availableBytes = 0;
long int stepIn = 0;
int stepError = 0;

///////////////////////////////////////////////////////
unsigned long timeSinceStep = 0;
unsigned long timeNow;
unsigned long timeAtStep;
unsigned long writeTime;


////////////////////////////////////////////////////////
// Messages
char data[54]; // Char array to write stepNo, pressure and time into
char endByte[3] = "E";
char disableMsg[3] = "D ";
char limitHit[3] = "L ";

////////////////////////////////////////////////////////
// Actuator geometry
float STROKE = 25.0; //Eq. triangle length in mm
float L0 = STROKE/(1.0 - 2.0/PI); // flat length for contraction = STROKE
float ACT_WIDTH = 30.0; // width of muscle in mm
float NUM_L = 3; // number of subdivisions in muscle
float A_SYRINGE = PI*pow(13.25, 2.0); // piston area in mm^2
float FACT_V = (ACT_WIDTH*pow(L0 , 2.0))/(2.0*NUM_L);
float MAX_V = FACT_V*(2.0/PI); // volume in mm^3 when fully actuated
// steps to fill actuator
int maxSteps = 5000; //((MAX_V/A_SYRINGE)*STEPS_PER_MM); 
int minSteps = 10;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // uStepper S setup - 
  stepper.setup(CLOSEDLOOP,200);
  stepper.setMaxAcceleration(MAXACCELERATION);
  stepper.setMaxVelocity(MAXVELOCITY);
  stepper.setControlThreshold(15);
  // stepper.stop();

  Wire1.begin();
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
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pressureRead() {
  pressureAbs = sensor.getPressure(ADC_4096);
  // Filter out false readings
  // Stop motor and wait if pressure exceeds maximum
  if (pressureAbs > PRESS_MAX){
    // extInterrupt = true;
    disconFlag = false; // Stop and disable
  }
  else if (pressureAbs < 0){
    pressureAbs = 0.00;
  }
  else if (pressureAbs < PRESS_MIN){
    disconFlag = false;
  }

}


void pressInitZeroVol() {
  //Set state for motor motion based on comparison of pressure signal with setpoint
  pressureRead();
  // pressureAbs = 1000.00;
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
      pressSteps = PRESS_STEPS_FINE;
      // stepper.setMaxVelocity(SPEEDP_LOW);
    }
    else{
      pressSteps = PRESS_STEPS_LG;
      // stepper.setMaxVelocity(SPEEDP_HIGH);
    }
  }

  switch (motorState) {
    case 0:
      // Stop motor
      break;
    case 1:
      //Move motor forwards
      stepper.moveSteps(pressSteps);
      break;
    case 2:
      //Move motor backwards 
      stepper.moveSteps(-pressSteps);
      break;
  }
}



//Function to read input from python script and confirm pump position (TOP, LHS, RHS).
void handShake() {
  while (Serial.available() > 0) {
    shakeInput = Serial.readStringUntil('\n');
    if (shakeInput != ""){
      sprintf(data, "%s\n", shakeKey);
      Serial.write(data);
      if (shakeInput == shakeKey){
        shakeFlag = true;
        // Enable the motor after handshaking
        shakeInput = "";
        // Initialise the time variables
        timeAtStep = micros();
        flushInputBuffer = Serial.readStringUntil('\n');
      }
    }
  }
}


//Function to read input in serial monitor and set the new desired pressure.
void readWriteSerial() {
  while (Serial.available() > 0){
    firstDigit = Serial.read();
    // Control code sends capital S to receive stepCount
    // Capital S in ASCII is 83, so check for that:
    if (firstDigit == 83) {
      stepRecv[0] = '\0'; // Zero the stepRecv char array
      byteRead = 0;
      int j = 0;
      while (byteRead != '\n'){
        if (Serial.available()>0){
          if (j < 5){ //Don't go over number of bytes in stepRecv
            byteRead = Serial.read();
            if (byteRead == 115){ // look for s as end byte
              break;
            }
            else if (byteRead == 83){
              // serErrorFlag = true; // if byteRead is S something went wrong
              break;
            }
            else if (byteRead != '\n'){
              stepRecv[j] = byteRead; // build stepRecv
              stepRecv[j+1] = '\0'; // Append a null
            }
            else{ // not start, end byte
              // serErrorFlag = true;
              break;
            }
          }
          else{ // too long for stepRecv
            // serErrorFlag = true;
            break;
          }
          j = j+1;
        }
        else{
          delayMicroseconds(500);
        }
      }

      if (strcmp(stepRecv, "Closed")==0){
        disconFlag = true;
        stepRecv[0] = '\0';
        return;
      }
      for (int k = 0; k <= strlen(stepRecv); k++){
        if(isDigit(stepRecv[k])==false){
          // serErrorFlag = true;
          // Serial.println(k);
          continue;
        }
      }
    }
    else {
      // flushInputBuffer = Serial.readStringUntil('\n');
      // availableBytes = Serial.available();
      // for(int i=0; i<availableBytes; i++){
      byteRead = Serial.read();
      // }
    }
  }
}


void setPosition(){
  stepIn = strtol(stepRecv, NULL, 10);
  if (stepIn > maxSteps){
    stepIn = maxSteps;
  }
  else if (stepIn < minSteps){
    stepIn = prevStepIn;
  }

  if(serErrorFlag == true){
    stepIn = prevStepIn;
    serErrorFlag = false;
  }
  else{
    prevStepIn = stepIn;
  }

  // if (serErrorFlag == false){
  //   stepIn = strtol(stepRecv, NULL, 10);
  //   // stepIn = atoi(stepRecv);
  //   // Serial.println(stepIn);
  //   if (stepIn > maxSteps){
  //     stepIn = maxSteps;
  //   }
  //   else if (stepIn < minSteps){
  //     stepIn = prevStepIn;
  //   }
  //   prevStepIn = stepIn;
  // }
  // else{
  //   stepIn = prevStepIn;
  //   serErrorFlag = false; // reset error flag
  // }

  angPos = DEG_PER_REV*(float(stepIn)/STEPS_PER_REV);
  angMeas = stepper.encoder.getAngleMoved();
  stepCount = int(angMeas*STEPS_PER_REV/DEG_PER_REV);
  stepError = stepIn - stepCount;
}


void writeSerial(char msg){
  writeTime = millis();
  if (msg == 'S'){ // Normal operation, send stepCount etc
    sprintf(data, "%s,%lu,%lu,%s", stepRecv, stepIn, writeTime, endByte);
  }
  else if (msg == 'D'){ // Python cut off comms, acknowledge this
    sprintf(data, "%s%s,%d,%lu%s", disableMsg, shakeKey, int(pressureAbs*10), writeTime, endByte);
  }
  else if (msg == 'L'){ // Limit switch hit, advise Python
    sprintf(data, "%s%s,%d,%lu%s", limitHit, shakeKey, int(pressureAbs*10), writeTime, endByte);
  }
  else if (msg == 'p'){ // Calibrating
    sprintf(data, "%06d%s,%d,%lu%s", STABLE_TIME-stateCount, shakeKey, int(pressureAbs*10), writeTime, endByte);
  }
  else if(msg = 'P'){ // Calibration finished
    sprintf(data, "%06d%s,%d,%lu%s", stepCount, shakeKey, int(pressureAbs*10), writeTime, endByte);
  }
  Serial.write(data);
}


void loop() {
  if(shakeFlag == false){
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

  timeNow = micros();
  timeSinceStep = timeNow - timeAtStep;
  while (timeSinceStep < 47500){
    timeNow = micros();
    timeSinceStep = timeNow - timeAtStep;
  }
  timeAtStep = timeNow;

  switch(pumpState){

    //////////////////////////////////////////////////////////////////////////////////////////
    //Handshake
    case 1:
      if (!Serial) { //if serial disconnected, go to disconected state
        disconFlag = true;
      }
      handShake();
      break;
    
    //////////////////////////////////////////////////////////////////////////////////////////
    //Disconnection
    case 2:
      writeSerial('D');
      stepper.setup(CLOSEDLOOP,200);
      stepper.setMaxAcceleration(MAXACCELERATION);
      stepper.setMaxVelocity(MAXVELOCITY);
      stepper.setControlThreshold(15);

      disconFlag = false; // No not disconnected
      shakeFlag = false; // No not handshaken
      pressFlag = true; // Yes do calibration
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Calibration
    case 3:

      if (!Serial) { //if serial disconnected, go to disconected state
        disconFlag = true;
      }

      pressInitZeroVol();

      // If enough time has passed, say volume is 0, tell python and move on
      if (stateCount >= STABLE_TIME){
        // Step count should now be zero - muscle empty.
        stepCount = 0;
        stepper.encoder.setHome();
        // Notify that calibration is done
        pressFlag = false;
        writeSerial('P');
      }
      else{
        // Check for disconnection
        readWriteSerial();
        // If no reply, say calibration in progres
        writeSerial('p');
      }
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Active
    case 4:

      if (!Serial) { //if serial disconnected, go to disconected state
        disconFlag = true;
      }

      pressureRead();

      // Read stepIn
      readWriteSerial(); 
      setPosition();
      //Send stepCount
      writeSerial('S');

      // Move to the desired position
      stepper.moveToAngle(angPos); // Could have problems with this - seems it is relative to current position, not zero position
      break;

    //////////////////////////////////////////////////////////////////////////////////////////
    //Default
    default:
      break;
  }
}
