/*

*/


// Mega to uStepper
// Byte 0: START_MESSAGE
// Byte 1: COMMAND_SET_POSITION
// Byte 2: sequence number N
// Byte 3: angle byte 0
// Byte 4: angle byte 1
// Byte 5: angle byte 2
// Byte 6: angle byte 3
// Byte 7: power flag
// Byte 8: END_MESSAGE

// uStepper to Mega
// Byte 0: START_MESSAGE
// Byte 1: status
// Byte 2: completed sequence number
// Byte 3: encoder byte 0
// Byte 4: encoder byte 1
// Byte 5: encoder byte 2
// Byte 6: encoder byte 3
// Byte 7: actual power-enabled flag
// Byte 8: END_MESSAGE

#include <SPI.h>
// #include <avr/dtostrf.h> // for atoi() function on Due
#include "linearAxis.h"
#include "pressRegPRE1-U08.h"

// #include <Wire.h>

// #include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
// #include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
// #include <Adafruit_ADS1X15.h>


unsigned long LOOP_FREQ = 100; // Hz
unsigned long LOOP_PERIOD_MICRO = round(1000000/LOOP_FREQ);
float LOOP_PERIOD = 1.0f/float(LOOP_FREQ);

const int NUMBER_PUMPS = 5;
const int NUMBER_PRESSSURE_REGS = 0;


/////////////////////////////////////////////////////////////
// State machine variables

#define DISCONNECTED 0
#define CALIBRATING 1
#define HOLDING 2
#define ACTIVE 3
int systemState = HOLDING;

#define ISOLATED 0
#define DEFLATING 1
#define INPUTPRESSURE 2
int inflationState = ISOLATED;
int prevInfState = inflationState;
int inflationCounter = 0;
int deflationCounter = 0;
int infStableTime = 3*LOOP_FREQ; // 1 second

#define STOP_GRASP 0
#define CLOSE_GRASP 1
#define OPEN_GRASP 2
int buttonState = STOP_GRASP;

#define POWER_ON 'Y'
#define POWER_OFF 'N' 
char powerState = POWER_OFF;
bool communicationShutdownTriggered = false;

int allCalibrated = 0;
int calibratedBytes = 0;

//////////////////////////////////////////////////////////
// Setup pins for uStepperS peripherals
int resetPin1 = 8;


/*

CS1  - 9
CS2  - 10
CS3  - 11
CS4  - 12
CS5  - 26
CS6  - 27
CS7  - 28
CS8  - 29
CS9  - 30
CS10 - 31
CS11 - 32
CS12 - 33
*/

// X is Left actuator, Y is Right, Z is Top
int selectPinRot = 10;   // Axial - first value from Pi - CS2
int selectPinTool = 11;  // Rotary - second value from Pi - CS3
int selectPinWrist = 12; // Tool - third from Pi - CS4
int selectPinAxial = 9;  // Wrist - 4th from Pi - CS1
int selectPinGrasp = 28; // Grasper - 5th value from Pi - CS9

int pressPinRot = A4;
int pressPinTool = A3;
int pressPinWrist = A2;
int pressPinAxial = A1;
int pressPinGrasp = A0;

int limitPinRot = 22;
int limitPinTool = 24;
int limitPinWrist = 23;
int limitPinAxial = 25;
int limitPinGrasp = 35;

int valvePin = 27;
int valvePinStruct = 41;

// Pins for grapser open and close functions
int grasper_RHS_FWD = 37;
int grasper_RHS_BWD = 39;
// int grasper_LHS_FWD = 27;
// int grasper_LHS_BWD = 29;


// Position and pressure variables
float posX = 0.0;
float posY = 0.0;
float posZ = 0.0;
float posP = 0.0;

int FLOAT_LEN = 9;
int FLOAT_PREC = 2;

char posXStr[10];
char posYStr[10]; 
char posZStr[10];
char posPStr[10];
char posGStr[10];

char pressXStr[10];
char pressYStr[10]; 
char pressZStr[10];
char pressPStr[10]; 

float angles[NUMBER_PUMPS];
// char *angleStrings[NUMBER_PUMPS] = {posXStr, posYStr, posZStr, posPStr}; // array of char pointers

int pressures[NUMBER_PUMPS + NUMBER_PRESSSURE_REGS];
float pressuresF[NUMBER_PUMPS + NUMBER_PRESSSURE_REGS];
float P_ATMOS = 0.0; // kPa gauge
float P_INFLATED = 100.0; //kPa gauge

// Time variables for loop frequency
unsigned long timeSinceExec = 0;
unsigned long timeNow;
unsigned long timeLastExecution;


////////////////////////////////////////////////////////
// uStepper data structures

LinAxis axisList[5] = {LinAxis(), LinAxis(), LinAxis(), LinAxis(), LinAxis()};


////////////////////////////////////////////////////////
// Output Messages
char data[150]; // Char array to write stepNo, pressure and time into
char startByte = '<';
char endByte = '>';
char calibrationFlag = 'N';
unsigned long writeTime;
const int SERIAL_MICRO_DELAY = 100;
const int SERIAL_TIMEOUT = 50;
const int EXPECTED_MESSAGE_SIZE = 41;

////////////////////////////////////////////////////////
// Setting Serial input variables
char firstDigit = 0;   // For checking incoming communications (motor state)
char secondDigit = 0;  // For checking incoming communications (inflation state)
char thirdDigit = 0;   // For checking incoming communications (button state)
String flushInputBuffer;

// Input string is comma delimited
// position 1, ..., position n
char inputArray[97];
char *positionInputs[NUMBER_PUMPS]; // an array of pointers to the pieces of the above array after strtok()
char *ptr = NULL;
bool correctNumPositions = false;


/////////////////////////////////////////////////////////
// Pressure regulator

// PressReg pressureRegulator;

/////////////////////////////////////////////////////////
// Multiplexer

// QWIICMUX muxPlex;
// NAU7802 loadCell; //Create pointer to a set of pointers to the sensor class
#define NUMBER_OF_SENSORS 4
byte currentPortNumber = 0;
bool initSuccess = true;
long loadArray[NUMBER_OF_SENSORS];
float loadIntercepts[NUMBER_OF_SENSORS] = {-157842.064, 120690.686, 50251.4, 203841.3};
float loadGradients[NUMBER_OF_SENSORS] = {298.89, 213.08, 291.40, 263.92};
float convLoads[NUMBER_OF_SENSORS];

char loadXStr[10];
char loadYStr[10]; 
char loadZStr[10];
char loadPStr[10]; 

int PORT_NUM_LOAD_CELLS[NUMBER_OF_SENSORS] = {3,0,1,4};


////////////////////////////////////////////////////////
// ADC

// Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// int PORT_NUMBER_ADS = 5;


const unsigned long ACK_TIMEOUT_MS = 100;
const uint16_t MAX_CONSECUTIVE_ACK_FAILURES = 5;

uint16_t consecutiveAckFailures[NUMBER_PUMPS] =
{
    0, 0, 0, 0, 0
};

bool allMotorCommunicationsHealthy = true;

////////////////////////////////////////////////////////
// Setup
void setup() {

  Serial.begin(115200);

  // Initialise pumps 
  axisList[0].init(selectPinRot, pressPinRot, LOOP_PERIOD, limitPinRot);     // Axial
  axisList[1].init(selectPinTool, pressPinTool, LOOP_PERIOD, limitPinTool);   // Rotary
  axisList[2].init(selectPinWrist, pressPinWrist, LOOP_PERIOD, limitPinWrist); // Wrist
  axisList[3].init(selectPinAxial, pressPinAxial, LOOP_PERIOD, limitPinAxial); // Tool
  axisList[4].init(selectPinGrasp, pressPinGrasp, LOOP_PERIOD, limitPinGrasp); // Grasper


  // Grasper control from haptic
  pinMode(grasper_RHS_FWD, OUTPUT);
  pinMode(grasper_RHS_BWD, OUTPUT);

  digitalWrite(grasper_RHS_FWD, HIGH);
  digitalWrite(grasper_RHS_BWD, HIGH);

  // Set uStepperS reset pin
  pinMode(resetPin1, OUTPUT);

  // Reset uSteppers
  delay(1000);
  resetMotors();
  delay(1000);



  // Setup the SPI pins:
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  // initialize SPI:
  SPI.begin();


}


/////////////////////////////////////////////////////////
// Functions

void resetMotors(){
  digitalWrite(resetPin1, LOW);
  delay(100);
  digitalWrite(resetPin1, HIGH);
}



void updateEncoderData(){
  // Read encoder value from each motor. 
  int i = 0;
  for(auto &item : axisList){

    // angles[i] = atof(positionInputs[i]);
    angles[i] = item.angleIn;
    i++;

  }
}




void setMotorAngleChanges(){
  // Update angle to be sent 
  for (auto &item : axisList){
    // desiredAngle is angle received from control computer
    // angleIn is angle received from given stepper motor


    // If any pressure is too high, prevent change of angle of any motor
    // if (checkPress == 0){
    item.compDesiredAngle = item.desiredAngle + item.angleAtZeroVol;  // Actual max angle is angle at max vol plus angletZeroVol
    // Serial.println(item.compDesiredAngle);
    item.dataOut.fData = item.compDesiredAngle;
    // }

    // Serial.print(int(item.deltaAngle)); Serial.print('\t');
    // Serial.print(int(item.compDesiredAngle)); Serial.print('\t');
    // Serial.print(int(item.angleIn)); Serial.print('\t');
    // Serial.print(int(item.angleAtZeroVol)); Serial.print('\t');
  }
}




// Move each motor individually by sending and receiving data
void moveMotors(){
  allMotorCommunicationsHealthy = true;

  uint8_t motorIndex = 0;
  // Iterate through list of uStepper objects
  for(auto &item : axisList){ 
    // For each motor, SPI transfer desired and true angular positions
    // item.sendRecvFloat(&item.dataOut, &item.dataIn);
    // item.sendRecvFloat_POWER(&item.dataOut, &item.dataIn, powerState);
    const bool previousCommandAcknowledged = item.sendCommandPipelined(item.dataOut.fData, powerState);
    /*
      * The first transaction after reset returns STATUS_IDLE,
      * so do not classify it as a failure solely because it is
      * not STATUS_COMPLETE.
      */
    const bool startupResponse =
        item.responseFramingValid &&
        item.communicationHealthy &&
        (item.previousCommandSequence == 0) &&
        (item.acknowledgedSequence) == 0 &&
        (item.acknowledgedStatus == STATUS_IDLE);

    const bool acknowledgementTimedOut =
        item.acknowledgementTime != 0 &&
        (millis() - item.acknowledgementTime > ACK_TIMEOUT_MS);

    if ((previousCommandAcknowledged || startupResponse) && !acknowledgementTimedOut)
    {
      // For this motor index, message received correctly
      consecutiveAckFailures[motorIndex] = 0;
    }
    else
    {
      consecutiveAckFailures[motorIndex]++;

      if (consecutiveAckFailures[motorIndex] >=
          MAX_CONSECUTIVE_ACK_FAILURES)
      {
          allMotorCommunicationsHealthy = false;
      }
    }

    motorIndex++;

  }
  /*
    * Choose the safe response appropriate to the mechanism.
    *
    * This example requests motor power off after repeated
    * communication failures.
    */
  if (!allMotorCommunicationsHealthy)
  {
    powerState = POWER_OFF;
    if (!communicationShutdownTriggered)
    {
      requestAllMotorsOff();
      communicationShutdownTriggered = true;
    }
  }
  else
  {
    communicationShutdownTriggered = false;
  }

}


void requestAllMotorsOff()
{
  for (auto &item : axisList)
  {
    item.sendCommandPipelined(
      item.dataOut.fData,
      POWER_OFF);
  }
}


bool confirmAllLatestCommands()
{
  bool allConfirmed = true;

  for (auto &item : axisList)
  {
    if (!item.readLatestAcknowledgement())
    {
      allConfirmed = false;
    }
  }

  return allConfirmed;
}


void stopAndConfirmMotors()
{
    for (auto &item : axisList)
    {
        item.sendCommandPipelined(
            item.dataOut.fData,
            POWER_OFF);
    }

    delay(2);

    const bool shutdownConfirmed =
        confirmAllLatestCommands();

    if (!shutdownConfirmed)
    {
        // Record fault, retry, or assert the common reset.
    }
}





// Send values to serial
void writeData(){
  // Write angle and pressure data to control computer.

  // // Save float values in char arrays as Arduino can't send them as is
  dtostrf(angles[0], FLOAT_LEN, FLOAT_PREC, posXStr);
  dtostrf(angles[1], FLOAT_LEN, FLOAT_PREC, posYStr);
  dtostrf(angles[2], FLOAT_LEN, FLOAT_PREC, posZStr);
  dtostrf(angles[3], FLOAT_LEN, FLOAT_PREC, posPStr);
  dtostrf(angles[4], FLOAT_LEN, FLOAT_PREC, posGStr);

  dtostrf(axisList[0].pressureRead, FLOAT_LEN, FLOAT_PREC, pressXStr);
  dtostrf(axisList[1].pressureRead, FLOAT_LEN, FLOAT_PREC, pressYStr);
  dtostrf(axisList[2].pressureRead, FLOAT_LEN, FLOAT_PREC, pressZStr);
  dtostrf(axisList[3].pressureRead, FLOAT_LEN, FLOAT_PREC, pressPStr);

  dtostrf(convLoads[0], FLOAT_LEN, FLOAT_PREC, loadXStr);
  dtostrf(convLoads[1], FLOAT_LEN, FLOAT_PREC, loadYStr);
  dtostrf(convLoads[2], FLOAT_LEN, FLOAT_PREC, loadZStr);
  dtostrf(convLoads[3], FLOAT_LEN, FLOAT_PREC, loadPStr);
  

  writeTime = millis();
  // Clear data char array
  data[0] ='\0';
  const int written = snprintf(data, sizeof(data), "%c,%s,%s,%s,%s,%s,%s,%s,%s,%s,%lu,%s,%s,%s,%s,%c\n", \
    startByte,\
    posXStr, posYStr, posZStr, posPStr, posGStr,\
    pressXStr, pressYStr, pressZStr, pressPStr,\
    writeTime,\
    loadXStr, loadYStr, loadZStr, loadPStr,\
    endByte);


  if ((written > 0) && (written < static_cast<int>(sizeof(data))))
  {
    Serial.write(
        reinterpret_cast<const uint8_t *>(data),
        written);
  }

}



//Function to read input in serial monitor and set the new desired pressure.
void readSerial() {
  if (Serial.available() > 0){
    int timeout1  = 0;
    char readChar = Serial.read();
    while (readChar !='B'){
      // Serial.println(readChar);
      delayMicroseconds(SERIAL_MICRO_DELAY);
      if (Serial.available() > 0){
        timeout1 = 0;
        readChar = Serial.read();
      }
      else{
        timeout1++;
        if (timeout1 > 20){
          return;
        }
      }
    }
    if (Serial.available() > 2){
      delayMicroseconds(SERIAL_MICRO_DELAY);
      firstDigit = Serial.read();
      delayMicroseconds(SERIAL_MICRO_DELAY);
      secondDigit = Serial.read();
      delayMicroseconds(SERIAL_MICRO_DELAY);
      thirdDigit = Serial.read();
      delayMicroseconds(SERIAL_MICRO_DELAY);

      int stateError = setStates();
      // Serial.println(stateError);

      if (stateError == 0){
        // Read in array of positions
        // Serial.println("Get input");
        if (getArray()){// THIS IS BLOCKING - WAITS for '\n' CHAR
          // Split array and assign desired positions to each pump
          // Serial.println("Split");
          splitArray();
        } 

      }
    }
  }
}


int setStates(){
    // Control code sends capital S to receive stepCount
    // H is hold, C is calibrate, S is active
    int  check = 0;

    if (firstDigit == 'C'){
      // This condition prevents entering calibration protocol twice in a row, like debouncing
      if ((systemState == CALIBRATING) && (allCalibrated == NUMBER_PUMPS)){
        systemState = HOLDING;
        powerState = POWER_ON;
        // Serial.println("Calibration done, holding");
      }
      else{
        systemState = CALIBRATING;
        powerState = POWER_ON;
        // Serial.println("Calibration Mode");
        // Set pumpCalibrated flags to zero?
      }

    }
    else if (firstDigit == 'S'){
      systemState = ACTIVE;
      powerState = POWER_ON;
      // Serial.println("Active Mode");
    }
    else if (firstDigit == 'H'){
      systemState = HOLDING;
      powerState = POWER_OFF;
      // Serial.println("Hold Mode");
    }
    else{
      check++;
    }

    prevInfState = inflationState;
    // Second digit determines inflatable structure state
    if (secondDigit == 'D'){
      inflationState = DEFLATING;
      // Serial.println("Deflation Mode");
    }
    else if (secondDigit == 'I'){
      inflationState = ISOLATED;
      // Serial.println("Isolated pressure supply Mode");
    }
    else if (secondDigit == '_'){
      inflationState = INPUTPRESSURE;
      // Serial.println("Input Mode");
    }
    else{
      check++;
    }


    // Third digit for button state
    if (thirdDigit == 'N'){
      // No button being pressed
      buttonState = STOP_GRASP;
    }
    else if (thirdDigit == 'F'){
      buttonState = CLOSE_GRASP;
    }
    else if (thirdDigit == 'B'){
      buttonState = OPEN_GRASP;
    }
    else{
      check++;
    }

    return check;
}



bool getArray(){
  // Begin by zeroing input arrays
  memset(inputArray, 0, sizeof(inputArray));
  inputArray[0] = '\0';
  // Zero all position input arrays
  for (int j = 0; j < NUMBER_PUMPS; j++){
    positionInputs[j] = nullptr;
  }

  char charRead = '\0';
  int i = 0;
  const unsigned long startTime = millis();
  // int timeout = 0;
  // while(charRead != '\n'){ // Add a timeout function?
  while ((millis() - startTime) < SERIAL_TIMEOUT){
    delayMicroseconds(SERIAL_MICRO_DELAY);

    if (Serial.available() > 0){

      charRead = Serial.read();
      if (charRead == '\n'){
        inputArray[i] = '\0';
        // Reached the end of the line, exit succesfully
        return true;
      }
      // Add latest char to end of array if not out of size bounds
      else if(i < sizeof(inputArray)-1){
        inputArray[i] = charRead;
      }
      // If at the end of inputArray size without finding \n
      // Something wrong: null terminate and exit function
      else{
        inputArray[i] = '\0';
        return false;
      }
      i++;
    }
  }
  // Serial command has timed out
  inputArray[i] = '\0';
  return false;

}



void splitArray(){
  int k = 0;
  int startIndex = 0;
  int endIndex = 0;
  bool startPresent = false;
  bool endPresent = false;
  char truncArray[sizeof(inputArray)];
  truncArray[0] = '\0';

  for (k = 0; k < sizeof(inputArray); k++){
    if (inputArray[k] == '['){
      startIndex = k;
      startPresent = true;
    }
    if (inputArray[k] == ']'){
      endIndex = k;
      endPresent = true;
    }
    if ((startPresent && endPresent) && (endIndex - startIndex == EXPECTED_MESSAGE_SIZE)){
      break;
    }
  }
  // Serial.print("Indices - Start: "); Serial.print(startIndex); Serial.print(" End: "); Serial.println(endIndex);
  // Serial.print("Flags - Start: "); Serial.print(startPresent); Serial.print(" End: "); Serial.println(endPresent);

  if ((startPresent && endPresent) && (startIndex < endIndex)){
    if(endIndex - startIndex == EXPECTED_MESSAGE_SIZE){
      int n = 0;
      for (n = 0; n < endIndex - startIndex - 1; n++){
        truncArray[n] = inputArray[startIndex + 1 + n];
      }
      truncArray[n] = '\0';
    }
  }


  // Serial.print("Trunc: "); Serial.println(truncArray);

  int i = 0;
  // Look for comma delimiters in inputArray with strtok()
  // puts next part of message into memory at ptr
  ptr = strtok(truncArray, ",");
  while (ptr != NULL){
    if (i < NUMBER_PUMPS){
      // For all the uStepper motors save desired positions based on index
      positionInputs[i] = ptr;
    }
    i++;
    ptr = strtok(NULL, ",");
  }
  // Serial.println(i);
  // Check if correct number of values were received
  if (i == NUMBER_PUMPS + NUMBER_PRESSSURE_REGS){
    // Serial.println("Correct number inputs");
    // Update desired angular positions relative to 'zero volume angle'
    int j = 0;
    for (auto &item : axisList){
      char *conversionEnd = nullptr;

      const double parsedValue =
          strtod(positionInputs[j], &conversionEnd);

      const bool valid =
          (conversionEnd != positionInputs[j]) &&
          (*conversionEnd == '\0') &&
          isfinite(parsedValue);

      if (valid){
        item.prevDesiredAngle = item.desiredAngle;
        item.desiredAngle = static_cast<float>(parsedValue);
        // Serial.print("Desired angular positions: ");
        // Serial.println(positionInputs[j]);
      }

      j++;
    }
  }
}




void calibrationProtocol(){
  // call pressInitZero on each motor to set individual states
  // Individual states determine forward, backward, or no motion.
  // Motion is achieved by incrementing/decrementing desiredAngle as approp.
  allCalibrated = 0;
  calibratedBytes = 0;
  int i = 0;
  for (auto &item : axisList){
    if (item.pumpCalibrated == 0){
      // Use pressure to calibrate for the hydraulic pumps
      if (i < 3){
        // This function sets the deltaAngle of each pump 
        item.pressInitZeroVol();
      }
      // Else use limit switch for prismatic stage
      else{
        // deltaAngle is also set in this function
        item.moveToLimit(); // Move prismatic stage to limit switch
      }
      // Set target based on results of homing functions
      item.caliDesiredAngle = item.angleIn + item.deltaAngle;
    }
    else{
      allCalibrated = allCalibrated + item.pumpCalibrated;
      bitSet(calibratedBytes, i); // Set the ith bit if calibrated
      item.deltaAngle = 0.0;
    }
    // Serial.print(int(item.deltaAngle)); Serial.print('\t');
    // Set desired angular position as the current position minus a small delta

    item.dataOut.fData = item.caliDesiredAngle;
    i++;
    // Serial.println(item.pumpCalibrated);
  }

  if (allCalibrated == NUMBER_PUMPS) {
    // Serial.println("Calibration complete.");
    calibrationFlag = 'Y';
  }
}


// void readLoadCells(){
//   for(int i = 0; i < NUMBER_OF_SENSORS; i++){
//     enableMuxPort(PORT_NUM_LOAD_CELLS[i]);
//     loadArray[i] = loadCell.getReading();
//     convLoads[i] = (loadArray[i] - loadIntercepts[i])/loadGradients[i];
//     // Serial.println(i);
//     disableMuxPort(PORT_NUM_LOAD_CELLS[i]);
//   }
// }


// void readADC(){
//   // enableMuxPort(PORT_NUMBER_ADS);
//   int16_t adc0, adc1, adc2, adc3;
//   float volts0, volts1, volts2, volts3;

//   adc0 = ads.readADC_SingleEnded(0);
//   adc1 = ads.readADC_SingleEnded(1);
//   adc2 = ads.readADC_SingleEnded(2);
//   adc3 = ads.readADC_SingleEnded(3);

//   volts0 = ads.computeVolts(adc0);
//   volts1 = ads.computeVolts(adc1);
//   volts2 = ads.computeVolts(adc2);
//   volts3 = ads.computeVolts(adc3);

  // Serial.println("-----------------------------------------------------------");
  // Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
  // Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.println("V");
  // Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
  // Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");


  // disableMuxPort(PORT_NUMBER_ADS);
// }


    









void loop() {
  readSerial();
  // flushInputBuffer = Serial.readStringUntil('\n');

  // Check if sampling period has been reached
  timeNow = micros();
  timeSinceExec = timeNow - timeLastExecution;
  // if enough time has passed, change actuator and structure behaviour
  if (timeSinceExec >= LOOP_PERIOD_MICRO){
    timeLastExecution = micros(); 


    // Change syringe pump behaviour based on systemState
    // systemState determined by firstDigit in readSerial()
    // Serial.println(systemState);
    switch(systemState){

      case CALIBRATING:
      // After moving out of calibration mode, state is determined by control computer
        // calibrationProtocol();
        break;

      case HOLDING:
        // Do nothing to alter motor angles
        break;

      case ACTIVE:
        setMotorAngleChanges();
        break;
    }// system state loop


    // Move the motors based on the angles set in the state machine
    moveMotors();

    // Put encoder values in array to be sent
    updateEncoderData();

    // // Read data from load cells
    // readLoadCells();

    // // Read pressure sensors from 16  bit ADC
    // readADC();

    // Send out data over serial
    writeData();
    
  
  } //Timing loop
  
}// main loop





