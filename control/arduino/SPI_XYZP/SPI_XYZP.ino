/*

Turn on
Reset motors
Calibrate and wait




S_B  664.8231,  664.8231,  664.8231,  450.0000,    0.0000
*/


#include <SPI.h>
// #include <avr/dtostrf.h> // for atoi() function on Due
#include "linearAxis.h"
#include "pressRegPRE1-U08.h"

#include <Wire.h>

// #include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include <Adafruit_ADS1X15.h>


unsigned long LOOP_FREQ = 100; // Hz
unsigned long LOOP_PERIOD_MICRO = round(1000000/LOOP_FREQ);
float LOOP_PERIOD = 1/float(LOOP_FREQ);

const int number_pumps = 5;
const int number_pressure_regs = 0;


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

int allCalibrated = 0;
int calibratedBytes = 0;

//////////////////////////////////////////////////////////
// Setup pins for uStepperS peripherals
int resetPin1 = 8;

// X is Left actuator, Y is Right, Z is Top
int selectPinRot = 10;
int selectPinTool = 11;
int selectPinWrist = 12;
int selectPinAxial = 9;
int selectPinGrasp = 13;

int pressPinX = A4;
int pressPinY = A3;
int pressPinZ = A2;
int pressPinP = A1;
int pressPinAir = A0;

int limitPinRot = 22;
int limitPinTool = 24;
int limitPinWrist = 26;
int limitPinAxial = 30;
int limitPinGrasp = 32;

int valvePin = 37;
int valvePinStruct = 41;

// Pins for grapser open and close functions
int grasper_RHS_FWD = 31;
int grasper_RHS_BWD = 33;
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

char pressXStr[10];
char pressYStr[10]; 
char pressZStr[10];
char pressPStr[10]; 
char pressRegStr[10]; 

float angles[number_pumps];
// char *angleStrings[number_pumps] = {posXStr, posYStr, posZStr, posPStr}; // array of char pointers

int pressures[number_pumps + number_pressure_regs];
float pressuresF[number_pumps + number_pressure_regs];
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

////////////////////////////////////////////////////////
// Setting Serial input variables
char firstDigit = 0;   // For checking incoming communications (motor state)
char secondDigit = 0;  // For checking incoming communications (inflation state)
char thirdDigit = 0;   // For checking incoming communications (button state)
String flushInputBuffer;

// Input string is comma delimited
// position 1, ..., position n
char inputArray[97];
char *positionInputs[number_pumps]; // an array of pointers to the pieces of the above array after strtok()
char *pressureInput[1];// = malloc(32);
char *ptr = NULL;
bool correctNumPositions = false;


/////////////////////////////////////////////////////////
// Pressure regulator

PressReg pressureRegulator;

/////////////////////////////////////////////////////////
// Multiplexer

// QWIICMUX muxPlex;
NAU7802 loadCell; //Create pointer to a set of pointers to the sensor class
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

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
int PORT_NUMBER_ADS = 5;

////////////////////////////////////////////////////////
// Setup

void setup() {

  Serial.begin(115200);
  // Serial.println("Start LRTP Stage");

  Wire.begin();
  // Wire.setClock(3400000);
  loadCell.begin();


  // Initialize all the sensors
  for (byte i = 0; i < NUMBER_OF_SENSORS; i++){
    enableMuxPort(PORT_NUM_LOAD_CELLS[i]);
    loadCell.begin();
    disableMuxPort(PORT_NUM_LOAD_CELLS[i]);
  }
  // enableMuxPort(PORT_NUMBER_ADS);
  // ads.begin();
  // disableMuxPort(PORT_NUMBER_ADS);
  // if (initSuccess == false)
  // {
  //   Serial.print("Freezing...");
  //   while (1)
  //     ;
  // }



  // Set reference for analogue pins to 1.1V instead of 5 V
  // P_measured limited to maximum of 154.85 kPa, resolution of 0.278 kPa
  // Set reference for analogue pins to 2.56V instead of 5 V
  // P_measured limited to maximum of 531.97 kPa, resolution of 0.65 kPa for actuators
  // P_measured limited to maximum of 354.65 kPa, resolution of 0.43 kPa for structure
  analogReference(INTERNAL2V56);

  // Initialise pumps 
  axisList[0].init(selectPinRot, pressPinX, LOOP_PERIOD, limitPinRot);
  axisList[1].init(selectPinTool, pressPinY, LOOP_PERIOD, limitPinTool);
  axisList[2].init(selectPinWrist, pressPinZ, LOOP_PERIOD, limitPinWrist);
  axisList[3].init(selectPinAxial, pressPinP, LOOP_PERIOD, limitPinAxial);
  axisList[4].init(selectPinGrasp, pressPinP, LOOP_PERIOD, limitPinGrasp);

  //Initialise pressure regulator
  pressureRegulator.init(selectPinGrasp, pressPinAir, valvePin, valvePinStruct);

  // Grasper control from haptic
  pinMode(grasper_RHS_FWD, OUTPUT);
  pinMode(grasper_RHS_BWD, OUTPUT);

  digitalWrite(grasper_RHS_FWD, HIGH);
  digitalWrite(grasper_RHS_BWD, HIGH);

  
  // Setup the SPI pins:
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  // initialize SPI:
  SPI.begin();



  // Set uStepperS reset pin
  pinMode(resetPin1, OUTPUT);

  // Reset uSteppers
  // delay(2000);
  resetMotors();
  // delay(1000);

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
    angles[i] = item.angleIn;
    i++;
  }
}



// void intialPositions(){
//   // Read encoder value from each motor. 
//   for(auto &item : axisList){
//     // For each motor, SPI transfer desired and true angular positions
//     // item.dataOut.fData = 0.0;
//     item.sendRecvFloat(&item.dataOut, &item.dataIn);
//     item.sendRecvFloat_POWER(&item.dataOut, &item.dataIn, powerState);
//     item.initEncoder = item.angleIn;
//   }
// }
  


// void intialPressures(){
//   // Read pressure sensor of each syringe. 
//   for(auto &item : axisList){
//     // For each syringe pump, read related pressure sensor
//     item.pressureBaseline = item.readPressure();
//   }
// }



void readActuatorPressures(){
  // Update pressure data for each syringe pump
  int i = 0;
  for(auto &item : axisList){ 
    // pressures[i] = item.readPressure();
    pressuresF[i] = item.readPressure();
    i++;
  }
}

void readPressStructure(){
  // Read pressure sensor of pneumatic structure
  if (number_pressure_regs == 1){
    pressuresF[number_pumps] = pressureRegulator.readStructPressure();
  }
}


void updateAllPressures(){
  readActuatorPressures();
  readPressStructure();
}


int checkPressures(){
  int i = 0;
  int checkPress = 0;
  for(auto &item : axisList){ 
    if (pressuresF[i]> item.MAX_PRESS_KPA){
      checkPress += 1;
    }
    i++; 
  }
  return checkPress;
}



void checkAnglesToMotors(){
  // Changes desiredAngle.
  // Limit the range of the motors to their respective maxima and minima
  // for hydraulic and prismatic stages

  int i = 0;
  for (auto &item : axisList){
    if (i < 3){
      // For hydraulic pumps, limit to between angle of approx max volume and almost zero
      if (item.desiredAngle > item.MAX_ANGLE){
        item.desiredAngle = item.MAX_ANGLE;
      }
      else if (item.desiredAngle < item.MIN_ANGLE){
        item.desiredAngle = item.MIN_ANGLE;
      }
    }
    else{
      // For prismatic stage, limit between max extension and zero
      if (item.desiredAngle > item.MAX_ANGLE_P){
        item.desiredAngle = item.MAX_ANGLE_P;
      }
      else if (item.desiredAngle < item.MIN_ANGLE_P){
        item.desiredAngle = item.MIN_ANGLE_P;
      }
    }
    i++;
  }
}


void setMotorAngleChanges(){
  // Update angle to be sent 
  for (auto &item : axisList){
    // desiredAngle is angle received from control computer
    // angleIn is angle received from given stepper motor
    int checkPress = checkPressures();
    // Serial.println(checkPress);
    checkAnglesToMotors();

    // If any pressure is too high, prevent change of angle of any motor
    if (checkPress == 0){
      item.compDesiredAngle = item.desiredAngle + item.angleAtZeroVol;  // Actual max angle is angle at max vol plus angletZeroVol
      // Serial.println(item.compDesiredAngle);
      item.dataOut.fData = item.compDesiredAngle;
    }

    // Serial.print(int(item.deltaAngle)); Serial.print('\t');
    // Serial.print(int(item.compDesiredAngle)); Serial.print('\t');
    // Serial.print(int(item.angleIn)); Serial.print('\t');
    // Serial.print(int(item.angleAtZeroVol)); Serial.print('\t');
  }
}


// void setZeroAngleChanges(){
//   // Set change of angle to zero to prevent motion when
//   // angles are sent to motors
//   for (auto &item : axisList){
//     item.deltaAngle = 0.0;
//     item.dataOut.fData = item.deltaAngle;
//   }
// }




// Move each motor individually by sending and receiving data
void moveMotors(){
  // Iterate through list of uStepper objects
  for(auto &item : axisList){ 
    // For each motor, SPI transfer desired and true angular positions
    // item.sendRecvFloat(&item.dataOut, &item.dataIn);
    item.sendRecvFloat_POWER(&item.dataOut, &item.dataIn, powerState);
  }
}

// // Send values to serial
// void writeData(){
//   // Write angle and pressure data to control computer.

//   // // Save float values in char arrays as Arduino can't send them as is
//   dtostrf(angles[0], FLOAT_LEN, FLOAT_PREC, posXStr);
//   dtostrf(angles[1], FLOAT_LEN, FLOAT_PREC, posYStr);
//   dtostrf(angles[2], FLOAT_LEN, FLOAT_PREC, posZStr);
//   dtostrf(angles[3], FLOAT_LEN, FLOAT_PREC, posPStr);

//   dtostrf(axisList[0].pressureRead, FLOAT_LEN, FLOAT_PREC, pressXStr);
//   dtostrf(axisList[1].pressureRead, FLOAT_LEN, FLOAT_PREC, pressYStr);
//   dtostrf(axisList[2].pressureRead, FLOAT_LEN, FLOAT_PREC, pressZStr);
//   dtostrf(axisList[3].pressureRead, FLOAT_LEN, FLOAT_PREC, pressPStr);

//   // dtostrf(loadArray[0], FLOAT_LEN, FLOAT_PREC, loadXStr);
//   // dtostrf(loadArray[1], FLOAT_LEN, FLOAT_PREC, loadYStr);
//   // dtostrf(loadArray[2], FLOAT_LEN, FLOAT_PREC, loadZStr);
//   // dtostrf(loadArray[3], FLOAT_LEN, FLOAT_PREC, loadPStr);

//   dtostrf(pressureRegulator.pressureRead, FLOAT_LEN, FLOAT_PREC, pressRegStr);
  

//   writeTime = millis();
//   // Clear data char array
//   data[0] ='\0';
//   // Send 4 encoder values, five pressure values, time and end byte
//   // sprintf(data, "%c,%s,%s,%s,%s,%s,%s,%s,%s,%s,%lu,%c,%d,%c\n", \
//   //   startByte,\
//   //   posXStr, posYStr, posZStr, posPStr,\
//   //   pressXStr, pressYStr, pressZStr, pressPStr, pressRegStr,\
//   //   writeTime,\
//   //   calibrationFlag,\
//   //   calibratedBytes,\
//   //   endByte);

//   sprintf(data, "%c,%s,%s,%s,%s,%s,%s,%s,%s,%s,%lu,%c,%d,%ld,%ld,%ld,%ld,%c\n", \
//     startByte,\
//     posXStr, posYStr, posZStr, posPStr,\
//     pressXStr, pressYStr, pressZStr, pressPStr, pressRegStr,\
//     writeTime,\
//     calibrationFlag,\
//     calibratedBytes,\
//     loadArray[0], loadArray[1], loadArray[2], loadArray[3],\
//     endByte);
//   Serial.write(data);
// }

// Send values to serial
void writeData(){
  // Write angle and pressure data to control computer.

  // // Save float values in char arrays as Arduino can't send them as is
  dtostrf(angles[0], FLOAT_LEN, FLOAT_PREC, posXStr);
  dtostrf(angles[1], FLOAT_LEN, FLOAT_PREC, posYStr);
  dtostrf(angles[2], FLOAT_LEN, FLOAT_PREC, posZStr);
  dtostrf(angles[3], FLOAT_LEN, FLOAT_PREC, posPStr);

  dtostrf(axisList[0].pressureRead, FLOAT_LEN, FLOAT_PREC, pressXStr);
  dtostrf(axisList[1].pressureRead, FLOAT_LEN, FLOAT_PREC, pressYStr);
  dtostrf(axisList[2].pressureRead, FLOAT_LEN, FLOAT_PREC, pressZStr);
  dtostrf(axisList[3].pressureRead, FLOAT_LEN, FLOAT_PREC, pressPStr);

  dtostrf(convLoads[0], FLOAT_LEN, FLOAT_PREC, loadXStr);
  dtostrf(convLoads[1], FLOAT_LEN, FLOAT_PREC, loadYStr);
  dtostrf(convLoads[2], FLOAT_LEN, FLOAT_PREC, loadZStr);
  dtostrf(convLoads[3], FLOAT_LEN, FLOAT_PREC, loadPStr);

  dtostrf(pressureRegulator.pressureRead, FLOAT_LEN, FLOAT_PREC, pressRegStr);
  

  writeTime = millis();
  // Clear data char array
  data[0] ='\0';
  // Send 4 encoder values, five pressure values, time and end byte
  // sprintf(data, "%c,%s,%s,%s,%s,%s,%s,%s,%s,%s,%lu,%c,%d,%c\n", \
  //   startByte,\
  //   posXStr, posYStr, posZStr, posPStr,\
  //   pressXStr, pressYStr, pressZStr, pressPStr, pressRegStr,\
  //   writeTime,\
  //   calibrationFlag,\
  //   calibratedBytes,\
  //   endByte);

  sprintf(data, "%c,%s,%s,%s,%s,%s,%s,%s,%s,%s,%lu,%c,%d,%s,%s,%s,%s,%c\n", \
    startByte,\
    posXStr, posYStr, posZStr, posPStr,\
    pressXStr, pressYStr, pressZStr, pressPStr, pressRegStr,\
    writeTime,\
    calibrationFlag,\
    calibratedBytes,\
    loadXStr, loadYStr, loadZStr, loadPStr,\
    endByte);
  Serial.write(data);
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
        getArray(); // THIS IS BLOCKING - WAITS for '\n' CHAR
        // Split array and assign desired positions to each pump
        // Serial.println("Split");
        // pressureRegulator.prevInputPress = pressureRegulator.inputPress;
        // Serial.println(pressureRegulator.prevInputPress);
        splitArray(); 
      }
    }
    // Send out data over serial
    // writeData();
  }
}


int setStates(){
    // Control code sends capital S to receive stepCount
    // H is hold, C is calibrate, S is active
    int  check = 0;

    if (firstDigit == 'C'){
      // This condition prevents entering calibration protocol twice in a row, like debouncing
      if ((systemState == CALIBRATING) & (allCalibrated == number_pumps)){
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



void getArray(){
  // Begin by zeroing input arrays
  memset(inputArray,0, 56);
  inputArray[0] = '\0';
  // Zero all position input arrays
  for (int j = 0; j < number_pumps; j++){
    *positionInputs[j] = '\0';
  }
  // Zero pressure input array
  pressureInput[0] = '\0';
  // memset(pressureInput, 0 ,32);
  // Serial.println(strlen(*pressureInput[0]));
  // int availableBytes = Serial.available();
  char charRead = '\0';
  int i = 0;
  int timeout = 0;
  while(charRead != '\n'){ // Add a timeout function?
    delayMicroseconds(SERIAL_MICRO_DELAY);
    if (i == 96){
      break;
    }
    if (Serial.available() > 0){
      timeout = 0;
      charRead = Serial.read();
      if (charRead != '\n'){
        inputArray[i] = charRead;
      }
      i++;
    }
  }
  inputArray[i] = '\0';
    // else{
    //   timeout++;
    //   if (timeout > 100){
    //     break;
    //   }
    // }
  


  // // Read until we find start character '{'
  // while (charRead != '['){
  //   if (Serial.available() > 0){
  //     charRead = Serial.read();
  //     Serial.println(charRead);
  //   }
  // }
  // // Read until we find end character '}'
  // while (charRead != ']'){
  //   if (Serial.available() > 0){
  //     charRead = Serial.read();
  //     Serial.println(charRead);
  //     // if (charRead == '\n'){
  //     //   // inputArray[0] = '\0';
  //     //   break;
  //     // }
  //     if (charRead != ']'){
  //       inputArray[i] = charRead;
  //       i++;
  //     }
  //   }
  // }
  // Serial.print("Rcvd: "); Serial.print(inputArray); Serial.println(" End. ");
}



void splitArray(){
  int k = 0;
  int startIndex = 0;
  int endIndex = 0;
  bool startPresent = false;
  bool endPresent = false;
  char truncArray[56];
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
    if ((startPresent && endPresent) && (endIndex - startIndex == 41)){
      break;
    }
  }
  // Serial.print("Indices - Start: "); Serial.print(startIndex); Serial.print(" End: "); Serial.println(endIndex);
  // Serial.print("Flags - Start: "); Serial.print(startPresent); Serial.print(" End: "); Serial.println(endPresent);

  if ((startPresent && endPresent) && (startIndex < endIndex)){
    if(endIndex - startIndex == 41){
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
    if (i < number_pumps){
      // For all the uStepper motors save desired positions based on index
      positionInputs[i] = ptr;
    }
    // else if(i == number_pumps){
    //   // For pressure regulator, save in designated char pointer
    //   pressureInput[0] = ptr;
    // }
    i++;
    ptr = strtok(NULL, ",");
  }
  // Serial.println(i);
  // Check if correct number of values were received
  if (i == number_pumps + number_pressure_regs){
    // Serial.println("Correct number inputs");
    // Update desired angular positions relative to 'zero volume angle'
    int j = 0;
    for (auto &item : axisList){
      item.prevDesiredAngle = item.desiredAngle;
      item.desiredAngle = atof(positionInputs[j]);
      // Serial.print("Desired angular positions: ");
      // Serial.println(positionInputs[j]);
      j++;
    }
    // float interPress = atof(*pressureInput);
    // if (interPress < pressureRegulator.STRUCT_MAX_PRESS){
    //   if ((interPress =! 100.0) || (interPress != 101.0)){
    //     interPress = 100.0;
    //   }
    //   pressureRegulator.inputPress = interPress;
    //   // Serial.print("Pressure: "); Serial.println(interPress);
    // }
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

  if (allCalibrated == number_pumps) {
    // Serial.println("Calibration complete.");
    calibrationFlag = 'Y';
  }
}


void readLoadCells(){
  for(int i = 0; i < NUMBER_OF_SENSORS; i++){
    enableMuxPort(PORT_NUM_LOAD_CELLS[i]);
    loadArray[i] = loadCell.getReading();
    convLoads[i] = (loadArray[i] - loadIntercepts[i])/loadGradients[i];
    // Serial.println(i);
    disableMuxPort(PORT_NUM_LOAD_CELLS[i]);
  }
}


void readADC(){
  enableMuxPort(PORT_NUMBER_ADS);
  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  volts0 = ads.computeVolts(adc0);
  volts1 = ads.computeVolts(adc1);
  volts2 = ads.computeVolts(adc2);
  volts3 = ads.computeVolts(adc3);

  // Serial.println("-----------------------------------------------------------");
  // Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
  // Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.println("V");
  // Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
  // Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");


  disableMuxPort(PORT_NUMBER_ADS);
}









///////////////////////////////////////////////////////////

// NEED TO DECIDE IF SWITCH CASE WILL BE INSIDE A FOR(ITEM) LOOP OR NOT

// READ STATE FROM CONTROL COMP IN MAIN LOOP? THEN ENTER SWITCH CASE IN TIMED STATEMENT?

// Can check state derived from individual motors against the desired controllerMode coming from comp 

// INDIVIDUAL PUMPS DETERMINE GLOBAL STATE - ALL HAVE TO BE CALIBRATED FOR 'GLOBAL' FLAG TO BE SET.
// NEED TO UPDATE CONTROL COMPUTER ON GLOBAL STATE? NOT REALLY, CONTROL COMP CAN JUST SEND SAME FORMAT ALWAYS 






    // CALIBRATION MODE:

    // Read in flag that tells motors to calibrate  like handshake()
    // Set desired pressures                        initialised in class
    // Read pressures from each pump                readActuatorPressures();
    // Filter pressures
    // Compare to desired pressure                  like pressInitZeroVol()
    // Calculate change in angle                    need a setAngles(), but this could be based on stage's linear speed as it is currently
    // Move motors                                  moveMotors()
    // Repeat until all pressures stable            need a flag for derived controller state
    // Tell control computer                        add start byte to writeData() to tell current state?
    // Go to Hold Mode                              change state variable



    // HOLD MODE:

    // Keep motors powered
    // wait for mode flag from control computer
    // Read pressures
    // Go to calibration or hold mode
 
    
  
    // ACTIVE MODE:

    // Read absolute angular position values from control computer
    // Check values / verify / filter
    // Compare with current angular positions
    // Find changes in angle 
    // Read pressures
    // Move motors






    // INFLATION

    // Only enter this state after control computer command
    // Set desired structure pressure
    // Pressurise the structure


    // DEFLATION

    // Can only go to this state after calibration and inflation
    // Go to 'zero' volumes for actuators
    // Set pressure on press reg to atmospheric
    









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
        calibrationProtocol();
        break;

      case HOLDING:
        // Do nothing to alter motor angles
        break;

      case ACTIVE:
        setMotorAngleChanges();
        break;
    }// system state loop




    // Set grasper state based on buttonState
    switch(buttonState){
      
      case STOP_GRASP:
        digitalWrite(grasper_RHS_FWD, HIGH);
        digitalWrite(grasper_RHS_BWD, HIGH);
        break;

      case CLOSE_GRASP:
        digitalWrite(grasper_RHS_FWD, HIGH);
        digitalWrite(grasper_RHS_BWD, LOW);
        break;

      case OPEN_GRASP:
        digitalWrite(grasper_RHS_FWD, LOW);
        digitalWrite(grasper_RHS_BWD, HIGH);
        break;

      default:
        digitalWrite(grasper_RHS_FWD, HIGH);
        digitalWrite(grasper_RHS_BWD, HIGH);
        break;
    } // Grasper state loop



    // Read pressures and put in array to be sent
    // updateAllPressures();

    // Put encoder values in array to be sent
    updateEncoderData();

    // // Read data from load cells
    // readLoadCells();

    // // Read pressure sensors from 16  bit ADC
    // readADC();

    // Send out data over serial
    writeData();

    // Move the motors based on the angles set in the state machine
    moveMotors();

    // Change state of the solenoid valve
    // pressureRegulator.latchValve();

    // Change inflatable structure behaviour based on inflatationState
    // inflationState determined by input in readSerial()
    // Firstly change from input_set state to isolated state if pressure has had time to settle
    // if (inflationState == INPUTPRESSURE){
    //   if (pressureRegulator.prevInputPress != pressureRegulator.inputPress){ // current desired pressure different from last time
    //       inflationCounter = 0;
    //   }
    //   else if (inflationCounter >= infStableTime){
    //     inflationState = ISOLATED;
    //   }
    // }
    // else if (inflationState == DEFLATING){
    //   if (deflationCounter >= 3*infStableTime){
    //     inflationState = ISOLATED;
    //   }
    // }

    // switch (inflationState){

    //   case ISOLATED:
    //     //Change valve states
    //     pressureRegulator.isolateSupply();
    //     // set to atmospheric pressure
    //     pressureRegulator.desStructPress = P_ATMOS; 
    //     // Change pressure regulator value 
    //     pressureRegulator.writePressureReg();
    //     break;
      
    //   case DEFLATING:
    //     if (prevInfState != DEFLATING){
    //       deflationCounter = 0;
    //       prevInfState = DEFLATING;
    //     }
        
    //     deflationCounter = deflationCounter + 1;
    //     if (deflationCounter >= 3*infStableTime){
    //       inflationState = ISOLATED;
    //     }

    //     pressureRegulator.desStructPress = P_INFLATED; // Allow regulator to open
    //     // Change pressure regulator value 
    //     pressureRegulator.writePressureReg();
    //     //Change valve states
    //     pressureRegulator.deflateStructure();
    //     break;

    //   case INPUTPRESSURE:
    //     if (prevInfState != INPUTPRESSURE){
    //       inflationCounter = 0;
    //     }
    //     else if (prevInfState == INPUTPRESSURE){
    //       inflationCounter = inflationCounter + 1;
    //     }
    //     // Serial.println(atof(*pressureInput));
    //     if (pressureRegulator.inputPress > 5.0){
    //       pressureRegulator.desStructPress = pressureRegulator.inputPress; // set by control computer
    //       // Change pressure regulator value 
    //       pressureRegulator.writePressureReg();
    //       //Change valve states
    //       pressureRegulator.inflateStructure();
    //     }
    //     break;
        
    // } //Inflation switch
  
  } //Timing loop
  
}// main loop





