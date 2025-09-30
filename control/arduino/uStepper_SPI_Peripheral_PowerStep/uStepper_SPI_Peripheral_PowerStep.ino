
#include <uStepperS.h>


uStepperS stepper;

const uint8_t SS_Pin = 3;

union dataFloat{
  float fData;
  uint8_t bData[4];
};


unsigned long LOOP_FREQ = 100; // Hz
float LOOP_PERIOD = 1/LOOP_FREQ;
unsigned long LOOP_PERIOD_MICRO = round(1000000*LOOP_PERIOD);

// Time variables for loop frequency
unsigned long timeSinceExec = 0;
unsigned long timeNow;
unsigned long timeLastExecution;

bool ENABLECLOSEDLOOP = false;
float angleEncoder = 0.0;
float prevAngleEncoder = angleEncoder;
float angleDelta = 0.0;
float angleDesired = 0.0;
int encoderChangeCount = 0;
int changeCountThresh = 50;
bool encoderOK = true;
float angPos = 0.0;
float brakeCurrent = 5.0; // percent 

volatile dataFloat posEncoder;
volatile dataFloat posFromController;
const int NUM_CHARS = 4;
const int LAST_CHAR_NUM = 5;

char inputArray[56];

volatile int posIndex = 0;
volatile uint8_t lastByte = 0;

volatile bool startMsgReceived = false; 
volatile bool endMsgReceived = false; 
volatile bool receivedOK = false;

int microDelay = 25;

char START_MESSAGE = '<';
char END_MESSAGE = '>';
char POWER_ON_FLAG = 'Y';
char POWER_OFF_FLAG = 'N';
volatile bool powerState = false;
volatile bool prevPowerState = false;


float LEAD = 8.0;
float STEPS_PER_MM = 200*32; //51200.0; // 32 microsteps per 1.8 deg step
long prevStepCountDesired = 0;
long stepCountDesired = 0;
long stepCountDesiredLocal = 0;

long stepCountDriver = 0;
long prevStepCountDriver = 0;

long stepCountLocal = 0;
long stepError = 0.0;

float angleEstDriver = 0.0;
float angleEstLocal = 0.0;


void setup(void)
{
  noInterrupts();
  Serial.begin(115200);
  if (ENABLECLOSEDLOOP){
    stepper.setup(CLOSEDLOOP,200);   // stepper.setup();     //Initiate the stepper object with default settings
  }
  else{
    stepper.setup();
  }
  // stepper.setup(CLOSEDLOOP,200);   // stepper.setup();     //Initiate the stepper object with default settings
  
  // Initialise data structures
  posEncoder.fData = 0.0;
  // angleEncoder = posEncoder.fData;
  posFromController.fData = 0.0;
  // Serial.println("Start");


  // For the closed loop position control the acceleration and velocity parameters define the response of the control:
  stepper.setMaxAcceleration(500);   //use an acceleration of 2000 fullsteps/s^2
  stepper.setMaxVelocity(1500);        //Max velocity of 800 fullsteps/s
  
  // stepper.checkOrientation(4.0);      //Check orientation of motor connector with +/- 30 microsteps movement
  // stepper.setControlThreshold(15);    //Adjust the control threshold - here set to 15 microsteps before making corrective action

  // After orientation check, turn off motors:
  stepper.setBrakeMode(FREEWHEELBRAKE);
  stepper.disablePid();

  ///////////////////////////////////////////
  // have to send on controller in, peripheral out
  pinMode(MISO0, OUTPUT);
  pinMode(SS0, INPUT);
  // pinMode(SCK0, INPUT);
  pinMode(SS_Pin, INPUT);
  
  // turn on SPI in peripheral mode, but with no interrupt
  SPCR0 = 0;
  SPCR0 |= (1<<SPIE0)|(1<<SPE0)|(0<<DORD0)|(0<<MSTR0)|(0<<CPOL0)|(0<<CPHA0)|(0<<SPR01)|(0<<SPR00); // SPI_MODE Defined by CPOL and CPHA)
  SPSR0 &= ~(0<<SPI2X0);

  interrupts();
}




ISR (SPI_STC_vect){
  // Continue if this peripheral is selected
  if (digitalRead(SS_Pin) == LOW){
    char c = SPDR0;
    // delayMicroseconds(microDelay);
    // If start byte was received in a previous call and
    // number of bytes received is in correct range, then store data
    // Serial.println(c);

    if (startMsgReceived == true){
      // Serial.println(posIndex);
      if (posIndex < NUM_CHARS){
        // Serial.println(c);
        posFromController.bData[posIndex] = uint8_t(c);
        // delayMicroseconds(microDelay);
        SPDR0 = posEncoder.bData[posIndex];
        // Serial.print(c, DEC);
        posIndex++;
      }
      else if(posIndex == NUM_CHARS){//(c is power state bit){
        // Serial.println(c, DEC);
        prevPowerState = powerState;
        if (c == POWER_ON_FLAG){
          powerState = true;
        }
        else{
          powerState = false;
        }
        posIndex++;
      }
      else if(posIndex == LAST_CHAR_NUM){//(c == END_MESSAGE){
        // Serial.println(c, DEC);
        if(c == END_MESSAGE){
          // Serial.println("End message");
          receivedOK = true;
          endMsgReceived = true;
          startMsgReceived = false;
        }
        posIndex++;
      }
      else{
        // posIndex is incorrect
        // data doesn't correspond to start or end bytes 
        startMsgReceived = false;
        endMsgReceived = false;
        receivedOK = false;
        // Serial.println(c, BIN);
      }
    }

    else if (c == START_MESSAGE){
      startMsgReceived = true;
      endMsgReceived = false;
      receivedOK = false;
      // Serial.println();
      // Serial.println("Start message");
      // Serial.println(c);    
      posIndex = 0;
      // SPDR0 = 0;//posEncoder.bData[posIndex];
    }
    else if (c == END_MESSAGE){
      endMsgReceived = true;
      startMsgReceived = false;
      receivedOK = false;
      // check for posIndex == 4    - this should always be the case
      // Serial.println("End message - wrong place");
    }
    else{
      startMsgReceived = false;
      receivedOK = false;
      // either startMsgReceived is false or posIndex is too high, and
      // data doesn't correspond to start or end bytes 
      // Serial.print(c, BIN);
      // posIndex++;
    }
  }
}// end of SPI interrupt routine



// Convert desired angle coming from controller to position in microsteps
long convertDesiredAngleToStepNumber(float angleDes){
  long stepCountDes = long(round(angleDes*LEAD*STEPS_PER_MM/360.0));
  return stepCountDes;
}


// Convert position in microsteps to anglular position in degrees
float convertStepToAngle(float stepCountDri){
  float angleDri = float(stepCountDri)*360.0/(LEAD*STEPS_PER_MM);
  return angleDri;
}




int checkInput(){
  if (receivedOK == true){
    startMsgReceived = false;
    endMsgReceived = false;
    receivedOK = false;
    return 1;    
  }
  else{
    return 0;
  }
}

int checkEncoder(bool encoderFlag){
    if (encoderFlag == true){
      return 1;
    }
    else{
      return 0;
    }
}


void encoderFlag(){
  // Check if the encoder values are changing 
  if (angleEncoder == prevAngleEncoder){
    encoderChangeCount = encoderChangeCount + 1;
  }
  else{
    encoderChangeCount = 0;
  }

  // if the encoder hasn't changed values for 50 loops then ignore incoming angle
  if (encoderChangeCount > changeCountThresh){
    encoderOK = false;
  }
  else{
    encoderOK = true;
  }
}


void toggleMotorPower(bool powState, bool prevPowState){
  // Disable motor power based on status byte from controllerz
  // First check for a change in status:
  if (prevPowState != powState){
    if(powState == true){
      stepper.setBrakeMode(HARDBRAKE, brakeCurrent);
      if (ENABLECLOSEDLOOP){
        stepper.enablePid();
      }
    }
    else {
      stepper.setBrakeMode(FREEWHEELBRAKE);
      stepper.disablePid();
    }
  }
}


void loop(void)
{

  // Check if sampling period has been reached
  timeNow = micros();
  timeSinceExec = timeNow - timeLastExecution;
  // if enough time has passed, change actuator and strucure behaviour
  if (timeSinceExec >= LOOP_PERIOD_MICRO){
    timeLastExecution = micros(); 

    // Get angle data from encoder
    prevAngleEncoder = angleEncoder;
    angleEncoder = stepper.encoder.getAngleMoved();
    // Get estimated (open loop) position from driver
    stepCountDriver = stepper.driver.getPosition();
    angleEstDriver = convertStepToAngle(stepCountDriver);

    // Put in float variable for SPI comms
    if (ENABLECLOSEDLOOP){
      posEncoder.fData = angleEncoder;
    }
    else{
      posEncoder.fData = angleEstDriver;
    }
    
    // encoderFlag();


  }
  // Serial.print("Est angle:  ");
  // Serial.println(angleEstDriver);
  // Serial.print("Angle in:   ");
  // Serial.println(angleDesired);

  // If both start and end messages received correctly,
  // change angular position received from controller
  if (checkInput() == 1){// Check the input array and reset the receivedOK flag
    toggleMotorPower(powerState, prevPowerState);
    checkEncoder(encoderOK);

    // Read desired angle from pump controller
    angleDesired = posFromController.fData;
    // Convert desired angle to number of microsteps
    stepCountDesired = convertDesiredAngleToStepNumber(angleDesired);    

    // Update actual position, expressed in microsteps
    stepCountDriver = stepper.driver.getPosition();
    // int load = stepper.driver.getStallValue();
    // Serial.println(load);

    // Calculate difference between desired position and actual position
    stepError = stepCountDesired - stepCountDriver;

    // Move towards target position
    if(powerState){
      // Checking the power first will make sure stepCountDriver is accurate
      stepper.moveSteps(stepError);
    }

    // IF angleEncoder FROM ENCODER DOESN'T MATCH THE ESTIMATED ANGLE 
    // angleEstDriver FROM THE DRIVER THEN WE HAVE A PROBLEM


    // Serial.print("Encoder:    ");
    // Serial.println(angleEncoder);

    // Serial.print("Est angle:  ");
    // Serial.println(angleEstDriver);
    // Serial.print("Angle in:   ");
    // Serial.println(angleDesired);

    // Serial.print("Est steps:  ");
    // Serial.println(stepCountDriver);
    // Serial.print("Step in:    ");
    // Serial.println(stepCountDesired);

    // Serial.print("Step error: ");
    // Serial.println(stepError);

    // Serial.print("Power:      ");
    // Serial.println(powerState);
    // Serial.println();

    // Clear message flags so that any one message
    // will only move the motor once.
    startMsgReceived = false;
    endMsgReceived = false;
  }
  
}
