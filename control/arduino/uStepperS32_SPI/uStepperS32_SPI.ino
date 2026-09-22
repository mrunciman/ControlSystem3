#include <UstepperS32.h>
// #include <callbacks.h>
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"


UstepperS32 stepper;

const uint8_t SS_Pin = D5;

union dataFloat{
  float fData;
  uint8_t bData[4];
};


unsigned long LOOP_FREQ = 100; // Hz
float LOOP_PERIOD = 1.0/LOOP_FREQ;
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

// char inputArray[56];

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
  stepper.setMaxAcceleration(500);
  stepper.setMaxVelocity(1500);        //Max velocity in fullsteps/s
  
  // stepper.checkOrientation(4.0);      //Check orientation of motor connector with +/- 30 microsteps movement
  // stepper.setControlThreshold(15);    //Adjust the control threshold - here set to 15 microsteps before making corrective action

  // After orientation check, turn off motors:
  stepper.setBrakeMode(FREEWHEELBRAKE);
  stepper.disablePid();

  // ///////////////////////////////////////////
  // // have to send on controller in, peripheral out
  // pinMode(MISO0, OUTPUT);
  // pinMode(SS0, INPUT);
  // // pinMode(SCK0, INPUT);
  // pinMode(SS_Pin, INPUT);
  
  // // turn on SPI in peripheral mode, but with no interrupt
  // SPCR0 = 0;
  // SPCR0 |= (1<<SPIE0)|(1<<SPE0)|(0<<DORD0)|(0<<MSTR0)|(0<<CPOL0)|(0<<CPHA0)|(0<<SPR01)|(0<<SPR00); // SPI_MODE Defined by CPOL and CPHA)
  // SPSR0 &= ~(0<<SPI2X0);

  pinMode(D6, INPUT); // SCK1 from master
  pinMode(D3, OUTPUT); // MISO1 to master
  pinMode(D2, INPUT); // MOSI1 from master
  pinMode(SS_Pin, INPUT_PULLUP);
  setupPeripheralSPI();

  interrupts();
}


void setupPeripheralSPI()
{
    /* Enable GPIOA and SPI1 clocks */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_GPIO_InitTypeDef gpio = {0};

    /*
    * PA5 = D6 = SPI1 SCK
    * PA6 = D3 = SPI1 MISO
    * PA7 = D2 = SPI1 MOSI
    */
    gpio.Pin = LL_GPIO_PIN_5 |
              LL_GPIO_PIN_6 |
              LL_GPIO_PIN_7;
    gpio.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio.Pull = LL_GPIO_PULL_NO;
    gpio.Alternate = LL_GPIO_AF_5;

    LL_GPIO_Init(GPIOA, &gpio);

    LL_SPI_Disable(SPI1);

    LL_SPI_SetTransferDirection(
        SPI1, LL_SPI_FULL_DUPLEX);

    LL_SPI_SetMode(
        SPI1, LL_SPI_MODE_SLAVE);

    LL_SPI_SetDataWidth(
        SPI1, LL_SPI_DATAWIDTH_8BIT);

    /* Same as the original CPOL=0, CPHA=0 */
    LL_SPI_SetClockPolarity(
        SPI1, LL_SPI_POLARITY_LOW);

    LL_SPI_SetClockPhase(
        SPI1, LL_SPI_PHASE_1EDGE);

    LL_SPI_SetTransferBitOrder(
        SPI1, LL_SPI_MSB_FIRST);

    /*
     * CS is checked manually on D5, rather than using
     * the SPI peripheral's hardware NSS pin.
     */
    LL_SPI_SetNSSMode(
        SPI1, LL_SPI_NSS_SOFT);

    NVIC_SetPriority(SPI1_IRQn, 1);
    NVIC_EnableIRQ(SPI1_IRQn);

    LL_SPI_EnableIT_RXNE(SPI1);
    LL_SPI_Enable(SPI1);

    /* Initial byte returned to the master */
    LL_SPI_TransmitData8(SPI1, 0);
}



// ISR (SPI_STC_vect){
//   // Continue if this peripheral is selected
//   if (digitalRead(SS_Pin) == LOW){
//     char c = SPDR0;
//     // delayMicroseconds(microDelay);
//     // If start byte was received in a previous call and
//     // number of bytes received is in correct range, then store data
//     if (startMsgReceived == true){
//       // Serial.println(posIndex);
//       if (posIndex < NUM_CHARS){
//         // Serial.println(c);
//         posFromController.bData[posIndex] = uint8_t(c);
//         // delayMicroseconds(microDelay);
//         SPDR0 = posEncoder.bData[posIndex];
//         // Serial.print(c, DEC);
//         posIndex++;
//       }
//       else if(posIndex == NUM_CHARS){//(c is power state bit){
//         // Serial.println(c, DEC);
//         prevPowerState = powerState;
//         if (c == POWER_ON_FLAG){
//           powerState = true;
//         }
//         else{
//           powerState = false;
//         }
//         posIndex++;
//       }
//       else if(posIndex == LAST_CHAR_NUM){//(c == END_MESSAGE){
//         // Serial.println(c, DEC);
//         if(c == END_MESSAGE){
//           // Serial.println("End message");
//           receivedOK = true;
//           endMsgReceived = true;
//           startMsgReceived = false;
//         }
//         posIndex++;
//       }
//       else{
//         // posIndex is incorrect
//         // data doesn't correspond to start or end bytes 
//         startMsgReceived = false;
//         endMsgReceived = false;
//         receivedOK = false;
//         // Serial.println(c, BIN);
//       }
//     }

//     else if (c == START_MESSAGE){
//       startMsgReceived = true;
//       endMsgReceived = false;
//       receivedOK = false;
//       // Serial.println();
//       // Serial.println("Start message");
//       // Serial.println(c);    
//       posIndex = 0;
//       // SPDR0 = 0;//posEncoder.bData[posIndex];
//     }
//     else if (c == END_MESSAGE){
//       endMsgReceived = true;
//       startMsgReceived = false;
//       receivedOK = false;
//       // check for posIndex == 4    - this should always be the case
//       // Serial.println("End message - wrong place");
//     }
//     else{
//       startMsgReceived = false;
//       receivedOK = false;
//       // either startMsgReceived is false or posIndex is too high, and
//       // data doesn't correspond to start or end bytes 
//       // Serial.print(c, BIN);
//       // posIndex++;
//     }
//   }
// }// end of SPI interrupt routine

extern "C" void SPI1_IRQHandler(void)
{
    if (LL_SPI_IsActiveFlag_RXNE(SPI1))
    {
        uint8_t c = LL_SPI_ReceiveData8(SPI1);
        // Serial.print(c);
        uint8_t reply = 0;

        if (digitalRead(SS_Pin) == LOW)
        {
            if (startMsgReceived)
            {
                if (posIndex < NUM_CHARS)
                {
                    posFromController.bData[posIndex] = c;
                    reply = posEncoder.bData[posIndex];
                    posIndex++;
                }
                else if (posIndex == NUM_CHARS)
                {
                    prevPowerState = powerState;
                    powerState = (c == POWER_ON_FLAG);
                    posIndex++;
                }
                else if (posIndex == LAST_CHAR_NUM)
                {
                    receivedOK = (c == END_MESSAGE);
                    endMsgReceived = receivedOK;
                    startMsgReceived = false;
                    posIndex++;
                }
                else
                {
                    startMsgReceived = false;
                    endMsgReceived = false;
                    receivedOK = false;
                    posIndex = 0;
                }
            }
            else if (c == START_MESSAGE)
            {
                startMsgReceived = true;
                endMsgReceived = false;
                receivedOK = false;
                posIndex = 0;
            }
            else
            {
                receivedOK = false;
            }
        }
        else
        {
            startMsgReceived = false;
            endMsgReceived = false;
            receivedOK = false;
            posIndex = 0;
        }

        // Loaded now, transmitted during the next master byte
        LL_SPI_TransmitData8(SPI1, reply);
    }

    if (LL_SPI_IsActiveFlag_OVR(SPI1))
    {
        volatile uint32_t clear;
        clear = SPI1->DR;
        clear = SPI1->SR;
        (void)clear;
    }
}



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
    angleEncoder = stepper.angleMoved();
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
    // Serial.println(angleDesired);

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
