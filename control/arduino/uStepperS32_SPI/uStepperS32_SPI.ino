#include <UstepperS32.h>
// #include <callbacks.h>
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"

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



UstepperS32 stepper;

const uint8_t SS_Pin = D5;

union dataFloat{
  float fData;
  uint8_t bData[4];
};


enum StatusCode : uint8_t
{
    STATUS_IDLE          = 0x00,
    STATUS_RECEIVING     = 0x01,
    STATUS_ACCEPTED      = 0x02,
    STATUS_BUSY          = 0x03,
    STATUS_COMPLETE      = 0x04,
    STATUS_BAD_LENGTH    = 0x10,
    STATUS_BAD_CRC       = 0x11,
    STATUS_BAD_COMMAND   = 0x12,
    STATUS_BAD_PARAMETER = 0x13,
    STATUS_DRIVER_FAULT  = 0x20
};


volatile uint8_t receivedSequence = 0;
volatile uint8_t completedSequence = 0;
volatile uint8_t commandStatus = STATUS_IDLE;

enum CommandCode : uint8_t
{
    COMMAND_NOP          = 0x00,
    COMMAND_SET_POSITION = 0x01
};

constexpr uint8_t PACKET_SIZE = 9;

constexpr uint8_t RX_START_INDEX    = 0;
constexpr uint8_t RX_COMMAND_INDEX  = 1;
constexpr uint8_t RX_SEQUENCE_INDEX = 2;
constexpr uint8_t RX_ANGLE_INDEX    = 3;
constexpr uint8_t RX_POWER_INDEX    = 7;
constexpr uint8_t RX_END_INDEX      = 8;

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

// volatile int posIndex = 0;
volatile uint8_t lastByte = 0;

// volatile bool startMsgReceived = false; 
// volatile bool endMsgReceived = false; 
// volatile bool receivedOK = false;

// New receive and response buffers
volatile uint8_t rxPacket[PACKET_SIZE];
volatile uint8_t txPacket[PACKET_SIZE];

volatile uint8_t rxIndex = 0;
volatile uint8_t txIndex = 0;

volatile bool packetReady = false;

volatile StatusCode = STATUS_IDLE;
voltaile commandStatus = COMMAND_NOP;
volatile uint8_t receivedSequence = 0;
volatile uint8_t completedSequence = 0;

volatile bool requestedPowerState = false;
bool powerState = false;
bool powerEnabled = false;

int microDelay = 25;

char START_MESSAGE = '<';
char END_MESSAGE = '>';
char POWER_ON_FLAG = 'Y';
char POWER_OFF_FLAG = 'N';
// volatile bool powerState = false;
// volatile bool prevPowerState = false;


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


  // After orientation check, turn off motors:
  stepper.setBrakeMode(FREEWHEELBRAKE);
  stepper.disablePid();

  // ///////////////////////////////////////////

  pinMode(D6, INPUT); // SCK1 from master
  pinMode(D3, OUTPUT); // MISO1 to master
  pinMode(D2, INPUT); // MOSI1 from master
  pinMode(SS_Pin, INPUT_PULLUP);
  noInterrupts();
  setupPeripheralSPI();
  interrupts();

  buildResponsePacket(
    STATUS_IDLE,
    completedSequence,
    powerEnabled,
    angleEstDriver);

}


void buildResponsePacket(
    StatusCode status,
    uint8_t sequence,
    bool actualPowerState,
    float currentPosition)
{
    dataFloat position;
    position.fData = currentPosition;

    noInterrupts();

    txPacket[0] = START_MESSAGE;
    txPacket[1] = static_cast<uint8_t>(status);
    txPacket[2] = sequence;

    txPacket[3] = position.bData[0];
    txPacket[4] = position.bData[1];
    txPacket[5] = position.bData[2];
    txPacket[6] = position.bData[3];

    txPacket[7] =
        actualPowerState
            ? POWER_ON_FLAG
            : POWER_OFF_FLAG;

    txPacket[8] = END_MESSAGE;

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
    // LL_SPI_TransmitData8(SPI1, 0);
    LL_SPI_TransmitData8(SPI1, txPacket[0]);
}



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



void processPendingCommand()
{
    uint8_t localPacket[PACKET_SIZE];
    bool localPacketReady = false;

    noInterrupts();

    if (packetReady)
    {
        for (uint8_t i = 0; i < PACKET_SIZE; i++)
        {
            localPacket[i] = rxPacket[i];
        }

        packetReady = false;
        commandStatus = STATUS_BUSY;
        localPacketReady = true;
    }

    interrupts();

    if (!localPacketReady)
    {
        return;
    }

    const uint8_t command =
        localPacket[RX_COMMAND_INDEX];

    const uint8_t sequence =
        localPacket[RX_SEQUENCE_INDEX];

    if (command == COMMAND_NOP)
    {
        /*
         * A NOP clocks out the previously prepared acknowledgement.
         * Do not replace that acknowledgement with a new sequence.
         */
        return;
    }

    if (command != COMMAND_SET_POSITION)
    {
        commandStatus = STATUS_BAD_COMMAND;

        buildResponsePacket(
            STATUS_BAD_COMMAND,
            sequence,
            powerEnabled,
            angleEstDriver);

        return;
    }

    dataFloat requestedAngle;

    requestedAngle.bData[0] =
        localPacket[RX_ANGLE_INDEX + 0];

    requestedAngle.bData[1] =
        localPacket[RX_ANGLE_INDEX + 1];

    requestedAngle.bData[2] =
        localPacket[RX_ANGLE_INDEX + 2];

    requestedAngle.bData[3] =
        localPacket[RX_ANGLE_INDEX + 3];

    bool localPowerState;

    if (localPacket[RX_POWER_INDEX] == POWER_ON_FLAG)
    {
        localPowerState = true;
    }
    else if (localPacket[RX_POWER_INDEX] == POWER_OFF_FLAG)
    {
        localPowerState = false;
    }
    else
    {
        commandStatus = STATUS_BAD_PARAMETER;

        buildResponsePacket(
            STATUS_BAD_PARAMETER,
            sequence,
            powerEnabled,
            angleEstDriver);

        return;
    }

    /*
     * Reject NaN or infinity before converting to microsteps.
     */
    if (!isfinite(requestedAngle.fData))
    {
        commandStatus = STATUS_BAD_PARAMETER;

        buildResponsePacket(
            STATUS_BAD_PARAMETER,
            sequence,
            powerEnabled,
            angleEstDriver);

        return;
    }

    powerState = localPowerState;
    angleDesired = requestedAngle.fData;

    /*
     * Apply the accepted command.
     */
    if (powerState)
    {
        if (!powerEnabled)
        {
            stepper.setBrakeMode(
                HARDBRAKE,
                brakeCurrent);

            if (ENABLECLOSEDLOOP)
            {
                stepper.enablePid();
            }

            powerEnabled = true;
        }
    }
    else
    {
        stepper.setBrakeMode(FREEWHEELBRAKE);
        stepper.disablePid();
        powerEnabled = false;
    }

    stepCountDesired =
        convertDesiredAngleToStepNumber(
            angleDesired);

    stepCountDriver =
        stepper.driver.getPosition();

    stepError =
        stepCountDesired -
        stepCountDriver;

    if (powerEnabled)
    {
        stepper.moveSteps(stepError);
    }

    /*
     * The command has been applied.
     */
    completedSequence = sequence;
    commandStatus = STATUS_COMPLETE;

    /*
     * Prepare the response that will be returned during the next
     * SPI transaction.
     */
    buildResponsePacket(
        STATUS_COMPLETE,
        completedSequence,
        powerEnabled,
        angleEstDriver);
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
    timeLastExecution = timeNow; 

    // Get angle data from encoder
    prevAngleEncoder = angleEncoder;
    angleEncoder = stepper.angleMoved();
    // Get estimated (open loop) position from driver
    stepCountDriver = stepper.driver.getPosition();
    angleEstDriver = convertStepToAngle(stepCountDriver);

    noInterrupts();
    // Put in float variable for SPI comms
    if (ENABLECLOSEDLOOP){
      posEncoder.fData = angleEncoder;
    }
    else{
      posEncoder.fData = angleEstDriver;
    }
    interrupts();
    
    // encoderFlag();
  }

  processPendingCommand();


  // If both start and end messages received correctly,
  // change angular position received from controller
  // if (checkInput() == 1){// Check the input array and reset the receivedOK flag
  //   toggleMotorPower(powerState, prevPowerState);
  //   checkEncoder(encoderOK);

  //   // Read desired angle from pump controller
  //   angleDesired = posFromController.fData;
  //   // Convert desired angle to number of microsteps
  //   stepCountDesired = convertDesiredAngleToStepNumber(angleDesired);    

  //   // Update actual position, expressed in microsteps
  //   stepCountDriver = stepper.driver.getPosition();
  //   // int load = stepper.driver.getStallValue();
  //   // Serial.println(angleDesired);

  //   // Calculate difference between desired position and actual position
  //   stepError = stepCountDesired - stepCountDriver;

  //   // Move towards target position
  //   if(powerState){
  //     // Checking the power first will make sure stepCountDriver is accurate
  //     stepper.moveSteps(stepError);
  //   }

  //   // IF angleEncoder FROM ENCODER DOESN'T MATCH THE ESTIMATED ANGLE 
  //   // angleEstDriver FROM THE DRIVER THEN WE HAVE A PROBLEM


  //   // Serial.print("Encoder:    ");
  //   // Serial.println(angleEncoder);

  //   // Serial.print("Est angle:  ");
  //   // Serial.println(angleEstDriver);
  //   // Serial.print("Angle in:   ");
  //   // Serial.println(angleDesired);

  //   // Serial.print("Est steps:  ");
  //   // Serial.println(stepCountDriver);
  //   // Serial.print("Step in:    ");
  //   // Serial.println(stepCountDesired);

  //   // Serial.print("Step error: ");
  //   // Serial.println(stepError);

  //   // Serial.print("Power:      ");
  //   // Serial.println(powerState);
  //   // Serial.println();

  //   // Clear message flags so that any one message
  //   // will only move the motor once.
  //   startMsgReceived = false;
  //   endMsgReceived = false;
  // }
  
}
