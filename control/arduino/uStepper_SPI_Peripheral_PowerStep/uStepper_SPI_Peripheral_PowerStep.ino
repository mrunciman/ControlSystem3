#include <uStepperS.h>
#include <math.h>


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


uStepperS stepper;


// Software chip-select from the Mega.
const uint8_t SS_Pin = 3;

/*
 * This direct register definition assumes Arduino pin 3 on the
 * uStepper S maps to PD3.
 *
 * Verify this against the selected uStepper board definition.
 */
#define SOFTWARE_SS_PIN_REG PIND
#define SOFTWARE_SS_BIT PD3


union dataFloat
{
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


const unsigned long LOOP_FREQ = 100;
const unsigned long LOOP_PERIOD_MICRO =
    1000000UL / LOOP_FREQ;


// Time variables for loop frequency.
unsigned long timeSinceExec = 0;
unsigned long timeNow = 0;
unsigned long timeLastExecution = 0;


bool ENABLECLOSEDLOOP = false;

float angleEncoder = 0.0f;
float prevAngleEncoder = 0.0f;
float angleDesired = 0.0f;

float brakeCurrent = 5.0f;


// Receive and response buffers.
volatile uint8_t rxPacket[PACKET_SIZE];
volatile uint8_t txPacket[PACKET_SIZE];

volatile uint8_t rxIndex = 0;
volatile uint8_t txIndex = 0;

volatile bool packetReady = false;

volatile StatusCode commandStatus = STATUS_IDLE;
volatile uint8_t receivedSequence = 0;
volatile uint8_t completedSequence = 0;


bool powerState = false;
bool powerEnabled = false;


const uint8_t START_MESSAGE = '<';
const uint8_t END_MESSAGE = '>';
const uint8_t POWER_ON_FLAG = 'Y';
const uint8_t POWER_OFF_FLAG = 'N';


float LEAD = 8.0f;
float STEPS_PER_MM = 200.0f * 32.0f;

long stepCountDesired = 0;
long stepCountDriver = 0;
long stepError = 0;

float angleEstDriver = 0.0f;


// Forward declarations.
void buildResponsePacket(
    StatusCode status,
    uint8_t sequence,
    bool actualPowerState,
    float currentPosition);

void setupPeripheralSPI();

void processPendingCommand();

long convertDesiredAngleToStepNumber(float angleDes);

float convertStepToAngle(long stepCountDri);


void setup(void)
{
    Serial.begin(115200);

    if (ENABLECLOSEDLOOP)
    {
        stepper.setup(CLOSEDLOOP, 200);
    }
    else
    {
        stepper.setup();
    }


    stepper.setMaxAcceleration(500);
    stepper.setMaxVelocity(1500);


    // Start with the motor disabled.
    stepper.setBrakeMode(FREEWHEELBRAKE);
    stepper.disablePid();

    powerState = false;
    powerEnabled = false;


    /*
     * Construct the initial response before enabling SPI.
     */
    buildResponsePacket(
        STATUS_IDLE,
        completedSequence,
        powerEnabled,
        angleEstDriver);


    setupPeripheralSPI();
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

    txPacket[1] =
        static_cast<uint8_t>(status);

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


    txIndex = 0;


    /*
     * If SPI0 is already enabled, preload response byte zero.
     */
    if (SPCR0 & (1 << SPE0))
    {
        SPDR0 = txPacket[0];
    }


    interrupts();
}


void setupPeripheralSPI()
{
    /*
     * This board returns data to the Mega through MISO0.
     */
    pinMode(MISO0, OUTPUT);

    /*
     * SPI0's hardware SS pin remains an input.
     *
     * The Mega selects this peripheral using SS_Pin instead.
     */
    pinMode(SS0, INPUT);

    /*
     * Software chip-select from the Mega.
     */
    pinMode(SS_Pin, INPUT_PULLUP);


    noInterrupts();


    rxIndex = 0;
    txIndex = 0;
    packetReady = false;

    commandStatus = STATUS_IDLE;
    receivedSequence = 0;
    completedSequence = 0;


    /*
     * Configure SPI0:
     *
     * SPIE0 = 1: SPI transfer-complete interrupt enabled
     * SPE0  = 1: SPI enabled
     * DORD0 = 0: MSB first
     * MSTR0 = 0: slave mode
     * CPOL0 = 0: clock idle low
     * CPHA0 = 0: sample on first edge
     *
     * This is SPI Mode 0.
     */
    SPCR0 =
        (1 << SPIE0) |
        (1 << SPE0);


    /*
     * Preload the first byte returned to the Mega.
     */
    SPDR0 = txPacket[0];


    interrupts();
}


/*
 * Triggered when SPI0 completes one eight-bit transfer, provided:
 *
 * - SPI0 is enabled;
 * - SPIE0 is set;
 * - global interrupts are enabled.
 *
 * A transition on the software chip-select pin does not itself
 * trigger this ISR.
 */
ISR(SPI_STC_vect)
{
    /*
     * Read the byte received during the completed transfer.
     */
    const uint8_t receivedByte = SPDR0;


    /*
     * The software chip-select is active-low.
     *
     * Ignore traffic while this board is not selected.
     */
    if ((SOFTWARE_SS_PIN_REG &
         _BV(SOFTWARE_SS_BIT)) != 0)
    {
        rxIndex = 0;
        txIndex = 0;

        /*
         * Prepare response byte zero for the next transaction.
         */
        SPDR0 = txPacket[0];

        return;
    }


    /*
     * The byte transmitted during the current transfer was already
     * in SPDR0 before the transfer began.
     *
     * Load the following response byte for the next transfer.
     */
    if (txIndex < (PACKET_SIZE - 1))
    {
        txIndex++;
        SPDR0 = txPacket[txIndex];
    }
    else
    {
        /*
         * Fallback value for any excess clock pulses.
         */
        SPDR0 = 0;
    }


    /*
     * Do not overwrite a complete command which loop() has not
     * yet copied and processed.
     */
    if (packetReady)
    {
        return;
    }


    /*
     * When idle, only a start marker begins a new packet.
     */
    if (rxIndex == 0)
    {
        if (receivedByte == START_MESSAGE)
        {
            rxPacket[RX_START_INDEX] =
                receivedByte;

            rxIndex = 1;

            commandStatus =
                STATUS_RECEIVING;
        }

        return;
    }


    /*
     * Store subsequent bytes in the fixed-size packet.
     */
    if (rxIndex < PACKET_SIZE)
    {
        rxPacket[rxIndex] =
            receivedByte;

        rxIndex++;
    }


    /*
     * A complete physical packet has arrived.
     */
    if (rxIndex == PACKET_SIZE)
    {
        const bool framingValid =
            (rxPacket[RX_START_INDEX] ==
             START_MESSAGE) &&
            (rxPacket[RX_END_INDEX] ==
             END_MESSAGE);


        if (framingValid)
        {
            receivedSequence =
                rxPacket[RX_SEQUENCE_INDEX];

            commandStatus =
                STATUS_ACCEPTED;

            packetReady = true;
        }
        else
        {
            commandStatus =
                STATUS_BAD_LENGTH;
        }


        /*
         * Prepare to receive the next transaction.
         *
         * The next response byte zero is normally preloaded by
         * buildResponsePacket() after the command is processed.
         */
        rxIndex = 0;
        txIndex = 0;
    }
}


void processPendingCommand()
{
    uint8_t localPacket[PACKET_SIZE];
    bool localPacketReady = false;


    noInterrupts();


    if (packetReady)
    {
        for (uint8_t i = 0;
             i < PACKET_SIZE;
             i++)
        {
            localPacket[i] =
                rxPacket[i];
        }

        packetReady = false;
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
         * A NOP only clocks out the previously prepared
         * acknowledgement.
         *
         * Restore byte zero so another status read begins with the
         * response start marker.
         */
        noInterrupts();

        txIndex = 0;
        SPDR0 = txPacket[0];

        interrupts();

        return;
    }


    commandStatus = STATUS_BUSY;


    if (command != COMMAND_SET_POSITION)
    {
        commandStatus =
            STATUS_BAD_COMMAND;

        buildResponsePacket(
            commandStatus,
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


    bool localPowerState = false;


    if (localPacket[RX_POWER_INDEX] ==
        POWER_ON_FLAG)
    {
        localPowerState = true;
    }
    else if (localPacket[RX_POWER_INDEX] ==
             POWER_OFF_FLAG)
    {
        localPowerState = false;
    }
    else
    {
        commandStatus =
            STATUS_BAD_PARAMETER;

        buildResponsePacket(
            commandStatus,
            sequence,
            powerEnabled,
            angleEstDriver);

        return;
    }


    /*
     * Reject NaN and infinity before converting the requested
     * angle to microsteps.
     */
    if (!isfinite(requestedAngle.fData))
    {
        commandStatus =
            STATUS_BAD_PARAMETER;

        buildResponsePacket(
            commandStatus,
            sequence,
            powerEnabled,
            angleEstDriver);

        return;
    }


    powerState = localPowerState;
    angleDesired = requestedAngle.fData;


    /*
     * Apply the requested motor-power state.
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
        stepper.setBrakeMode(
            FREEWHEELBRAKE);

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
     * Prepare the response returned during the next transaction.
     */
    buildResponsePacket(
        commandStatus,
        completedSequence,
        powerEnabled,
        angleEstDriver);
}


// Convert desired angle to position in microsteps.
long convertDesiredAngleToStepNumber(float angleDes)
{
    return static_cast<long>(round(angleDes * LEAD * STEPS_PER_MM / 360.0f));
}


// Convert position in microsteps to angular position in degrees.
float convertStepToAngle(long stepCountDri)
{
    return static_cast<float>(stepCountDri) * 360.0f / (LEAD * STEPS_PER_MM);
}


void loop(void)
{
    timeNow = micros();
    timeSinceExec = timeNow - timeLastExecution;

    if (timeSinceExec >= LOOP_PERIOD_MICRO)
    {
        timeLastExecution = timeNow;

        prevAngleEncoder = angleEncoder;

        angleEncoder = stepper.encoder.getAngleMoved();

        stepCountDriver = stepper.driver.getPosition();

        angleEstDriver = convertStepToAngle(stepCountDriver);
    }


    processPendingCommand();
}