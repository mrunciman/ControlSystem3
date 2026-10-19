#include <UstepperS32.h>

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"


UstepperS32 stepper;

const uint8_t SS_Pin = D5;


union dataFloat
{
    float fData;
    uint8_t bData[4];
};


const unsigned long LOOP_FREQ = 100; // Hz
const unsigned long LOOP_PERIOD_MICRO =
    1000000UL / LOOP_FREQ;


// Time variables for loop frequency
unsigned long timeSinceExec = 0;
unsigned long timeNow = 0;
unsigned long timeLastExecution = 0;


bool ENABLECLOSEDLOOP = false;

float angleEncoder = 0.0f;
float prevAngleEncoder = 0.0f;
float angleDesired = 0.0f;

int encoderChangeCount = 0;
int changeCountThresh = 50;

bool encoderOK = true;
float brakeCurrent = 5.0f; // percent


volatile dataFloat posEncoder;
volatile dataFloat posFromController;

const int NUM_CHARS = 4;
const int LAST_CHAR_NUM = 5;


volatile int posIndex = 0;

volatile bool startMsgReceived = false;
volatile bool endMsgReceived = false;
volatile bool receivedOK = false;

const uint8_t START_MESSAGE = '<';
const uint8_t END_MESSAGE = '>';
const uint8_t POWER_ON_FLAG = 'Y';
const uint8_t POWER_OFF_FLAG = 'N';

volatile bool requestedPowerState = false;
volatile bool requestedPowerStateValid = false;

bool powerState = false;
bool powerEnabled = false;


float LEAD = 8.0f;
float STEPS_PER_MM = 200.0f * 32.0f;

long stepCountDesired = 0;
long stepCountDriver = 0;
long stepError = 0;

float angleEstDriver = 0.0f;


/*
 * Reset the active message parser.
 *
 * This does not clear receivedOK, because receivedOK may indicate
 * that a complete packet is waiting for loop() to consume it.
 */
inline void resetMessageParser()
{
    startMsgReceived = false;
    endMsgReceived = false;
    requestedPowerStateValid = false;
    posIndex = 0;
}


void setupPeripheralSPI()
{
    /*
     * Enable GPIOA and SPI1 clocks.
     */
    LL_AHB1_GRP1_EnableClock(
        LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_APB2_GRP1_EnableClock(
        LL_APB2_GRP1_PERIPH_SPI1);


    LL_GPIO_InitTypeDef gpio = {0};

    /*
     * PA5 = D6 = SPI1 SCK
     * PA6 = D3 = SPI1 MISO
     * PA7 = D2 = SPI1 MOSI
     */
    gpio.Pin =
        LL_GPIO_PIN_5 |
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
        SPI1,
        LL_SPI_FULL_DUPLEX);

    LL_SPI_SetMode(
        SPI1,
        LL_SPI_MODE_SLAVE);

    LL_SPI_SetDataWidth(
        SPI1,
        LL_SPI_DATAWIDTH_8BIT);

    /*
     * SPI Mode 0:
     * CPOL = 0
     * CPHA = 0
     */
    LL_SPI_SetClockPolarity(
        SPI1,
        LL_SPI_POLARITY_LOW);

    LL_SPI_SetClockPhase(
        SPI1,
        LL_SPI_PHASE_1EDGE);

    LL_SPI_SetTransferBitOrder(
        SPI1,
        LL_SPI_MSB_FIRST);

    /*
     * D5 is checked manually as software chip select.
     */
    LL_SPI_SetNSSMode(
        SPI1,
        LL_SPI_NSS_SOFT);


    /*
     * Clear parser state before enabling reception.
     */
    startMsgReceived = false;
    endMsgReceived = false;
    receivedOK = false;
    requestedPowerState = false;
    requestedPowerStateValid = false;
    posIndex = 0;


    NVIC_SetPriority(SPI1_IRQn, 1);
    NVIC_EnableIRQ(SPI1_IRQn);

    LL_SPI_EnableIT_RXNE(SPI1);
    LL_SPI_Enable(SPI1);

    /*
     * Initial byte returned during the first transaction.
     */
    LL_SPI_TransmitData8(SPI1, 0);
}


void setup(void)
{
    Serial.begin(115200);

    /*
     * Complete uStepper initialization before configuring
     * the external SPI peripheral.
     */
    if (ENABLECLOSEDLOOP)
    {
        stepper.setup(CLOSEDLOOP, 200);
    }
    else
    {
        stepper.setup();
    }


    posEncoder.fData = 0.0f;
    posFromController.fData = 0.0f;


    stepper.setMaxAcceleration(500);
    stepper.setMaxVelocity(1500);

    /*
     * Orientation check intentionally disabled.
     */
    // stepper.checkOrientation(4.0);

    /*
     * Begin with the motor unpowered.
     */
    stepper.setBrakeMode(FREEWHEELBRAKE);
    stepper.disablePid();

    powerState = false;
    powerEnabled = false;


    /*
     * SPI1 connections:
     *
     * D6 / PA5 = SCK from Mega
     * D3 / PA6 = MISO to Mega
     * D2 / PA7 = MOSI from Mega
     * D5       = software chip select
     */
    pinMode(D6, INPUT);
    pinMode(D3, OUTPUT);
    pinMode(D2, INPUT);
    pinMode(SS_Pin, INPUT_PULLUP);


    noInterrupts();
    setupPeripheralSPI();
    interrupts();
}


/*
 * SPI1 interrupt handler.
 *
 * Trigger:
 * The ISR is entered when SPI1 has received a complete byte and
 * RXNE is set, provided:
 *
 * - SPI1 is enabled;
 * - the RXNE interrupt is enabled;
 * - SPI1_IRQn is enabled in the NVIC;
 * - global interrupts are enabled.
 *
 * A transition on D5 does not itself trigger this ISR.
 */
extern "C" void SPI1_IRQHandler(void)
{
    if (LL_SPI_IsActiveFlag_RXNE(SPI1))
    {
        /*
         * Reading DR clears the received-data condition.
         */
        const uint8_t c =
            LL_SPI_ReceiveData8(SPI1);

        uint8_t reply = 0;


        /*
         * Software chip select is active-low.
         *
         * Ignore traffic when this board is not selected.
         */
        if (digitalRead(SS_Pin) != LOW)
        {
            resetMessageParser();

            /*
             * Keep the response register loaded for a future transfer.
             */
            LL_SPI_TransmitData8(SPI1, reply);
        }
        else if (receivedOK)
        {
            /*
             * A complete packet is waiting for loop().
             *
             * Do not overwrite its payload or clear receivedOK.
             */
            LL_SPI_TransmitData8(SPI1, reply);
        }
        else if (startMsgReceived)
        {
            /*
             * Receive the four bytes of the desired-angle float.
             */
            if (posIndex < NUM_CHARS)
            {
                posFromController.bData[posIndex] = c;

                reply = posEncoder.bData[posIndex];

                posIndex++;
            }

            /*
             * Receive and validate the requested power state.
             */
            else if (posIndex == NUM_CHARS)
            {
                if (c == POWER_ON_FLAG)
                {
                    requestedPowerState = true;
                    requestedPowerStateValid = true;
                    posIndex++;
                }
                else if (c == POWER_OFF_FLAG)
                {
                    requestedPowerState = false;
                    requestedPowerStateValid = true;
                    posIndex++;
                }
                else
                {
                    receivedOK = false;
                    resetMessageParser();
                }
            }

            /*
             * Receive and validate the end marker.
             */
            else if (posIndex == LAST_CHAR_NUM)
            {
                if ((c == END_MESSAGE) &&
                    requestedPowerStateValid)
                {
                    receivedOK = true;
                    endMsgReceived = true;
                }
                else
                {
                    receivedOK = false;
                    endMsgReceived = false;
                }

                startMsgReceived = false;
                requestedPowerStateValid = false;
                posIndex = 0;
            }

            /*
             * Invalid parser position.
             */
            else
            {
                receivedOK = false;
                resetMessageParser();
            }

            /*
             * Loaded now and transmitted during the next
             * transfer from the Mega.
             */
            LL_SPI_TransmitData8(SPI1, reply);
        }

        /*
         * Start a new packet only while the parser is idle.
         */
        else if (c == START_MESSAGE)
        {
            startMsgReceived = true;
            endMsgReceived = false;
            requestedPowerStateValid = false;
            posIndex = 0;

            LL_SPI_TransmitData8(SPI1, reply);
        }

        /*
         * Unexpected byte while idle.
         */
        else
        {
            resetMessageParser();

            LL_SPI_TransmitData8(SPI1, reply);
        }
    }


    /*
     * Clear an overrun condition.
     *
     * OVR can occur if another byte arrives before the previous
     * received byte is read.
     */
    if (LL_SPI_IsActiveFlag_OVR(SPI1))
    {
        volatile uint32_t clear;

        clear = SPI1->DR;
        clear = SPI1->SR;

        (void)clear;

        /*
         * Reject the current partially received packet because at
         * least one byte may have been lost.
         */
        receivedOK = false;
        resetMessageParser();
    }
}


// Convert desired angle to position in microsteps.
long convertDesiredAngleToStepNumber(float angleDes)
{
    return long(
        round(
            angleDes *
            LEAD *
            STEPS_PER_MM /
            360.0f));
}


// Convert position in microsteps to angular position in degrees.
float convertStepToAngle(float stepCountDri)
{
    return
        stepCountDri *
        360.0f /
        (LEAD * STEPS_PER_MM);
}


void encoderFlag()
{
    if (angleEncoder == prevAngleEncoder)
    {
        encoderChangeCount++;
    }
    else
    {
        encoderChangeCount = 0;
    }

    encoderOK =
        (encoderChangeCount <= changeCountThresh);
}


void toggleMotorPower(bool requestedState)
{
    if (requestedState)
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
}


void loop(void)
{
    /*
     * Update encoder/driver feedback at LOOP_FREQ.
     */
    timeNow = micros();
    timeSinceExec =
        timeNow - timeLastExecution;

    if (timeSinceExec >= LOOP_PERIOD_MICRO)
    {
        timeLastExecution = timeNow;

        prevAngleEncoder = angleEncoder;

        /*
         * uStepper S32 API differs from the 8-bit uStepper S API.
         */
        angleEncoder = stepper.angleMoved();

        stepCountDriver =
            stepper.driver.getPosition();

        angleEstDriver =
            convertStepToAngle(stepCountDriver);


        /*
         * Prevent SPI1_IRQHandler() from reading a partly
         * updated four-byte float.
         */
        noInterrupts();

        if (ENABLECLOSEDLOOP)
        {
            posEncoder.fData = angleEncoder;
        }
        else
        {
            posEncoder.fData = angleEstDriver;
        }

        interrupts();

        // encoderFlag();
    }


    /*
     * Atomically consume a complete accepted packet.
     */
    bool packetAvailable = false;
    bool localRequestedPowerState = false;
    float localRequestedAngle = 0.0f;

    noInterrupts();

    if (receivedOK)
    {
        localRequestedPowerState =
            requestedPowerState;

        localRequestedAngle =
            posFromController.fData;

        receivedOK = false;
        endMsgReceived = false;

        packetAvailable = true;
    }

    interrupts();


    /*
     * Only act on the atomically copied packet.
     */
    if (packetAvailable)
    {
        powerState =
            localRequestedPowerState;

        angleDesired =
            localRequestedAngle;


        toggleMotorPower(powerState);


        stepCountDesired =
            convertDesiredAngleToStepNumber(
                angleDesired);


        stepCountDriver =
            stepper.driver.getPosition();


        stepError =
            stepCountDesired -
            stepCountDriver;


        if (powerState)
        {
            stepper.moveSteps(stepError);
        }
    }
}