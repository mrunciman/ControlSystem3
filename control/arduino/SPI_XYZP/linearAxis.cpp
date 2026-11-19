#include "linearAxis.h"
// #include <avr/dtostrf.h> // for atoi() function on Due
#include <SPI.h>


LinAxis::LinAxis():  settingsSPI(500000, MSBFIRST, SPI_MODE0) 
{

}


void LinAxis::init(int CS_Pin, int P_Pin, float periodSeconds, int L_Pin)
{    
  PERIOD = periodSeconds;
  selectPin = CS_Pin;
  pressPin = P_Pin;
  limitPin = L_Pin;
  pinMode(selectPin, OUTPUT);
  digitalWrite(selectPin, HIGH);
  pinMode(limitPin, INPUT_PULLUP);
  // pinMode(pressPin, INPUT);
  dataIn.fData = 0.0;
  dataOut.fData = 0.0;
}




void LinAxis::sendRecvFloat_POWER(dataFloat *outData, dataFloat *inData, char powerStatus)
{
  prevAngleIn = angleIn;
  SPI.beginTransaction(settingsSPI);
  // take the select pin low to activate buffer
  digitalWrite(selectPin, LOW);
  
  //Send first byte and discard last byte that was sent (lastByte)
  // transfer first sends data on MOSI, then waits and receives from MISO

  // firstByte should be 0 sent by motor at end of SPI interrupt
  firstByte = SPI.transfer(firstOut); 
  // Serial.print(firstOut);
  delayMicroseconds(microDelay);

  // firstByte will now be 0 from start of SPI interrupt on motor
  firstByte = SPI.transfer(outData->bData[0]);
  // Serial.print(outData->bData[0], BIN);
  // Serial.print(firstByte);
  delayMicroseconds(microDelay);
  for (int i = 0; i < NUM_CHARS; i++){
    if (i < NUM_CHARS-2){
      // Send bytes 2 - 4 and receive bytes 1 -3
      inData->bData[i] = SPI.transfer(outData->bData[i+1]);
      // Serial.print(outData->bData[i+1], BIN);
      // Serial.print(inData->bData[i], BIN);
    }
    else if (i == NUM_CHARS-2){
      // Send placeholder last byte and receive byte 4
      inData->bData[i] = SPI.transfer(powerStatus);
      // Serial.print(powerStatus);
      // Serial.print(inData->bData[i], BIN);
    }
    else if (i == NUM_CHARS-1){
      firstByte = SPI.transfer(lastOut);
      // Serial.print(lastOut);
      // Serial.print(firstByte);
      
    }
    delayMicroseconds(microDelay); // delay between transmissions
    // Serial.println(inData->bData[i]);
  }
  // take the select pin high to de-select the chip:
  digitalWrite(selectPin, HIGH);
  delayMicroseconds(microDelay);
  SPI.endTransaction();

  angleIn = REAL_DIRECTION*inData->fData;
}


bool LinAxis::acknowledgementMatchesPreviousCommand() const
{
    return responseFramingValid && (acknowledgedSequence == previousCommandSequence);
}



bool LinAxis::acknowledgementSuccessful() const
{
    return acknowledgementMatchesPreviousCommand() && (acknowledgedStatus == STATUS_COMPLETE);
}



bool LinAxis::sendCommandPipelined(
    float desiredAngle,
    char requestedPowerState)
{

  if (!isfinite(desiredAngle))
  {
    acknowledgementErrorCount++;
    return false;
  }
    
  dataFloat desired;
  desired.fData = desiredAngle;

  /*
  * The response received during this transaction should normally
  * acknowledge the previous command.
  */
  previousCommandSequence = lastCommandSequence;
  lastCommandSequence = nextSequence;
  responseFramingValid = false;
  responseSequenceValid = false;
  communicationHealthy = false;

  txPacket[PACKET_START_INDEX] =
      '<';

  txPacket[PACKET_TYPE_INDEX] =
      COMMAND_SET_POSITION;

  txPacket[PACKET_SEQUENCE_INDEX] =
      lastCommandSequence;

  txPacket[PACKET_FLOAT_INDEX + 0] =
      desired.bData[0];

  txPacket[PACKET_FLOAT_INDEX + 1] =
      desired.bData[1];

  txPacket[PACKET_FLOAT_INDEX + 2] =
      desired.bData[2];

  txPacket[PACKET_FLOAT_INDEX + 3] =
      desired.bData[3];

  txPacket[PACKET_POWER_INDEX] =
      static_cast<uint8_t>(requestedPowerState);

  txPacket[PACKET_END_INDEX] =
      '>';


  SPI.beginTransaction(settingsSPI);

  digitalWrite(selectPin, LOW);
  delayMicroseconds(microDelay);

  for (uint8_t i = 0; i < SPI_PACKET_SIZE; i++)
  {
      rxPacket[i] =
          SPI.transfer(txPacket[i]);

      delayMicroseconds(microDelay);
  }

  digitalWrite(selectPin, HIGH);
  delayMicroseconds(microDelay);

  SPI.endTransaction();

  /*
    * Validate the fixed response frame.
    */
  responseFramingValid =
      (rxPacket[PACKET_START_INDEX] == '<') &&
      (rxPacket[PACKET_END_INDEX] == '>');

  if (!responseFramingValid)
  {
      framingErrorCount++;
      acknowledgementErrorCount++;
      communicationHealthy = false;
  }
  else
  {
      acknowledgedStatus =
          static_cast<StatusCode>(
              rxPacket[PACKET_TYPE_INDEX]);

      acknowledgedSequence =
          rxPacket[PACKET_SEQUENCE_INDEX];

      dataFloat returnedAngle;

      returnedAngle.bData[0] =
          rxPacket[PACKET_FLOAT_INDEX + 0];

      returnedAngle.bData[1] =
          rxPacket[PACKET_FLOAT_INDEX + 1];

      returnedAngle.bData[2] =
          rxPacket[PACKET_FLOAT_INDEX + 2];

      returnedAngle.bData[3] =
          rxPacket[PACKET_FLOAT_INDEX + 3];

      prevAngleIn = angleIn;
      angleIn =
          REAL_DIRECTION *
          returnedAngle.fData;

      actualPowerState =
          static_cast<char>(
              rxPacket[PACKET_POWER_INDEX]);

      responseSequenceValid = (acknowledgedSequence == previousCommandSequence);

      /*
      * STATUS_IDLE with sequence 0 is expected on the first
      * transaction after startup.
      */
      const bool startupResponse =
          (previousCommandSequence == 0) &&
          (acknowledgedSequence == 0) &&
          (acknowledgedStatus == STATUS_IDLE);

      if (responseSequenceValid || startupResponse)
      {
          communicationHealthy = true;
          acknowledgementTime = millis();
      }
      else
      {
          sequenceErrorCount++;
          acknowledgementErrorCount++;
          communicationHealthy = false;
      }
  }


  /*
    * Advance sequence number. Zero is reserved for startup/idle.
    */
  nextSequence++;

  if (nextSequence == 0)
  {
      nextSequence = 1;
  }


  return acknowledgementSuccessful();
}



bool LinAxis::readLatestAcknowledgement()
{
    for (uint8_t i = 0; i < SPI_PACKET_SIZE; i++)
    {
        txPacket[i] = 0;
    }

    txPacket[PACKET_START_INDEX] = '<';
    txPacket[PACKET_TYPE_INDEX] = COMMAND_NOP;

    /*
     * The NOP carries the most recent command sequence for
     * diagnostic clarity. The peripheral does not execute it.
     */
    txPacket[PACKET_SEQUENCE_INDEX] = lastCommandSequence;
    txPacket[PACKET_POWER_INDEX] = 'N';
    txPacket[PACKET_END_INDEX] = '>';


    SPI.beginTransaction(settingsSPI);

    digitalWrite(selectPin, LOW);
    delayMicroseconds(microDelay);

    for (uint8_t i = 0; i < SPI_PACKET_SIZE; i++)
    {
        rxPacket[i] =
            SPI.transfer(txPacket[i]);

        delayMicroseconds(microDelay);
    }

    digitalWrite(selectPin, HIGH);
    delayMicroseconds(microDelay);

    SPI.endTransaction();


    responseFramingValid =
        (rxPacket[PACKET_START_INDEX] == '<') &&
        (rxPacket[PACKET_END_INDEX] == '>');

    if (!responseFramingValid)
    {
        framingErrorCount++;
        acknowledgementErrorCount++;
        communicationHealthy = false;
        return false;
    }


    acknowledgedStatus =
        static_cast<StatusCode>(
            rxPacket[PACKET_TYPE_INDEX]);

    acknowledgedSequence =
        rxPacket[PACKET_SEQUENCE_INDEX];

    dataFloat returnedAngle;

    for (uint8_t i = 0; i < 4; i++)
    {
        returnedAngle.bData[i] =
            rxPacket[PACKET_FLOAT_INDEX + i];
    }

    prevAngleIn = angleIn;
    angleIn =
        REAL_DIRECTION *
        returnedAngle.fData;

    actualPowerState =
        static_cast<char>(
            rxPacket[PACKET_POWER_INDEX]);

    responseSequenceValid =
        (acknowledgedSequence ==
         lastCommandSequence);

    communicationHealthy =
        responseSequenceValid;

    if (communicationHealthy)
    {
        acknowledgementTime = millis();
    }
    else
    {
        sequenceErrorCount++;
        acknowledgementErrorCount++;
    }

    return communicationHealthy && (acknowledgedStatus == STATUS_COMPLETE);
}


float LinAxis::readPressure()
{
  bitsSensor = analogRead(pressPin);
  V_sensor = bitsSensor * ADC_V;
  pressureRead = PSI_TO_KPA*((V_sensor - 0.1*V_SUPPLY)*(P_MAX_SENSOR - P_MIN_SENSOR)/(0.8*V_SUPPLY) + P_MIN_SENSOR) - pressureBaseline; // FOR GAGE PRESSURE, PMIN IS ATMOSPHERIC PRESSURE
  pressure = int(pressureRead);
  return pressureRead;
}



float LinAxis::readPressureADC(float v_adc){
  pressureRead = PSI_TO_KPA*((v_adc - 0.1*V_SUPPLY)*(P_MAX_SENSOR - P_MIN_SENSOR)/(0.8*V_SUPPLY) + P_MIN_SENSOR) - pressureBaseline; // FOR GAGE PRESSURE, PMIN IS ATMOSPHERIC PRESSURE
  return pressureRead;
}




void LinAxis::pressInitZeroVol()
{
  //Set state for motor motion based on comparison of pressure signal with setpoint
  readPressure();
  // pressureRead = 100;

  float pressureError = pressureRead - desiredPress;

  if (motorState == 3){
    startTime = millis();
  }

  prevMotorState = motorState;
  // If time/pressure threshold hasn't been reached, enter algorithm
  if ((pumpCalibrated == 0) & (stateCount < STABLE_TIME)){
    // Assign motor state based on pressure error
    if (pressureRead < desiredPress - P_THRESH){ // The pressure is less than setpoint minus threshold
      lowFlag = true;
      if (pressureRead > desiredPress - 2*P_THRESH){
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
        if (pressureRead <= desiredPress + P_THRESH){
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
    // Serial.println(motorState);
  }
  else{
    // Show that pump has been calibrated
    pumpCalibrated = 1;

    // Stop motor
    deltaAngle = 0.0;

    // record encoder value at which the actuator is 'empty'
    angleAtZeroVol = angleIn;

    // Set motor state
    motorState = 0;

    // reset stateCount
    stateCount = 0;
  }



  switch (motorState) {
    case 0:
      // Stop motor
      // Serial.println("Stop");
      deltaAngle = 0.0;
      break;


    case 1:
      //Move motor forwards
      //Serial.println("INCREASE PRESSURE");
      // If close to target pressure, use finer movements
      // if within 50 mbar of target pressure go slower
      if (abs(pressureError) < P_FINE){  // CHECK P_FINE
        deltaAngle = REAL_DIRECTION*convertSpeedToAngle(CAL_SPEED_FINE);
      }
      else{
        deltaAngle = REAL_DIRECTION*convertSpeedToAngle(CALIBRATION_SPEED);
      }
      break;


    case 2:
      //Move motor back
      //Serial.println("DECREASE PRESSURE");
      // If close to target pressure, use finer movements
      // if within 50 mbar of target pressure go slower
      if (abs(pressureError) < P_FINE){  // CHECK P_FINE
        deltaAngle = REAL_DIRECTION*convertSpeedToAngle(-CAL_SPEED_FINE);
      }
      else{
        deltaAngle = REAL_DIRECTION*convertSpeedToAngle(-CALIBRATION_SPEED);
      }
      break;


    default:
      //Just in case nothing matches, stop motor
      // Serial.println("Default");
      deltaAngle = 0.0;
      break;
  }
  

}



float LinAxis::convertFlowRateToAngle(float flow_rate){
  // Takes a desired flow rate and converts it to a 
  // an angle change to be executed in the next time step
  float angleChange = 360.0*(flow_rate*PERIOD)/(PISTON_AREA*LEAD); // Degrees
  return angleChange;
}


float LinAxis::convertSpeedToAngle(float stage_speed){
  // takes the desired speed of the linear stage in mm/s and 
  // converts it to an angle change to be executed in the next time step
  float angleChange = 360.0*(stage_speed*PERIOD)/(LEAD); // Degrees
  return angleChange;
}

float LinAxis::convertDistanceToAngle(float distance){
  // takes a desired distance for the stage to move in mm and 
  // converts it to an angle change to be executed
  float angleChange = 360.0*(distance)/(LEAD); // Degrees
  return angleChange;
}


float LinAxis::convertAngleToSpeed(float angleChange){
  // takes the desired angle change in deg/s and 
  // converts it to an angle change to be executed in the next time step
  // angleChange = (stage_speed*PERIOD)/(LEAD);
  float stage_speed = ((angleChange/360.0)*LEAD)/PERIOD; // mm/s
  return stage_speed;
}


bool LinAxis::moveToLimit(){
  if (pumpCalibrated == 0){
    if (digitalRead(limitPin) == LOW){ // MAKE SURE HIGH IS THE CORRECT CHOICE
      //Move forward off the switch and set flag
      deltaAngle = REAL_DIRECTION*convertDistanceToAngle(2); // THis will be a move at max speed
      // Serial.println(deltaAngle);
      pumpCalibrated = 1;
      // record encoder value at which the actuator is 'empty' / calibrated in case of prismatic joint
      angleAtZeroVol = angleIn;
      // Serial.println("Limit hit");
    }
    else{
      // Keep moving back until switch hit
      deltaAngle = REAL_DIRECTION*convertSpeedToAngle(-CAL_SPEED_FINE);
    }
  }

}


