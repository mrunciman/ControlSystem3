#include "pressRegPRE1-U08.h"
#include <SPI.h>

PressReg::PressReg():  pressSPI(500000, MSBFIRST, SPI_MODE0) 
{

}


void PressReg::init(int CS_Pin, int P_Pin, int V_Pin, int S_Pin)
{
  selectPinAir = CS_Pin;
  pressPinAir = P_Pin;
  valvePin = V_Pin;
  valvePinStruct = S_Pin;
  pinMode(selectPinAir, OUTPUT);
  digitalWrite(selectPinAir, HIGH);

  //Initialise valve digital output pins
  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, LOW);
  pinMode(valvePinStruct, OUTPUT);
  digitalWrite(valvePinStruct, LOW);
}



float PressReg::readStructPressure()
{
  bitsSensor = analogRead(pressPinAir);
  V_sensor = bitsSensor * ADC_V;
  pressureRead = PSI_TO_KPA*((V_sensor - 0.1*V_SUPPLY)*(P_MAX_SENSOR - P_MIN_SENSOR)/(0.8*V_SUPPLY) + P_MIN_SENSOR) - pressureBaseline; // FOR GAGE PRESSURE, PMIN IS ATMOSPHERIC PRESSURE
  pressure = int(pressureRead);
  return pressureRead;
}

float PressReg::convStructPressADC(float v_adc){
  pressureRead = PSI_TO_KPA*((v_adc - 0.1*V_SUPPLY)*(P_MAX_SENSOR - P_MIN_SENSOR)/(0.8*V_SUPPLY) + P_MIN_SENSOR) - pressureBaseline; // FOR GAGE PRESSURE, PMIN IS ATMOSPHERIC PRESSURE
  return pressureRead;
}


void PressReg::writePressureReg() 
{
  // Tell DAC what settings/output to use

  // Scale pressure value to maximum pressure value that the regulator can deliver
  // MAKE PMAX AND PMIN VARIABLES, VMAX AND VMIN

  V_DAC_BITS = (desStructPress/((P_MAX_REG - P_MIN_REG)*(V_DAC_REF/V_REG_REF))) * DAC_RESOLUTION;
  // float commandPressure = ((V_REG_REF/V_DAC_REF)*(P_MAX_REG - P_MIN_REG)) * (V_12bit/DAC_RESOLUTION);
  // Convert from float to unsigned 16 bit value
  uint16_t V_12bit = round(V_DAC_BITS);

  V_12bit |=0xf000;             // B15(A/B)=1 B, B14(BUF)=1 on, B13(GAn) 1=x1  B12(SHDNn) 1=off
  if (!outSelect)  V_12bit &= ~0x8000; // When zero clear B15 for A.

  SPI.beginTransaction(pressSPI);
  digitalWrite(selectPinAir, LOW);
  SPI.transfer( (0xff00 & V_12bit)>>8 ); // This is necessary because a 16 bit int is being sent
  delayMicroseconds(microDelay_PR);
  SPI.transfer(  0x00ff & V_12bit );
  delayMicroseconds(microDelay_PR);
  digitalWrite(selectPinAir, HIGH);
  SPI.endTransaction();
}



void PressReg::setStructValveState()
{
  if(pressureRead > DEPLOY_PRESSURE){
    // If pressure is high enough, close the valve and hold
    digitalWrite(valvePin, LOW);
  }
  else if (pressureRead > STRUCT_MAX_PRESS){
    digitalWrite(valvePin, HIGH);
  }
}

void PressReg::closeValve()
{
  digitalWrite(valvePin, LOW);
}

void PressReg::openValve()
{
  digitalWrite(valvePin, HIGH);
}


void PressReg::latchValve()
{
    // NEED TO DEBOUNCE 
    // Set valve state based on desired Pressure
    if (pressLatch == 0){
      // The pressure is too low, so open valve to regulate
      digitalWrite(valvePin, HIGH);
      // Serial.println("Valve open");
      desiredPress = inflatingPress;

      if (pressureRead >= DEPLOY_PRESSURE){
        // Start a count 
        correctPressCount = correctPressCount + 1;
        if (correctPressCount > correctPressThresh){
          desiredPress = 0.0;
          // Close valve when correct pressure reached
          digitalWrite(valvePin, LOW);
          // Serial.println("Valve closed");
          pressLatch = 1;
        }
      }
      else{
        correctPressCount = 0;
      }
    }
    else{ 
      // Pressure in structure too high or too low, so open valve to regulate
      if((pressureRead < STRUCT_MIN_PRESS)||(pressureRead > STRUCT_MAX_PRESS)){
        digitalWrite(valvePin, HIGH);
        // Serial.println("Valve open");
        desiredPress = DEPLOY_PRESSURE;
        pressLatch = 0;
        correctPressCount = 0;
      }
    }

    // Prevent max regulator pressure going too high
    if (desiredPress > P_MAX){
      desiredPress = P_MAX;
    }
}


void PressReg::pressureProfile()
{
  // Ramp pressure
  if (profileCount != countLimit+1){
      inflatingPress = DEPLOY_PRESSURE*(profileCount/countLimit);
      profileCount = profileCount + 1;
  }
}


void PressReg::inflateStructure()
{
  // supply valve: open
  // struct valve: open
  // press reg:    P_inf

  digitalWrite(valvePin, HIGH);
  digitalWrite(valvePinStruct, HIGH);

}



void PressReg::isolateSupply()
{
  // supply valve: closed
  // struct valve: closed
  // press reg:    0 kPa
  digitalWrite(valvePin, LOW);
  digitalWrite(valvePinStruct, LOW);

}


void PressReg::deflateStructure()
{
  // supply valve: closed
  // struct valve: open
  // press reg:    P_inf
  digitalWrite(valvePin, LOW);
  digitalWrite(valvePinStruct, HIGH);
}