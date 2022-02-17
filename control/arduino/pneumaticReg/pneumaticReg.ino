// MCP4922 Demo code sinewave at 16 res top to bot.
// For comparison to MCP4725 operation (DAC_RESOLUTION=-5).
#include <SPI.h>

#define DAC_RESOLUTION    5
#define DAC_ARRAY_INDICES (pow(2,DAC_RESOLUTION))

SPISettings settingsA(16000000, MSBFIRST, SPI_MODE0);  // At 16 = SPI Clock = 8MHz.

const PROGMEM uint16_t SineLookup_5bits[32]
{
2048, 2447, 2831, 3185, 3495, 3750, 3939, 4056,
4095, 4056, 3939, 3750, 3495, 3185, 2831, 2447,
2048, 1648, 1264,  910,  600,  345,  156,   39,
   0,   39,  156,  345,  600,  910, 1264, 1648
};

int RCLKPin  = 10;   // pin 12 on the 74hc595 latch - nSS
int SRCLKPin = 13;  // pin 11 on the 74hc595 shift register clock - SCK
int SERPin   = 11;    //  MOSI
#define MARKER 4
//////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);        // Start serial port (debug).

  pinMode(RCLKPin,  OUTPUT);   // Set SPI control PINs to output.
  pinMode(SRCLKPin, OUTPUT);
  pinMode(SERPin,   OUTPUT);
  pinMode(MARKER,   OUTPUT);

  SPI.begin();

  Serial.println("MCP4922 SPI Dual DAC SPI hardware mode");
  noInterrupts();
}

//////////////////////////////////////////////////////////////////////////////
// 0 - A, 1 - B
//
void writeMCP4922_AB(byte AB, uint16_t v) {

    v |=0xf000;             // B15(A/B)=1 B, B14(BUF)=1 on, B13(GAn) 1=x1  B12(SHDNn) 1=off
    if (!AB)  v &= ~0x8000; // When zero clear B15 for A.

    SPI.beginTransaction(settingsA);
    digitalWrite(RCLKPin, LOW);
    SPI.transfer( (0xff00 & v)>>8 );
    SPI.transfer(      0x00ff & v );
    digitalWrite(RCLKPin, HIGH);
    SPI.endTransaction;
}

void loop() {
   for (int i = 0; i < DAC_ARRAY_INDICES; i++) {
      digitalWrite(MARKER, HIGH);
      digitalWrite(MARKER, LOW);
      writeMCP4922_AB( 0, pgm_read_word(&(SineLookup_5bits[i])) );
   }
}
