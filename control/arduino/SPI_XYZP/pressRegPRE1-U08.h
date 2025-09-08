#include <SPI.h>


class PressReg{

  ///////////////////////////////////////////////////////////////////////////
  public:
    // Constructor
    PressReg();

    float P_MAX_REG = 800.0; // kPa
    float P_MIN_REG = 0.0;   // kPa

    // analog pin for related pressure sensor
    int pressPinAir;
    int selectPinAir;
    int valvePin;
    int valvePinStruct;

    SPISettings pressSPI;
    float desStructPress; // Must be in kPa
    float inputPress = 0.0;
    float prevInputPress = 0.0;
    byte outSelect = 0;
    int microDelay_PR = 25;

    /////////////////////////////////////////////////////////////////////////
    // Pressure reading constants and variables

    //Convert from psi to kPa
    float PSI_TO_KPA = 6.89476;
    // conversion multiplier from Arduino ADC value to voltage in mV
    const float ADC_V = 2.56/pow(2, 10);   //NEEDS TO CHANGE FOR DUE BOARD, CHECK THIS
    // ARDUINO DUE RUNS AT 3.3V 
    // Constant values for conversion equation
    float V_SUPPLY = 5.0;
    float P_MAX_SENSOR = 100.0; // psi
    float P_MIN_SENSOR = 0.0;   // psi

    float V_DAC_BITS = 0.0; // bits
    uint16_t V_12bit = round(V_DAC_BITS); // bits
    float V_DAC_REF = 5.0; // V
    float V_REG_REF = 8.0; // V
    float DAC_RESOLUTION = pow(2,12);

    // Variables to read into
    int bitsSensor = 0;
    float V_sensor = 0.0; // V 
    float pressureRead = 0.0; // in kPa
    int pressure = 0; // in kPa -  int equal to int(pressureRead)
    int desiredPress = -15; //kPa
    // Initial value:
    float pressureBaseline = 0.0;// kPa
    float DEPLOY_PRESSURE = 100; // kPa
    float STRUCT_MAX_PRESS = 150; // kPa
    float STRUCT_MIN_PRESS = 50;  // kPa
    float P_MAX = 200.0;//P_MAX_REG/2;  //kPa


    // Latch
    int pressLatch = 0;
    float inflatingPress = DEPLOY_PRESSURE;
    int correctPressCount = 0;
    int correctPressThresh = 200;
    float profileCount = 0;
    float countLimit = 1000.0;

    ////////////////////////////////////////////////////////////////////////////
    // Functions
    void init(int CS_Pin, int P_Pin, int V_Pin, int S_Pin);
    float readStructPressure();
    float convStructPressADC(float v_adc);
    void writePressureReg();
    void setStructValveState();
    void pressureProfile();
    void latchValve();
    void closeValve();
    void openValve();
    void inflateStructure();
    void isolateSupply();
    void deflateStructure();

};
