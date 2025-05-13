#include <SPI.h>

typedef union {
  float fData;
  uint8_t bData[4];
}dataFloat;

class LinAxis{


  ///////////////////////////////////////////////////////////////////////////
  public:
    // Constructor
    LinAxis(); //uint8_t CS_Pin, uint8_t LM_Pin

    // peripheral select pin
    int selectPin;
    // analog pin for related pressure sensor
    int pressPin;
    // Pin for limit switch to set miniumum (for prismatic calibration too)
    int limitPin;



    /////////////////////////////////////////////////////////////////////////
    // Variables for pressure and position data

    dataFloat dataIn;
    dataFloat dataOut;

    int REAL_DIRECTION = 1;

    float angleIn = 0.0;
    float prevAngleIn = 0.0;
    float desiredAngle = 0.0;
    float prevDesiredAngle = 0.0;
    float cumulAngle = 0.0;
    float compDesiredAngle = 0.0;
    float caliDesiredAngle = 0.0;
    float deltaAngle = 0.0;
    float initEncoder = 0.0;
    float angleAtZeroVol = 0.0;
    float homeOffset = 0.0;

    int stepCount = 0;
    int motorState = 3;
    int prevMotorState = 0;

    bool lowFlag = false;
    int startTime;
    int stateCount = 0;
    int STABLE_TIME = 3000; // time in milliseconds reqd for pressure to be at calibration setpoint

    float PERIOD;
    float LEAD = 8.0; // mm per revolution
    float PISTON_RAD = 6.3; //mm
    float PISTON_AREA = PI*pow(PISTON_RAD, 2); // mm^2
    float CALIBRATION_SPEED = 2.0; // mm/s
    float CAL_SPEED_FINE = CALIBRATION_SPEED/2.0; // mm/s
    float distStage; // mm
    float flowRate; // mm^3/s
    float speedStage;
    // float deltaAngle = distStage/LEAD;
    // float deltaAngle = veloStage*PERIOD/LEAD;
    // float deltaAngle = (flow_rate*PERIOD)/(AREA_PISTON*LEAD);
    float MAX_SPEED = 10.0; //mm/s
    float MAX_ANGLE = 1600.0; // 1600 for L_0 = 54 // 1065 for L_0 = 39 mm
    float MIN_ANGLE = -MAX_ANGLE;
    float MAX_ANGLE_P = 3150.0; // 70 mm max motion
    float MIN_ANGLE_P = -MAX_ANGLE_P;




    /////////////////////////////////////////////////////////////////////
    // State setting variables
    int pumpCalibrated = 0;



    ////////////////////////////////////////////////////////////////////
    // SPI message variables

    //Use start and end bytes to set state on pumps/peripherals
    char firstOut = '<';
    char lastOut = '>';

    byte firstByte = 0;
    byte lastByte = 255;

    SPISettings settingsSPI;

    int microDelay = 25;

    const int NUM_CHARS = 5;



    /////////////////////////////////////////////////////////////////////////
    // Pressure reading constants and variables

    //Convert from psi to kPa
    float PSI_TO_KPA = 6.89476;
    // conversion multiplier from Arduino ADC value to voltage in mV
    const float ADC_V = 2.56/pow(2, 10);   //NEEDS TO CHANGE FOR DUE BOARD, CHECK THIS
    // ARDUINO DUE RUNS AT 3.3V AND IS 12 BIT     -  3.3/pow(2, 12); 
    // Constant values for conversion equation
    float V_SUPPLY = 5.0;
    float P_MAX_SENSOR = 150.0; // kPa
    float P_MIN_SENSOR = 0.0; // kPa
    int P_THRESH = 5; // kPa
    int P_FINE = 10; // kPa - When within this threshold, use finer movements

    float MAX_PRESS_KPA = 110.0; // kPa

    // Variables to read into
    int bitsSensor = 0;
    float V_sensor = 0.0; 
    float pressureRead = 0.0; // in kPa
    int pressure = 0; // in kPa -  int equal to int(pressure_read)
    int desiredPress = -20; //kPa
    // Initial value:
    float pressureBaseline = 0.0;



    ////////////////////////////////////////////////////////////////////////////
    // Functions
    void init(int CS_Pin, int P_Pin, float periodSeconds, int L_Pin);
    void pressInitZeroVol();
    void sendRecvFloat_POWER(dataFloat *outData, dataFloat *inData, char powerStatus);
    void sendRecvFloat(dataFloat *outData, dataFloat *inData);
    float readPressure();
    float readPressureADC(float v_adc);
    float convertFlowRateToAngle(float flow_rate);
    float convertSpeedToAngle(float stage_speed);
    float convertAngleToSpeed(float changeAngle);
    bool moveToLimit();
    float convertDistanceToAngle(float distance);


};