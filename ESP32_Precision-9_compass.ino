bool invertRoll = false;
bool invertPitch = true;

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <EEPROM.h>             // include library to read and write from flash memory
const uint8_t EEPROM_SIZE = 1 + (sizeof(float) * 3 * 4) + 4;
bool b_calibrated = false;
long gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0}, calibrated_gyro_bias[3] = {0,0,0}, calibrated_accel_bias[3] = {0,0,0};

enum EEP_ADDR
{
    EEP_CALIB_FLAG = 0x00,
    EEP_ACC_BIAS = 0x01,
    EEP_GYRO_BIAS = 0x0D,
    EEP_MAG_BIAS = 0x19,
    EEP_MAG_SCALE = 0x25,
    EEP_HEADING_OFFSET = 0x31,
    EEP_HEEL_OFFSET = 0x32,
    EEP_TRIM_OFFSET = 0x33,
    EEP_AUTOCALIBRATION = 0x34
};

#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32
#include "N2kMessages.h"
#include <NMEA2000_esp32.h>
#include "runningAngle.h"

#include <SparkFunBME280.h>

#define LED_BUILTIN 2

// List here messages your device will transmit.
const unsigned long TransmitMessagesCompass[] PROGMEM = { 127250L, 127251L, 127257L , 0 }; //Vessel Heading, Rate of Turn, Attitude
const unsigned long TransmitMessagesEnv[] PROGMEM = { 130310L, 130311L, 0 }; //Atmospheric Pressure, Temperature

#define DEV_COMPASS 0 // 60-140

tNMEA2000* nmea2000;
tN2kMsg N2kMsg;
tN2kMsg N2kMsgReply;
int DEVICE_ID = 65;
int SID;

// Running angles
runningAngle AP(runningAngle::DEGREES);
runningAngle AR(runningAngle::DEGREES);
runningAngle AY(runningAngle::DEGREES);
runningAngle AG(runningAngle::DEGREES);

// B&G calibration stop/start
bool calibrationStart = false;
bool calibrationStop = false;
bool calibrationFinished = false;
 
// Deviation variables
float Deviation[128];
bool  calibrationRecording = false;
unsigned long t_dev;
int dev_num = 0;

// Barometer variables
BME280  bmp;
double T, P, P0;
unsigned long recentMaxPressureTimestamp;
float recentMinPressure;
const int numReadings = 250;
double readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

unsigned long t_next;             // Timer

// ----- Arduino pin definitions
byte intPin = 2;                                                    // Interrupt pin (not used) ... 2 and 3 are the Arduinos ext int pins

bool showOutput = false;

// ----- Select a TASK
/*
  Choose a TASK from the following list:
  #define TASK 0    // Calibrate each time at start-up
  #define TASK 1    // Calibrate all sensors once and store in EEPROM
  #define TASK 2    // Calibrate gyro and accel sensors once and store in EEPROM
  #define TASK 5    // No calibration
*/

#define TASK 5

// ----- user offsets and scale-factors
/*
  Each of the following values must be overwritten with the offsets and scale - factors for
  YOUR location otherwise you will have to "tumble" your compass every time you switch it on.
  here are two methods for obtaining this data :

  Method 1 :
  ----------
  Set "#define TASK 1". Upload this change to your Arduino.
  You will be asked to "tumble" your  compass for 30 seconds.
  Results will be stored in EEPROM
  
  Method 2 :
  ----------
  Set "#define TASK 2". Upload this change to your Arduino.
  Only accel & gyro are calibrated.
  Results will be stored in EEPROM  
*/

float
Mag_x_offset = -150.55,
Mag_y_offset = 467.50,
Mag_z_offset = -132.20,
Mag_x_scale = 1.00,
Mag_y_scale = 1.07,
Mag_z_scale = 0.94;

#define True_North false          // change this to "true" for True North                

char InputChar;                   // incoming characters stored here

// ----- software timer
unsigned long Timer1 = 500000L;   // 500mS loop ... used when sending data to to Processing
unsigned long Stop1;              // Timer1 stops when micros() exceeds this value

// ----- MPU-9250 addresses
#define MPU9250_ADDRESS     0x68  // Device address when ADO = 0; Use 0x69 when AD0 = 1
#define AK8963_ADDRESS      0x0C  //  Address of magnetometer

/*
  See also MPU-9250 Register Map and Descriptions, Revision 4.0, and
  RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in the above document;
  The MPU9250 and MPU9150 are virtually identical but the latter has a different register map
  The MPU9255 is similar but WHO_AM_I reports 0x73
*/

// ----- MPU-9250 register map
#define AK8963_WHO_AM_I     0x00  // should return 0x48
#define AK8963_INFO         0x01
#define AK8963_ST1          0x02  // data ready status bit 0
#define AK8963_XOUT_L       0x03  // data
#define AK8963_XOUT_H       0x04
#define AK8963_YOUT_L       0x05
#define AK8963_YOUT_H       0x06
#define AK8963_ZOUT_L       0x07
#define AK8963_ZOUT_H       0x08
#define AK8963_ST2          0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL         0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC         0x0C  // Self test control
#define AK8963_I2CDIS       0x0F  // I2C disable
#define AK8963_ASAX         0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY         0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ         0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO    0x00
#define SELF_TEST_Y_GYRO    0x01
#define SELF_TEST_Z_GYRO    0x02

/*
  #define X_FINE_GAIN         0x03   // [7:0] fine gain
  #define Y_FINE_GAIN         0x04
  #define Z_FINE_GAIN         0x05
  #define XA_OFFSET_H         0x06   // User-defined trim values for accelerometer
  #define XA_OFFSET_L_TC      0x07
  #define YA_OFFSET_H         0x08
  #define YA_OFFSET_L_TC      0x09
  #define ZA_OFFSET_H         0x0A
  #define ZA_OFFSET_L_TC      0x0B
*/

#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E
#define SELF_TEST_Z_ACCEL   0x0F

#define SELF_TEST_A         0x10

#define XG_OFFSET_H         0x13   // User-defined trim values for gyroscope
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define LP_ACCEL_ODR        0x1E
#define WOM_THR             0x1F

#define MOT_DUR             0x20   // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR            0x21   // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR           0x22   // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39  // Check DMP interrupt
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A   // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1          0x6B   // Device defaults to the SLEEP mode
#define PWR_MGMT_2          0x6C
#define DMP_BANK            0x6D   // Activates a specific bank in the DMP
#define DMP_RW_PNT          0x6E   // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG             0x6F   // Register in DMP from which to read or to which to write
#define DMP_REG_1           0x70
#define DMP_REG_2           0x71
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I_MPU9250    0x75   // Should return 0x71; MPU9255 will return 0x73
#define XA_OFFSET_H         0x77
#define XA_OFFSET_L         0x78
#define YA_OFFSET_H         0x7A
#define YA_OFFSET_L         0x7B
#define ZA_OFFSET_H         0x7D
#define ZA_OFFSET_L         0x7E

// ----- Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0,               // 0.6 mG per LSB;
  MFS_16BITS                    // 0.15 mG per LSB
};

enum M_MODE {
  M_8HZ = 0x02,                 // 8 Hz ODR (output data rate) update
  M_100HZ = 0x06                // 100 Hz continuous magnetometer
};

// ----- Specify sensor full scale
byte Gscale = GFS_250DPS;
byte Ascale = AFS_2G;
byte Mscale = MFS_14BITS;                           // Choose either 14-bit or 16-bit magnetometer resolution (AK8963=14-bits)
byte Mmode = 0x02;                                  // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;                             // scale resolutions per LSB for the sensors

short accelCount[3];                                // Stores the 16-bit signed accelerometer sensor output
short gyroCount[3];                                 // Stores the 16-bit signed gyro sensor output
short magCount[3];                                  // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0},
      magBias[3] = {0, 0, 0},
      magScale[3] = {0, 0, 0};         // Factory mag calibration, mag offset , mag scale-factor
float gyroBias[3] = {0, 0, 0},
      accelBias[3] = {0, 0, 0};        // Bias corrections for gyro and accelerometer
short tempCount;                                    // temperature raw count output
float temperature;                                  // Stores the real internal chip temperature in degrees Celsius
float SelfTest[6];                                  // holds results of gyro and accelerometer self test

/*
  There is a tradeoff in the beta parameter between accuracy and response speed.
  In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
  However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
  Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
  By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
  I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
  the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
  In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
*/

// ----- global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);        // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);        // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

// ----- Madgwick free parameters
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;     // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;     // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

// ----- Mahony free parameters
#define Kp 2.0f * 5.0f                            // original Kp proportional feedback parameter in Mahony filter and fusion scheme
//#define Kp 40.0f                                    // Kp proportional feedback parameter in Mahony filter and fusion scheme
#define Ki 0.0f                                     // Ki integral parameter in Mahony filter and fusion scheme

unsigned long delt_t = 0;                           // used to control display output rate
unsigned long count = 0, sumCount = 0;              // used to control display output rate
float pitch, roll, yaw;
float deltat = 0.0f, sum = 0.0f;                    // integration interval for both filter schemes
unsigned long lastUpdate = 0, firstUpdate = 0;      // used to calculate integration interval
unsigned long Now = 0;                              // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz;           // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};              // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};                 // vector to hold integral error for Mahony method

float heading, heading_true, heading_variation = 0,
      heading_offset_rad = 0,
      heel_offset_rad = 0, 
      trim_offset_rad = 0; 
int heading_offset_deg = 0,      
    heel_offset_deg = 0,
    trim_offset_deg = 0;
bool send_heading = true;                           // to do 20 vs 10hz
bool send_heading_true = false;                     // to do 20 vs 10hz
unsigned char compass_autocalibration = 0x00;

// -----------------
// setup()
// -----------------
void setup()
{
  Wire.begin(16,17);
  Wire.setClock(400000);                            // 400 kbit/sec I2C speed
  Serial.begin(115200);
  while (!Serial);                                  // required for Feather M4 Express

  // Setup LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("BMP Start");
  bmp.settings.commInterface = I2C_MODE;
  bmp.settings.I2CAddress = 0x76;
  bmp.settings.runMode = 3;
  bmp.settings.filter = 0;
  bmp.settings.tempOverSample = 1;
  bmp.settings.pressOverSample = 1;
  //bmp.settings.disableHumidity = true;
  if (!bmp.begin()) {
    Serial.println(F("BMP init failed!"));
    while (1);
  }
  else Serial.println(F("BMP init success!"));
  delay(2000);
  
  // ----- Display title
  Serial.println("\nMPU-9250 Quaternion Compass");
  Serial.println("");
  delay(2000);


  // ----- Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  delay(1000);

  if ((c == 0x71) || (c == 0x73)) // MPU9250=0x68; MPU9255=0x73
  {
    Serial.println(F("MPU9250 is online..."));
    Serial.println("");
    delay(2000);
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.println(F("Self test (14% acceptable)"));
    Serial.print(F("x-axis acceleration trim within : ")); Serial.print(SelfTest[0], 1); Serial.println(F("% of factory value"));
    Serial.print(F("y-axis acceleration trim within : ")); Serial.print(SelfTest[1], 1); Serial.println(F("% of factory value"));
    Serial.print(F("z-axis acceleration trim within : ")); Serial.print(SelfTest[2], 1); Serial.println(F("% of factory value"));
    Serial.print(F("x-axis gyration trim within : ")); Serial.print(SelfTest[3], 1); Serial.println(F("% of factory value"));
    Serial.print(F("y-axis gyration trim within : ")); Serial.print(SelfTest[4], 1); Serial.println(F("% of factory value"));
    Serial.print(F("z-axis gyration trim within : ")); Serial.print(SelfTest[5], 1); Serial.println(F("% of factory value"));
    Serial.println("");

    Serial.println("EEPROM start");
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        Serial.println("EEPROM start failed");
    }

    if (MPU9250isCalibrated() && TASK == 5) {
      printCalibration();
      if (loadMPU9250Calibration()) {
        Serial.println("MPU9250 calibration sucessfully read from EEPROM");
        calibrateMPU9250(gyroBias, accelBias); // Load biases in bias registers
        // printBiases2();
      } else {
        Serial.println("MPU9250 calibration reading from EEPROM failed");        
      }
    } else {
      Serial.println("MPU9250 needs calibration.");
      // ----- Level surface message
      Serial.println("Place the compass on a level surface");
      Serial.println("");
      delay(1000);
      calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      if (saveMPU9250Calibration()) {
        Serial.println("MPU9250 calibration saved to EEPROM");
      } else {
        Serial.println("MPU9250 calibration saving to EEPROM failed");        
      }
    }
    printBiases();

    delay(1000);

    initMPU9250();
    Serial.println(F("MPU9250 initialized for active data mode....")); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    Serial.println("");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
    if (d == 0x48)
    {
      // ----- AK8963 found
      Serial.println(F("AK8963 is online ..."));

      // Get magnetometer calibration from AK8963 ROM
      initAK8963(magCalibration);
      Serial.println(F("AK8963 initialized for active data mode ...")); // Initialize device for active mode read of magnetometer
      Serial.println("");

      Serial.println(F("Asahi sensitivity adjustment values"));
      Serial.print(F("ASAX = ")); Serial.println(magCalibration[0], 2);
      Serial.print(F("ASAY = ")); Serial.println(magCalibration[1], 2);
      Serial.print(F("ASAZ = ")); Serial.println(magCalibration[2], 2);
      Serial.println("");
      delay(1000);
    }
    else
    {
      // ----- AK8963 not found
      Serial.print(F("AK8963 WHO_AM_I = ")); Serial.println(c, HEX);
      Serial.println(F("I should be 0x48"));
      Serial.print(F("Could not connect to AK8963: 0x"));
      Serial.println(d, HEX);
      Serial.println("");
      while (1) ; // Loop forever if communication doesn't happen
    }
  }
  else
  {
    // ----- MPU9250 not found
    Serial.print(F("MPU9250 WHO_AM_I = ")); Serial.println(c, HEX);
    Serial.println(F("I should be 0x71 or 0x73"));
    Serial.print(F("Could not connect to MPU9250: 0x"));
    Serial.println(c, HEX);
    Serial.println("");
    while (1) ; // Loop forever if communication doesn't happen
  }

  // --------------------------
  // TASK 0,1 tasks & messages
  // --------------------------
  if ((TASK == 0) || (TASK == 1))
  {
    // ----- Calculate magnetometer offsets & scale-factors
    Serial.println(F("Magnetometer calibration ..."));
    Serial.println(F("Tumble/wave device for 30 seconds in a figure 8"));
    delay(2000);
    magCalMPU9250(magBias, magScale);

    Serial.println(F("Stop tumbling"));
    Serial.println("");
    delay(4000);
  }

  if (TASK == 1) {
    // ----- Message
    Serial.println(F("------------------------------------------"));
    Serial.println(F("The following offsets were found and will "));
    Serial.println(F("be stored in EEPROM"));
    Serial.println(F("------------------------------------------"));
    Serial.println(F(""));
    Serial.println(F("float"));
    Serial.print(F("Mag_x_offset = "));
    Serial.print(magBias[0]);
    Serial.println(",");
    Serial.print(F("Mag_y_offset = "));
    Serial.print(magBias[1]);
    Serial.println(",");
    Serial.print(F("Mag_z_offset = "));
    Serial.print(magBias[2]);
    Serial.println(",");
    Serial.print(F("Mag_x_scale = "));
    Serial.print(magScale[0]);
    Serial.println(",");
    Serial.print(F("Mag_y_scale = "));
    Serial.print(magScale[1]);
    Serial.println(",");
    Serial.print(F("Mag_z_scale = "));
    Serial.print(magScale[2]);
    Serial.println(F(";"));

    // ----- Halt program
    while (true);
  }

  // Initialise canbus
  Serial.println("Set up NMEA2000 device");
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);
  
  // nmea2000->SetDeviceCount(1);
  // nmea2000->SetInstallationDescription1("");
  // nmea2000->SetInstallationDescription2("");
  nmea2000->SetProductInformation("107018103", // Manufacturer's Model serial code
                                 13233, // Manufacturer's product code
                                 "Precision-9 Compass",  // Manufacturer's Model ID
                                 "2.0.3-0",  // Manufacturer's Software version code
                                 "", // Manufacturer's Model version
                                 1,  // load equivalency *50ma
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_COMPASS
                                );
 
  // Set device information
  nmea2000->SetDeviceInformation(1048678, // Unique number. Use e.g. Serial number.
                                140, // Device function=Temperature See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                60, // Device class=Sensor Communication Interface. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                275, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4,
                                DEV_COMPASS
                               );
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, DEVICE_ID);
  nmea2000->EnableForward(false);
  nmea2000->SetForwardStream(&Serial);
  nmea2000->SetForwardType(tNMEA2000::fwdt_Text);
  nmea2000->ExtendTransmitMessages(TransmitMessagesCompass, DEV_COMPASS);
  nmea2000->SetN2kCANMsgBufSize(20);
  nmea2000->SetMsgHandler(HandleNMEA2000Msg);
  nmea2000->Open();

  // Running angles weights
  AP.setWeight(0.3);
  AR.setWeight(0.3);
  AY.setWeight(0.3);
  AG.setWeight(0.3);

  t_next = 0;
}

// ----------
// loop()
// ----------
void loop()
{
  refresh_data();                              // This must be done each time through the loop
  calc_quaternion();                           // This must be done each time through the loop
  calc_pitchRollYaw();

  if (t_next + 500 <= millis())                  // Set timer for 50ms
  {
    t_next = millis();                        
    calc_heading();
    printValues();
    

    if (send_heading) {
      SetN2kPGN127250(N2kMsg, SID, heading * DEG_TO_RAD, N2kDoubleNA, N2kDoubleNA, N2khr_magnetic);
      nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
      if (send_heading_true) {
        SetN2kPGN127250(N2kMsg, SID, heading_true * DEG_TO_RAD, N2kDoubleNA, N2kDoubleNA, N2khr_true);
        nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
      }
      SetN2kRateOfTurn(N2kMsg, SID, gz * 0.017448352875489); // radians
      nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
    }
    send_heading = !send_heading;             // Toggle to do 10hz

    SetN2kAttitude(N2kMsg, SID, N2kDoubleNA, pitch * DEG_TO_RAD, roll * DEG_TO_RAD);
    nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
    SID++; if (SID > 253) {
      SID = 1;
    }
      // Check for commands
    if (Serial.available()) {
      InputChar = Serial.read();
      if (InputChar == 'o') {
        showOutput = !showOutput;
      }
      if (InputChar == 'd') {
        Serial.println("Starting deviation calibration. Start turning at a steady 3 degrees/second and press 's' to start.");
        calibrationStart = true;
      }
      if (InputChar == 's' &&  calibrationRecording == false) {
        Serial.println("Calibration recording started. Keep turning at a steady 3 degrees/second for 390 degrees.");
         calibrationRecording = true;
      }
    }
    if (calibrationStart == true && calibrationStop == false) {
        Serial.println("Calibration recording started. Keep turning at a steady 3 degrees/second for 390 degrees.");
        calibrationRecording = true;
    }
    if ( calibrationRecording) {
      recordDeviation();
      t_dev = millis();
    }
    if (calibrationStop == true && calibrationStart == true) {
      calibrationStart = false;
      calibrationStop = false;
       calibrationRecording = false;
      Serial.println("Calibration cancelled.");
    }
    if (calibrationFinished) {
      Serial.println("Calibration finished.");
       calibrationRecording = false;
      calibrationStart = false;
      showDeviationTable();
    }
    ToggleLed();                              // Toggle led
  }
  nmea2000->ParseMessages();

}

void printBiases() {
  Serial.println(F("MPU9250 accelerometer bias"));
  Serial.print(F("x = ")); Serial.println((int)(1000 * accelBias[0]));
  Serial.print(F("y = ")); Serial.println((int)(1000 * accelBias[1]));
  Serial.print(F("z = ")); Serial.print((int)(1000 * accelBias[2]));
  Serial.println(F(" mg"));
  Serial.println("");
  Serial.println(F("MPU9250 gyro bias"));
  Serial.print(F("x = ")); Serial.println(gyroBias[0], 1);
  Serial.print(F("y = ")); Serial.println(gyroBias[1], 1);
  Serial.print(F("z = ")); Serial.print(gyroBias[2], 1);
  Serial.println(F(" o/s"));
  Serial.println("");
  Serial.println(F("MPU9250 magnetometer bias"));
  Serial.print(F("x = ")); Serial.println(Mag_x_offset, 1);
  Serial.print(F("y = ")); Serial.println(Mag_y_offset, 1);
  Serial.print(F("z = ")); Serial.print(Mag_z_offset, 1);
  Serial.println("\n");
}

void printBiases2() {
  Serial.println(F("MPU9250 accelerometer bias"));
  Serial.print(F("x = ")); Serial.println((int)(accel_bias[0]));
  Serial.print(F("y = ")); Serial.println((int)(accel_bias[1]));
  Serial.print(F("z = ")); Serial.print((int)(accel_bias[2]));
  Serial.println("");
  Serial.println(F("MPU9250 gyro bias"));
  Serial.print(F("x = ")); Serial.println(gyro_bias[0], 1);
  Serial.print(F("y = ")); Serial.println(gyro_bias[1], 1);
  Serial.print(F("z = ")); Serial.print(gyro_bias[2], 1);
  Serial.println("");
  Serial.println(F("MPU9250 magnetometer bias"));
  Serial.print(F("x = ")); Serial.println(Mag_x_offset, 1);
  Serial.print(F("y = ")); Serial.println(Mag_y_offset, 1);
  Serial.print(F("z = ")); Serial.print(Mag_z_offset, 1);
  Serial.println("\n");
}

void recordDeviation() {
  // Record compass headings to create deviation table
  // With 3 degrees/second we can record the heading every 0.5 seconds in a table of 120 measurements
  if (millis() > t_dev) {
    t_dev += 500; // Next measurement to happen 500ms from now
    if (calibrationRecording) {    
      Deviation[dev_num] = heading;
      Serial.printf("Rate of turn: % 5.2f dps   Deviation[%3d] = %05.2f\n", gz, dev_num, Deviation[dev_num]);
      dev_num++;
      if (dev_num > 120) {
        dev_num = 0;
      }
    } else {
      Serial.printf("Rate of turn: % 3.0f dps   Press 's' to start recording.\n", gz);  
    }
  }
}

void showDeviationTable() {
  Serial.printf("num heading\n");  
  for (dev_num = 0; dev_num <= 120; dev_num++) {
    Serial.printf("%3d, %05.2f\n", dev_num, Deviation[dev_num]);
  }
}

int num_n2k_messages = 0;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  // N2kMsg.Print(&Serial);
  // Serial.printf("PGN: %u\n", (unsigned int)N2kMsg.PGN);

  switch (N2kMsg.PGN) {
    case 130850L:
      if (ParseN2kPGN130850(N2kMsg)) {
        Serial.printf("calibrationStart: %s  calibrationStop: %s\n", String(calibrationStart), String(calibrationStop));
      }
      break;
    case 127258L:
      unsigned char SID;
      tN2kMagneticVariation source;
      uint16_t DaysSince1970;
      double variation;
      ParseN2kMagneticVariation(N2kMsg, SID, source, DaysSince1970, variation);
      heading_variation = (float)variation * RAD_TO_DEG;
      send_heading_true = true;
      break;
    case 130845L:
      uint16_t Key, Command, Value;
      if (ParseN2kPGN130845(N2kMsg, Key, Command, Value)) {
        // Serial.printf("Key: %d Command: %d Value: %d\n", Key, Command, Value);
      }
      break;
  }
  num_n2k_messages++;
  // Serial.printf("Message count: %d\n", num_n2k_messages);
  ToggleLed();
}

bool ParseN2kPGN130850(const tN2kMsg &N2kMsg) {
  Serial.println("Entering ParseN2kPGN130850");
  if (N2kMsg.PGN!=130850L) return false;
  int Index=2;
  unsigned char Command1=N2kMsg.GetByte(Index);
  unsigned char Command2=N2kMsg.GetByte(Index);
  unsigned char Command3=N2kMsg.GetByte(Index);
  unsigned char Command4=N2kMsg.GetByte(Index);
  unsigned char CalibrationStopStart=N2kMsg.GetByte(Index);
  //Serial.printf("Command1: %u  Command4: %u  CalibrationStopStart: %u  ", (unsigned int)Command1, (unsigned int)Command4, (unsigned int)CalibrationStopStart);
  if (Command1 == DEVICE_ID && Command4 == 18 && CalibrationStopStart == 0) {
    calibrationStart = true;
    // Send ack
    Send130851Ack(0); 
    return true;
  } else {
    if (Command1 == DEVICE_ID && Command4 == 18 && CalibrationStopStart == 1) {
      calibrationStop = true;
      // Send ack
      Send130851Ack(1);
      return true;
    }
  }
  return false;
}

bool ParseN2kPGN130845(const tN2kMsg &N2kMsg, uint16_t &Key, uint16_t &Command, uint16_t &Value) {
  // Serial.println("Entering ParseN2kPGN130845");
  if (N2kMsg.PGN != 130845L) return false;
  int Index=2;
  unsigned char Target = N2kMsg.GetByte(Index);
  if (Target == 65) {
    N2kMsg.Print(&Serial);
    tN2kMsg N2kMsgReply;
    unsigned char source = N2kMsg.Source;
    Index = 6;
    Key = N2kMsg.Get2ByteUInt(Index);
    Command = N2kMsg.Get2ByteUInt(Index);    
    if (Command == 0x0000) {
      // Get
      if (SetN2kPGN130845(N2kMsgReply, DEVICE_ID, Key, 2))   // 2 = Ack
        nmea2000->SendMsg(N2kMsgReply, DEV_COMPASS);
    }
    if (Command == 0x0100) {
      // Set
      switch (Key) {
        case 0x0000:  // Heading offset
          heading_offset_rad = N2kMsg.Get2ByteUDouble(0.0001, Index);
          heading_offset_deg = (int)round(heading_offset_rad * RAD_TO_DEG);
          Serial.printf("heading_offset_rad: %f  heading_offset_deg: %d\n", heading_offset_rad, heading_offset_deg);
          EEPROM.writeByte(EEP_HEADING_OFFSET, (unsigned char)heading_offset_deg);
          EEPROM.commit();
          break;
        case 0x0039:  // Heel offset
          heel_offset_rad = N2kMsg.Get2ByteUDouble(0.0001, Index);
          heel_offset_deg = (int)round(heel_offset_rad * RAD_TO_DEG);
          Serial.printf("heel_offset_rad: %f  heel_offset_deg: %d\n", heel_offset_rad, heel_offset_deg);
          EEPROM.writeByte(EEP_HEEL_OFFSET, (unsigned char)heel_offset_deg);
          EEPROM.commit();
          break;
        case 0x0031:  // Trim offset
          trim_offset_rad = N2kMsg.Get2ByteUDouble(0.0001, Index);
          trim_offset_deg = (int)round(trim_offset_rad * RAD_TO_DEG);
          Serial.printf("trim_offset_rad: %f  trim_offset_deg: %d\n", trim_offset_rad, trim_offset_deg);
          EEPROM.writeByte(EEP_TRIM_OFFSET, (unsigned char)trim_offset_deg);
          EEPROM.commit();
          break;
        case 0xd200:  // Auto calibration (01=on, 02=auto locked)
          compass_autocalibration = N2kMsg.GetByte(Index);
          if (SetN2kPGN130845(N2kMsgReply, DEVICE_ID, 0xd400, 2))
            nmea2000->SendMsg(N2kMsgReply, DEV_COMPASS);
          EEPROM.writeByte(EEP_AUTOCALIBRATION, (unsigned char)compass_autocalibration);
          EEPROM.commit();
          break;
        case 0xd300:  // Warning
          break;
        case 0xd400:  // Status
          break;      }
        }
    if (SetN2kPGN130845(N2kMsgReply, DEVICE_ID, Key, 2))
      nmea2000->SendMsg(N2kMsgReply, DEV_COMPASS);      
    return true;
  } else {
    // Serial.printf("Skipping this one...\n");
    return false;
  }
  return false;
}

void Send130851Ack(int StopStart) {
  tN2kMsg N2kMsg;
  SetN2k130851Ack(N2kMsg, DEVICE_ID, 18, (unsigned char)StopStart);
  nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
}

bool SetN2kPGN130845(tN2kMsg &N2kMsg, unsigned char DEVICE_ID, uint16_t Key, uint16_t Command) {
  N2kMsg.SetPGN(130845L);
  N2kMsg.Priority=2;
  N2kMsg.AddByte(0x41); // Reserved
  N2kMsg.AddByte(0x9f); // Reserved
  N2kMsg.AddByte((unsigned char)DEVICE_ID);
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.Add2ByteUInt((uint16_t)Key);
  if (Command == 2) {
    N2kMsg.Add2ByteUInt(0x0200);
  }
  switch (Key) {
    case 0x0000:  // Heading offset
      N2kMsg.Add2ByteDouble(heading_offset_rad, 0.0001);
      N2kMsg.AddByte(0xff); // Reserved
     break;
    case 0x0039:  // Heel offset
      N2kMsg.Add2ByteDouble(heel_offset_rad, 0.0001);
      break;
    case 0x0031:  // Trim offset
      N2kMsg.Add2ByteDouble(trim_offset_rad, 0.0001);
      break;
    case 0xd200:  // Auto calibration (01=on, 02=auto locked)
      N2kMsg.AddByte(compass_autocalibration); // Reserved
      N2kMsg.AddByte(0xff); // Reserved
      break;
    case 0xd300:  // Warning
      N2kMsg.AddByte(0x00); // Reserved  // 00=No warning, 01=First calibration in progress, 02=Parameters in use are not valid
      N2kMsg.AddByte(0xff); // Reserved
      break;
    case 0xd400:  // Status
      N2kMsg.AddByte(0x00); // Reserved  // 00=Is calibrated, 01=Not calibrated, 02=Is calibrating
      N2kMsg.AddByte(0xff); // Reserved
      break;
    default:
      return false;
  }
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.AddByte(0xff); // Reserved
  return true;
}

void SetN2kPGN130851(tN2kMsg &N2kMsg, int DEVICE_ID, unsigned char Command, unsigned char CalibrationStopStart) {
    N2kMsg.SetPGN(130851L);
    N2kMsg.Priority=2;
    N2kMsg.AddByte(0x41); // Reserved
    N2kMsg.AddByte(0x9f); // Reserved
    N2kMsg.AddByte((unsigned char)DEVICE_ID);
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte((unsigned char)Command);
    N2kMsg.AddByte((unsigned char)CalibrationStopStart);
    N2kMsg.AddByte(0x00); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
}
                     
void SetN2k130851Ack(tN2kMsg &N2kMsg, int DEVICE_ID, unsigned char Command, unsigned char CalibrationStopStart) {
  SetN2kPGN130851(N2kMsg, DEVICE_ID, Command, CalibrationStopStart);
}

void SetN2k130845(tN2kMsg &N2kMsg, int DEVICE_ID, uint16_t Key, uint16_t Command) {
  SetN2kPGN130845(N2kMsg, DEVICE_ID, Key, Command);
}


// -------------------
// getMres()
// -------------------
/* Get magnetometer resolution */
void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}

// -------------------
// getGres()
// -------------------
/* Get gyro resolution */
void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

// -------------------
// getAres()
// -------------------
/* Get accelerometer resolution */
void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}

// -------------------
// readAccelData()
// -------------------
/* Read accelerometer registers */
void readAccelData(short * destination)
{
  byte rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((short)rawData[0] << 8) | rawData[1] ;   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((short)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((short)rawData[4] << 8) | rawData[5] ;
}

// -------------------
// readGyroData()
// -------------------
/* Read gyro registers */
void readGyroData(short * destination)
{
  byte rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((short)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((short)rawData[4] << 8) | rawData[5] ;
}

// -------------------
// readMagData()
// -------------------
/* Read magnetometer registers */
void readMagData(short * destination)
{
  byte rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    byte c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((short)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((short)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((short)rawData[5] << 8) | rawData[4] ;
    }
  }
}

// -------------------
// readTempData()
// -------------------
/* Read temperature */
short readTempData()
{
  byte rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((short)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

// -------------------
// initAK8963()
// -------------------
/* Initialize the AK8963 magnetometer */
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  byte rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

// -------------------
// initMPU9250()
// -------------------
/* Initialize the MPU9250|MPU6050 chipset */
void initMPU9250()
{
  // -----wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // -----get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // ----- Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // -----Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  byte c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // ----- Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // ----- Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // ----- Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}

// -------------------
// calibrateMPU9250()
// -------------------
/*
  Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
  of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
*/
void calibrateMPU9250(float * dest1, float * dest2)
{
  byte data[12]; // data array to hold accelerometer and gyro x, y, z, data
  unsigned short ii, packet_count, fifo_count;

  // ----- reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // ----- get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // ----- Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // ----- Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  unsigned short  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  unsigned short  accelsensitivity = 16384;  // = 16384 LSB/g

  // ----- Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // ----- At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((unsigned short)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    short accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (short) (((short)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (short) (((short)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (short) (((short)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (short) (((short)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (short) (((short)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (short) (((short)data[10] << 8) | data[11]) ;

    accel_bias[0] += (long) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (long) accel_temp[1];
    accel_bias[2] += (long) accel_temp[2];
    gyro_bias[0]  += (long) gyro_temp[0];
    gyro_bias[1]  += (long) gyro_temp[1];
    gyro_bias[2]  += (long) gyro_temp[2];
  }
  accel_bias[0] /= (long) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (long) packet_count;
  accel_bias[2] /= (long) packet_count;
  gyro_bias[0]  /= (long) packet_count;
  gyro_bias[1]  /= (long) packet_count;
  gyro_bias[2]  /= (long) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (long) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (long) accelsensitivity;
  }
  if (TASK == 5) {
    accel_bias[0] = calibrated_accel_bias[0];
    accel_bias[1] = calibrated_accel_bias[1];
    accel_bias[2] = calibrated_accel_bias[2];
    gyro_bias[0] = calibrated_gyro_bias[0];
    gyro_bias[1] = calibrated_gyro_bias[1];
    gyro_bias[2] = calibrated_gyro_bias[2];
  }
  writeBiastoMPU9250(dest1, dest2);
}


void writeBiastoMPU9250(float * dest3, float * dest4) {
  Serial.println("Writing biases to MPU9250");
  byte data[6]; // data array to hold accelerometer and gyro x, y, z, data
  unsigned short  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  unsigned short  accelsensitivity = 16384;  // = 16384 LSB/g

  printBiases2();
  // ----- Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // ----- Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // ----- Output scaled gyro biases for display in the main program
  dest3[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest3[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest3[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  long accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (long) (((short)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (long) (((short)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (long) (((short)data[0] << 8) | data[1]);

  unsigned long mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  byte mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (int ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // ----- Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8);     // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFE;
  data[1] = data[1] | mask_bit[0];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFE;
  data[3] = data[3] | mask_bit[1];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFE;
  data[5] = data[5] | mask_bit[2];              // preserve temperature compensation bit when writing back to accelerometer bias registers
  // see https://github.com/kriswiner/MPU9250/issues/215

  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // ----- Output scaled accelerometer biases for display in the main program
  dest4[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest4[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest4[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

// -----------------
// magCalMPU9250()
// -----------------
/*
  Function which accumulates magnetometer data after device initialization.
  It calculates the bias and scale in the x, y, and z axes.
*/
void magCalMPU9250(float * bias_dest, float * scale_dest)
{
  unsigned short ii = 0, sample_count = 0;
  short mag_max[3]  = { -32768, -32768, -32768},
        mag_min[3]  = {32767, 32767, 32767},
        mag_temp[3] = {0, 0, 0};
  long mag_bias[3] = {0, 0, 0};
  float mag_chord[3] = {0, 0, 0};
  float avg_chord;

  // ----- Make sure resolution has been calculated
  getMres();

  // ----- Tumble compass for 30 seconds
  /*
    At 8 Hz ODR (output data rate), new mag data is available every 125 ms
    At 100 Hz ODR, new mag data is available every 10 ms
  */

  if (Mmode == M_8HZ) sample_count = 240;         // 240*125mS=30 seconds
  if (Mmode == M_100HZ) sample_count = 3000;      // 3000*10mS=30 seconds

  for (ii = 0; ii < sample_count; ii++)
  {
    readMagData(mag_temp);  // Read the raw mag data

    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }

    if (Mmode == M_8HZ) delay(135);               // At 8 Hz ODR, new mag data is available every 125 ms
    if (Mmode == M_100HZ) delay(12);              // At 100 Hz ODR, new mag data is available every 10 ms
  }

  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // ----- Get hard iron correction
  /* long data-type  */
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;                       // data-type: long
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

  // ----- Save mag biases in G for main program
  /* float data-type  */
  bias_dest[0] = (float)mag_bias[0] * magCalibration[0] * mRes;       // rawMagX * ASAX * 0.6
  bias_dest[1] = (float)mag_bias[1] * magCalibration[1] * mRes;       // rawMagY * ASAY * 0.6
  bias_dest[2] = (float)mag_bias[2] * magCalibration[2] * mRes;       // rawMagZ * ASAZ * 0.6

  // ----- Get soft iron correction estimate
  /* float data-type */
  mag_chord[0]  = ((float)(mag_max[0] - mag_min[0])) / 2.0;
  mag_chord[1]  = ((float)(mag_max[1] - mag_min[1])) / 2.0;
  mag_chord[2]  = ((float)(mag_max[2] - mag_min[2])) / 2.0;
  avg_chord = (mag_chord[0] + mag_chord[1] + mag_chord[2]) / 3.0;

  // ----- calculate scale-factors
  /* Destination data-type is float */
  scale_dest[0] = avg_chord / mag_chord[0];
  scale_dest[1] = avg_chord / mag_chord[1];
  scale_dest[2] = avg_chord / mag_chord[2];

  //  Serial.print("bias_dest[0] "); Serial.println(bias_dest[0]);
  //  Serial.print("bias_dest[1] "); Serial.println(bias_dest[1]);
  //  Serial.print("bias_dest[2] "); Serial.println(bias_dest[2]);
  //  Serial.print("");
  //
  //  Serial.print("mag_chord[0]] "); Serial.println(mag_chord[0]);
  //  Serial.print("mag_chord[1]] "); Serial.println(mag_chord[1]);
  //  Serial.print("mag_chord[2]] "); Serial.println(mag_chord[2]);
  //  Serial.print("avg_chord] "); Serial.println(avg_chord);
  //  Serial.print("");
  //
  //  Serial.print("scale_dest[0] "); Serial.println(scale_dest[0]);
  //  Serial.print("scale_dest[1] "); Serial.println(scale_dest[1]);
  //  Serial.print("scale_dest[2] "); Serial.println(scale_dest[2]);
}

// ------------------
// MPU9250SelfTest()
// ------------------
/* Accelerometer and gyroscope self test; check calibration wrt factory settings */
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  byte rawData[6] = {0, 0, 0, 0, 0, 0};
  byte selfTest[6];
  long gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  byte FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (short)(((short)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (short)(((short)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (short)(((short)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (short)(((short)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (short)(((short)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (short)(((short)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (short)(((short)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (short)(((short)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (short)(((short)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (short)(((short)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (short)(((short)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (short)(((short)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.; // Report percent differences
    destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
  }
}

// --------------
// writeByte()
// --------------
void writeByte(byte address, byte subAddress, byte data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

byte readByte(byte address, byte subAddress)
{
  byte data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (byte) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

// --------------
// readBytes()
// --------------
void readBytes(byte address, byte subAddress, byte count, byte * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  byte i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}

// ---------------------------
// MadgwickQuaternionUpdate()
// ---------------------------
/*
  Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
  (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
  which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
  device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
  The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
  but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
*/
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // ----- Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // ----- Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // ----- Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // ----- Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // ----- Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // ----- Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // ----- Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

// --------------------------
// MahonyQuaternionUpdate()
// --------------------------
/*
  Similar to Madgwick scheme but uses proportional and integral filtering
  on the error between estimated reference vectors and measured ones.
*/
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // ----- Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // ----- Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // ----- Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // ----- Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // ----- Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  //  ----- Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // ----- Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // ----- Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // ----- Normalise quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

// ------------------------
// refresh_data()
// ------------------------
/* Get current MPU-9250 register values */
void refresh_data()
{
  // ----- If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    readAccelData(accelCount);                          // Read the accelerometer registers
    getAres();                                          // Get accelerometer resolution

    // ----- Accelerometer calculations
    ax = (float)accelCount[0] * aRes;                   // - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes;                   // - accelBias[1];
    az = (float)accelCount[2] * aRes;                   // - accelBias[2];
     
    // ----- Gyro calculations
    readGyroData(gyroCount);                            // Read the gyro registers
    getGres();                                          // Get gyro resolution

    // ----- Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    // ----- Magnetometer calculations
    readMagData(magCount);                              // Read the magnetometer x|y| registers
    getMres();                                          // Get magnetometer resolution

    //    // ----- Kris Winer hard-iron offsets
    //    magBias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    //    magBias[1] = +120.;  // User environmental x-axis correction in milliGauss
    //    magBias[2] = +125.;  // User environmental x-axis correction in milliGauss

    // ----- Copy hard-iron offsets
    magBias[0] = Mag_x_offset;                          // Get hard-iron offsets (from compass_cal)
    magBias[1] = Mag_y_offset;
    magBias[2] = Mag_z_offset;

    // ----- Copy the soft-iron scalefactors
    magScale[0] = Mag_x_scale;
    magScale[1] = Mag_y_scale;
    magScale[2] = Mag_z_scale;

    //    // ----- Calculate the magnetometer values in milliGauss
    //    /* Include factory calibration per data sheet and user environmental corrections */
    //    mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0];    // get actual magnetometer value, this depends on scale being set
    //    my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
    //    mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];

    // ----- Calculate the magnetometer values in milliGauss
    /* The above formula is not using the soft-iron scale factors */
    mx = ((float)magCount[0] * mRes * magCalibration[0] - magBias[0]) * magScale[0];    // (rawMagX*ASAX*0.6 - magOffsetX)*scalefactor
    my = ((float)magCount[1] * mRes * magCalibration[1] - magBias[1]) * magScale[1];    // (rawMagY*ASAY*0.6 - magOffsetY)*scalefactor
    mz = ((float)magCount[2] * mRes * magCalibration[2] - magBias[2]) * magScale[2];    // (rawMagZ*ASAZ*0.6 - magOffsetZ)*scalefactor
  }
  // Get Temperature & Pressure
  T = bmp.readTempC();
  P = bmp.readFloatPressure() / 100;
}

// ------------------------
// calc_quaternion()
// ------------------------
/* Send current MPU-9250 register values to Mahony quaternion filter */
void calc_quaternion()
{
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  /*
    Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
    For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    This is ok by aircraft orientation standards!
    Pass gyro rate as rad/s
  */

  // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

  // ----- Apply NEU (north east up)signs when parsing values
  MahonyQuaternionUpdate(
    ax,                -ay,                  az,
    gx * DEG_TO_RAD,   -gy * DEG_TO_RAD,     gz * DEG_TO_RAD,
    my,                -mx,                 -mz);
}

void calc_pitchRollYaw()
{
  /*
    Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    In this coordinate system, the positive z-axis is down toward Earth.
    Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    applied in the correct order which for this configuration is yaw, pitch, and then roll.
    For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  */

  pitch = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);

    
  // ----- convert to degrees
  pitch *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;

  // Apply offset for pitch and roll
  pitch += trim_offset_deg;
  roll += heel_offset_deg;
}

void calc_heading () {
  // ----- calculate the heading
  /*
     The yaw and compass heading (after the next two lines) track each other 100%
  */

  /*
  // Use running angles to smoothen
  gz = AG.add(gz);
  pitch = AP.add(pitch);
  roll = AR.add(roll);
  yaw = AY.add(yaw);
  */

  heading = yaw;
  if (heading < 0) heading += 360.0;                        // Yaw goes negative between 180 amd 360 degrees
  // if (True_North == true) heading += Declination;        // Calculate True North
  if (heading < 0) heading += 360.0;                        // Allow for under|overflow
  if (heading >= 360) heading -= 360.0;
  if (send_heading_true) {
    heading_true = heading + heading_variation + heading_offset_deg;
    if (heading_true < 0) heading_true += 360.0;                        // Allow for under|overflow
    if (heading_true >= 360) heading_true -= 360.0;
  }
  if (invertPitch) {
    pitch = pitch * -1;
  }
  if (invertRoll) {
    roll = roll * -1;
  }
}

void printValues() {
  // ----- send the results to the Serial Monitor
  if (showOutput) {  
    // ----- Print gyro values
    Serial.print(" Gyro:");
    Serial.printf(" % 7.5f dps", gz);
    Serial.printf(" Pitch % 6.1f", pitch);
    Serial.printf(" Roll % 6.1f", roll);
    Serial.printf(" Yaw % 6.1f", yaw);
    Serial.printf(" Heading % 5.1f", heading);
    if (send_heading_true) {
      Serial.printf(" Variation % 3.1f", heading_variation);
      Serial.printf(" HeadingT % 5.1f\n", heading_true);      
    } else {
      Serial.printf("\n");
    }
  } 
}

bool saveMPU9250Calibration() {
    printBiases2();
    EEPROM.writeByte(EEP_CALIB_FLAG, 1);
    EEPROM.writeLong(EEP_ACC_BIAS + 0, accel_bias[0]);
    EEPROM.writeLong(EEP_ACC_BIAS + 4, accel_bias[1]);
    EEPROM.writeLong(EEP_ACC_BIAS + 8, accel_bias[2]);
    EEPROM.writeLong(EEP_GYRO_BIAS + 0, gyro_bias[0]);
    EEPROM.writeLong(EEP_GYRO_BIAS + 4, gyro_bias[1]);
    EEPROM.writeLong(EEP_GYRO_BIAS + 8, gyro_bias[2]);
    EEPROM.writeFloat(EEP_MAG_BIAS + 0, Mag_x_offset);
    EEPROM.writeFloat(EEP_MAG_BIAS + 4, Mag_y_offset);
    EEPROM.writeFloat(EEP_MAG_BIAS + 8, Mag_z_offset);
    EEPROM.writeFloat(EEP_MAG_SCALE + 0, Mag_x_scale);
    EEPROM.writeFloat(EEP_MAG_SCALE + 4, Mag_y_scale);
    EEPROM.writeFloat(EEP_MAG_SCALE + 8, Mag_z_scale);
    EEPROM.commit();
    return true;
}

bool loadMPU9250Calibration() {
    if (EEPROM.readByte(EEP_CALIB_FLAG) == 0x01)
    {
        Serial.println("calibrated? : YES");
        Serial.println("load calibrated values");
        calibrated_accel_bias[0] = EEPROM.readLong(EEP_ACC_BIAS + 0);
        calibrated_accel_bias[1] = EEPROM.readLong(EEP_ACC_BIAS + 4);
        calibrated_accel_bias[2] = EEPROM.readLong(EEP_ACC_BIAS + 8);
        calibrated_gyro_bias[0]  = EEPROM.readLong(EEP_GYRO_BIAS + 0);
        calibrated_gyro_bias[1]  = EEPROM.readLong(EEP_GYRO_BIAS + 4);
        calibrated_gyro_bias[2]  = EEPROM.readLong(EEP_GYRO_BIAS + 8);
        Mag_x_offset = EEPROM.readFloat(EEP_MAG_BIAS + 0);
        Mag_y_offset = EEPROM.readFloat(EEP_MAG_BIAS + 4);
        Mag_z_offset = EEPROM.readFloat(EEP_MAG_BIAS + 8);
        Mag_x_scale  = EEPROM.readFloat(EEP_MAG_SCALE + 0);
        Mag_y_scale  = EEPROM.readFloat(EEP_MAG_SCALE + 4);
        Mag_z_scale  = EEPROM.readFloat(EEP_MAG_SCALE + 8);
        heading_offset_deg = (unsigned char)EEPROM.readByte(EEP_HEADING_OFFSET);
        if (heading_offset_deg == 255) heading_offset_deg = 0;
        heading_offset_rad = (float)heading_offset_deg * DEG_TO_RAD;
        Serial.printf("heading_offset_rad: %f  heading_offset_deg: %d\n", heading_offset_rad, heading_offset_deg);
        heel_offset_deg = (unsigned char)EEPROM.readByte(EEP_HEEL_OFFSET);
        if (heel_offset_deg == 255) heel_offset_deg = 0;
        heel_offset_rad = (float)heel_offset_deg * DEG_TO_RAD;
        Serial.printf("heel_offset_rad: %f  heel_offset_deg: %d\n", heel_offset_rad, heel_offset_deg);
        trim_offset_deg = (unsigned char)EEPROM.readByte(EEP_TRIM_OFFSET);
        if (trim_offset_deg == 255) trim_offset_deg = 0;
        trim_offset_rad = (float)trim_offset_deg * DEG_TO_RAD;
        Serial.printf("trim_offset_rad: %f  trim_offset_deg: %d\n", trim_offset_rad, trim_offset_deg);
        compass_autocalibration = EEPROM.readByte(EEP_AUTOCALIBRATION);        
        if (compass_autocalibration == 255) compass_autocalibration = 0;
        writeBiastoMPU9250(gyroBias, accelBias);
        return true;
    }
    else
    {
        Serial.println("calibrated? : NO");
        return false;
    }
}

void clearCalibration()
{
    for (size_t i = 0; i < EEPROM_SIZE; ++i) {
      EEPROM.writeByte(i, 0xFF);
    }
    EEPROM.commit();
}

bool  MPU9250isCalibrated()
{
    return (EEPROM.readByte(EEP_CALIB_FLAG) == 0x01);
}

void printCalibration()
{
    Serial.println("< calibration parameters >");
    Serial.print("calibrated? : ");
    Serial.println(EEPROM.readByte(EEP_CALIB_FLAG) ? "YES" : "NO");
    Serial.print("acc bias x  : ");
    Serial.println(EEPROM.readLong(EEP_ACC_BIAS + 0));
    Serial.print("acc bias y  : ");
    Serial.println(EEPROM.readLong(EEP_ACC_BIAS + 4));
    Serial.print("acc bias z  : ");
    Serial.println(EEPROM.readLong(EEP_ACC_BIAS + 8));
    Serial.print("gyro bias x : ");
    Serial.println(EEPROM.readLong(EEP_GYRO_BIAS + 0));
    Serial.print("gyro bias y : ");
    Serial.println(EEPROM.readLong(EEP_GYRO_BIAS + 4));
    Serial.print("gyro bias z : ");
    Serial.println(EEPROM.readLong(EEP_GYRO_BIAS + 8));
    Serial.print("mag bias x  : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_BIAS + 0));
    Serial.print("mag bias y  : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_BIAS + 4));
    Serial.print("mag bias z  : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_BIAS + 8));
    Serial.print("mag scale x : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_SCALE + 0));
    Serial.print("mag scale y : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_SCALE + 4));
    Serial.print("mag scale z : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_SCALE + 8));
    Serial.print("heading_offset_rad: ");
    Serial.println(EEPROM.readByte(EEP_HEADING_OFFSET));
    Serial.print("heel_offset_rad   : ");
    Serial.println(EEPROM.readByte(EEP_HEEL_OFFSET));
    Serial.print("trim_offset_rad   : ");
    Serial.println(EEPROM.readByte(EEP_TRIM_OFFSET));
    Serial.print("compass_autocalibration: "); 
    Serial.println(EEPROM.readByte(EEP_AUTOCALIBRATION));        
}

void printBytes()
{
    for (size_t i = 0; i < EEPROM_SIZE; ++i)
        Serial.println(EEPROM.readByte(i), HEX);
}
