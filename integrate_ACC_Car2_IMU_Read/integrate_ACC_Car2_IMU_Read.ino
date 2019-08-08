//*************************************************************************
//This code combines the ACC functionality
//with the RC Remote control. The ability to control
//the set ACC speed and estop using the remote has been added
//*************************************************************************

#include <Servo.h>
#include <DueTimer.h>
#include "Semaphore.h"
#include "Ultrasonic.h"
#include "ACC.h"
//#include "Drive.h"
#include "PwmChannel.h"
#include "Encoder.h"
//#include <TimerOne.h>

//---IMU files ----
#include <Wire.h>
//#include "Razor_AHRS_Define.h"
//---IMU files ----

Servo myservo_drive;
Servo myservo_steer;

//Throttle
const int THROTTLE_IN_PIN = 3;
                            const int MOTOR_PIN = 13;
//Steering
const int STEERING_IN_PIN = 2;
const int S_MOTOR_PIN = 12;
const int ULTRASONIC_FRONT = 48 ;//car 1 is 54; car 2 is 48
const int ULTRASONIC_LEFT = 56;
const int ULTRASONIC_RIGHT = 64;

//Channel 3 (VR) and Channel 4 (SW)
const int VR_IN_PIN = 5; //channel 3 connected to pin 5
const int SW_IN_PIN = 6; //channel 4 connected to pin 6

//we actually dont need an output for these pins, but the class is setup such that we need to provide an output pin here
const int VR_OUT_PIN = 8;
const int ESTOP_PIN  = 7;
                                const int THROTTLE_OUT_PIN = 9;
const int STEERING_OUT_PIN = 11;
int SET_VCRUISE = 0; // set the command cruise velocity from input from channel 3
int SET_STEER = 1500;
int SET_SPEED = 1500;

int setspeed_cmd;
int pispeed_cmd = 200;
int speedChannelold_2 = 1500;
int speedChannelold_1 = 1500;

int throttle_cmd;
int throttleChannelold_2 = 1500;
int throttleChannelold_1 = 1500;
int channel3_old = 1500;

int speedChanneltemp = 1500;
int throttleChanneltemp = 1500;
int steerChanneltemp = 1500;

int loop_ct = 0;
int ultrasonic_ct = 0;

float ff_pwm = 0;
float fb_pwm = 0;

const int DIST_BRAKE_FORWARD = 5; // cm
const int DIST_BRAKE_REVERSE = 50; // cm

const int OVERRIDE_PIN = 10;
bool autoOverride = false;

// PWM pulse widths +/- PWM_DEADZONE are ignored
const int PWM_DEADZONE = 30; // us
const int PWM_FUDGE = 25; // us

bool pinUltrasonicState;
bool pinUltrasonicState_r;
bool pinUltrasonicState_l;
int DistF = 0;// Dist are for debugging
int DistR = 0;
int DistL = 0;
int avgDist;
int avgDistF;
int avgDistFL;
int avgDistFR;


unsigned long tUltrasonicStart;
unsigned long tUltrasonicEnd;
unsigned long tUltrasonicStart_r;
unsigned long tUltrasonicEnd_r;
unsigned long tUltrasonicStart_l;
unsigned long tUltrasonicEnd_l;
int str_buf[3] = {0, 0, 0};
const int us_arb_del = 6;


int steer_cmd_pi = 0;

// Updated 6/10/2016 F. Dang /////////////////////////////
// Raspi - Arduino Serial Commands
const char SOP = '{';
const char EOP = '}';
char PacketType = 0;
char IncomingByteTmp = 0;
char Arry[15];
int i = 0;
int j = 0;
int PacketLength = 0;
int value = 0;
int cntrl_mode = 0;
//0 for ACC/Lanekeeping (default)
//1 for manual override
boolean manual_flag = false;
boolean brake_flag = false;
int Tog = false;
unsigned int ct_main = 0;
unsigned int tmp = 0;
float V_CLC_TRGT = 0;
float V_CL_TRGT_BUF[5] = {0, 0, 0, 0, 0};
const unsigned int idx = 0;
int array_length = 0;
const float batt_thresh = 6.4;

///////////////////////////////////////////////////////////////////


PwmChannel throttleChannel = PwmChannel(THROTTLE_IN_PIN, THROTTLE_OUT_PIN, true, PWM_FUDGE, PWM_DEADZONE, 2, true);
PwmChannel steeringChannel = PwmChannel(STEERING_IN_PIN, STEERING_OUT_PIN, false, 0, 0, 1, false);
PwmChannel setspeedChannel = PwmChannel(VR_IN_PIN, VR_OUT_PIN, false, 0, 0, 1, false);
PwmChannel estopChannel    = PwmChannel(SW_IN_PIN, ESTOP_PIN, false, 0, 0, 1, false);

// Semaphore
Semaphore sem_PWM = FREE;      // Semaphore to disable pwm output

Ultrasonic frontUltrasonic = Ultrasonic(ULTRASONIC_FRONT, sem_PWM);
Ultrasonic rightUltrasonic = Ultrasonic(ULTRASONIC_RIGHT, sem_PWM);
Ultrasonic leftUltrasonic = Ultrasonic(ULTRASONIC_LEFT, sem_PWM);

//Wireless Communication
#include <SPI.h>
#include "RF24.h"
#define CE 27
#define CSN 29
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(CE, CSN);
/**********************************************************/
byte addresses[][6] = {"1Node", "2Node"};
// Used to control whether this node is sending or receiving
int payload[6] = {0};
static int payloadSize = sizeof(payload);


// Comment out for ACC Testing - Jun 17 2016
//Ultrasonic backUltrasonic = Ultrasonic(ULTRASONIC_BACK, sem_PWM);

// ------------------------------ IMU ----------------------------------------------------
/*
* Razor AHRS Firmware v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
*
*
* History:
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
*
*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
*
*   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
*     * v1.3.0
*       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
*       * Added sensor calibration (improves precision and responsiveness a lot!).
*       * Added binary yaw/pitch/roll output.
*       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
*       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
*       * Wrote new easier to use test program (using Processing).
*       * Added support for new version of "9DOF Razor IMU": SEN-10736.
*       --> The output of this code is not compatible with the older versions!
*       --> A Processing sketch to test the tracker is available.
*     * v1.3.1
*       * Initializing rotation matrix based on start-up sensor readings -> orientation OK right away.
*       * Adjusted gyro low-pass filter and output rate settings.
*     * v1.3.2
*       * Adapted code to work with new Arduino 1.0 (and older versions still).
*     * v1.3.3
*       * Improved synching.
*     * v1.4.0
*       * Added support for SparkFun "9DOF Sensor Stick" (versions SEN-10183, SEN-10321 and SEN-10724).
*     * v1.4.1
*       * Added output modes to read raw and/or calibrated sensor data in text or binary format.
*       * Added static magnetometer soft iron distortion compensation
*     * v1.4.2
*       * (No core firmware changes)
*
* TODOs:
*   * Allow optional use of EEPROM for storing and reading calibration values.
*   * Use self-test and temperature-compensation features of the sensors.
***************************************************************************************************************/

/*
  "9DOF Razor IMU" hardware versions: SEN-10125 and SEN-10736

  ATMega328@3.3V, 8MHz

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10125
  HMC5883L : Magnetometer on SEN-10736
  ITG-3200 : Gyro

  Arduino IDE : Select board "Arduino Pro or Pro Mini (3.3v, 8Mhz) w/ATmega328"
*/

/*
  "9DOF Sensor Stick" hardware versions: SEN-10183, SEN-10321 and SEN-10724

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10183 and SEN-10321
  HMC5883L : Magnetometer on SEN-10724
  ITG-3200 : Gyro
*/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.
    
  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up
  
  Transformation order: first yaw then pitch then roll.
*/

/*
  Serial commands that the firmware understands:
  
  "#o<params>" - Set OUTPUT mode and parameters. The available options are:
  
      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.
      
      // Angles output
      "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).
      
      // Sensor calibration
      "#oc" - Go to CALIBRATION output mode.
      "#on" - When in calibration mode, go on to calibrate NEXT sensor.
      
      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
                In fact it's too much and an output frame rate of 50Hz can not be maintained. #osbb.
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).
      
      // Error message output        
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.
    
    
  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.
         
         
  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.
          
          
  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)
  
  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.
  
  The status LED will be on if streaming output is enabled and off otherwise.
  
  Byte order of binary output is little-endian: least significant byte comes first.
*/



/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)


// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 9600 //115200  //57600

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float


// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false


// Bluetooth
// You can set this to true, if you have a Rovering Networks Bluetooth Module attached.
// The connect/disconnect message prefix of the module has to be set to "#".
// (Refer to manual, it can be set like this: SO,#)
// When using this, streaming output will only be enabled as long as we're connected. That way
// receiver and sender are synchronzed easily just by connecting/disconnecting.
// It is not necessary to set this! It just makes life easier when writing code for
// the receiving side. The Processing test sketch also works without setting this.
// NOTE: When using this, OUTPUT__STARTUP_STREAM_ON has no effect!
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false


// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -260) //old value for all +/-250
#define ACCEL_X_MAX ((float) 230)
#define ACCEL_Y_MIN ((float) -243)
#define ACCEL_Y_MAX ((float) 257)
#define ACCEL_Z_MIN ((float) -256)
#define ACCEL_Z_MAX ((float) 242)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -1077)
#define MAGN_X_MAX ((float) 255)
#define MAGN_Y_MIN ((float) -643)
#define MAGN_Y_MAX ((float) 605)
#define MAGN_Z_MIN ((float) -885)
#define MAGN_Z_MAX ((float) 361)

// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {0, 0, 0};
//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 9.51) //old value 0.0
#define GYRO_AVERAGE_OFFSET_Y ((float) 35.10)
#define GYRO_AVERAGE_OFFSET_Z ((float) 9.62)

/*
// Calibration example:

// "accel x,y,z (min/max) = -277.00/264.00  -256.00/278.00  -299.00/235.00"
#define ACCEL_X_MIN ((float) -277)
#define ACCEL_X_MAX ((float) 264)
#define ACCEL_Y_MIN ((float) -256)
#define ACCEL_Y_MAX ((float) 278)
#define ACCEL_Z_MIN ((float) -299)
#define ACCEL_Z_MAX ((float) 235)

// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
//#define MAGN_X_MIN ((float) -511)
//#define MAGN_X_MAX ((float) 581)
//#define MAGN_Y_MIN ((float) -516)
//#define MAGN_Y_MAX ((float) 568)
//#define MAGN_Z_MIN ((float) -489)
//#define MAGN_Z_MAX ((float) 486)

// Extended magn
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {91.5, -13.5, -48.1};
const float magn_ellipsoid_transform[3][3] = {{0.902, -0.00354, 0.000636}, {-0.00354, 0.9, -0.00599}, {0.000636, -0.00599, 1}};

// Extended magn (with Sennheiser HD 485 headphones)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {72.3360, 23.0954, 53.6261};
//const float magn_ellipsoid_transform[3][3] = {{0.879685, 0.000540833, -0.0106054}, {0.000540833, 0.891086, -0.0130338}, {-0.0106054, -0.0130338, 0.997494}};

//"gyro x,y,z (current/average) = -40.00/-42.05  98.00/96.20  -18.00/-18.36"
#define GYRO_AVERAGE_OFFSET_X ((float) -42.05)
#define GYRO_AVERAGE_OFFSET_Y ((float) 96.20)
#define GYRO_AVERAGE_OFFSET_Z ((float) -18.36)
*/


// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME true


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/


// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
  // Generate compile error
  #error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.ino!
#endif

//#include <Wire.h>

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define STATUS_LED_PIN 4  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi


// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
unsigned long imu_loop_time;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;
// ------------------------------ IMU ----------------------------------------------------

void Speed_Cntrl()
{
  //speed unit cm/s
  array_length = sizeof(ff_target_PWM) / INT_LENGTH;
  if (V_CL_TRGT < ff_target_speed[0])
  {
    ff_pwm = ff_target_PWM[0];
  }
  else if (V_CL_TRGT > ff_target_speed[array_length - 1])

  {
    ff_pwm = ff_target_PWM[array_length - 1];
  }
  else
  {

    for (int i = 0; i < (array_length - 2); i++)
    {
      float pwm_slope = 0;
      if ( abs(V_CL_TRGT - ff_target_speed[i]) <= 0.01 ) //&& (V_CL_TRGT < (ff_target_speed[i]+10)) )
      {
        ff_pwm = ff_target_PWM[i];
        break;
      }

      if ( (V_CL_TRGT > ff_target_speed[i]) && (V_CL_TRGT < ff_target_speed[i + 1]) ) //&& (V_CL_TRGT < (ff_target_speed[i]+10)) )
      {
        pwm_slope = (ff_target_PWM[i + 1] - ff_target_PWM[i]) / (ff_target_speed[i + 1] - ff_target_speed[i]);
        ff_pwm = pwm_slope * V_CL_TRGT + (ff_target_PWM[i] - pwm_slope * ff_target_speed[i]);      // y = mx + C
        break;
      }
    }
  }

  if (batt_voltage > 5) {
    ff_pwm = max(min(7.2 / batt_voltage, 1.2), 1) * ff_pwm; // A&M
  }
  else {
    ;
  }
  ff_pwm = max(1440.0F, min(ff_pwm, 1660.0F));

  /*Model Reference the CLC - Yue Sun 12/27/2018, runs the same loop interval as encoder speed filter*/
  V_CL_TRGT_BUF[0] = V_CL_TRGT;

  V_CLC_TRGT = speed_wt * V_CL_TRGT_BUF[idx] + (1 - speed_wt) * V_CLC_TRGT;

  V_CL_TRGT_BUF[4] = V_CL_TRGT_BUF[3];
  V_CL_TRGT_BUF[3] = V_CL_TRGT_BUF[2];
  V_CL_TRGT_BUF[2] = V_CL_TRGT_BUF[1];
  V_CL_TRGT_BUF[1] = V_CL_TRGT_BUF[0];

  float speed_error = V_CLC_TRGT - car_speed;
  //scale dowm the error as per the feedback and FF control distribution
  //speed_error = 0.4*speed_error;

  /*noise rejection*/
  if ( speed_error > 2.0F) {
    enc_Kp = 1;
    enc_Ki = 0.5;
  }
  else {
    if (speed_error > -2.0F) {
      speed_error = 0.0F;
      enc_Kp = 1;
      enc_Ki = 0.5;
    }
    else {
      enc_Kp = 1;
      enc_Ki = 0.5;
    }
  }

  prop_error = speed_error * enc_Kp;
  intgl_error += speed_error * enc_Ki * 0.025;
  fb_pwm = prop_error + intgl_error;

  if (fb_pwm > 100.0)
  {
    fb_pwm = 100.0;
    intgl_error -= speed_error * enc_Ki * 0.025;
  }
  if (fb_pwm < -150.0)
  {
    fb_pwm = -150.0;
    intgl_error -= speed_error * enc_Ki * 0.025;
  }
  motor_PWM = ff_pwm + fb_pwm ; // + 90.0 with ff_pwm;


    setDrive(motor_PWM, 0);

  //setDrive( 1600,  0);
}

void ACC_Func_Handler() { //running every 25ms

  battery_feedback();
  encoder_feedback();

  if (manual_flag) {
//    setDrive(SET_SPEED, 0);
  }
  /*else if (brake_flag) {
    setDrive(1500, 0); //[0 180] due to servo.h module, 90 means 0 speed
    //[1000 2000] 1500 means 0 speed
  }*/
  else {
    DistF = frontUltrasonic.getDistance();
    DistL = leftUltrasonic.getDistance();
    DistR = rightUltrasonic.getDistance();
    avgDistFL = leftUltrasonic.getAverageDistance();
    avgDistFR = rightUltrasonic.getAverageDistance();
    avgDistF = frontUltrasonic.getAverageDistance();

    if(steer_cmd_pi > 3)
    {
    avgDist = min(avgDistF, avgDistFL);
    if (avgDistFR <= (d_thres_lo-10))
        avgDist = min(avgDist, avgDistFR);
    }
    else if(steer_cmd_pi < -3)
    {
    avgDist = min(avgDistF, avgDistFR);
    if (avgDistFL <= (d_thres_lo-10))
        avgDist = min(avgDist, avgDistFL);
    }
    else
    {
    avgDist = min(min(avgDistFL,avgDistFR),avgDistF);
    }
    
    
    
    ACC(avgDist, 10, brake_flag); //Distance, times of 10ms

    /*12 26 2018 YSUN - re-map RC car speed range, 1500 + [-500, 500]*/
    //M&A
    SET_VCRUISE = 120;
    /*
      tmp=Cmd_To_PWM(v_cmd,SET_VCRUISE,pispeed_cmd);
      setDrive(1500 + tmp, 0);*/

    /*12 27 2018 - reengage speed control, remap speed targets*/

    //v_cmd is an output from ACC func call

    float speed_control = Cmd_To_PWM(v_cmd, SET_VCRUISE, pispeed_cmd);

    //[0 120] - PI, RC, v_cmd
    //[0 60] - cm/s linear wheel speed
    V_CL_TRGT = map(speed_control, -120, 120, -60, 60);

    //encoder feedback is mapped to linear wheel speed in unit of [cm/s]
    Speed_Cntrl();
  }

  
  //Trigger ultrasonics every 75ms
  ultrasonic_ct++;
  if(ultrasonic_ct >= 1)
  {
    frontUltrasonic.readDistance();
    rightUltrasonic.readDistance();
    leftUltrasonic.readDistance();
    ultrasonic_ct = 0;
  }

}

void Lane_Keep_Hanlder() {
  if (manual_flag) {
//    setSteer(SET_STEER, 0);
  } else {
    serialEvent();
  }
}

void setup()
{
  PWM_SERVO_SETUP();
  
  
  //Serial1.begin(115200);
  Serial.begin(9600);
  //SerialUSB.begin(9600);
  //SerialUSB.begin(14400);

  pinMode(OVERRIDE_PIN, INPUT);
  pinMode(37, OUTPUT);

  //Ultrasonic setup use to be

  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards

  for (int j = 4; j <= 8; ++j)
    pinMode(j, INPUT);

  pinMode(ESTOP_PIN, OUTPUT);
  digitalWrite(ESTOP_PIN, LOW);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  
  IMU_CALL_SETUP(); //-----IMU setup --- 
  
  digitalWrite(STATUS_LED_PIN, LOW);


  //Timer1.initialize(PERIOD_SCALE); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  //  throttleChannel = PwmChannel(THROTTLE_IN_PIN, MOTOR_PIN, true, PWM_FUDGE, PWM_DEADZONE, 2, true);
  //  steeringChannel = PwmChannel(STEERING_IN_PIN, S_MOTOR_PIN, false, 0, 0, 1, false);
  //PwmChannel.setSemaphore(sem_PWM);     // Set semaphore for PWM/Ultrasonic



  //  //Input
  attachInterrupt(THROTTLE_IN_PIN, throttleChannel.channelISR_arr[throttleChannel.getChannelNum()], CHANGE); //Throttle Channel 2, mean 1500, [1000 2000]
  attachInterrupt(STEERING_IN_PIN, steeringChannel.channelISR_arr[steeringChannel.getChannelNum()], CHANGE); //Steering Channel 1, mean 1500, [1000 2000]
  attachInterrupt(digitalPinToInterrupt(VR_IN_PIN), setspeedChannel.channelISR_arr[setspeedChannel.getChannelNum()], CHANGE);  //Channel 3, [1000 2000]
  attachInterrupt(digitalPinToInterrupt(SW_IN_PIN), estopChannel.channelISR_arr[estopChannel.getChannelNum()], CHANGE);        //Channel 4, mean 1000 -> 2000

  myservo_drive.attach(13);
  myservo_steer.attach(12);

  //PwmChannel::initAndSetSemaphore(sem_PWM);
  delay(200);

  Timer6.attachInterrupt(ACC_Func_Handler);
  Timer6.start(25000); // Calls every 50ms (10Hz)

  Timer7.attachInterrupt(Lane_Keep_Hanlder);
  Timer7.start(2000); // Calls every 2ms (500Hz) - Improve Com Speed, see if steers earlier

  pinMode(channelA, INPUT_PULLUP);
  pinMode(channelB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelA), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelB), encoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt((frontUltrasonic.getPin()) + frontUltrasonic.getEchoOffset()), ultrasonicChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt((rightUltrasonic.getPin()) + rightUltrasonic.getEchoOffset()), ultrasonicChange_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt((leftUltrasonic.getPin()) + leftUltrasonic.getEchoOffset()), ultrasonicChange_l, CHANGE);
 
  /* Wireless communication */
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(80); 
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setAutoAck(0);
  radio.stopListening();
  
  Serial.print("Hello World");

//  disableWatchDog();
//  wdt_disable();

}

void loop()
{

  //channel 3 filtering & fault rejection
  //if valid input signal, record it to temp
  if ( (setspeedChannel.getMicroDiff() > 900) && (setspeedChannel.getMicroDiff() < 2100)) {
    speedChanneltemp = setspeedChannel.getMicroDiff();
  }
  else { //else load last step of valid value to temp
    ;//speedChanneltemp = speedChannelold_1;
  }
  setspeed_cmd = (speedChanneltemp + speedChannelold_1 + speedChannelold_2) / 3;
  speedChannelold_2 = speedChannelold_1;
  speedChannelold_1 = speedChanneltemp;


  //channel 2 filtering
  //if valid input signal, record it to temp
  if ( (throttleChannel.getMicroDiff() > 900) && (throttleChannel.getMicroDiff() < 2100)) {
    throttleChanneltemp = throttleChannel.getMicroDiff();
  }
  else { //else load last step of valid value to temp
    ;//throttleChanneltemp = throttleChannelold_1;
  }
  throttle_cmd = ( throttleChanneltemp + throttleChannelold_1 + throttleChannelold_2) / 3;
  throttleChannelold_2 = throttleChannelold_1;
  throttleChannelold_1 = throttleChanneltemp;

  //channel 3 fault rejection
  if ((steeringChannel.getMicroDiff() > 900) && (steeringChannel.getMicroDiff() < 2100)) {
    steerChanneltemp = steeringChannel.getMicroDiff();
  }
  else {
    steerChanneltemp = 1500;
  }

  SET_VCRUISE = map(speedChanneltemp, 1000, 2000, 0, 120); //set cruise velocity target map here, change values (0,200) for different mapping, channel 3 - Speed
  SET_SPEED = map(throttleChanneltemp, 1000, 2000, 1300, 1700); //set speed target map here, change values (1300,1700) for different mapping, channel 2 - PWM [1000 2000]
  SET_STEER = map(steerChanneltemp, 900, 2100, 180, 0); //set steer target map here, change values (0,180) for different mapping, channel 1

  //  Serial.print("\t");
  //  Serial.print("\t");
  //  Serial.println(throttleChannel.getMicroDiff());

  //M&A
  cntrl_mode = 0;

  switch (cntrl_mode) {

    case 0://auto state
      manual_flag = false;
      if ( (throttle_cmd > 1600) || (throttle_cmd < 1400)) { //channel 2
        cntrl_mode = 1;
        channel3_old = setspeed_cmd;

      }
      break;
    case 1://manual state
      manual_flag = true;
      if (abs(setspeed_cmd - channel3_old) > 200) { //channel 3
        cntrl_mode = 0;
      }
      break;
    default:
      manual_flag = false;
      cntrl_mode = 0;
      break;

  }

  //***** estop conditions
  if (estopChannel.getMicroDiff() > 1700 && estopChannel.getMicroDiff() < 2500)
  { //digitalWrite(ESTOP_PIN, HIGH);
    //Serial.print("ESTOP!!! Press reset if everything is OK");
  }
  else
  { digitalWrite(ESTOP_PIN, LOW);
  }
  //end ***** estop conditions
 
  //12/26/2018 Yue Sun Comment Out All print screens
  // Read average ultrasonic distance for automatic braking
  //int avgDistF = frontUltrasonic.getAverageDistance();
  /*
    Serial.print("Channel 2:");
    Serial.print(throttleChannel.getMicroDiff());
    Serial.print("\t");

    Serial.print("Channel 3:");
    Serial.print(setspeedChannel.getMicroDiff());
    Serial.print("\t");

    Serial.print("CntrlState:");
    Serial.print(cntrl_mode);
    Serial.print("\t");

    Serial.print("DriveCmd:");
    Serial.print(tmp);
    Serial.print("\t");

    Serial.print("MappedChan3:");
    Serial.print(SET_VCRUISE);
    Serial.print("\t");

    Serial.print("PI");
    Serial.print(pispeed_cmd);
    Serial.print("\t");

    Serial.print("Ultra");
    Serial.print(avgDistF);
    Serial.print("\t");


    Serial.println(estopChannel.getMicroDiff());
  */
  //  Serial.println(batt_voltage);
  /*
    //Yue Sun 12/27/2018
    Serial.print(V_CL_TRGT);
    Serial.print("\t");
    Serial.print(V_CLC_TRGT);
    Serial.print("\t");
    Serial.print(car_speed);
    Serial.print("\t");
    Serial.print(batt_voltage);
    Serial.print("\t");
    Serial.print(ff_pwm);
    Serial.print("\t");
    Serial.print(fb_pwm);
    Serial.print("\t");
    Serial.print(prop_error);
    Serial.print("\t");
    Serial.print(intgl_error);
    Serial.print("\t");
    Serial.print(loop_delay);
    Serial.print("\t");
    Serial.print(array_length);
    Serial.print("\t");
    Serial.println(enc_count);

    //  Serial.println(setspeedChannel.getMicroDiff());
  */

  loop_ct++;
  if(loop_ct>=50)
  {
    /* Call this every 50 loops */
    wireless_communication();
    loop_ct = 0;
  if(batt_voltage <= batt_thresh)
      {
  
     digitalWrite(37,HIGH);
  
      }
     else{
        digitalWrite(37,LOW);
    
      }
  }
  
  ct_main++;
  if (ct_main % 100 == 0) {
    Tog = 1 - Tog;
  }
  //digitalWrite(13, boolean(Tog));
  //Serial.println(batt_call);

  digitalWrite(STATUS_LED_PIN, HIGH);

  IMU_LOOP_CALL() ; // ----------- IMU call ---------

  digitalWrite(STATUS_LED_PIN, LOW);


//actual IMU acc data
// divide the values by 256 or GRAVITY to get acceleration data in g
//Serial.print("\t Ax:");     Serial.print(accel[0]);
//Serial.print("\t Ay:");     Serial.print(accel[1]);
//Serial.print("\t Az:");     Serial.print(accel[2]);
//Serial.print("\t Az:");

Serial.println(""); //line break
delay(5);
  
}

// Updated 6/10/2016 F. Dang Serial Communicaiton from PI to Arduino///////////////////////////////////////////////////////////////
void serialEvent() {
  // Schedule new pulse width
  //  dutyRequest = Serial.parseFloat();
  //  oldDutyOutput = outputPeriod * dutyRequest / 100;
  //  Serial.print("Requested: "); Serial.println(dutyRequest);
  if (Serial.available() && i < 14)
  {
    IncomingByteTmp = Serial.read();

    switch (IncomingByteTmp) {
      case SOP:
        i = 0;
        //Serial.println("SOP found");
        /*for(int j=0;j<sizeof(Arry);j++){
            Arry[j] = '\0';
          }*/
        break;

      case EOP:
        PacketType = Arry[0];
        PacketLength = i;
        Arry[i] = '\0';

        value = atoi(Arry + 1);
        //Serial.print("Packet type: ");
        //Serial.println(PacketType);
        //Serial.println(value);

        //Right now this function runs the same rate as serialEvent() in 1ms, we need to be cautious
        driveHandler(PacketType, value);

        break;

      default:
        Arry[i] = IncomingByteTmp;
        i = i + 1;
        //Serial.print("Char found");
        //Serial.println(Arry[i-1]);

    }
  }
  else if(i >= 14) { /*No char on the bus*/
    /*12/26/2018 - Yue Sun Open Loop Max Reduction*/
//    pispeed_cmd = 120; /*invalid inpupt, set to max*/
//    brake_flag = false; /*reset brake flag*/
    i = 0;
  }


}

// Handle Raspi commands

void driveHandler(char packetType, int value) {
  switch (packetType) {
    //not used since PI sends 'D' - Drive
    case 'D': // Forward
      //Serial.print("Drive F: "); Serial.println(value);
      //Do not follow speed command from Lane Control
      //setDrive(1500 + value * 5 , 0);                       // Drive range: [ -100, 100 ]
      if (value > 50 && value < 150) { //honored speed command
        pispeed_cmd = value;
      }
      else {
        pispeed_cmd = 120; //Invalid speed command (almost stoped, set to max)
      }
      break;
    case 'B':
      //Serial.print("Brake: "); Serial.println(value);
      if (value > 50) {            // Demo brake function. not best implementation
        // Then brake                                       // Brake range: [    0, 100 ]
        brake_flag = true;
      }
      else {
        brake_flag = false;
      }
      break;
    case 'S':
      //Serial.print("Steer: "); Serial.println(value);
      //setSteer(1500 + (500/30) * value , 0);                // Steer range: [  -30,  30 ]
  
      str_buf[0] = str_buf[1];
      str_buf[1] = str_buf[2];
      str_buf[2] = value;
      
      setSteer(90 + (90 / 30 * value) , 0);              // Steer range: [  -30,  30 ]
      if(str_buf[2] - str_buf[1] < us_arb_del)           // condition to select left or right ultrasonic
      {
          steer_cmd_pi = str_buf[2];
      }
      else
      {
          steer_cmd_pi = (str_buf[0] + str_buf[1] + str_buf[2])/3;
      }
      
      break;
    default:
      ;
      //Serial.print("Error: Unrecognized command: "); Serial.println(packetType);
  }

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//us [0 180]
void setSteer(int us, int dly) {
  //steeringChannel.setDuty(us);
  //myservo_steer.write(us);
  if (us <= 180)
    us = map(us,0,180,540,2380);
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = us;
  delay(dly);
}

/*
  void brake (int dly){
  throttleChannel.setDuty(1000);
  delay(dly);
  throttleChannel.setDuty(1500);
  }
*/
//us [0 180]
void setDrive(int us, int dly) {

  //M&A
  /*
    Serial.println("P ");
    Serial.print(us);
    Serial.print("\t");
    Serial.print("M ");
    Serial.print(En_Vel);
    Serial.print("\t");
    Serial.print("F ");
    Serial.print(Rel_Pos);
    Serial.print("\t");
    Serial.print("U ");

    Serial.println(DistF);
  */
//  myservo_drive.writeMicroseconds(us);
//    myservo_drive.write(us);

  if (us <= 180)
    us = map(us,0,180,1000,2000);
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = us;
  
//  throttleChannel.setDuty(us);
  delay(dly);
}
void wireless_communication()
{
  payload[0] = (int)(TO_DEG(yaw));
  payload[1] = (int)(TO_DEG(pitch));
  payload[2] = (int)(TO_DEG(roll));
  payload[3] = (int)(accel[0]);
  payload[4] = (int)(accel[1]);
  payload[5] = (int)(imu_loop_time);
  radio.writeFast( &payload, payloadSize); //WARNING FAST WRITE
  //when using fast write there are three FIFO buffers.
  //If the buffers are filled the 4th request will become blocking.
  //Ensure Fast write is not called too quickly (around 1 ms)

}


void ultrasonicChange()
{
  pinUltrasonicState = digitalRead(frontUltrasonic.getPin() + frontUltrasonic.getEchoOffset());

  if (pinUltrasonicState)
  {
    tUltrasonicStart = micros();
    //Rising_Toggle++;
    //RisingFalling_Toggle = 1;
  }
  else
  {
    tUltrasonicEnd = micros() - tUltrasonicStart;
    frontUltrasonic.set_pulse_dur(tUltrasonicEnd);
    //RisingFalling_Toggle = 0;
  }
}


void ultrasonicChange_r()
{
  pinUltrasonicState_r = digitalRead(rightUltrasonic.getPin() + rightUltrasonic.getEchoOffset());

  if (pinUltrasonicState_r)
  {
    tUltrasonicStart_r = micros();
    //Rising_Toggle++;
    //RisingFalling_Toggle = 1;
  }
  else
  {
    tUltrasonicEnd_r = micros() - tUltrasonicStart_r;
    rightUltrasonic.set_pulse_dur(tUltrasonicEnd_r);
    //RisingFalling_Toggle = 0;
  }
}


void ultrasonicChange_l()
{
  pinUltrasonicState_l = digitalRead(leftUltrasonic.getPin() + leftUltrasonic.getEchoOffset());

  if (pinUltrasonicState_l)
  {
    tUltrasonicStart_l = micros();
    //Rising_Toggle++;
    //RisingFalling_Toggle = 1;
  }
  else
  {
    tUltrasonicEnd_l = micros() - tUltrasonicStart_l;
    leftUltrasonic.set_pulse_dur(tUltrasonicEnd_l);
    //RisingFalling_Toggle = 0;
  }
}

void PWM_SERVO_SETUP()
{
    // PWM set-up on pins D38 and D36 for channels 1 and 2 respectively
  REG_PMC_PCER1 |= PMC_PCER1_PID36;                  // Enable PWM 

  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);  // Set the PWM clock A rate to 2MHz (84MHz/42)
  

  PWM->PWM_CH_NUM[1].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;      // Enable dual slope PWM and set the clock source as CLKA
  PWM->PWM_CH_NUM[1].PWM_CPRD = 20000;                                // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz;
  PWM->PWM_CH_NUM[2].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;      // Enable dual slope PWM and set the clock source as CLKA
  PWM->PWM_CH_NUM[2].PWM_CPRD = 20000;                                // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz;
  
  REG_PWM_ENA = PWM_ENA_CHID2 | PWM_ENA_CHID1;                        //Enable PWM channels 1 and 2;
  
  delay(1);
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = 1500;        // Set initial PWM
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = 1500;  
  delay(3000);                                  // Give ESC time to reset after pins reset to low
  REG_PIOC_ABSR |= PIO_ABSR_P6 | PIO_ABSR_P4;   // Set the port C PWM pins to peripheral type B
  REG_PIOC_PDR  |= PIO_PDR_P6 | PIO_PDR_P4;     // Set the port C PWM pins to outputs
  delay(250);
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = 1500;        // Set the PWM duty cycle to center / 50% / 1500 
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = 1500;  
}

void IMU_CALL_SETUP(){
  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);
  
  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  delay(100);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  
  // Read sensors, init DCM algorithm
  delay(100);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif
}

void IMU_LOOP_CALL(){

//  output_mode = OUTPUT__MODE_SENSORS_CALIB; // change ---------------->
  output_mode = OUTPUT__MODE_ANGLES;
  
  // Time to read the sensors again?
  if((micros()/1000 - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = micros()/1000;//millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();

    if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
    {
      check_reset_calibration_session();  // Check if this session needs a reset
      if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
    }
    else if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
    {
      // Apply sensor calibration
      compensate_sensor_errors();
    
      // Run DCM algorithm
      Compass_Heading(); // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();
      
      if (output_stream_on || output_single_on) output_angles();
    }
    else  // Output sensor values
    {      
      if (output_stream_on || output_single_on) output_sensors();
    }
    
    output_single_on = false;
    
#if DEBUG__PRINT_LOOP_TIME == true
//    Serial.print("loop time (ms) = ");
//    Serial.println(micros()/1000 - timestamp);
    imu_loop_time = micros()/1000 - timestamp;
#endif
  }
#if DEBUG__PRINT_LOOP_TIME == true
  else
  {
    //Serial.println("waiting...");
  }
#endif

}
