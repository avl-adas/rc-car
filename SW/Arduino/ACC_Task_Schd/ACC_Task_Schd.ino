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
  else if (brake_flag) {
    setDrive(1500, 0); //[0 180] due to servo.h module, 90 means 0 speed
    //[1000 2000] 1500 means 0 speed
  }
  else {
    DistF = frontUltrasonic.getDistance();
    DistL = leftUltrasonic.getDistance();
    DistR = rightUltrasonic.getDistance();
    avgDistFL = leftUltrasonic.getAverageDistance();
    avgDistFR = rightUltrasonic.getAverageDistance();
    avgDistF = frontUltrasonic.getAverageDistance();

    if(steer_cmd_pi > 2)
    {
    avgDist = min(avgDistF, avgDistFL);
      
    }
    else if(steer_cmd_pi < -2)
    {
    avgDist = min(avgDistF, avgDistFR);
    }
    else
    {
    avgDist = min(min(avgDistFL,avgDistFR),avgDistF);
    }
    
    
    
    ACC(avgDist, 10); //Distance, times of 10ms

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
  //Serial1.begin(115200);
  //Serial.begin(9600);
  //SerialUSB.begin(9600);
  Serial.begin(9600);
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
  
  //Serial.print("Hello World");

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
  if(loop_ct>=1000)
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
      setSteer(90 + (90 / 30 * value) , 0);              // Steer range: [  -30,  30 ]
      steer_cmd_pi = value;
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
  myservo_steer.write(us);
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
    myservo_drive.write(us);

//  throttleChannel.setDuty(us);
  delay(dly);
}
void wireless_communication()
{
  payload[0] = (int)(DistL);
  payload[1] = (int)(avgDistFL);
  payload[2] = (int)(DistF);
  payload[3] = (int)(avgDistF);
  payload[4] = (int)(DistR);
  payload[5] = (int)(avgDistFR);
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
