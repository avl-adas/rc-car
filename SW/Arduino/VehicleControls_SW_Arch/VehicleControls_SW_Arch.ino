//*************************************************************************
//This code combines the ACC functionality
//with the RC Remote control. The ability to control
//the set ACC speed and estop using the remote has been added
//*************************************************************************

#include <Servo.h>
#include <DueTimer.h>
#include "Semaphore.h"

//#include "Drive.h"
#include "PwmChannel.h"
#include "Encoder.h"

#include "RemoteInput.h"
#include "Pin_Init.h"
#include "Ultrasonic.h"
#include "ACC_Task.h"
#include "Lane_Control.h"

// These are used for Remote inputs processing
int SET_VCRUISE = 0; // set the command cruise velocity from input from channel 3
int SET_STEER = 1500;
int SET_SPEED = 1500;




boolean manual_flag = false;   // Used in main and ACC
boolean brake_flag = false;    // Unused. DELETE

unsigned int ct_main = 0;
unsigned int tmp = 0;

// Semaphore
Semaphore sem_PWM = FREE;      // Semaphore to disable pwm output

// Ultrasonic Objects
Ultrasonic frontUltrasonic = Ultrasonic(ULTRASONIC_FRONT, sem_PWM);
Ultrasonic rightUltrasonic = Ultrasonic(ULTRASONIC_RIGHT, sem_PWM);
Ultrasonic leftUltrasonic = Ultrasonic(ULTRASONIC_LEFT, sem_PWM);

void setup()
{
  PWM_SERVO_SETUP();
  Serial.begin(9600);

  pinMode(OVERRIDE_PIN, INPUT);
  pinMode(LOW_BATTERY_INDICATOR, OUTPUT);

  remote_Pin_Setup();

  // Are we still using servo
  myservo_drive.attach(13);
  myservo_steer.attach(12);

  //PwmChannel::initAndSetSemaphore(sem_PWM);
  delay(200);

  Timer6.attachInterrupt(ACC_Func_Handler);
  Timer6.start(25000); // Calls every 50ms (10Hz)

  Timer7.attachInterrupt(Lane_Keep_Hanlder);
  Timer7.start(2000); // Calls every 2ms (500Hz) - Improve Com Speed, see if steers earlier

  encoder_setup();
  ultrasonic_setup();
  wireless_comm_setup();
}

void loop()
{
  // speed channel varibles
  static uint16_t speedChanneltemp = 1500;
  static uint16_t speedChannelold_1 = 1500;
  static uint16_t speedChannelold_2 = 1500;

  // throttle channel variables
  static uint16_t throttleChanneltemp = 1500;
  static uint16_t throttleChannelold_1 = 1500;
  static uint16_t throttleChannelold_2 = 1500;

  // steer channel variables
  static uint16_t steerChanneltemp = 1500;

  static uint8_t loop_ct = 0;     // used for Wireless communication
  static uint8_t ct_main = 0;     // used for battery call???
  static bool Tog = false;        // Used for ??

  int setspeed_cmd;

  //channel 3 - Speed setting from Remote. Filtering & fault rejection
  //if valid input signal, record it to temp
  speedChanneltemp = setspeedChannel.getMicroDiff();
  if ( (speedChanneltemp < 900) && (speedChanneltemp > 2100) ) 
  { //if out of range load last step of valid value to temp
      speedChanneltemp = speedChannelold_1;
  }
  setspeed_cmd = (speedChanneltemp + speedChannelold_1 + speedChannelold_2) / 3;
  speedChannelold_2 = speedChannelold_1;
  speedChannelold_1 = speedChanneltemp;

  //channel 1 - Throttle command from Remote. Fault filtering
  //if valid input signal, record it to temp
  throttleChanneltemp = throttleChannel.getMicroDiff();
  if ( (throttleChanneltemp < 900) && (throttleChanneltemp > 2100)) 
  {   //if out of range load last step of valid value to temp
      throttleChanneltemp = throttleChannelold_1; 
  }
  throttle_cmd = ( throttleChanneltemp + throttleChannelold_1 + throttleChannelold_2) / 3;
  throttleChannelold_2 = throttleChannelold_1;
  throttleChannelold_1 = throttleChanneltemp;

  //channel 2 - Steering commands from Remote. Fault rejection
  steerChanneltemp = steeringChannel.getMicroDiff();
  if ((steerChanneltemp < 900) && (steerChanneltemp > 2100)) 
  {   // if out of range load defaul center value
      steerChanneltemp = 1500;
  }

  SET_VCRUISE = map(speedChanneltemp, 1000, 2000, 0, 120); //set cruise velocity target map here, change values (0,200) for different mapping, channel 3 - Speed
  SET_SPEED = map(throttleChanneltemp, 1000, 2000, 1300, 1700); //set speed target map here, change values (1300,1700) for different mapping, channel 2 - PWM [1000 2000]
  SET_STEER = map(steerChanneltemp, 900, 2100, 180, 0); //set steer target map here, change values (0,180) for different mapping, channel 1

  //M&A
  cntrl_mode = 0;   // Start in Auto Mode
  switch (cntrl_mode) 
  {
    case 0://auto state
      manual_flag = false;
      if ( (throttle_cmd > 1600) || (throttle_cmd < 1400)) 
      { //channel 2
        cntrl_mode = 1;
        channel3_old = setspeed_cmd;    // what is channel3_old used for?
      }
      break;

    case 1://manual state
      manual_flag = true;
      if (abs(setspeed_cmd - channel3_old) > 200) { //channel 3
        cntrl_mode = 0;
      }
      break;

    default:  // Auto mode is DEFAULT operation
      manual_flag = false;
      cntrl_mode = 0;
      break;
  }

  //***** estop conditions
  if (estopChannel.getMicroDiff() > 1700 && estopChannel.getMicroDiff() < 2500)
  { 
    //digitalWrite(ESTOP_PIN, HIGH);
    //Serial.print("ESTOP!!! Press reset if everything is OK");
  }
  else
  { 
    digitalWrite(ESTOP_PIN, LOW);
  }
  //end ***** estop conditions

  loop_ct++;
  if(loop_ct>= WIRELESS_COMM_COUNT)
  {
    /* Call this every 50 loops */
    wireless_communication();
    loop_ct = 0;
    if(batt_voltage <= BATTERY_LOW_THRESHOLD)
    {
        digitalWrite(LOW_BATTERY_INDICATOR,HIGH); // pin 37??
    }
    else
    {
        digitalWrite(LOW_BATTERY_INDICATOR,LOW);
    }
  }
} // Loop() END


//us [0 180]
void setSteer(int us, int dly) 
{
  //steeringChannel.setDuty(us);
  //myservo_steer.write(us);

  // Constants
  if (us <= 180)
    us = map(us,0,180,540,2380);
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = us;
  delay(dly);
}

void setDrive(int us, int dly) 
{
  if (us <= 180)
    us = map(us,0,180,1000,2000);
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = us;
  
//  throttleChannel.setDuty(us);
  delay(dly);
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
