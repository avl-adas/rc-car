#include <arduino.h>
#include <Servo.h>
#include "Drive.h"


Servo steer_servo;

void drive_setup()
{
  steer_servo.attach(P_SRV_CMD);

  analogWriteFrequency(P_MTR_PWM, PWM_FREQ);
  pinMode(P_MTR_PWM,OUTPUT);
  pinMode(P_SRV_CMD,OUTPUT);
}


void drive_cmd(int dvalue,uint8_t dir)
{
  /* Map 0-100% duty cycle to 1-255 analog out */
  dvalue = map(dvalue, 0, 100, 0, 255);
  
  digitalWrite(P_MTR_DIR, dir);
  analogWrite(P_MTR_PWM, dvalue);
}


void steer_cmd(int svalue)
{
  int steer_input = svalue;//map steering angle to servo value
    
  //analogWrite(steerpin,steer_input);
  //steerservo.write(steer_input);
}


int steering_feedback()
{
  int srv_feedback_raw = analogRead(P_SRV_FBACK); //raw voltage from feedback servo
  int srv_feedback_angle = map(srv_feedback_raw, 174, 676, 0, 180); //map from voltage to angle

  return srv_feedback_angle;
}
