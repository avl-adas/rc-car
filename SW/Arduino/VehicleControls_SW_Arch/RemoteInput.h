#ifndef _REMOTE_INPUT_H_
#define _REMOTE_INPUT_H_

#include "PwmChannel.h"

//Throttle
const uint8_t THROTTLE_IN_PIN = 3;
const uint8_t THROTTLE_OUT_PIN = 9;

//Steering
const uint8_t STEERING_IN_PIN = 2;
const uint8_t STEERING_OUT_PIN = 11;

//Channel 3 (VR) 
const uint8_t VR_IN_PIN = 5; //channel 3 connected to pin 5
const uint8_t VR_OUT_PIN = 8;

//Channel 4 (SW)
const uint8_t SW_IN_PIN = 6; //channel 4 connected to pin 6
const uint8_t ESTOP_PIN  = 7;

//we actually dont need an output for these pins, 
//but the class is setup such that we need to provide an output pin here

// Remote Control Channels
PwmChannel throttleChannel = PwmChannel(THROTTLE_IN_PIN, THROTTLE_OUT_PIN, true, PWM_FUDGE, PWM_DEADZONE, 2, true);
PwmChannel steeringChannel = PwmChannel(STEERING_IN_PIN, STEERING_OUT_PIN, false, 0, 0, 1, false);
PwmChannel setspeedChannel = PwmChannel(VR_IN_PIN, VR_OUT_PIN, false, 0, 0, 1, false);
PwmChannel estopChannel    = PwmChannel(SW_IN_PIN, ESTOP_PIN, false, 0, 0, 1, false);

void remote_Pin_Setup()
{
  for (int j = 5; j <= 6; ++j)
  {
    pinMode(j, INPUT);
  }
  pinMode(ESTOP_PIN, OUTPUT);
  digitalWrite(ESTOP_PIN, LOW);

  pinMode(VR_OUT_PIN, OUTPUT);
  digitalWrite(VR_OUT_PIN, LOW);

  // Input
  attachInterrupt(THROTTLE_IN_PIN, throttleChannel.channelISR_arr[throttleChannel.getChannelNum()], CHANGE); //Throttle Channel 2, mean 1500, [1000 2000]
  attachInterrupt(STEERING_IN_PIN, steeringChannel.channelISR_arr[steeringChannel.getChannelNum()], CHANGE); //Steering Channel 1, mean 1500, [1000 2000]
  attachInterrupt(digitalPinToInterrupt(VR_IN_PIN), setspeedChannel.channelISR_arr[setspeedChannel.getChannelNum()], CHANGE);  //Channel 3, [1000 2000]
  attachInterrupt(digitalPinToInterrupt(SW_IN_PIN), estopChannel.channelISR_arr[estopChannel.getChannelNum()], CHANGE);        //Channel 4, mean 1000 -> 2000
}
#endif