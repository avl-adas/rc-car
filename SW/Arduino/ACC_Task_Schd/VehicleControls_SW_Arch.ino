#include <DueTimer.h>
#include "Task_Setup.h"
#include "Wireless_Comm.h"
#include "Battery_Voltage.h"

boolean manual_flag = false;  // Will be used when we start using Remote for Manual Control

void setup() 
{
  // put your setup code here, to run once:
  PWM_SERVO_SETUP();
  Serial.begin(9600);
  pinMode(LOW_BATTERY_INDICATOR, OUTPUT);
  Timer6.attachInterrupt(ACC_Func_Handler);
  Timer6.start(25000); // Calls every 15ms
  Timer7.attachInterrupt(Lane_Keep_Handler);
  Timer7.start(2000); // Calls every 2 ms
  encoder_setup();
  ultrasonic_setup();
  wireless_comm_setup();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  static unsigned long timeout = 0;
  if(millis() - timeout > 2000)
  {
    timeout = millis();
    wireless_communication();
    battery_feedback();
  }
}
