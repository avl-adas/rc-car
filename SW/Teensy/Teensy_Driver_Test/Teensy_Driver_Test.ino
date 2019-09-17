#include "Drive.h"
#include <Servo.h>


void setup()
{
  Serial.begin(9600);
  drive_setup();
}


void loop() 
{
  drive_cmd(20,1);
  delay(5000);
  drive_cmd(60,1);
  delay(5000);
  
    
  Serial.print(steering_feedback());
  Serial.print("/n");
}
