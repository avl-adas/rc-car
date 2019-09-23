#include "Task_Setup.h"
#include "Drive.h"

extern long enc_count;
extern float movement_angles_pr;
extern float car_speed_print;

IntervalTimer tmr_Drive_Control;

void setup()
{
  Serial.begin(9600);
  drive_setup();
  encoder_setup();

  /* Timer interrupt */
  tmr_Drive_Control.begin(Drive_Control, 2000000);
}


void loop()
{
  /* main loop */
  
  Serial.println(car_speed_print);
  
  //Serial.print("call");
  //Serial.print("/n");
}
