#include <arduino.h>
#include "Encoder.h"

void encoder_setup()
{
  pinMode(channelA, INPUT_PULLUP);
  pinMode(channelB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelA), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelB), encoder, CHANGE);  
}

int cnt_encoder =0;

void encoder()
{ 
  
  if(cnt_encoder > 10000000)
  {cnt_encoder = 0;}
  else{
  cnt_encoder++;
  }
  
  // Read A and B signals
  boolean A_val = digitalRead(channelA);
  boolean B_val = digitalRead(channelB);

  // Record the A and B signals in seperate sequences
  seqA <<= 1;
  seqA |= A_val;
  
  seqB <<= 1;
  seqB |= B_val;
  
  // Mask the MSB four bits
  seqA &= 0b00001111;
  seqB &= 0b00001111;
     
  // Compare the recorded sequence with the expected sequence
  if (seqA == 0b00001001 && seqB == 0b00000011) 
  { 
    enc_count++;
    //signed long roll over protection
    if(enc_count == 2147483647) 
      {   enc_count = 0;  }
    enc_left = true;
    enc_right = false;
  }
  else if (seqA == 0b00000011 && seqB == 0b00001001) 
  {
    enc_count--;
    //signed long roll over protection
    if(enc_count == -2147483648) 
      {   enc_count = 0;   }
    enc_right = true;
    enc_left = false;
  }
    //Serial.println(enc_count);

  // capture time stamp needed for speed calc accuracy
  oldTime = newTime;
  newTime = micros()/1000;
}// end of encoder ISR

float encoder_speed_feedback()
{
  // RC car wheel variables
  float movement_angles_ds = 0.0F;
  float movement_angles_rd = 0.0F;
  
  // distance variables
  float new_dist = 0.0F;
  static float old_dist = 0.0F;
  
  // speed variables
  float raw_car_speed = 0.0F;
  static float car_speed = 0.0F;

  //Encoder calculations Yue Sun 12/27/2018, overflow protection
  if (enc_count > 2000000)
  {
      enc_count = 0;
      enc_count_old = 0;
  }
  //Serial.println(enc_count);
  //noInterrupts();   // Disable interrupts
  // calc drive shaft movement in angles
  movement_angles_ds = (360.0/256.0)*(float)enc_count;   
  //Serial.println(movement_angles_ds);

  movement_angles_pr = movement_angles_ds;
  // calc rear differential movement in angles     
  movement_angles_rd = movement_angles_ds * (DRIVE_SHAFT_RADIUS / REAR_DIFF_RADIUS) * DRIVE_BELT_EFF;
  //Serial.println(movement_angles_rd);
  // calc wheel movement in centimetres
  new_dist = (movement_angles_rd / 360.0F) * 2 * 3.14 * TIRE_RADIUS;
  //Serial.println(new_dist);
  //Serial.println(old_dist);
  speed_loop_end = micros()/1000;
  speed_loop_delay = speed_loop_end - speed_loop_start;
  //Serial.println(speed_loop_delay);
  if( abs(new_dist - old_dist) <= 0.01F)
  {
    raw_car_speed = 0.0F;
    //Serial.println("IF is TRUE");
  }
  else
  {
    raw_car_speed = (new_dist - old_dist)/( ((newTime - oldTime) + speed_loop_delay) * 0.001 );   // in cm/sec
    //Serial.println(raw_car_speed);
  }
    //Serial.println(raw_car_speed);
  car_speed = raw_car_speed * SPEED_FILTER_WT + car_speed * (1 - SPEED_FILTER_WT);
  //Serial.println(car_speed);
  //Serial.println(newTime);
  //Serial.println(oldTime);
  //interrupts();   // Enable interrupts
  speed_loop_start = micros()/1000;
 
  old_dist = new_dist;
  enc_count_old = enc_count;

  
  return car_speed;
}
