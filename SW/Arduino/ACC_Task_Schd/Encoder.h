// Encoder pin setup and variable initialization
#define INT_LENGTH 4
const byte channelA = P_ENCODER_A;
const byte channelB = P_ENCODER_B;
const byte enc_index = P_ENCODER_I;
volatile boolean chA = 0;

volatile boolean enc_left = false;
volatile boolean enc_right = false;
volatile byte seqA = 0;
volatile byte seqB = 0;
volatile long enc_count = 0;
volatile long enc_count_old = 0;
//volatile unsigned int cnt2 = 0;

long newPosition = 0;
long oldPosition = 0;
unsigned long newTime = 0;
unsigned long oldTime = 0;
unsigned int loop_start=0, loop_end=0, loop_delay=0;

const float rear_diff_rad = 3.0;   // rear differential radius in cm
const float drive_shaft_rad = 1.5; // drive shaft radius in cm where the encoder is connected
const float drive_belt_eff = 0.95;
const float tire_rad = 3.7;        // tire radius in cm
const float speed_wt = 0.75; //Yue Sun 12/27/2018 - 
float raw_car_speed = 0.0;
float car_speed = 0.0;
float old_dist = 0.0;
float new_dist = 0.0;

float movement_angles_ds = 0;
float movement_angles_rd = 0;
float movement_wheel = 0;

float prop_error=0;
float intgl_error=0;
float enc_Kp= 1;
float enc_Ki=0.5;

float motor_PWM = 0.0F;
float V_CL_TRGT = 60.0F;
float adjust = 40.0F;

float ff_target_speed[11] = {-30.0F, -20.0F, -10.0F, 0.0F, 10.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F};          // in cm/sec
float ff_target_PWM[11] = {1422.0F-adjust, 1430.0F-adjust, 1440.0F, 1500.0F, 1560.0F, 1578.0F+adjust, 1583.0F+adjust, 1590.0F+adjust, 1595.0F+adjust, 1600.0F+adjust, 1610.0F+adjust};     // pulse width in uS, [1000 2000]

// Battery protection variables
#define NUM_SAMPLES 100
int batt_sum = 0;                    // sum of samples taken
unsigned char sample_count = 0; // current sample number
float batt_voltage = 0.0;            // calculated voltage
float batt_voltage_raw = 0.0;            // calculated voltage raw
unsigned char batt_call = 0U;


void encoder()
{  
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
      if(enc_count == 2147483647) enc_count = 0;
      enc_left = true;
      oldTime = newTime;
      newTime = micros()/1000;
      //capture time stamp
    }
     
     else if (seqA == 0b00000011 && seqB == 0b00001001) 
    {
      enc_count--;
      //signed long roll over protection
      if(enc_count == -2147483648) enc_count = 0;
      enc_right = true;
      oldTime = newTime;
      newTime = micros()/1000;
    }
}// end of encoder ISR

void battery_feedback()
{
    // take a number of analog samples and add them up
    if(sample_count < NUM_SAMPLES) 
    {
        batt_sum += analogRead(A4);
        sample_count++;
        //delay(5);
    }
    else
    {
      // calculate the voltage
      // use 5.0 for a 5.0V ADC reference voltage
      // 5.015V is the calibrated reference voltage
      batt_voltage_raw = ((float)batt_sum / (float)NUM_SAMPLES * 3.3) / 1024.0;
      batt_voltage = batt_voltage_raw * 2.67;
      
      sample_count = 0;
      batt_sum = 0;
      
      
      // send voltage for display on Serial Monitor
      // voltage multiplied by 11 when using voltage divider that
      // divides by 11. 11.132 is the calibrated voltage divide
      // value
      //Serial.println((float)batt_sum / (float)NUM_SAMPLES);

      //Serial.print(batt_voltage);
      //Serial.println (" V");
    }

    //delay(1000);
}

void encoder_feedback()
{
  //Encoder calculations Yue Sun 12/27/2018, overflow protection
//  if (abs(enc_count-enc_count_old)> 200000){
  if (enc_count> 2000000){
      enc_count = 0;
      enc_count_old = 0;
//    old_dist = 0.0F;
    /*
    if(enc_count_old>0){
      movement_angles_ds = (360.0/256.0)*float( (2,147,483,647-enc_count_old) + enc_count);        // drive shaft movement in angles
    }
    else{
      movement_angles_ds = (360.0/256.0)*float( (-2,147,483,648-enc_count_old) + enc_count);        // drive shaft movement in angles
    }*/
  }

  movement_angles_ds = (360.0/256.0)*float(enc_count);        // drive shaft movement in angles
  movement_angles_rd = movement_angles_ds*(drive_shaft_rad/rear_diff_rad)*drive_belt_eff;   //rear differential movement in angles
  movement_wheel = (movement_angles_rd/360)* 2*3.14 * tire_rad;
  new_dist = movement_wheel;

  
  loop_end = micros()/1000;
  loop_delay = loop_end - loop_start;
   //Serial.print("Loop Start = ");
  //Serial.println(loop_start);
  if((new_dist-old_dist) == 0)
      raw_car_speed = 0.0;
  else if ((newTime - oldTime)!= 0)
      raw_car_speed = (new_dist-old_dist)/((newTime - (oldTime-loop_delay))*0.001);      // in cm/sec
  
  car_speed = raw_car_speed*speed_wt + car_speed*(1-speed_wt);
  loop_start = micros()/1000;
  /*
  Serial.print("Loop Delay = ");
  Serial.println(loop_delay);
  Serial.println("  ");
  
  Serial.print("Car speed = ");
  Serial.println(car_speed);
  
  Serial.print("Car PWM = ");
  Serial.println(motor_PWM);
  
  Serial.print("Ff PWM = ");
  Serial.println(ff_pwm);
  
  Serial.print("Fb PWM = ");
  Serial.println(fb_pwm);
  
  Serial.print("Spd_Tgr_cmps =");
  Serial.println(V_CL_TRGT);
  */
  old_dist = new_dist;
  enc_count_old = enc_count;
}
