#include <arduino.h>
#include "ACC_Task.h"
#include "Ultrasonic.h"
#include "ACC_Task_Data.h"

float REF_Speed;
float CUR_Speed;
float PI_Speed;
float FF_PWM;
float FB_PWM;
float MTR_PWM;
unsigned long acc_task_speed;

void ACC_Func_Handler() 
{   //running every 25ms
  unsigned long acc_task_start = micros();
  ultrasonic_distances();     // Minimum distance arbitration
  Mode_Selector();            // Updates CAR_MODE

  float raspi_speed = (float)pi_speed_cmd;    // Using a local variable for data integrity
  raspi_speed = map(raspi_speed, -160.0F, 160.0F, -80.0F, 80.0F);
  PI_Speed = raspi_speed;

  //Temporary
  //CAR_MODE = CRUISE_CONTROL;
  
  switch (CAR_MODE)
  {
    case MANUAL:
    //Serial.println("In Manual Mode    ");
    //Speed_Control(1625.0);
    break;
      
    case CRUISE_CONTROL: 
      float reference_speed;
      reference_speed = min(raspi_speed, CRUISE_VELOCITY);
      
      if (pi_brake_flag)
      {
        reference_speed = HARD_BRAKE;
      }

      Speed_Control(reference_speed);
      break;

    case ACC:
      float ultraSonic_data = avgDist;
      //Sensor_read(ultraSonic_data, sample_time);    // Updates Rel_Velocity and Rel_position
      float ACC_Speed = Position_Control(ultraSonic_data);
      if (ultraSonic_data < DIST_TO_OBSTACLE_LO)  
      {
        ACC_Speed = HARD_BRAKE;
      }
      else
      {
        //ACC_Speed = speed_command_arbitration(ACC_Speed, raspi_speed);
      }
      Speed_Control(ACC_Speed);
  }
  acc_task_speed = micros() - acc_task_start; 
}

void Mode_Selector()
{
  //Serial.println("Mode_Selector() ");
  if(manual_flag)               // Manual Control
  {
    CAR_MODE = MANUAL;     
  } 
  else if(avgDist > DIST_TO_OBSTACLE_HI)   
  {
    CAR_MODE = CRUISE_CONTROL;    
  }
  else if(avgDist < DIST_TO_OBSTACLE_HI)  
  {
    CAR_MODE = ACC;   // 
  }
  else
  {
    CAR_MODE = CRUISE_CONTROL; // Default CC Mode
  }
}

float feedforward_pwm(float reference_speed, uint8_t ff_arr_len)
{
  float ff_pwm;
  if (reference_speed < ff_target_speed[0])
    {
      ff_pwm = ff_target_PWM[0];
    }
  else if (reference_speed > ff_target_speed[ff_arr_len - 1])
    {
      ff_pwm = ff_target_PWM[ff_arr_len - 1];
    }
  else
  {
    for (int i = 0; i < (ff_arr_len - 1); i++)
    {
      float pwm_slope = 0;
      if ( abs(reference_speed - ff_target_speed[i]) <= 0.01 ) //&& (reference_speed < (ff_target_speed[i]+10)) ) // CHECK 1 - CONDITION STATEMENT
      {
        ff_pwm = ff_target_PWM[i];
        break;
      }

      if ( (reference_speed > ff_target_speed[i]) && (reference_speed < ff_target_speed[i + 1]) ) //&& (reference_speed < (ff_target_speed[i]+10)) )
      {
        pwm_slope = (ff_target_PWM[i + 1] - ff_target_PWM[i]) / (ff_target_speed[i + 1] - ff_target_speed[i]);
        ff_pwm = pwm_slope * reference_speed + (ff_target_PWM[i] - pwm_slope * ff_target_speed[i]);      // y = mx + C
        break;
      }
    }
  }
  return ff_pwm;
}

void Speed_Control(float reference_speed)
{
  REF_Speed = reference_speed;  // BROADCAST
	float speed_error = 0.0F,	prop_term = 0.0F, ff_pwm = 1500.0F, fb_pwm = 0.0F;
	static float intgl_term = 0, prev_error = 0;
	static boolean intgl_wind_up = false;
	float current_speed = encoder_speed_feedback();
  CUR_Speed = current_speed;    // BROADCAST
	static uint8_t ff_arr_len = sizeof(ff_target_PWM) / sizeof(float);
	ff_pwm = feedforward_pwm(reference_speed, ff_arr_len);
  FF_PWM = ff_pwm;    // BROADCAST
  
	speed_error = reference_speed - current_speed;
	if ( (speed_error < 1.0F) && (speed_error > -1.0F)) 
  {
    speed_error = 0.0F;
  }

  prop_term = speed_error * KP;
  
  if (!intgl_wind_up)
  {
    intgl_term += speed_error * KI * DELTA_TIME;
  }
  
  fb_pwm = prop_term + intgl_term;

if (fb_pwm > PI_POS_SAT)
  {
    // fb_pwm is positively saturated
    fb_pwm = PI_POS_SAT;
    if (speed_error > 0.0F) // checking if error term is positive
    {
      intgl_wind_up = true;
    } 
    else 
    {
      intgl_wind_up = false;
    }
  }
  else if (fb_pwm < PI_NEG_SAT)  // rarely in this condition
  {
    // fb_pwm is negatively saturated
    fb_pwm = PI_NEG_SAT;
    if (speed_error < 0.0F) // checking if error term is negative
    {
      intgl_wind_up = true;
    } 
    else 
    {
      intgl_wind_up = false;
    }
  }
  else 
  {
    intgl_wind_up = false;
  }
  
  FB_PWM = fb_pwm;    // BROADCAST
  
	motor_PWM = ff_pwm + fb_pwm;
	if ( (reference_speed > 0.0F) && (motor_PWM > MOTOR_PWM_MAX) )
	{
		motor_PWM = MOTOR_PWM_MAX;
	}
	else if ( (reference_speed < 0.0F) && (motor_PWM < MOTOR_PWM_MIN) )
	{
		motor_PWM = MOTOR_PWM_MIN;
	}
	else
	{
		// DO NOTHING 
	}

  if(CAR_MODE == CRUISE_CONTROL)
  {
    motor_PWM = constrain(motor_PWM, 1500, 2000);
  }
  else if((CAR_MODE == ACC) && reference_speed > 10)
  {
    motor_PWM = constrain(motor_PWM, 1450, 2000);
  }
  
	prev_error = speed_error;

  MTR_PWM = motor_PWM;    // BROADCAST
 
	setDrive(motor_PWM, 0);
  // setDrive(1625, 0);
}

// Function copied over from old code. Not used now. 
void Sensor_read(float ultraSonic_data, float sample_time)
{   
  // Need to think about sample_time 
	// Here the term Rel means relative positions and relative velocity w.r.t to the detected obstacle
	static float rel_vel_old[] = { 0.0F, 0.0F };
	static float rel_pos_old = 0.0F;

	uint8_t rel_vel_filter[REL_VEL_FIL_LEN] = { 1, 1, 1 };
	float rel_velocity = 0;
   
  rel_pos_old = Rel_Pos;
 	Rel_Pos = ultraSonic_data;		// New uS data

	rel_vel_old[1] = rel_vel_old[0];
	rel_vel_old[0] = rel_velocity;
  rel_velocity = (Rel_Pos - rel_pos_old) / sample_time;  //cm/10ms = m/s
  	
	rel_velocity = (rel_vel_filter[0] * rel_velocity + rel_vel_filter[1] * rel_vel_old[0] + 
				          rel_vel_filter[2] * rel_vel_old[1]) / REL_VEL_FIL_LEN;
}

float speed_command_arbitration(float ACC_Speed, float pi_speed_cmd)
{
	return min(ACC_Speed, pi_speed_cmd);
}

float Position_Control(float ultraSonic_data)
{
  float Pos_Kp = CRUISE_VELOCITY/(DIST_TO_OBSTACLE_HI - DIST_TO_OBSTACLE_LO);	// CRUISE_VELOCITY = 60 in cm/s, e.g. P_Kp = 60/(50-30) = 3
	// when the car is approaching the Lower threshold the ref speed will proportionally go down till 0
	return Pos_Kp * (ultraSonic_data - DIST_TO_OBSTACLE_LO);
}

void setDrive(int us, int dly) 
{

  //Serial.println("setDrive() ");
  if (us <= 180)
    us = map(us,0,180,1000,2000);
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = us;
  
//  throttleChannel.setDuty(us);
  delay(dly);
}
