#include "ACC_Task.h"
#include "Ultrasonic.h"

void Speed_Control(float reference_speed)
{
	float speed_error = 0,	prop_term = 0;
	static float intgl_term = 0, prev_error = 0;
	static bool intgl_wind_up = false;
	float current_speed = encoder_speed_feedback();
	uint8_t ff_arr_len = sizeof(ff_target_PWM) / sizeof(float);
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
	    for (int i = 0; i < (ff_arr_len - 2); i++)
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

  	battery_feedback();
  	// *************** CHECK ************** //
  	if (batt_voltage > BATTERY_LOW_THRESHOLD) 
	{
	   ff_pwm = max(min(BATTERY_MAX_VOLTAGE / batt_voltage, 1.2), 1) * ff_pwm; // A&M
	}
	else 
	{
	   //IDEALLY E-STOP COMMAND SHOULD BE GIVEN HERE TO PREVENT BATTERY DRAIN
	}
	ff_pwm = max(1440.0F, min(ff_pwm, 1660.0F));
  	// CHECK 3 - 1440 CORRESPONDS TO -10 CM/SEC...WHICH MEANS WE CANT GO LOWER THAN THAT BASED ON THIS LIMIT
  	// ***************************************

	speed_error = reference_speed - current_speed;
	if ( (speed_error < 2.0F) && (speed_error > -2.0F)) 
    {
      speed_error = 0.0F;
    }

    // dt = micros() - dt;
    prop_term = speed_error * KP;
    if (intgl_wind_up)
    {	// Either disable Integral control completely or reduce the term 
    	 intgl_term = 0;
    	 // intgl_term -= speed_error * KI; // * delta_time;
    }
    else 
    {
    	intgl_term += speed_error * KI; // * delta_time;
    }

    fb_pwm = prop_term + intgl_term;

    if (fb_pwm > PI_POS_SAT)
  	{
    	fb_pwm = PI_POS_SAT;
    	// if the fb_pwm is saturated and the error is still building up in positive
    	// we need to disable the Integral part of the controller
      intgl_wind_up = false;
    	if (speed_error > prev_error)
    	{
    		intgl_wind_up = true;
    	}
  	}
  	else if (fb_pwm < PI_NEG_SAT)
  	{
    	fb_pwm = PI_NEG_SAT;
    	// if the fb_pwm is saturated and the error is still building up in negative
    	// we need to disable the Integral part of the controller
    	if (speed_error < prev_error)
    	{
    		intgl_wind_up = true;
    	}
  	}
  	else 
  	{
  		intgl_wind_up = false;
  	}

  	motor_PWM = ff_pwm + fb_pwm;
  	if ( (reference_speed > 0) && (motor_PWM > MOTOR_PWM_MAX) )
  	{
  		motor_PWM = MOTOR_PWM_MAX;
  	}
  	else if ( (reference_speed < 0) && (motor_PWM < MOTOR_PWM_MIN) )
  	{
  		motor_PWM = MOTOR_PWM_MIN;
  	}
  	else
  	{
  		// DO NOTHING 
  	}
  	prev_error = speed_error;
  	setDrive(motor_PWM, 0);
}

void ACC_Func_Handler() 
{ 	//running every 25ms
	ultrasonic_distances();			// Minimum distance arbitration
	Mode_Selector();				// Updates CAR_MODE
	float raspi_speed = pi_speed_cmd;		// Using a local variable for data integrity
	raspi_speed = map(raspi_speed, -120.0F, 120.0F, -60.0F, 60.0F);
	switch (CAR_MODE)
	{
		case MANUAL: break;

		case CRUISE_CONTROL: 
			
			/*
			if (raspi_speed > 50.0F)
			{	
				raspi_speed = map(raspi_speed, 50.0F, 150.0F, 10.0F, 60.0F); 	// PI can command from 50 to 150 PWM raw value
			}
			else if (raspi_speed < -50.0F)
			{
				raspi_speed = map(raspi_speed, 50.0F, 150.0F, 10.0F, 60.0F); 	// PI can command from 50 to 150 PWM raw value				
			}
			*/

			// Lateral control also sends Brake flag command. Need to incorporate that
			float reference_speed = min(CRUISE_VELOCITY, raspi_speed);
			Speed_Control(reference_speed);
			break;

		case ACC:
			float ultraSonic_data = avgDist;
			Snsr_Proc(ultraSonic_data, sample_time);		// Updates Rel_Velocity and Rel_position
			float ACC_Speed = Position_Control(ultraSonic_data);
			if (ultraSonic_data < DIST_TO_OBSTACLE_LO)	
			{
				ACC_Speed = HARD_BRAKE;
			}
			else
			{
				ACC_Speed = speed_command_arbitration(ACC_Speed, raspi_speed);
			}
			Speed_Control(ACC_Speed);
	}
  // ##################### CHECK #############  
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

void Snsr_Proc(float ultraSonic_data, int sample_time)
{    
	// Here the term Rel means relative positions and relative velocity w.r.t to the detected obstacle
	static float rel_vel_old = { 0.0F, 0.0F };
	static rel_pos_old = 0.0F;

	uint8_t rel_vel_filter[REL_VEL_FIL_LEN] = { 1, 1, 1 };
	float rel_velocity = 0;
   
  	rel_pos_old = Rel_Pos;
 	Rel_Pos = ultraSonic_data;		// New uS data

	rel_vel_old[1] = rel_vel_old[0];
	rel_vel_old[0] = Rel_Vel;
  	rel_velocity = (Rel_Pos - rel_pos_old) / sample_time;  //cm/10ms = m/s
  	
	rel_velocity = (rel_vel_filter[0] * rel_velocity + rel_vel_filter[1] * rel_vel_old[0] + 
				    rel_vel_filter[2] * rel_vel_old[1]) / REL_VEL_FIL_LEN;
}

void Mode_Selector()
{
  if(manual_flag)               // Manual Control
  {
    CAR_MODE = 0; 		// MANUAL Mode
  } 
  else if(avgDist > DIST_TO_OBSTACLE_HI)   
  {
   	CAR_MODE = 1;		// CRUISE_CONTROL Mode
  }
  else if(avgDist < DIST_TO_OBSTACLE_HI)  
  {
    CAR_MODE = 2;		// ACC Mode
  }
}

float speed_command_arbitration(float ACC_Speed, float pi_speed_cmd)
{
	return 
}

float Position_Control(float ultraSonic_data)
{
  	float Pos_Kp = CRUISE_VELOCITY/(DIST_TO_OBSTACLE_HI - DIST_TO_OBSTACLE_LO);	// CRUISE_VELOCITY = 60 in cm/s, e.g. P_Kp = 60/(50-30) = 3
	// when the car is approaching the Lower threshold the ref speed will proportionally go down till 0
	return Pos_Kp * (ultraSonic_data - d_thres_lo);
}


				// ACC speed 		
        /*			
float Cmd_To_PWM(float VelCmd, int set_vcruise, int pi_cmd_v)
{
  // pi_cmd_v is from 0-120 in PWM units. Need to scale it down to cm/sec for e.g. (pi_cmd_v/120 * max_speed (in cm/sec))
  int v_tmp = min(set_vcruise,pi_cmd_v);
  // Better implementation
  // v_cruise = min(set_vcruise,pi_cmd_v);
  v_cruise = v_tmp; // Assign v_cruise target for next loop, so that ACC command can pick it up
  
  if (VelCmd < 0)   // Negative/Reverse/Brake Command
  { // If negative speed command
    // Coast for 1 cycle
    // Brake hard for next 9 cycles
    // Keep Coasting
    VelCmd = -100;  // This intruction will always be overriden, might as well remove it
    if(Rev_Counter<1)
    {
      VelCmd = 0;
      Rev_Counter++;
    }
    else if(Rev_Counter<10)
    {
      Rev_Counter++;
    }
    else 
    {
      VelCmd = 0;
    }
  }
  else if(VelCmd>0)
  {
        Rev_Counter =0;   // reset rev counter for forward speed command
  }
  //VelCmd [-500 500]
  //return ( (float ( VelCmd*55) )/500 + 90);
  return VelCmd;     // changed on June 6 by sunny and rahul
  //PWM [35 145]
}
*/
