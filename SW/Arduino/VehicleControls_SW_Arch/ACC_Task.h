#ifndef _ACC_TASK_H_
#define _ACC_TASK_H_

#define MANUAL 0
#define CRUISE_CONTROL 1
#define ACC 2

uint8_t CAR_MODE = 1;		// By default we run in CRUISE_CONTROL mode

int ultrasonic_ct = 0;
// PI Controller Variables
const float KP = 1.0F;
const float KI = 1.0F;
const float PI_POS_SAT =  100.0F;
const float PI_NEG_SAT = -100.0F;
const float MOTOR_PWM_MAX = 1750.0F;
const float MOTOR_PWM_MIN = 1250.0F;
float DELTA_TIME = 0.025F;

// Speed Control
float motor_PWM = 0.0F;
float V_CL_TRGT = 60.0F;
float adjust = 0.0F;


//float ff_target_speed[11] = {-30.0F, -20.0F, -10.0F, 0.0F, 10.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F};          // in cm/sec
//float ff_target_PWM[11] = {1422.0F-adjust, 1430.0F-adjust, 1440.0F, 1500.0F, 1560.0F, 1578.0F+adjust, 1583.0F+adjust, 1590.0F+adjust, 1595.0F+adjust, 1600.0F+adjust, 1610.0F+adjust};     // pulse width in uS, [1000 2000]

float ff_target_speed[12] = {-30.00F,  -20.00F,  -10.00F,  0.00F,  10.00F, 20.00F, 30.00F, 40.00F, 50.00F, 60.00F, 70.00F, 80.00F};
float ff_target_PWM[12] = {1423.00F-adjust , 1430.00F-adjust,  1440.00F, 1500.00F, 1560.00F, 1570.00F+adjust, 1577.00F+adjust, 1585.00F+adjust, 1592.00F+adjust, 1600.00F+adjust, 1615.00F+adjust, 1630.00F+adjust};     // pulse width in uS, [1000 2000]


// ACC variables
const uint8_t REL_VEL_FIL_LEN = 3;
float Rel_Pos = 0.0F;
const float DIST_TO_OBSTACLE_HI = 50.0F;
extern const float DIST_TO_OBSTACLE_LO = 30.0F;
extern const float CRUISE_VELOCITY = 75.0F; // in cm/sec
const float HARD_BRAKE = 0.0F; 	 // in cm/sec

void Speed_Control(float );
void ACC_Func_Handler();
// void ACC_function(float , int sample_time, bool pi_brake_flag);

//Undecided
void Sensor_read(float , float );
void Mode_Selector();
float speed_command_arbitration(float , float );
float Position_Control(float );
void setDrive(int , int );
float feedforward_pwm(float , uint8_t );
#endif
