#include "pins.h"

const int PWM_FREQ = 5000;  //5 kHz

float encoder_speed_feedback(); // Car speed calc

void drive_cmd(int,uint8_t);
void steer_cmd(int);
void drive_setup(void);
int steering_feedback();
void Drive_Control(void);
