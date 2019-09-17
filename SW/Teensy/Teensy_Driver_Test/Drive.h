#include "pins.h"

const int PWM_FREQ = 5000;  //5 kHz

void drive_cmd(int,uint8_t);
void steer_cmd(int);
void drive_setup(void);
int steering_feedback();
