#include "pins.h"

extern int steer_cmd_pi;

const uint8_t ULTRASONIC_FRONT = 48 ;//car 1 is 54; car 2 is 48
const uint8_t ULTRASONIC_LEFT = 56;
const uint8_t ULTRASONIC_RIGHT = 64;
const uint8_t extern pos_uS_delta = 10;
extern const float DIST_TO_OBSTACLE_HI;
extern const float DIST_TO_OBSTACLE_LO;

bool pinUltrasonicState;
bool pinUltrasonicState_r;
bool pinUltrasonicState_l;

// used in ACC_func and also wireless comm
float avgDistF;
float avgDistFL;
float avgDistFR;
float avgDist;

unsigned long tUltrasonicStart;
unsigned long tUltrasonicEnd;
unsigned long tUltrasonicStart_r;
unsigned long tUltrasonicEnd_r;
unsigned long tUltrasonicStart_l;
unsigned long tUltrasonicEnd_l;

// Semaphore
Semaphore sem_PWM = FREE;      // Semaphore to disable pwm output

// Ultrasonic Objects
Ultrasonic frontUltrasonic = Ultrasonic(P_US_F_T,P_US_F_E, sem_PWM);  // Front us on PCB used here
Ultrasonic rightUltrasonic = Ultrasonic(P_US_FR_T,P_US_FR_E, sem_PWM); // FR us on PCB used here
Ultrasonic leftUltrasonic = Ultrasonic(P_US_S_T,P_US_S_E, sem_PWM); // Right side us on PCB used here
