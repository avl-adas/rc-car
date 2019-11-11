#ifndef _LANE_CONTROL_H_
#define _LANE_CONTROL_H_

extern const float CRUISE_VELOCITY;
int db_lane_keep = 0;
int db_serial_event = 0;
// Lane Control
// Raspi - Arduino Serial Commands
const char SOP = '{';
const char EOP = '}';

char PacketType = 0;
char IncomingByteTmp = 0;

char Pi_SerialData[15];
uint8_t byte_index = 0;
uint8_t PacketLength = 0;
int value = 0;

int str_buf[3] = {0, 0, 0};   // Steering buffer
int steer_cmd_pi = 0;         // Used for Ultrasound arbitration
const uint8_t ULTRASONIC_ARB = 6;     // UltraSonic hysterisis 
extern int pi_speed_cmd = 200;

extern const float CRUISE_VELOCITY;

const int K_vcmd_x[5] =   {0,5,10,15,30};
const float K_vcmd_y[5] = {1.0,1.0,0.9,0.8,0.7};

void Lane_Keep_Handler();
void setSteer(int , int );
void serialEvent();
void driveHandler(char packetType, int value);
float interp_1d(const int * ,const float * ,unsigned int ,unsigned int);

extern boolean manual_flag;
extern boolean pi_brake_flag = false; 
#endif
