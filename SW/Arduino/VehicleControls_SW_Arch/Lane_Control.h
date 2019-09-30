#ifndef _LANE_CONTROL_H_
#define _LANE_CONTROL_H_

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

void Lane_Keep_Handler();
void setSteer(int , int );
void serialEvent();
void driveHandler(char packetType, int value);

extern boolean manual_flag;
extern boolean pi_brake_flag = false; 
#endif
