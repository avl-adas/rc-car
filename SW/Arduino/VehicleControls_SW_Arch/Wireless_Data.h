#include "RF24.h"
extern float avgDist;
extern uint8_t CAR_MODE;
extern float REF_Speed;
extern float CUR_Speed;
extern float FF_PWM;
extern float FB_PWM;
extern float MTR_PWM;
extern unsigned long acc_task_speed;
extern int pi_speed_cmd;
extern int value;
extern boolean pi_brake_flag;
extern float batt_voltage;

//Wireless Communication
const byte CE = 27;
const byte CSN = 29;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(CE, CSN);
byte addresses[][6] = {"1Node", "2Node"};
// Used to control whether this node is sending or receiving
int payload[6] = {0};
int payloadSize = sizeof(payload);
uint16_t WIRELESS_COMM_COUNT = 1000;
