//Wireless Communication
#include <SPI.h>
#include "RF24.h"
const byte CE = 27;
const byte CSN = 29;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(CE, CSN);

byte addresses[][6] = {"1Node", "2Node"};
// Used to control whether this node is sending or receiving
int payload[6] = {0};
static int payloadSize = sizeof(payload);
uint16_t WIRELESS_COMM_COUNT = 1000;

void wireless_comm_setup();
void wireless_communication();
