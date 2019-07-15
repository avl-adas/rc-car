#ifndef _WIRELESS_COMM_H_
#define _WIRELESS_COMM_H_

#include "wireless_comm.h"

void wireless_comm_setup()
{
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(80); 
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setAutoAck(0);
  radio.stopListening();
  //  disableWatchDog();
  //  wdt_disable();
}

void wireless_communication()
{
  payload[0] = (int)(avgDistFL);
  payload[1] = (int)(avgDistF);
  payload[2] = (int)(avgDistFR);
  payload[3] = (int)(motor_PWM);
  payload[4] = (int)(v_cmd);
  payload[5] = (int)(batt_voltage*100);
  radio.writeFast( &payload, payloadSize); //WARNING FAST WRITE
  //when using fast write there are three FIFO buffers.
  //If the buffers are filled the 4th request will become blocking.
  //Ensure Fast write is not called too quickly (around 1 ms)
}

#endif