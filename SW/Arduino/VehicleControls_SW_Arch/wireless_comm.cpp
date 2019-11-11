#include <arduino.h>
#include "Wireless_Comm.h"
#include "Wireless_Data.h"

void wireless_comm_setup()
{
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
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
  /*payload[0] = (int)(CAR_MODE);
  payload[1] = (int)(CUR_Speed);
  payload[2] = (int)(REF_Speed);
  payload[3] = (int)(PI_Speed);
  payload[4] = (int)(avgDist);
  payload[5] = (int)(avgDist);*/

  payload[0] = (int)(REF_Speed);
  payload[1] = (int)(CUR_Speed);
  payload[2] = (int)(PI_Speed);
  payload[3] = (int)(avgDistFL);
  payload[4] = (int)(avgDistF);
  payload[5] = (int)(avgDistFR);
  radio.writeFast( &payload, payloadSize); //WARNING FAST WRITE
  //when using fast write there are three FIFO buffers.
  //If the buffers are filled the 4th request will become blocking.
  //Ensure Fast write is not called too quickly (around 1 ms)

}
