#include "Lane_control.h"

void Lane_Keep_Handler() 
{
  if (manual_flag) 
  {
	// setSteer(SET_STEER, 0);
  } 
  else 
  {
     serialEvent();
  }
}

void serialEvent() 
{
  if (Serial.available() && byte_index < 14)
  {
    IncomingByteTmp = Serial.read();
    switch (IncomingByteTmp) 
    {
      case SOP:
        byte_index = 0;
        break;

      case EOP:
      	char PacketType;
        PacketType = Pi_SerialData[0];		// First element in buffer is Packet type Info
        PacketLength = byte_index;
        Pi_SerialData[byte_index] = '\0';

        value = atoi(Pi_SerialData + 1);	// Second element is the raw value

        //Right now this function runs the same rate as serialEvent() in 1ms, we need to be cautious
        driveHandler(PacketType, value);
        break;

      default:
        Pi_SerialData[byte_index] = IncomingByteTmp;
        byte_index = byte_index + 1;
    }
  }
  else if(byte_index >= 14) 
  { /*No char on the bus*/
    byte_index = 0;
  }
}

// Handle Raspi commands

void driveHandler(char packetType, int value) 
{
  switch (packetType) 
  {
    //not used since PI sends 'D' - Drive
    // We can also map the RasPIs PWM command to cm/sec here itself
    case 'D': // Forward
      	if (value > 50 && value < 150) 
      	{ //honored speed command
        	pispeed_cmd = value;
      	}
      	else 
      	{	// What if the 
        	pispeed_cmd = 120; //Invalid speed command (almost stoped, set to max)
      	}
      	break;

    case 'B':
		//Serial.print("Brake: "); Serial.println(value);
		if (value > 50) 
		{            // Demo brake function. not best implementation
		// Then brake                                       // Brake range: [    0, 100 ]
		brake_flag = true;
		}
		else 
		{
		brake_flag = false;
		}
		break;

    case 'S':
		//Serial.print("Steer: "); Serial.println(value);
		//setSteer(1500 + (500/30) * value , 0);                // Steer range: [  -30,  30 ]

		str_buf[0] = str_buf[1];
		str_buf[1] = str_buf[2];
		str_buf[2] = value;

		setSteer(90 + (90 / 30 * value) , 0);              // Steer range: [  -30,  30 ]
		if( abs(str_buf[2] - str_buf[1]) < ULTRASONIC_ARB )           // condition to select left or right ultrasonic
		{
		  steer_cmd_pi = str_buf[2];
		}
		else
		{
		  steer_cmd_pi = (str_buf[0] + str_buf[1] + str_buf[2])/3;
		}

		/* Recommended to avoid steering JITTER
		setSteer(90 + (90 / 30 * steer_cmd_pi) , 0);
		*/
		break;
    default:
      ;
      //Serial.print("Error: Unrecognized command: "); Serial.println(packetType);
  }
}
