#include <arduino.h>
#include "Lane_Control.h"

float PI_Steer_Raw;

void Lane_Keep_Handler() 
{
   db_lane_keep = 1;
   serialEvent();
}

void setSteer(int us, int dly) 
{
  //Serial.println("setSteer() ");
  //steeringChannel.setDuty(us);
  //myservo_steer.write(us);

  // Constants
  if (us <= 180)
    us = map(us,0,180,540,2380);
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = us;
  delay(dly);
}
void serialEvent() 
{
  db_serial_event = 1;
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
        PacketType = Pi_SerialData[0];    // First element in buffer is Packet type Info
        PacketLength = byte_index;
        Pi_SerialData[byte_index] = '\0';

        value = atoi(Pi_SerialData + 1);  // Second element is the raw value

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
  //Serial.println("driveHandler() ");
  switch (packetType) 
  {
    //not used since PI sends 'D' - Drive
    // We can also map the RasPIs PWM command to cm/sec here itself
    case 'D': // Forward
        /*
        if (value > 50 && value < 150) 
        { //honored speed command
          pi_speed_cmd = value * 1.1;
        }
        else 
        { // What if the 
          pi_speed_cmd = 130; //Invalid speed command (almost stoped, set to max)
        }*/
      	break;

    case 'B':
		//Serial.print("Brake: "); Serial.println(value);
		if (value > 50) 
		{            // Demo brake function. not best implementation
		// Then brake                                       // Brake range: [    0, 100 ]
		pi_brake_flag = true;
		}
		else 
		{
		pi_brake_flag = false;
		}
		break;

    case 'S':
		//Serial.print("Steer: "); Serial.println(value);
		//setSteer(1500 + (500/30) * value , 0);                // Steer range: [  -30,  30 ]
    pi_speed_cmd = interp_1d(K_vcmd_x,K_vcmd_y,5,abs(value-4))* 0.975 * CRUISE_VELOCITY * 2;

		str_buf[0] = str_buf[1];
		str_buf[1] = str_buf[2];
		str_buf[2] = value;

    PI_Steer_Raw = value;

   /* Offset addition to fix steering deviation from mid-point */
   value +=-3.5;

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


float interp_1d(const int *array_x,const float *array_y,unsigned int size,unsigned int value){

  unsigned int index = value/(array_x[size-1]/(size-1)); /*locate index of value, use integer math to cut residue*/

  if ( index <0 ){
    return array_y[0]; /*No extropolation beyond the min*/
  }
  else if( index < (size-1)){
    return (    array_y[index] + (array_y[index+1]-array_y[index])/(array_x[index+1]-array_x[index])*(value-array_x[index])   ); /*interpolation*/
  }
  else{
    return array_y[size-1];/*No extropolation beyond the max*/
  }
  
}
