#ifndef _PINS_H_
#define _PINS_H_

/**
 * Setup pins for hardware
 */
#define PCBCAR

#ifdef  PCBCAR
  
  #define P_BATLEVEL   A0 //
  #define P_ESTOP      A1
  #define P_LED_OK     A2
  #define P_LED_SPR    A3
  #define P_ENCODER_A  A5 //
  #define P_ENCODER_B  41 //
  #define P_ENCODER_I  A6 //
  #define P_CE         A7 //
  #define P_CSN        A8 //
  #define P_BTN_SPR    A11
  
  #define P_US_RR_T    66 //Rear Right T
  #define P_US_RR_E    49 //Rear Right E
  #define P_US_B_T     67 //Back T
  #define P_US_B_E     48 //Back E
  #define P_US_RL_T    25 //Rear Left T
  #define P_US_RL_E    50 //Rear Left E
  #define P_US_F_T     45  //Front T
  #define P_US_F_E     51 //Front E
  #define P_US_FR_T    43 //Front Right T
  #define P_US_FR_E    52 //Front Right E
  #define P_US_FL_T    4  //Left Side T
  #define P_US_FL_E    53 //Left Side E
  #define P_US_S_T     3  //Right Side T
  #define P_US_S_E     46 //Right Side E
  #define P_STEER      11 //PWM_CH2 20, PortPin:D7 
  #define P_DRIVE      12 //PWM_CH4 21, PortPin:D8 
  #define P_REC_STEER  7 //
  #define P_REC_THRTL  8 // 
  #define P_REC_VR     9 //
  #define P_REC_SWITCH 10 //
  #define P_PI_GPIO    17 //
  #define P_PI_TX      18 //Serial 1
  #define P_PI_RX      19 //Serial 1
  #define P_IMU_SDA    20 //
  #define P_IMU_SCL    21 //

#endif
#endif
