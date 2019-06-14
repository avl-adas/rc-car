/**
 * Setup pins for hardware
 */
#define PCBCAR

#ifdef  PCBCAR
  
  #define P_BATLEVEL   A0 //
  #define P_ESTOP      A1
  #define P_LED_OK     A2
  #define P_LED_SPR    A3
  #define P_ENCODER_A  A4 //
  #define P_ENCODER_B  A5 //
  #define P_ENCODER_I  A6 //
  #define P_CE         A7 //
  #define P_CSN        A8 //
  #define P_BTN_SPR    A11
  #define P_US_RR_T    66 //
  #define P_US_RR_E    49 //
  #define P_US_B_T     67 //
  #define P_US_B_E     48 //
  #define P_US_RL_T    25 //
  #define P_US_RL_E    50 //
  #define P_US_F_T     6 //
  #define P_US_F_E     51 //
  #define P_US_FR_T    5  //
  #define P_US_FR_E    52 //
  #define P_US_FL_T    4  //
  #define P_US_FL_E    53 //
  #define P_US_S_T     3  //
  #define P_US_S_E     46 //
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
