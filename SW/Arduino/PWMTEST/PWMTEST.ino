
/**
 * DUE Hardware based PWM PPM servo control demo.
 * 
 *  !!!PLACE CAR ON BOX WITH WHEELS OFF THE GROUND BEFORE RUNNING!!!
 *  
 * Connect Steering and drive control signal pins to D36 and D38.
 * System will slowly steer left/Right and go forwards and backwards.
 * 
 * Signal range:
 * 1000 is minimum
 * 1500 is center
 * 2000 is max
 */



/**
 *    // PWM pins listed on Table 38-2 for Atmel ATSAM3X8E 
      // datasheet available in www.atmel.com
      // NOTE: only it is possible to use one pin for each
      // PWM channel 
      PA8 , // PWM_CH0 -> RX0
      PB12, // PWM_CH0 -> SDA (SDA0)!
      PC3 , // PWM_CH0 -> D35
      PA21, // PWM_CH0 -> LED TX!
      PB16, // PWM_CH0 -> DAC1 (audio)
      PC2 , // PWM_CH0 -> D34

      PA19, // PWM_CH1 -> D42
      PB13, // PWM_CH1 -> SCL (SCL0)!
      PC5 , // PWM_CH1 -> D37
      PA12, // PWM_CH1 -> RX2
      PB17, // PWM_CH1 -> AI8
      PC4 , // PWM_CH1 -> D36 <--

      PA13, // PWM_CH2 -> TX2
      PB14, // PWM_CH2 -> D53 (SPISS)
      PC7 , // PWM_CH2 -> D39
      PA20, // PWM_CH2 -> D43
      PB18, // PWM_CH2 -> AI9
      PC6 , // PWM_CH2 -> D38 <--

      PA9 , // PWM_CH3 -> TX0
      PB15, // PWM_CH3 -> DAC0 (audio)
      PC9 , // PWM_CH3 -> D41
      PA0 , // PWM_CH3 -> CANTX
      PB19, // PWM_CH3 -> AI10
      PC8 , // PWM_CH3 -> D40

      PC20, // PWM_CH4 -> na
      PC21, // PWM_CH4 -> D9 (PWN)!

      PC19, // PWM_CH5 -> D44
      PC22, // PWM_CH5 -> D8 (PWN)!

      PC18, // PWM_CH6 -> D45
      PC23, // PWM_CH6 -> D7 (PWN)!

      PC24  // PWM_CH7 -> D6 (PWN)!
 */

 
int i = 1500;
int j = 1;
void setup() {
  PWM_SERVO_SETUP(); //setup PWM registers for due pins D36 and D38
}

void loop() {

  if (i >= 2000 || i <= 1000) j *= -1; //reverse counter
  
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = i;      //Set D36 PPM%
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = 3000-i; //Set D38 PPM%

  
  i += j;  //tick counter
  delay(3);
  }



void PWM_SERVO_SETUP()
{
    // PWM set-up on pins D38 and D36 for channels 1 and 2 respectively
  REG_PMC_PCER1 |= PMC_PCER1_PID36;                  // Enable PWM 
  REG_PIOC_ABSR |= PIO_ABSR_P6 | PIO_ABSR_P4;        // Set the port C PWM pins to peripheral type B
  REG_PIOC_PDR  |= PIO_PDR_P6 | PIO_PDR_P4;          // Set the port C PWM pins to outputs
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);  // Set the PWM clock A rate to 2MHz (84MHz/42)
  

  PWM->PWM_CH_NUM[1].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;     // Enable dual slope PWM and set the clock source as CLKA
  PWM->PWM_CH_NUM[1].PWM_CPRD = 20000;                               // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz;
  PWM->PWM_CH_NUM[2].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;     // Enable dual slope PWM and set the clock source as CLKA
  PWM->PWM_CH_NUM[2].PWM_CPRD = 20000;                               // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz;
  
  REG_PWM_ENA = PWM_ENA_CHID2 | PWM_ENA_CHID1;                       //Enable PWM channels 1 and 2;

  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = 1500;                             // Set the PWM duty cycle to center / 50% / 1500 
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = 1500;                             // Set the PWM duty cycle to center / 50% / 1500    
  delay(3000); // 3 seconds of neutral to arm??   
}
