// Function declarations 
void encoder_setup();  // PIN Initialization
void ultrasonic_setup();
void ACC_Func_Handler();
void Lane_Keep_Handler();

void PWM_SERVO_SETUP()
{
    // PWM set-up on pins D38 and D36 for channels 1 and 2 respectively
  REG_PMC_PCER1 |= PMC_PCER1_PID36;                  // Enable PWM 

  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);  // Set the PWM clock A rate to 2MHz (84MHz/42)
  

  PWM->PWM_CH_NUM[1].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;      // Enable dual slope PWM and set the clock source as CLKA
  PWM->PWM_CH_NUM[1].PWM_CPRD = 20000;                                // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz;
  PWM->PWM_CH_NUM[2].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;      // Enable dual slope PWM and set the clock source as CLKA
  PWM->PWM_CH_NUM[2].PWM_CPRD = 20000;                                // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz;
  
  REG_PWM_ENA = PWM_ENA_CHID2 | PWM_ENA_CHID1;                        //Enable PWM channels 1 and 2;
  
  delay(1);
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = 1500;        // Set initial PWM
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = 1500;  
  delay(3000);                                  // Give ESC time to reset after pins reset to low
  REG_PIOC_ABSR |= PIO_ABSR_P6 | PIO_ABSR_P4;   // Set the port C PWM pins to peripheral type B
  REG_PIOC_PDR  |= PIO_PDR_P6 | PIO_PDR_P4;     // Set the port C PWM pins to outputs
  delay(250);
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = 1500;        // Set the PWM duty cycle to center / 50% / 1500 
  PWM->PWM_CH_NUM[2].PWM_CDTYUPD = 1500;  
}
