
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
}
