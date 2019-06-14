
// Battery protection variables
#define NUM_SAMPLES 100
int batt_sum = 0;                    // sum of samples taken
unsigned char sample_count = 0; // current sample number
float batt_voltage = 0.0;            // calculated voltage
float batt_voltage_raw = 0.0;            // calculated voltage raw
float batt_limit = 6.8;
unsigned char batt_call = 0U;

float test;


void battery_feedback()
{
    // take a number of analog samples and add them up
    if(sample_count < NUM_SAMPLES) 
    {
        batt_sum += analogRead(P_BATLEVEL);
        sample_count++;
        //delay(5);
    }
    else
    {
      // calculate the voltage
      // use 5.0 for a 5.0V ADC reference voltage
      // 5.015V is the calibrated reference voltage
      batt_voltage_raw = ((float)batt_sum / (float)NUM_SAMPLES * 3.3) / 1024.0;
    
      batt_voltage = batt_voltage_raw * 2.67;
      
      sample_count = 0;
      batt_sum = 0;
         // Serial.print(batt_voltage);
          //Serial.print(" ");
      
      // send voltage for display on Serial Monitor
      // voltage multiplied by 11 when using voltage divider that
      // divides by 11. 11.132 is the calibrated voltage divide
      // value
      //Serial.println((float)batt_sum / (float)NUM_SAMPLES);

      //Serial.print(batt_voltage);
      //Serial.println (" V");
    }
  




  
  


    
}
