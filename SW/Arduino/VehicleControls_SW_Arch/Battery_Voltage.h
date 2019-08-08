#ifndef _BATTERY_VOLTAGE_H_
#define _BATTERY_VOLTAGE_H_

// Battery protection variables
#define NUM_SAMPLES 100
const int BATTERY_INPUT = A4;
const uint8_t LOW_BATTERY_INDICATOR = 37;
extern const float BATTERY_MAX_VOLTAGE = 7.4F;
extern const float BATTERY_LOW_THRESHOLD = 6.8F;

int batt_sum = 0;                    // sum of samples taken
uint8_t sample_count = 0;            // current sample number
uint8_t batt_call = 0U;
extern float batt_voltage = 0.0;            // calculated voltage
float batt_voltage_raw = 0.0;        // calculated voltage raw

void battery_feedback()
{
    // take a number of analog samples and add them up
    if(sample_count < NUM_SAMPLES) 
    {
        batt_sum += analogRead(BATTERY_INPUT);
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
#endif
