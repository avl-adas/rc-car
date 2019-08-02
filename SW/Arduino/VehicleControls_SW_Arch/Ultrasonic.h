#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include "Semaphore.h"

// //Ultrasonic_sensor
class Ultrasonic
{
  private:
      // Base pin number for HC SR04 sensors (added with TRIG and ECHO OFFSETs above)
      uint8_t ultrasonicPin;
      const uint8_t TRIG_OFFSET = 0;
      const uint8_t ECHO_OFFSET = 1;
      float distance;
      float lastDist[3] = { 201, 201, 201};
      float avg_uS = 0;

      // Distance at which automatic braking starts
      int brakeDist;
      unsigned long dur_ultraSonic_pulse;
      Semaphore sem_PWM;
      
  public: 
      Ultrasonic(uint8_t pin, Semaphore &sem_PWM_in);
      uint8_t readDistance();
      uint8_t getPin();
      uint8_t getEchoOffset();
      void set_pulse_dur(unsigned long);
      float getDistance();
      float getAverageDistance();   
};

void ultrasonic_setup();
void ultrasonicChange();
void ultrasonicChange_r();
void ultrasonicChange_l();
void ultrasonic_distances();

#endif
