#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include "Semaphore.h"

// //Ultrasonic_sensor
class Ultrasonic
{
  private:
      // Base pin number for HC SR04 sensors (added with TRIG and ECHO OFFSETs above)
      uint8_t ultrasonicTPin;
      uint8_t ultrasonicEPin;
      float distance;
      float lastDist[3] = { 201, 201, 201};
      float avg_uS = 0;

      // Distance at which automatic braking starts
      int brakeDist;
      unsigned long dur_ultraSonic_pulse;
      Semaphore sem_PWM;
      
  public: 
      Ultrasonic(uint8_t pinT,uint8_t pinE, Semaphore &sem_PWM_in);
      uint8_t readDistance();
      uint8_t getTPin();
      uint8_t getEPin();
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
