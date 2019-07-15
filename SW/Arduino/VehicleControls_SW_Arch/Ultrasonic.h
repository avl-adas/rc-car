#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

const uint8_t ULTRASONIC_FRONT = 48 ;//car 1 is 54; car 2 is 48
const uint8_t ULTRASONIC_LEFT = 56;
const uint8_t ULTRASONIC_RIGHT = 64;
const uint8_t pos_uS_delta = 10;

bool pinUltrasonicState;
bool pinUltrasonicState_r;
bool pinUltrasonicState_l;

// used in ACC_func and also wireless comm
float avgDistF;
float avgDistFL;
float avgDistFR;
float avgDist;

unsigned long tUltrasonicStart;
unsigned long tUltrasonicEnd;
unsigned long tUltrasonicStart_r;
unsigned long tUltrasonicEnd_r;
unsigned long tUltrasonicStart_l;
unsigned long tUltrasonicEnd_l;

//Ultrasonic_sensor
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
