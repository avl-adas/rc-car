#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

const int pos_uS_delta = 10;

//Ultrasonic_sensor
class Ultrasonic{
private:
    const int TRIG_OFFSET = 0;
    const int ECHO_OFFSET = 1;
    long distance;
    int lastDist[3] = { 201, 201, 201};
// Base pin number for HC SR04 sensors (added with TRIG and ECHO OFFSETs above)
    int ultrasonicPin;

// Distance at which automatic braking starts
     int brakeDist;
    unsigned long dur_ultraSonic_pulse;
    Semaphore sem_PWM;
    int avg_uS = 0;
public: 
    Ultrasonic(int pin, Semaphore &sem_PWM_in);
    int readDistance();
    int getPin();
    int getDistance();
    int getAverageDistance();
    int getEchoOffset();
    void set_pulse_dur(unsigned long);
};


//Constructor
 Ultrasonic::Ultrasonic(int pin, Semaphore &sem_PWM_in){
  
   ultrasonicPin = pin;
   sem_PWM = sem_PWM_in;
  
   pinMode(TRIG_OFFSET + pin, OUTPUT);
   pinMode(ECHO_OFFSET + pin, INPUT);
 
 }

//Main run method from original program
  int Ultrasonic::readDistance() {
  long duration;
  while (sem_PWM == HELD) Serial.println(sem_PWM);
  noInterrupts();
  sem_PWM = HELD;
  interrupts();

//  noInterrupts();
  digitalWrite(TRIG_OFFSET + ultrasonicPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(TRIG_OFFSET + ultrasonicPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(12); // Added this line
  digitalWrite(TRIG_OFFSET + ultrasonicPin, LOW);
  // int duration2 = pulseIn(ECHO_OFFSET + ultrasonicPin, HIGH,10000);

  duration = dur_ultraSonic_pulse;
  distance = (duration/2) / 29.1;

  noInterrupts();
  sem_PWM = FREE;
  interrupts();
}

void Ultrasonic::set_pulse_dur(unsigned long dur){
  dur_ultraSonic_pulse = dur;
}


//Returns ultrasonics pin number
int Ultrasonic::getPin(){
  return ultrasonicPin;
}

int Ultrasonic::getEchoOffset()
{
  return ECHO_OFFSET;
}

int Ultrasonic::getDistance(){

  return distance;
}

int Ultrasonic::getAverageDistance(){
    int tempDist = getDistance();

    if (tempDist < 2)
    {
      tempDist = 2;
    }
    else if(tempDist > 80)
    {
      tempDist = 80 + 5;
    }

    // Store last 3
    lastDist[0] = lastDist[1];
    lastDist[1] = lastDist[2];
    
  
    if ((tempDist > lastDist[2]) && (tempDist - avg_uS > pos_uS_delta))
    {
      lastDist[2] = avg_uS + pos_uS_delta;   
    }
    else
    {
      lastDist[2] = tempDist;
    }

    avg_uS = (lastDist[0] + lastDist[1] + lastDist[2])/3;
    // average
  return (lastDist[0] + lastDist[1] + lastDist[2])/3;
    
}

#endif
