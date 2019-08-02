#include <arduino.h>
#include "Ultrasonic.h"
#include "Ultrasonic_Data.h"


void ultrasonic_setup()
{
  attachInterrupt(digitalPinToInterrupt((frontUltrasonic.getPin()) + frontUltrasonic.getEchoOffset()), ultrasonicChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt((rightUltrasonic.getPin()) + rightUltrasonic.getEchoOffset()), ultrasonicChange_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt((leftUltrasonic.getPin()) + leftUltrasonic.getEchoOffset()), ultrasonicChange_l, CHANGE);
}
//Constructor
 Ultrasonic::Ultrasonic(uint8_t pin, Semaphore &sem_PWM_in)
 { 
   ultrasonicPin = pin;
   sem_PWM = sem_PWM_in;
   pinMode(TRIG_OFFSET + pin, OUTPUT);
   pinMode(ECHO_OFFSET + pin, INPUT);
 }

//Main run method from original program
 uint8_t Ultrasonic::readDistance() 
 {
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
  distance = (duration/2) / 29.1F;

  noInterrupts();
  sem_PWM = FREE;
  interrupts();
}

void Ultrasonic::set_pulse_dur(unsigned long dur)
{
  dur_ultraSonic_pulse = dur;
}


//Returns ultrasonics pin number
uint8_t Ultrasonic::getPin()
{
  return ultrasonicPin;
}

uint8_t Ultrasonic::getEchoOffset()
{
  return ECHO_OFFSET;
}

float Ultrasonic::getDistance()
{
  return distance;
}

float Ultrasonic::getAverageDistance()
{
    float tempDist = getDistance();
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
    
    // When the object is removed from path, the distance should increase gradually so that 
    // car doesn't take off immediately
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
    return avg_uS;
}


void ultrasonicChange()
{
  pinUltrasonicState = digitalRead(frontUltrasonic.getPin() + frontUltrasonic.getEchoOffset());
  if (pinUltrasonicState)
  {
    tUltrasonicStart = micros();
  }
  else
  {
    tUltrasonicEnd = micros() - tUltrasonicStart;
    frontUltrasonic.set_pulse_dur(tUltrasonicEnd);
  }
}

void ultrasonicChange_r()
{
  pinUltrasonicState_r = digitalRead(rightUltrasonic.getPin() + rightUltrasonic.getEchoOffset());
  if (pinUltrasonicState_r)
  {
    tUltrasonicStart_r = micros();
  }
  else
  {
    tUltrasonicEnd_r = micros() - tUltrasonicStart_r;
    rightUltrasonic.set_pulse_dur(tUltrasonicEnd_r);
  }
}

void ultrasonicChange_l()
{
  pinUltrasonicState_l = digitalRead(leftUltrasonic.getPin() + leftUltrasonic.getEchoOffset());
  if (pinUltrasonicState_l)
  {
    tUltrasonicStart_l = micros();
  }
  else
  {
    tUltrasonicEnd_l = micros() - tUltrasonicStart_l;
    leftUltrasonic.set_pulse_dur(tUltrasonicEnd_l);
  }
}

void ultrasonic_distances()
{
  frontUltrasonic.readDistance();
  rightUltrasonic.readDistance();
  leftUltrasonic.readDistance();
  avgDistFL = leftUltrasonic.getAverageDistance();
  avgDistFR = rightUltrasonic.getAverageDistance();
  avgDistF = frontUltrasonic.getAverageDistance();
  // steer_cmd_pi coming from Lateral control function
  if(steer_cmd_pi > 3)
  {
    avgDist = min(avgDistF, avgDistFL);
    if (avgDistFR <= (DIST_TO_OBSTACLE_LO-10))
        { avgDist = min(avgDist, avgDistFR);  }
  }
  else if(steer_cmd_pi < -3)
  {
    avgDist = min(avgDistF, avgDistFR);
    if (avgDistFL <= (DIST_TO_OBSTACLE_LO-10))
        { avgDist = min(avgDist, avgDistFL);  }
  }
  else
  {
    avgDist = min(min(avgDistFL,avgDistFR),avgDistF);
  }
}
