#ifndef _DRIVE_H_
#define _DRIVE_H_


#include "PwmChannel.h"
#include "Ultrasonic.h"


class Drive{

private:
    void setSteer(int us, int dly);
    void brake (int dly);
    void setDrive(int us, int dly);
    
public:
    void Drive();
    void driveCircleRight();
    void driveCircleLeft();
    void driveThreePointLeft();
    void slalom();
    void autoForward();
    void autoReverse();
    

}

//Constructor

void Drive(){

  
}
void Drive::driveCircleRight(){

Serial.println("Step1");
    noInterrupts();
    PwmChannel.oldDutyOutput = 1000 / PwmChannel.PERIOD_SCALE;
    interrupts();
    delay(1000);
    Serial.println("Step2");
    noInterrupts();
    PwmChannel.oldDutyOutput = 1600 / PwmChannel.PERIOD_SCALE;
    interrupts();
    delay(3000);
    Serial.println("Step3");

    noInterrupts();
    PwmChannel.oldDutyOutput = PwmChannel.DUTY_IDLE;
    interrupts();
}

void Drive::driveCircleLeft(){

Serial.println("Step1");
    noInterrupts();
    PwmChannel.oldDutyOutput = 2000 / PwmChannel.PERIOD_SCALE;
    interrupts();
    delay(1000);
    Serial.println("Step2");
    noInterrupts();
    PwmChannel.oldDutyOutput = 1600 / PwmChannel.PERIOD_SCALE;
    interrupts();
    delay(3000);
    Serial.println("Step3");

    noInterrupts();
    PwmChannel.oldDutyOutput = DUTY_IDLE;
    interrupts();
}



void Drive::driveThreePointLeft(){
  // Forward, Back-Left, Forward-Right
    setSteer(1500, 100);
    setDrive(1600, 1000);
    setDrive(1000, 500);  // brake half second
    setDrive(1500, 1500); // idle
    setSteer(2000, 1100);
    setDrive(1400, 750 * 4 / 3 * 2.6);
    setDrive(1500, 0);
    setSteer(1000, 100);
    setDrive(1600, 750 * 1.2);
    setDrive(1500, 0);
    setSteer(1500, 0);
}

void Drive::slalom(){
    setDrive(1600, 0);
    setSteer(1000, 3000 / 7);   //RIGHT
    setSteer(2000, 750);  //LEFT
    setSteer(1500, 80);
    setSteer(1000, 750);  //RIGHT
    setSteer(1500, 80);
    setSteer(2000, 750);  //LEFT
    setSteer(1500, 80);
    setSteer(1000, 750);  //RIGHT
    setSteer(2000, 3000 / 9);  //LEFT
    setDrive(1500, 0);
    setSteer(1500, 0);
}

void Drive::autoForward(){
  //setSteer(1500, 0)
  unsigned long count = 0;
  const int COUNT_THRESH = 60;   // Threshold in cycles for braking to be enabled
  do {
    
    setDrive(1600, 0);
   
    // Increment count only if less than threshold
    count = (count > COUNT_THRESH ? count : count + 1);

   Serial.print("Forward ultrasonic: "); //Serial.println(distance);

    delay(5);
  } while(frontUltrasoinc.getAverageDistance() > DIST_BRAKE_FORWARD);

  // Brake only if driven forward long enough
  if (oldDutyOutput > DUTY_IDLE && count >= COUNT_THRESH)
    brake(1500);
}

void Drive::autoReverse(){
  //setSteer(1500, 0)
  setDrive(1500, 1100);
//  delay(2500);
  do {
    setDrive(1400, 0);
    Serial.print("Reverse ultrasonic: ");//Serial.println(distance);

    delay(5);
  } while(  backUltrasonic.getAverageDistance() >= DIST_BRAKE_REVERSE);
// TODO: Add brake function for reverse
  setDrive(1500, 0);
}



void Drive::setSteer(int us, int dly){
  noInterrupts();
  PwmChannel.oldDutyOutput = us / PwmChannel.PERIOD_SCALE;
  interrupts();
  delay(dly);
}

void Drive::brake (int dly){
  noInterrupts();
  PwmChannel.oldDutyOutput = 1000 / PwmChannel.PERIOD_SCALE;
  interrupts();
  delay(dly);
  noInterrupts();
  PwmChannel.oldDutyOutput = 1500 / PwmChannel.PERIOD_SCALE;
  interrupts();
}

void Drive::setDrive(int us, int dly){
  noInterrupts();
  PwmChannel.oldDutyOutput = us / PwmChannel.PERIOD_SCALE;
  interrupts();
  delay(dly);
}

#endif
