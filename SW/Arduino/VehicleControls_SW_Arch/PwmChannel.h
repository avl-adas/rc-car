#ifndef _PWM_CHANNEL_H_
#define _PWM_CHANNEL_H_
//#include <DueTimer.h>

//#include "Semaphore.h"

// PWM pulse widths +/- PWM_DEADZONE are ignored
const int PWM_DEADZONE = 30; // us
const int PWM_FUDGE = 25; // us

class PwmChannel{

private:
  struct PwmChannelData {
    int channel_InputPin;
    unsigned long microlow = 0; 
    unsigned long microhigh = 0;
    unsigned long microdiff = 0;

   
    float dutyRequest = 0;
    unsigned long newDutyRequest = 0;
    const unsigned long dutyTolerance = 100/ PERIOD_SCALE; 
    
    int pwmFudge = 0; // PWM_FUDGE for throttle
    int pwmDeadzone = 0;  // PWM_DEADZONE for throttle
    int pwmScaleFactor = 1; // 2 for throttle
    unsigned int heartbeat = 0; 
    
    //output
    unsigned long oldDutyOutput = 0;
    unsigned long oldDutyRequest = 0;
    unsigned long outputCounter = 0;
    
    int channel_OutputPin;
    unsigned long heartbeatCounter = 0;
    const unsigned int minHeartbeat = 2;
    const unsigned long heartbeatPeriod = (minHeartbeat + 1+2) * outputPeriod;
   
   //New Variables
    bool isThrottleObject = false;  // true for throttle channel, false otherwise
    bool deadmanUsed = isThrottleObject;
  };

  static Semaphore semPWM;

  const static int MAX_NUM_CHANNELS = 4;
  static PwmChannelData channelData[MAX_NUM_CHANNELS];
  static int numChannels;
  int channelNum;

  // Semaphore
  static Semaphore sem_PWM;
  
public:
  static const unsigned long PERIOD_SCALE = 50;
  static const unsigned long outputPeriod = 17020 / PERIOD_SCALE; // *10us
  static const unsigned long DUTY_IDLE = 1500 / PERIOD_SCALE;
  static const int MICRO_MAX = 2100; // PERIOD_SCALE;
  static const int MICRO_MIN = 900; // PERIOD_SCALE;

  // Deadman switch status
  static bool deadman;
  
  
  static bool override_completeAutonomous;
  static bool override_throttleForward;
  static bool override_throttleReverse;

  // Constructor
  PwmChannel(int pin, int motorIn, bool isThrottle, int fudge=0, int dead=0, int scale=1, bool deadmanUsedIn=false);

  // Initialize static variables. Must call this before using any other functionality
  static void initAndSetSemaphore(Semaphore &sem_PWM_in);
  
  static void timerISR();
  
  static void channelISR (int currChannel);
  static void channelISR0() {  channelISR(0);  }
  static void channelISR1() {  channelISR(1);  }
  static void channelISR2() {  channelISR(2);  }
  static void channelISR3() {  channelISR(3);  }

  void (*channelISR_arr[4])() = {channelISR0, channelISR1, channelISR2, channelISR3};

  void setDuty(int duty_ms);

  int getChannelNum() { return channelNum;  }
  int getMicroDiff()  { return channelData[channelNum].microdiff; }
  int getDutyOutput() { return channelData[channelNum].oldDutyOutput; }
};


// Define variables
int PwmChannel::numChannels = 0;
Semaphore PwmChannel::sem_PWM = FREE;
bool PwmChannel::deadman = false;
bool PwmChannel::override_completeAutonomous = false;
bool PwmChannel::override_throttleForward = false;
bool PwmChannel::override_throttleReverse = false;

PwmChannel::PwmChannelData PwmChannel::channelData[MAX_NUM_CHANNELS];

// Constructor
PwmChannel::PwmChannel(int pin, int motorIn, bool isThrottle, int fudge, int dead, int scale, bool deadmanUsedIn)
{
  channelNum = numChannels++;
  PwmChannelData* currChannel = (PwmChannelData*)(&channelData[channelNum]);

  currChannel->channel_InputPin = pin;
  currChannel->channel_OutputPin = motorIn;
  currChannel->isThrottleObject = isThrottle;
  currChannel->pwmFudge = fudge;
  currChannel->pwmDeadzone = dead;
  currChannel->pwmScaleFactor = scale;
  currChannel->deadmanUsed = deadmanUsedIn;

  pinMode(motorIn, OUTPUT);
  digitalWrite(motorIn, LOW);
}

void PwmChannel::initAndSetSemaphore(Semaphore &sem_PWM_in)
{
//  numChannels = 0;
  sem_PWM = sem_PWM_in;
//  deadman = false;
//  static bool override_completeAutonomous = false;
//  static bool override_throttleForward = false;
//  static bool override_throttleReverse = false;
  Timer1.setPeriod(PERIOD_SCALE);
  Timer1.attachInterrupt(timerISR); // attach the service routine here
  Timer1.start();
}


  
// Read Throttle/Steering channel
void PwmChannel::channelISR(int currChannel)    // Channel 1 = throttle, 2 = steering
{
//Serial.print("c");
  // Disable interrupts
  noInterrupts();

//  if (override_completeAutonomous == false ){ //&& throttleOverride == false){
    
    if (digitalRead(channelData[currChannel].channel_InputPin) == HIGH)
    {
        channelData[currChannel].microlow = micros();
        channelData[currChannel].heartbeat++;
    }
    else 
    {
      // Grab current time, increment hearbeat
      channelData[currChannel].microhigh = micros();
      channelData[currChannel].heartbeat++;
    
      // Calculate new positive pulse width
      channelData[currChannel].microdiff = channelData[currChannel].microhigh - channelData[currChannel].microlow + channelData[currChannel].pwmFudge;      
      //newDutyRequest = microdiff / PERIOD_SCALE;
      
      if (channelData[currChannel].microdiff > 1500 + channelData[currChannel].pwmDeadzone)
      {
        channelData[currChannel].newDutyRequest = ((((int)channelData[currChannel].microdiff - channelData[currChannel].pwmDeadzone) - 1500) / channelData[currChannel].pwmScaleFactor + 1500) / PERIOD_SCALE;

        // Deadman switch enable
        if (channelData[currChannel].deadmanUsed) deadman = true; 
      } 
      else if (channelData[currChannel].microdiff < 1500 - channelData[currChannel].pwmDeadzone)
      {
        channelData[currChannel].newDutyRequest = ((((int)channelData[currChannel].microdiff + channelData[currChannel].pwmDeadzone) - 1500) / channelData[currChannel].pwmScaleFactor + 1500) / PERIOD_SCALE;

        // Deadman switch disable
        if (channelData[currChannel].deadmanUsed) deadman = false; 
      }
      else
      {
        channelData[currChannel].newDutyRequest = DUTY_IDLE;

        // Deadman switch disable
        if (channelData[currChannel].deadmanUsed) deadman = false; 
      }

    // Check for directional override
    // Don't save result if forward override_completeAutonomous and trying to go forward
    if (override_completeAutonomous || channelData[currChannel].isThrottleObject && (override_throttleForward && channelData[currChannel].newDutyRequest > DUTY_IDLE
        || override_throttleReverse && channelData[currChannel].newDutyRequest < DUTY_IDLE)){
      interrupts(); // Reenable interrupts and bail
      return;
    }
    
    // Check for bounds 
    if (channelData[currChannel].microdiff > MICRO_MAX ||
        channelData[currChannel].microdiff < MICRO_MIN){
      channelData[currChannel].oldDutyOutput = channelData[currChannel].oldDutyRequest;
    }
    else
    {
      // Update output if new request is within range from previous request
      if (channelData[currChannel].newDutyRequest >= (channelData[currChannel].oldDutyRequest - channelData[currChannel].dutyTolerance) &&
          channelData[currChannel].newDutyRequest <= (channelData[currChannel].oldDutyRequest + channelData[currChannel].dutyTolerance))
      {
          channelData[currChannel].oldDutyOutput = channelData[currChannel].oldDutyRequest;
      }
    
      // Save new request as old
      channelData[currChannel].oldDutyRequest = channelData[currChannel].newDutyRequest;
    }
  }
//  }
// Re-enable interrupts
interrupts(); 
}




void PwmChannel::timerISR()
{
//Serial.print("t");
  int currDutyOutput; // current output duty cycle
  
  noInterrupts();
//Serial.println("1");
//Serial.println(numChannels);
  
  for (int currChannel = 0; currChannel < numChannels; currChannel++){
    
//Serial.print("2");
    int currDutyOutput;
    
    // Don't do anything if in autonmous mode and deadman == false
    if (override_completeAutonomous && deadman == false){
      currDutyOutput = DUTY_IDLE;
    } else {
      currDutyOutput = channelData[currChannel].oldDutyOutput;
    }
//Serial.print("3");
  
    // Check for turn off
    if (channelData[currChannel].outputCounter >= currDutyOutput){
      digitalWrite(channelData[currChannel].channel_OutputPin, LOW);
      //Serial.println(oldDutyOutput);
  
      sem_PWM = FREE;   // Free semaphore
    }
    
//Serial.print("4");
    // Check for turn on/reset
    if (channelData[currChannel].outputCounter >= outputPeriod)
    {
      if (sem_PWM == HELD){
        interrupts();
        return;
      }
      sem_PWM = HELD;   // Grab semaphore
      digitalWrite(channelData[currChannel].channel_OutputPin, HIGH);
      channelData[currChannel].outputCounter = 0;
    }
  
//Serial.print("5");
    
    // Increment counter
    
    channelData[currChannel].outputCounter++;
  
    // Heartbeat check, skip if autonmous
    if (override_completeAutonomous == false){
      channelData[currChannel].heartbeatCounter++;
      if (channelData[currChannel].heartbeatCounter >= channelData[currChannel].heartbeatPeriod){
        if (channelData[currChannel].heartbeat < (channelData[currChannel].minHeartbeat * 2)){
           channelData[currChannel].oldDutyOutput = channelData[currChannel].oldDutyRequest = DUTY_IDLE;// / 50;
           digitalWrite(channelData[currChannel].channel_OutputPin, LOW);
           Serial.print("H"); Serial.print(currChannel);
        }
        channelData[currChannel].heartbeatCounter = 0;
        channelData[currChannel].heartbeat = 0;
      }
    }
//Serial.print("6");
  }
//Serial.print("7");
  interrupts();
}

void PwmChannel::setDuty(int duty_ms){
    noInterrupts();
    channelData[channelNum].oldDutyOutput = duty_ms / PERIOD_SCALE;
    interrupts();
  
}


#endif
