#ifndef _ENCODER_H_
#define _ENCODER_H_

// Encoder Pin Setup
const byte channelA = 23;
const byte channelB = 25;
const byte enc_index = 30;

// Encoder ISR variables
volatile boolean enc_left = false;
volatile boolean enc_right = false;
volatile byte seqA = 0;
volatile byte seqB = 0;
volatile long enc_count = 0;
volatile long enc_count_old = 0;
unsigned long newTime = 0;  // used in encoder ISR
unsigned long oldTime = 0;  // used in encoder ISR

// Speed feedback calculations
unsigned long speed_loop_start = 0;
unsigned long speed_loop_end = 0;
unsigned long speed_loop_delay = 0;

//const float PI = 3.14;
const float REAR_DIFF_RADIUS = 3.0;   // rear differential radius in cm
const float DRIVE_SHAFT_RADIUS = 1.5; // drive shaft radius in cm where the encoder is connected
const float DRIVE_BELT_EFF = 0.95;
const float TIRE_RADIUS = 3.7;        // tire radius in cm
const float SPEED_FILTER_WT = 0.75; //Yue Sun 12/27/2018 - 

// Encoder Functions
void encoder_setup();  // PIN Initialization
void encoder();        // ISR
float encoder_speed_feedback(); // Car speed calc

#endif
