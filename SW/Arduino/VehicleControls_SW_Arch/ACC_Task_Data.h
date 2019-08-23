// All the functions and global variables needed by ACC Task

// Function declarations
float encoder_speed_feedback(); // Car speed calc
extern void battery_feedback();

// Global Variables
extern boolean manual_flag;
extern boolean pi_brake_flag; 
extern float avgDistF;
extern float avgDistFL;
extern float avgDistFR;
extern float avgDist;

extern int pi_speed_cmd;
extern const float BATTERY_MAX_VOLTAGE;
extern const float BATTERY_LOW_THRESHOLD;
extern float batt_voltage;

extern Ultrasonic frontUltrasonic;
extern Ultrasonic rightUltrasonic;
extern Ultrasonic leftUltrasonic;
