extern boolean manual_flag;
extern boolean pi_brake_flag; 
extern float avgDistF;
extern float avgDistFL;
extern float avgDistFR;
extern float avgDist;

extern int pi_speed_cmd;

float encoder_speed_feedback(); // Car speed calc

extern const float BATTERY_MAX_VOLTAGE;
extern const float BATTERY_LOW_THRESHOLD;
extern float batt_voltage;
extern void battery_feedback();

extern Ultrasonic frontUltrasonic;
extern Ultrasonic rightUltrasonic;
extern Ultrasonic leftUltrasonic;
