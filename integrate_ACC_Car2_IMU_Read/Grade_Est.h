
float a = 0;            // vehicle acceleration - encoder
const float g = 9.8;    // gravity (m/s)
float grade = 0;        // grade
float h_ac = 0;         // horizontal accelerometer
float v_ac = 0;         // vertical accelerometer
float g_ac = 0;         // grade acceleration   

float car_speed_m  = 0; // car speed in m/s
float a_newtime = 0;    // new time for acceleration calculation
float a_oldtime = 0;    // old time for acceleration calculation
float new_speed = 0;
float old_speed = 0;

const float cg_h = .10;  // center of gravity height (m)                    ***estimated***
const float cg_l = .15;  // center of gravity distance from rear wheel (m)  ***estimated***
const float k = .4;      // spring constant for the suspension (N/m)        ***estimated***
const float wb = .24;    // wheelbase (m)                                   ***estimated***
const float m = 1;      // mass (kg)                                        ***estimated***


float phi = 0;          // pitch angle

float grade_tan = 0;
float grade_sin = 0;

void Grade_Est()
{
float Ff = 0;           // vertical force on the front wheel
float Fr = 0;           // vertical force on the rear wheel
float Fa = 0;           // horizontal force from acceleration
float Ffb = 0;          // front wheel static
float Frb = 0;          // rear wheel static
float Ffd = 0;          // front wheel force delta
float Frd = 0;          // rear wheel force delta
float F_d = 0;          // Front displacement (cm)
float R_d = 0;          // Rear displacement  (cm)
/*

read h_ac
read v_ac
 
 */

// calculate vehicle acceleration from encoder speed
// car_speed is in cm/sec
car_speed_m = car_speed / 100;

a_oldtime = a_newtime;
old_speed = new_speed;

a_newtime = micros();
new_speed = car_speed_m;

a = (new_speed - old_speed) / (a_newtime - a_oldtime); //vehicle acceleration


g_ac = h_ac - a; // calculate grade acceleration

grade_tan = atan(g_ac / v_ac);
grade_sin = asin(g_ac / g);



//Suspension pitch estimation



Fa = m * a;

Ffb = (m * g * cg_l) / wb;
Frb = (m * g * (wb - cg_l)) / wb;

Ff = (m * g * cg_l - Fa * h) / wb;
Fr = (m * g * (wb - cg_l) + Fa * h) / wb;

Ffd = Ffb - Ff
Frd = Frb - Fr

F_d = Ffd * k;
R_d = Frd * k;

phi = atan((F_d + R_d) / wb); //pitch

}
