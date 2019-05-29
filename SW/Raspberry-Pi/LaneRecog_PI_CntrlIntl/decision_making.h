#ifndef DECISION_MAKING_H_INCLUDED
#define DECISION_MAKING_H_INCLUDED

#include <string>
#include <fstream>
using namespace std;

//extern float Pos[3];
//extern unsigned long RGB[3];


// Function declarations
extern void trackingLogic(void);// Main function for proccessing camera data

extern int Drive_Cmd;

extern int CntlCom[3]; 

extern const float TaskPeriod;

const string serialPath = "/dev/ttyACM0";

#endif
