#include <stdio.h>
#include <math.h>
#include "decision_making.h"
//#include "blob.h"
#include "lane_recog.h"
#include "cntrl_limits.h"
#include "Sign_Reg.h"
#include "obstacle_detection.h"



//Outputs
//steer, drive and brake
//steer and drive ranges [-100 100], brake [0 100] 
int CntlCom[3] = 	{0,0,0};
#define IDX_STEER	0	// Range: [ -20 ?, 20 ? ]	degrees
#define IDX_DRIVE	1	// Range: [ -100, 100 ]		%
#define IDX_BRAKE	2	// Range: [ 0, 100 ]		%

//0 nothing, 1 Normal Keeping, 2 Sharp Turn Speed Reduction 3. ACC return speed less than cmd
#define STATE_INIT 0
#define STATE_KEEPING 1
#define STATE_SPD_RED 2
#define STATE_ACC 3

unsigned int CntlState = 0;

// PID controller for one channel
template<class type>
class PID{
	Limit<type> antiwindupLim;
	type err_old;
	type err_old_old;
	type err_I;
	type err_D;
	type* K;
	type K_tmp[1]={0.11};
	
public:
	// Constructor for PID object
	PID(type* inK, type antiwindupMin, type antiwindupMax)
		: K(inK) {
		// Initializations
		err_old = 0;
		err_old_old = 0;
		err_I = 0;
		err_D = 0;
		
		antiwindupLim = Limit<type>(antiwindupMin, antiwindupMax);
	}
	
	// Do PID control given desired value, feedback value, and time step
	type control(type desired, type feedback, float dt){
	//Could add piecewise non-linear terms in the future
		type err = desired - feedback;						// Calculate error on this step
		err_I = err_I + err;								// Integrate error
		saturate(err_I, antiwindupLim);						// Anti-Windup
		
		err_D = 0.5*(err_old - err_old_old) + 0.5*(err - err_old);		//Low pass filter the derivative term
		//err_D = (err - err_old);

		type err_filt = 0.3*err_old + 0.7*err; 				//Low pass filter the error term

		if (err_filt < 0)
		{
			K_tmp[0] = 0.95*K[0];
		}
		else
		{
			K_tmp[0] = 0.90*K[0];
		}
		

		err_old_old = err_old;						//store the previous error as old old error
		err_old = err;							// Store this error as previous

		return (K_tmp[0]*err_filt + K[1]*err_I*dt + K[2]*err_D/dt);	// return PID
	}
};



//Yue Sun
//Implement steering gain scheduling based on actual feedback speed
//Return is a interpolated float between [0 1] to better shape the Kp, Ki and Kd gains
//Could also implement 2D table as discussed, but start with something simple

const int K_s_x[5] = {0,25,50,75,100};
const float K_s_y[5] = {1.4,1.3,1.2,1.1,1};

//Implement speed COMMAND gain scheduling based on difference between target and lane center return
//Return is a interpolated float between [0 1] to better shape the speed command then cast back
const int K_vcmd_x[5] =   {0,5,10,15,30};
const float K_vcmd_y[5] = {1.0,1.0,0.9,0.8,0.7};

//Yue Sun Testing Purpose
//const float K_vcmd_y[5] = {0.6,0.6,0.6,0.6,0.6};

/*always put x in accending order with equal incrementals*/
/*this avoids sorting the input value into correct indexing of array*/
float interp_1d(const int *array_x,const float *array_y,unsigned int size,unsigned int value){

	unsigned int index = value/(array_x[size-1]/(size-1)); /*locate index of value, use integer math to cut residue*/

	if ( index <0 ){
		return array_y[0]; /*No extropolation beyond the min*/
	}
	else if( index < (size-1)){
		return (    array_y[index] + (array_y[index+1]-array_y[index])/(array_x[index+1]-array_x[index])*(value-array_x[index])   ); /*interpolation*/
	}
	else{
		return array_y[size-1];/*No extropolation beyond the max*/
	}
	
}

//steer
//PID
float K_x[3] = {0.11,0.0,0.002}; //was 0.07: as of 19-03-27
float K_x2[3] = {0.115,0.0,0.005};
PID<float> xPID = PID<float>(K_x, -1000, 1000);
PID<float> xPID2 = PID<float>(K_x2,-1000,1000);

//drive
//PID
float K_r[3] = {0.2,0.2,0.5};
PID<float> rPID = PID<float>(K_r, -1000, 1000);


//CAL parameters
//Targets
const int X_Target = FRAME_WIDTH/2; //steer factor

// Count since last paddle seen
int countSinceLastPaddle = 0;

float Pos[3] = {0,0,0};
float Pos_Old[3] = {0,0,0};

int lane_state =0;
int Drive_Cmd = 120; //Sunny Update 12/21/2018 for OLT, Sunny & Rahul 7/6/2017, SAS 4/3/2019
//gain was 0.9, 0.8 at steerting angle 15 30 
const float TaskPeriod = 0.05; //50ms Control Loop


template<class type> void saturate(type& value, Limit<type> limits);		// Saturates value to provided limits
bool txCommand(int command, int value);							// Sends the character command followed by value in packet


void trackingLogic(void){//10ms

	cout << "Sign Flag  " << Sign_Flag << endl;
	cout << "Obstacle Flag " << ObstacleFlag << endl;
	//check if traffic light is red (Stop)
	//check if obstacle is detected (Stop)
// SAS 4/3/2019		
//	if (SignFlag == 1 || ObstacleFlag == 1 || (LaneFlag1 == 1 && LaneFlag2 == 1))

	if (Sign_Flag == 1 || ObstacleFlag == 1 ){
		CntlCom[IDX_BRAKE] = BrakeLim.max;
		txCommand(IDX_BRAKE, CntlCom[IDX_BRAKE]);
		CntlCom[IDX_STEER] = 0.7*xPID.control(X_Target, lane_reg_flt_x, TaskPeriod) + 0.3*xPID2.control(X_Target,lane_reg_flt_x2,TaskPeriod);
		CntlCom[IDX_STEER] += 4; //sunny 07/06/2017 - offset steering inPI
		saturate(CntlCom[IDX_STEER], SteerLim);	//Left and Right

		txCommand(IDX_STEER, CntlCom[IDX_STEER]);
	}
	//Traffic light is green (Go) or no traffic light detected
	else {
	   switch(lane_state){

		case STATE_INIT:
			//entry condition to the state machine, reset values, jump to keeping
			lane_state = STATE_KEEPING;
			
		case STATE_KEEPING:
			CntlCom[IDX_BRAKE] = BrakeLim.min;		// Disable braking

			//Steer + saturate limits
			//2016-07-01 F. Dang fliter X coordinate value: x_lane_cross -> lane_reg_flt_x
			/*********************Need to use the FEEDBACK speed (real_v)*******************************/
			/*Yue Sun - 2018-12-21 weight higher on future vision, implement ROI 2 and weighted control distribution, gains are arbitrary*/
			CntlCom[IDX_STEER] = 0.7*xPID.control(X_Target, lane_reg_flt_x, TaskPeriod) + 0.3*xPID2.control(X_Target,lane_reg_flt_x2,TaskPeriod);
			CntlCom[IDX_STEER] += 4; //sunny 07/06/2017 - offset steering inPI
			saturate(CntlCom[IDX_STEER], SteerLim);	//Left and Right

			/*Then use the steering cmd and and delta to determin a speed cmd*/
			//Drive + saturate limits
			CntlCom[IDX_DRIVE] = interp_1d(K_vcmd_x,K_vcmd_y,5,abs(CntlCom[IDX_STEER]-4))*Lane_Mod_Gn*Drive_Cmd;
			saturate(CntlCom[IDX_DRIVE], DriveLim);	//Forward and Reverse

			Pos_Old[0] = Pos[0];
			Pos_Old[1] = Pos[1];
			Pos_Old[2] = Pos[2];
			
			// Send commands over serial
			txCommand(IDX_BRAKE, CntlCom[IDX_BRAKE]);
			txCommand(IDX_DRIVE, CntlCom[IDX_DRIVE]);
			txCommand(IDX_STEER, CntlCom[IDX_STEER]);
			
			// Reset counter
			countSinceLastPaddle = 0;
			break;

		//Need to add STATE_SPD_RED and STATE_ACC later
			
		default:	//color others (no command)
			
			//TODO: If no command found, should stop after a certain # of cycles.
			if(++countSinceLastPaddle > MAX_COUNT_SINCE_LAST_PADDLE){
				// Brake the car, we've lost the paddle!
				CntlCom[IDX_DRIVE] = 0;
				CntlCom[IDX_STEER] = 0;
				CntlCom[IDX_BRAKE] = BrakeLim.max; //brake, only takes Positive
				cerr << "Lost track of paddle for " << countSinceLastPaddle << " frames" << endl;
			}

	/* 		if (STATE_TRACKING == CntlState){//Were in tracking, fall into searching
				// TODO: Just steering? No driving?
				if(Pos_Old[0]>0){
					CntlCom[IDX_STEER] = 30;//Steer Positive to Search
					CntlCom[IDX_DRIVE] = 0;//Do not drive
				}
				else{
					CntlCom[IDX_STEER] = -30;//Steer Negative to Search
					CntlCom[IDX_DRIVE] = 0;//Do not drive
				}
			}
			else{//Were in stop mode, fall into initiliazed
				CntlState = STATE_UNINIT;
			
			} */
			
			
			// Send commands over serial
			txCommand(IDX_BRAKE, CntlCom[IDX_BRAKE]);
			txCommand(IDX_DRIVE, CntlCom[IDX_DRIVE]);
			txCommand(IDX_STEER, CntlCom[IDX_STEER]);
	   }
			
	}
}

template<class type>
void saturate(type& value, Limit<type> limits){
	if (value > limits.max){
		value = limits.max;
	} else if(value < limits.min){
		value = limits.min;
	}
}

bool txCommand(int command, int value){
	

	string commandChar = " ";

	switch (command){
		case IDX_BRAKE:
			commandChar = "B";
			break;
		case IDX_DRIVE:
			commandChar = "D";
			break;
		case IDX_STEER:
			commandChar = "S";
			break;
		default:
			commandChar = "?";
	}

	if (value == 0)
	{
		commandChar += "0";
		
	}
	cout << "SENDING CMD: " << '{' << commandChar << value << '}' << endl;
	try {
		ofstream fout (serialPath.c_str());
		fout << '{' << commandChar << value << '}';
		fout << '\n';
		fout.close();
	} catch (...) {
		return false;
	}
	return true;
}


