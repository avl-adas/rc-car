#ifndef _ACC_H_
#define _ACC_H_

unsigned int Manual_Enable = 0; //manual mode enable
unsigned int En_Pos = 0; //
unsigned int En_Vel = 0; //
unsigned int Man_Ctrl =0;//

unsigned int Rev_Counter = 0;


int Rel_Pos = 0; //filtered altrosonic reading [cm]
int Rel_Vel = 0; //filtered altrosonic velocity [m/s]
float v_cmd = 0; //control command, should be in PWM

int Rel_PosOld_1 =0;
int Rel_PosOld_2 =0;
int Rel_PosOld_3 =0;
int Rel_PosOld_4 =0;
//float Rel_PosOld_3 =0;

int Rel_VelOld_1 =0;
int Rel_VelOld_2 =0;
//float Rel_PosVel_3 =0;

const int d_thres_hi = 80;
const int d_thres_lo = 35;
int v_cruise = 100; //scaling between [-500 500]
float P_Kp = 0;// = v_cruise/(d_thres_hi - d_thres_lo);
float P_Kd = 0;


//VelCmd set by ACC, set_vcruise set by RC Transmitter, pi_cmd_v set by PI
float Cmd_To_PWM(float VelCmd, int set_vcruise, int pi_cmd_v){

  int v_tmp = min(set_vcruise,pi_cmd_v);
  
  v_cruise = v_tmp; // Assign v_cruise target for next loop, so that ACC command can pick it up
  
  if (VelCmd < 0){
    
    VelCmd = -100;
    
    if(Rev_Counter<1){
      VelCmd =0;
      Rev_Counter++;
    }
    else if(Rev_Counter<10){
      Rev_Counter++;
    }
    else {
      VelCmd = 0;
    }
  }
  else if(VelCmd>0){
        Rev_Counter =0;
  }
  //VelCmd [-500 500]
  //return ( (float ( VelCmd*55) )/500 + 90);
  return VelCmd;     // changed on June 6 by sunny and rahul
  //PWM [35 145]


}

void Mode_Selector()
{
	if(Manual_Enable){
		Man_Ctrl = 1;
		En_Pos = 0;
		En_Vel = 0;
	} 
	else{
		if(Rel_Pos<d_thres_hi){
			Man_Ctrl = 0;
			En_Pos = 1;
			En_Vel = 0;
		}
		else if(Rel_Pos>d_thres_hi){
			Man_Ctrl = 0;
			En_Pos = 0;
			En_Vel = 1;
		}
	
	}

}

void Vel_Cntrl(bool brk_flg){
	v_cmd = v_cruise;
 
 if(brk_flg)
 {v_cmd=0;}
 
}

void Pos_Cntrl(bool brk_flg){
        P_Kp = v_cruise/(d_thres_hi - d_thres_lo);
	v_cmd = constrain(P_Kp*(Rel_Pos - d_thres_lo), 0 , v_cruise); //+P_Kd*(Rel_Vel);
  
 if(brk_flg)
 {v_cmd=0;}
  
}


void Snsr_Proc(int SnsrRead, int dt){

	int C0_Pos = 1;
	int C1_Pos = 1;
	int C2_Pos = 1;
        int C3_Pos = 0;
        int C4_Pos = 0;
        int C_Pos_Scl = 3;
        //4th order FIR filter, cut off frequency at 30Hz based on a sampling rate of 100Hz(0.1 ms)


	int C0_Vel = 1;
        int C1_Vel = 1;
	int C2_Vel = 1;
        int C_Vel_Scl = 3;
        
        
        Rel_PosOld_4 = Rel_PosOld_3;
        Rel_PosOld_3 = Rel_PosOld_2;
	Rel_PosOld_2 = Rel_PosOld_1;
	Rel_PosOld_1 = Rel_Pos;
        Rel_Pos = SnsrRead;

	Rel_VelOld_2 = Rel_VelOld_1;
	Rel_VelOld_1 = Rel_Vel;
        Rel_Vel = (Rel_Pos - Rel_PosOld_1)/dt;  //cm/10ms = m/s

        //Rel_Pos = (C0_Pos*Rel_Pos + C1_Pos* Rel_PosOld_1 + C2_Pos * Rel_PosOld_2 + C3_Pos*Rel_PosOld_3 + C4_Pos*Rel_PosOld_4)/C_Pos_Scl;
        
	Rel_Vel = (C0_Vel*Rel_Vel + C1_Vel* Rel_VelOld_1 + C2_Vel * Rel_VelOld_2)/C_Vel_Scl;

}


void ACC(int SnsrRead, int dt, bool brk_flg){
	Snsr_Proc(SnsrRead,dt);
	Mode_Selector();

	if(Man_Ctrl){
		v_cmd = 0; //this should be interfaced with manual input
	}
	else if(En_Pos){
		Pos_Cntrl(brk_flg);
	}
	else if(En_Vel){
		Vel_Cntrl(brk_flg);
	}


}


#endif
