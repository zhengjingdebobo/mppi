#include "pid.h"


/*
		float shell_P;    //�⻷p
		float shell_I;
		float shell_D;
    float shell_delta;
	
		float core_P;    //�ڻ�p
		float core_I;
		float core_D;
		float core_delta;
	
		float shell_p_part;	       //�⻷ p����
		float shell_i_part;
		float shell_i_part_maxlimit;
		float shell_i_part_detach;
		float shell_d_part;
		float shell_d_part_maxlimit;
	
		float core_p_part;        //�ڻ�p����
		float out_max_limit;
		float shell_out;	
		float core_out;	
		float out;                //���??
		Ҫ����д�϶����� 
*/


One_PID_Para_t  Chassis_follow_pid  =
{
		.shell=
	{
		.shell_P = 175,//175
		.shell_I = 0.5,
		.shell_D = 0,


		.shell_p_part_maxlimit = 500,
		.shell_i_part_maxlimit = 500,
		.shell_i_part_detach   = 15.0f,
		.shell_d_part_maxlimit = 0,
		.shell_max_limit       = 800 //1000
	}

};




One_PID_Para_t  Chassis_speed_pid =
{
		.shell=
	{
		.shell_P = 3.2,
		.shell_I = 0,
		.shell_D = 0,
		
		.shell_p_part_maxlimit = 10000,
		.shell_i_part_maxlimit = 0,
		.shell_d_part_maxlimit = 0,		
		.shell_max_limit       = 15000 //1000	
	}

};





One_PID_Para_t Yaw_pid_reset1=
{
	.shell=
	{
		.shell_P = 550,//-5
		.shell_I = 1,
		.shell_D = 0,	
		.shell_p_part_maxlimit = 25000,
		.shell_i_part_maxlimit = 10000,
		.shell_i_part_detach   = 15,
		.shell_d_part_maxlimit = 0,		
		.shell_max_limit       = 25000 //1000	
	},

};



Dual_PID_Para_t chassise_yaw_pid_reset_dual=
{
	.shell=
	{
		.shell_P = 30,//605
		.shell_I = 0,
		.shell_D = 0, //0.020
		
		.shell_p_part_maxlimit =400,
		.shell_i_part_maxlimit = 0,
		.shell_d_part_maxlimit = 0.2,
     	.shell_max_limit       = 400
	
	
	},
	.core=
	{
		.core_P = 3,//-680,
		.core_I = 0,
		.core_D = 0,		
		.core_p_part_maxlimit = 10000,
		.core_i_part_maxlimit = 0,
		.core_i_part_detach   = 0,
		.core_d_part_maxlimit = 0,		
		.core_max_limit       = 10000
	
	}

};







Dual_PID_Para_t Yaw_pid=
{
		.shell=
	{
		.shell_P = 4.2,//605
		.shell_I = 0,
		.shell_D = 0, //0.020
		.shell_p_part_maxlimit =210,
		.shell_i_part_maxlimit = 0,
		.shell_d_part_maxlimit = 0,
  	.shell_max_limit       = 210
	
	
	},
	.core=
	{
		.core_P = -1050,//-680,
		.core_I = 0,
		.core_D = -1500,		
		.core_p_part_maxlimit = 30000,
		.core_i_part_maxlimit = 10000,
		.core_i_part_detach   =  0.5,
		.core_d_part_maxlimit = 5000,		
		.core_max_limit       = 30000
	
	}

};








Dual_PID_Para_t Yaw_pid_long_distance=
{
		.shell=
	{
		.shell_P = 4,//605
		.shell_I = 0,
		.shell_D = 0, //0.020
		.shell_p_part_maxlimit =210,
		.shell_i_part_maxlimit = 0,
		.shell_d_part_maxlimit = 0,
  	.shell_max_limit       = 210
	
	
	},
	.core=
	{
		.core_P = -1800,//-680,
		.core_I = 0,
		.core_D = -3000,		
		.core_p_part_maxlimit = 30000,
		.core_i_part_maxlimit = 10000,
		.core_i_part_detach   =  0.5,
		.core_d_part_maxlimit = 5000,		
		.core_max_limit       = 30000
	
	}

};








float Double_Pid_Ctrl(float shell_target,float shell_feedback,float core_feedback,Dual_PID_Para_t* PID)
{
	
	
	PID->shell.shell_delta = shell_target-shell_feedback;
	
	
	
	/************ P����  ************/
	

	
	/************ I����  ************/
	
	PID->shell.shell_i_part += PID->shell.shell_delta * PID->shell.shell_I;
	
	if(abs(PID->shell.shell_delta) > PID->shell.shell_i_part_detach)   PID->shell.shell_i_part=0;
	
	
		if( (PID->shell.shell_i_part) > (PID->shell.shell_i_part_maxlimit) )   PID->shell.shell_i_part  = PID->shell.shell_i_part_maxlimit;
	  if( (PID->shell.shell_i_part) < -(PID->shell.shell_i_part_maxlimit) )  PID->shell.shell_i_part = -PID->shell.shell_i_part_maxlimit;
		
		
		
		
	/************ D����  ************/
		
		PID->shell.shell_d_part = ( PID->shell.shell_delta - PID->shell.shell_delta_last )*PID->shell.shell_D;
		
		if( (PID->shell.shell_d_part) > (PID->shell.shell_d_part_maxlimit) )   PID->shell.shell_d_part  = PID->shell.shell_d_part_maxlimit;
	  if( (PID->shell.shell_d_part) < -(PID->shell.shell_d_part_maxlimit) )  PID->shell.shell_d_part = -PID->shell.shell_d_part_maxlimit;
		
		PID->shell.shell_delta_last=PID->shell.shell_delta;
		
	/************ �⻷���??  ************/
		PID->shell.shell_out =(PID->shell.shell_p_part+PID->shell.shell_i_part+PID->shell.shell_d_part); 
		
		if( (PID->shell.shell_out) > (PID->shell.shell_max_limit) )   PID->shell.shell_out  = PID->shell.shell_max_limit;
	  if( (PID->shell.shell_out) < -(PID->shell.shell_max_limit) )  PID->shell.shell_out = -PID->shell.shell_max_limit;
		
		
		
	PID->core.core_delta = PID->shell.shell_out - core_feedback;	
		
		
			/************ P����  ************/
	
	PID->core.core_p_part = PID->core.core_delta * PID->core.core_P;

	if( (PID->core.core_p_part) > (PID->core.core_p_part_maxlimit) )   PID->core.core_p_part  = PID->core.core_p_part_maxlimit;
	if( (PID->core.core_p_part) < -(PID->core.core_p_part_maxlimit) )  PID->core.core_p_part = -PID->core.core_p_part_maxlimit;
	
	
	/************ I����  ************/
	
	PID->core.core_i_part += PID->core.core_delta * PID->core.core_I;
	
	if(abs(PID->core.core_delta)> PID->core.core_i_part_detach)   PID->core.core_i_part=0;
	
	
		if( (PID->core.core_i_part) > (PID->core.core_i_part_maxlimit) )   PID->core.core_i_part  = PID->core.core_i_part_maxlimit;
	  if( (PID->core.core_i_part) < -(PID->core.core_i_part_maxlimit) )  PID->core.core_i_part = -PID->core.core_i_part_maxlimit;
		
		
		
		
	/************ D����  ************/
		
		PID->core.core_d_part = ( PID->core.core_delta - PID->core.core_delta_last )*PID->core.core_D;
		
		if( (PID->core.core_d_part) > (PID->core.core_d_part_maxlimit) )   PID->core.core_d_part  = PID->core.core_d_part_maxlimit;
	  if( (PID->core.core_d_part) < -(PID->core.core_d_part_maxlimit) )  PID->core.core_d_part = -PID->core.core_d_part_maxlimit;
		PID->core.core_delta_last=PID->core.core_delta;
		
	/************ �ڻ����??  ************/
		PID->core.core_out =(PID->core.core_p_part+PID->core.core_i_part+PID->core.core_d_part); 
		
		if( (PID->core.core_out) > (PID->core.core_max_limit) )   PID->core.core_out  = PID->core.core_max_limit;
	  if( (PID->core.core_out) < -(PID->core.core_max_limit) )  PID->core.core_out = -PID->core.core_max_limit;
	
	
	return PID->core.core_out ;

}


float One_Pid_Ctrl(float shell_target,float shell_feedback,One_PID_Para_t* PID)
{
	PID->shell.shell_delta = shell_target-shell_feedback;
	
	
	
	
	/************ P����  ************/

	PID->shell.shell_p_part = PID->shell.shell_delta * PID->shell.shell_P;

	if( (PID->shell.shell_p_part) > (PID->shell.shell_p_part_maxlimit) )   PID->shell.shell_p_part  = PID->shell.shell_p_part_maxlimit;
	if( (PID->shell.shell_p_part) < -(PID->shell.shell_p_part_maxlimit) )  PID->shell.shell_p_part = -PID->shell.shell_p_part_maxlimit;


	/************ I����  ************/
	
		
	PID->shell.shell_i_part += PID->shell.shell_delta * PID->shell.shell_I;
	
	if(abs(PID->shell.shell_delta) > PID->shell.shell_i_part_detach)   PID->shell.shell_i_part=0;
	
	
		if( (PID->shell.shell_i_part) > (PID->shell.shell_i_part_maxlimit) )   PID->shell.shell_i_part  = PID->shell.shell_i_part_maxlimit;
	  if( (PID->shell.shell_i_part) < -(PID->shell.shell_i_part_maxlimit) )  PID->shell.shell_i_part = -PID->shell.shell_i_part_maxlimit;
		
		
		
		
	/************ D����  ************/
		
		PID->shell.shell_d_part = ( PID->shell.shell_delta - PID->shell.shell_delta_last )*PID->shell.shell_D;
		
		if( (PID->shell.shell_d_part) > (PID->shell.shell_d_part_maxlimit) )   PID->shell.shell_d_part  = PID->shell.shell_d_part_maxlimit;
	  if( (PID->shell.shell_d_part) < -(PID->shell.shell_d_part_maxlimit) )  PID->shell.shell_d_part = -PID->shell.shell_d_part_maxlimit;
		
		PID->shell.shell_delta_last=PID->shell.shell_delta;
		
	/************ �⻷���??  ************/
		PID->shell.shell_out =(PID->shell.shell_p_part+PID->shell.shell_i_part+PID->shell.shell_d_part); 
		
		if( (PID->shell.shell_out) > (PID->shell.shell_max_limit) )   PID->shell.shell_out  = PID->shell.shell_max_limit;
	  if( (PID->shell.shell_out) < -(PID->shell.shell_max_limit) )  PID->shell.shell_out = -PID->shell.shell_max_limit;
		
		
		
	
	
	return  PID->shell.shell_out ;

}



float pid_ctrl_zxj(float shell_target,float shell_feedback,float core_feedback,Dual_PID_Para_t* PID)
{
	float detal = shell_target-shell_feedback;
	
	if(abs(detal) < PID->shell.shell_p_part_maxlimit)
		PID->shell.shell_out = PID->shell.shell_P * detal;
	else if(abs(detal) < PID->shell.shell_i_part_maxlimit)
	{
		if(detal > 0)
			PID->shell.shell_out = (PID->shell.shell_p_part_maxlimit  *  PID->shell.shell_P) + ((abs(detal) - PID->shell.shell_p_part_maxlimit) * PID->shell.shell_I);
		else if(detal < 0)
			PID->shell.shell_out = -(PID->shell.shell_p_part_maxlimit  *  PID->shell.shell_P) - ((abs(detal) - PID->shell.shell_p_part_maxlimit) * PID->shell.shell_I);

	}
	
	else if( abs(detal) < PID->shell.shell_d_part_maxlimit )
	{
		if(detal > 0)
			PID->shell.shell_out = (PID->shell.shell_p_part_maxlimit  *  PID->shell.shell_P) + (( PID->shell.shell_i_part_maxlimit - PID->shell.shell_p_part_maxlimit )* PID->shell.shell_I) + ((abs(detal) - PID->shell.shell_i_part_maxlimit) * PID->shell.shell_D);
		else if(detal < 0)
		 PID->shell.shell_out = -(PID->shell.shell_p_part_maxlimit  *  PID->shell.shell_P) - (( PID->shell.shell_i_part_maxlimit - PID->shell.shell_p_part_maxlimit )* PID->shell.shell_I) - ((abs(detal) - PID->shell.shell_i_part_maxlimit) * PID->shell.shell_D);

	}
		
	
	
		PID->core.core_delta = PID->shell.shell_out - core_feedback;	
		
		
			/************ P����  ************/
	
	PID->core.core_p_part = PID->core.core_delta * PID->core.core_P;

	if( (PID->core.core_p_part) > (PID->core.core_p_part_maxlimit) )   PID->core.core_p_part  = PID->core.core_p_part_maxlimit;
	if( (PID->core.core_p_part) < -(PID->core.core_p_part_maxlimit) )  PID->core.core_p_part = -PID->core.core_p_part_maxlimit;
	
	
	/************ I����  ************/
	
	PID->core.core_i_part += PID->core.core_delta * PID->core.core_I;
	
	if(abs(PID->core.core_delta)< PID->core.core_i_part_detach)   PID->core.core_i_part=0;
	
	
		if( (PID->core.core_i_part) > (PID->core.core_i_part_maxlimit) )   PID->core.core_i_part  = PID->core.core_i_part_maxlimit;
	  if( (PID->core.core_i_part) < -(PID->core.core_i_part_maxlimit) )  PID->core.core_i_part = -PID->core.core_i_part_maxlimit;
		
		
		
		
	/************ D����  ************/
		
		PID->core.core_d_part = ( PID->core.core_delta - PID->core.core_delta_last )*PID->core.core_D;
		
		if( (PID->core.core_d_part) > (PID->core.core_d_part_maxlimit) )   PID->core.core_d_part  = PID->core.core_d_part_maxlimit;
	  if( (PID->core.core_d_part) < -(PID->core.core_d_part_maxlimit) )  PID->core.core_d_part = -PID->core.core_d_part_maxlimit;
		PID->core.core_delta_last=PID->core.core_delta;
		
	/************ �ڻ����??  ************/
		PID->core.core_out =(PID->core.core_p_part+PID->core.core_i_part+PID->core.core_d_part); 
		
		if( (PID->core.core_out) > (PID->core.core_max_limit) )   PID->core.core_out  = PID->core.core_max_limit;
	  if( (PID->core.core_out) < -(PID->core.core_max_limit) )  PID->core.core_out = -PID->core.core_max_limit;
	
	
	return PID->core.core_out ;










}























