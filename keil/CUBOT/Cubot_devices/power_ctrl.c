#include "power_ctrl.h"
//#include "pid.h"

//One_PID_Para_t Power_ctrl_pid=
//{
//	.shell=
//	{
//		.shell_P = 920,
//		.shell_I = 0.0002,
//		.shell_D = 40,
//		.shell_i_part_detach   = 5,	
//	}
//	
//};


//float Power_Ctrl(uint16_t Energy_Buffer_t,uint16_t Real_time_Power_t,uint16_t Max_Power,One_PID_Para_t * Power_pid)
//{
//	
//	float Energy_Buffer = (float)Energy_Buffer_t;
//	float Real_time_Power = (float)Real_time_Power_t;
//  float Target_Power_Buff=20;       //缓冲能量剩余目标值为20
//	Power_pid->shell.shell_delta   = Energy_Buffer-Target_Power_Buff;
//	Power_pid->shell.shell_p_part  = Power_pid->shell.shell_P * Power_pid->shell.shell_delta;
//	
//	Power_pid->shell.shell_i_part += Power_pid->shell.shell_I * Power_pid->shell.shell_delta;
//	if(Power_pid->shell.shell_delta > Power_pid->shell.shell_i_part_detach)
//	{
//		Power_pid->shell.shell_i_part=0;
//	}
//	if(Power_pid->shell.shell_delta < -(Power_pid->shell.shell_i_part_detach))
//	{
//		Power_pid->shell.shell_i_part=0;
//	}
//	
//	Power_pid->shell.shell_d_part = (-1) * Power_pid->shell.shell_D * Real_time_Power;
//  

//  Power_pid->shell.shell_out = Power_pid->shell.shell_p_part + Power_pid->shell.shell_i_part  +  Power_pid->shell.shell_d_part;
//	
//	return Power_pid->shell.shell_out;

//}



















