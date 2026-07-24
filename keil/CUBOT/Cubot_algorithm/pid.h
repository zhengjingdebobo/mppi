#ifndef __PID_H
#define __PID_H
#include "main.h"

#define abs(x) ((x)>0? (x):(-(x)))

typedef  struct 
{	
      float Kp;//比例系数
      float Ki;//积分系数
      float Kd;//微分系数  
      float E0;//差值
			float Last_E0;
      float Ep_Part;//比例部分控制量
      float Ei_Part;//积分部分控制量
      float Ed_Part;//微分部分控制量
			float Ei_Part_Limit;
			float Out_Limit;//输出限幅
      float Out;//最后输出值
	    float Target; //目标值
	    float Feedback;//反馈值
}Pid_Info;

typedef   struct  //双闭环
{
		float shell_P;    //外环p
		float shell_I;
		float shell_D;
    float shell_delta;
	
		float core_P;    //内环p
		float core_I;
		float core_D;
		float core_delta;
	
		float shell_p_part;	       //外环 p部分
		float shell_i_part;
		float shell_i_part_maxlimit;
		float shell_i_part_detach;
		float shell_d_part;
		float shell_d_part_maxlimit;
	
		float core_p_part;        //内环p部分
		float out_max_limit;
		float shell_out;	
		float core_out;	
		float out;                //输出
}Dual_PID_Para;

typedef   struct  //双闭环
{
	
	struct {
		float shell_P;    //外环p
		float shell_I;
		float shell_D;
        float shell_delta;
		float shell_delta_last;
		float shell_feedback;
		
	  
		
		
		float shell_p_part;	       //外环 p部分
	    float shell_p_part_maxlimit;
		float shell_i_part;
		float shell_i_part_maxlimit;
		float shell_i_part_detach;  //死区
		float shell_d_part;
		float shell_d_part_maxlimit;
		float shell_max_limit;
		float shell_out;
		
	}shell;
	
}One_PID_Para_t;





extern Dual_PID_Para Pid_Test_Power;

typedef   struct  //双闭环
{
	
	struct {
		float shell_P;    //外环p
		float shell_I;
		float shell_D;
    float shell_delta;
		float shell_delta_last;
	
		float shell_p_part;	       //外环 p部分
	  float shell_p_part_maxlimit;
		float shell_i_part;
		float shell_i_part_maxlimit;
		float shell_i_part_detach;  //死区
		float shell_d_part;
		float shell_d_part_maxlimit;
	  float shell_max_limit;
		float shell_out;
	}shell;
	
	
	
	
	struct{
	
		float core_P;    //内环p
		float core_I;
		float core_D;
		float core_delta;
		float core_delta_last;
	

	
		float core_p_part;        //内环p部分
		float core_p_part_maxlimit;
		float core_i_part;
		float core_i_part_maxlimit;
		float core_i_part_detach;  //死区
		float core_d_part;
		float core_d_part_maxlimit;
	  float core_max_limit;
  	float core_out;	
	}core;




}Dual_PID_Para_t;


extern Dual_PID_Para_t chassise_yaw_pid_reset_dual;

extern One_PID_Para_t  Chassis_speed_pid;
extern One_PID_Para_t  Chassis_follow_pid;

//extern Dual_PID_Para_t Pitch_pid;
//extern Dual_PID_Para_t Yaw_pid;
//extern Dual_PID_Para_t Pitch_pid_vision;
//extern Dual_PID_Para_t Yaw_pid_vision;
//extern Dual_PID_Para_t Yaw_pid_long_distance;
//extern Dual_PID_Para_t Pitch_pid_long_distance;

//extern Dual_PID_Para_t Pitch_pid_zxj;
//extern Dual_PID_Para_t Yaw_pid_reset;


float Double_Pid_Ctrl(float shell_target,float shell_feedback,float core_feedback,Dual_PID_Para_t* PID);
float pid_ctrl_zxj(float shell_target,float shell_feedback,float core_feedback,Dual_PID_Para_t* PID);
float One_Pid_Ctrl(float shell_target,float shell_feedback,One_PID_Para_t* PID);




#endif