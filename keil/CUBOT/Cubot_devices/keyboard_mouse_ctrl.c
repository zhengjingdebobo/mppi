#include "keyboard_mouse_ctrl.h"
//#include "dr16.h"
//#include "holder.h"
//#include "shoot.h"
//#include "vision.h"
//#include "super_cap.h"
//#include "chassis.h"
//#include "referee.h"
//#include "tim.h"
//PC_sensitivity_t  PC_sensitivity =
//{  
//  .pc_yaw     = 0.03, //0.045,
//	.pc_pitch   = -0.03,//-0.006,
//	.dr16_yaw   = 0.003,
//	.dr16_pitch = 0.003,
//	.reset_time_zxj = 0
//};

//void PC_dr16_ctrl(void)
//{
//	

//	
///********************************遥控器控制***************************************/
///********************************遥控器控制***************************************/	
///********************************遥控器控制***************************************/	
///********************************遥控器控制***************************************/	
///********************************遥控器控制***************************************/	

//{	
//		if(abs(rc_Ctrl.rc.ch2-1024)>10)
//		{Hero_Holder.Yaw.Target_Angle -= (rc_Ctrl.rc.ch2-1024)*PC_sensitivity.dr16_yaw; }
//		if(abs(rc_Ctrl.rc.ch3-1024)>10)
//		{Hero_Holder.Pitch.Target_Angle += (rc_Ctrl.rc.ch3-1024)*PC_sensitivity.dr16_pitch;}
//		
//			if(Hero_Holder.Flag.Long_distance_shoot_model==1)	//吊射改慢累加速度
//		{
//			Hero_Holder.Yaw.Target_Angle   += (-rc_Ctrl.mouse.x)* PC_sensitivity.pc_yaw*0.5f;//转换成yaw的目标角度
//			Hero_Holder.Pitch.Target_Angle += (rc_Ctrl.mouse.y)*PC_sensitivity.pc_pitch*0.5;
//		}
//		else 
//		{
//			Hero_Holder.Yaw.Target_Angle +=  (-rc_Ctrl.mouse.x)* PC_sensitivity.pc_yaw;//转换成yaw的目标角度
//			Hero_Holder.Pitch.Target_Angle+= (rc_Ctrl.mouse.y)*PC_sensitivity.pc_pitch;
//		}
//}
//	
//	
//	
//	//打弹
//	if(rc_Ctrl.rc.s1==1&&rc_Ctrl.rc.s1_last==3)
//	{
//		Hero_Shoot.Flag.Fire_Hit = 1;      
//	}
//	
//	
//	//超级电容充电
//	if(rc_Ctrl.rc.s1==2)
//	{
//			my_cap.charge = 0;   	
//	}
//	else 	
//	{
//			my_cap.charge = 1;   	
//	}
//							
//	

//	//超级电容
//	
//	Super_Cap_Ctrl();

//	/*//跟随*/
//	if(rc_Ctrl.rc.s2==2&&rc_Ctrl.rc.s2_last==3)
//	{
//		Hero_Chassis.ChassisState.Follow =! Hero_Chassis.ChassisState.Follow;
//	}

//	//摇摆
//	if(rc_Ctrl.rc.sw < 824)
//	{
//		Hero_Chassis.ChassisState.Rock =1;  //摇摆是负自旋
//		
//	}
//	//自旋
//	else if(rc_Ctrl.rc.sw > 1224)
//	{
//		Hero_Chassis.ChassisState.Spin =1;  //自旋是正自旋

//	}
//	else 
//	{
//		Hero_Chassis.ChassisState.Rock =0;
//		Hero_Chassis.ChassisState.Spin =0;
//	}
//	
//	
//	
///****************************键鼠控制***************************************/	
///****************************键鼠控制***************************************/
///****************************键鼠控制***************************************/	
///****************************键鼠控制***************************************/	
///****************************键鼠控制***************************************/	
//	
//				if(rc_Ctrl.key_ctrl_flag == 0 )
//				{
//					/***********************打弹 视觉 吊射 自旋 跟随*************************/	
//					
//								//打弹
//								if(rc_Ctrl.mouse.press_l_flag == 1 && rc_Ctrl.mouse.last_press_l_flag == 0)
//								{
//									Hero_Shoot.Flag.Fire_Hit = 1;   	
//								}
//								
//				
//							
//								if(rc_Ctrl.key_F == 1)
//								{
//										Hero_Shoot.Flag.Load_start=0;
//										Hero_Shoot.Flag.Fire_Hit =0;
//										Hero_Shoot.Load.angle =0;		
//								}
//								
//								
//							 //改变射速
//								if(rc_Ctrl.key_V_flag == 1 && rc_Ctrl.laste_key_V_flag ==0)
//								{
//									 Hero_Shoot.Flag.change_speed =! Hero_Shoot.Flag.change_speed;
//								}
//								
//								//吊射
//								if(rc_Ctrl.key_G_flag == 1 && rc_Ctrl.laste_key_G_flag ==0)
//								{
//									 Hero_Holder.Flag.Long_distance_shoot_model =! Hero_Holder.Flag.Long_distance_shoot_model;
//								}
//								
//								//自旋
//								if(rc_Ctrl.key_R_flag ==1&& rc_Ctrl.laste_key_R_flag ==0)
//								{  								
//									Hero_Chassis.ChassisState.ING_Spin ++;
//								}
//								if(rc_Ctrl.key_Q_flag ==1 )
//								{
//									Hero_Chassis.ChassisState.Spin =1;
//								}
//								else if(rc_Ctrl.key_Q_flag ==0 && rc_Ctrl.rc.sw < 1223)
//								{
//									Hero_Chassis.ChassisState.Spin =0;
//								}
//								//左自旋
//								if(rc_Ctrl.key_E_flag == 1)
//								{
//									Hero_Chassis.ChassisState.Rock =1;
//								}
//								else if(rc_Ctrl.key_Q_flag ==0 && rc_Ctrl.rc.sw > 824)
//								{
//									Hero_Chassis.ChassisState.Rock =0;
//								}
//								//重启
//								if(rc_Ctrl.key_B_flag == 1)
//								{
//									
//									PC_sensitivity.reset_time_zxj++;
//								
//								}
//								
//								
//								
//								
//								
//								
//								//跟随
//								if(rc_Ctrl.key_X_flag ==1 && rc_Ctrl.laste_key_X_flag ==0)
//								{
//								  Hero_Chassis.ChassisState.Follow =! Hero_Chassis.ChassisState.Follow;
//						 		 
//								}

//				}
//				
//				
//				if(rc_Ctrl.key_ctrl_flag == 1 )//&& rc_Ctrl.key_shift_flag ==0 )
//				{
//					/***********************重启自瞄 自爆 自爆锁云台*************************/	
//						//自爆 按住不要松手
//								if(rc_Ctrl.mouse.press_l_flag ==1)
//								{
//									Hero_Shoot.Mode.Shoot = 1;
//								
//								}
//		
//				}
//				else 
//				{
//					Hero_Shoot.Mode.Shoot = 0;
//				}
//				
//	
//	
//	
//	
//	
// rc_Ctrl.rc.s1_last = rc_Ctrl.rc.s1;
// rc_Ctrl.rc.s2_last = rc_Ctrl.rc.s2;
//	
//	rc_Ctrl.mouse.last_press_l_flag = rc_Ctrl.mouse.press_l_flag;
//	rc_Ctrl.mouse.last_press_r_flag = rc_Ctrl.mouse.press_r_flag;
//	
//	rc_Ctrl.laste_key_W_flag = rc_Ctrl.key_W_flag;
//	rc_Ctrl.laste_key_A_flag = rc_Ctrl.key_A_flag;
//	rc_Ctrl.laste_key_S_flag = rc_Ctrl.key_S_flag;
//	rc_Ctrl.laste_key_D_flag = rc_Ctrl.key_D_flag;
//	rc_Ctrl.laste_key_shift_flag = rc_Ctrl.key_shift_flag;
//	rc_Ctrl.laste_key_ctrl_flag = rc_Ctrl.key_ctrl_flag;
//	rc_Ctrl.laste_key_Q_flag = rc_Ctrl.key_Q_flag;
//	rc_Ctrl.laste_key_E_flag = rc_Ctrl.key_E_flag;
//	rc_Ctrl.laste_key_V_flag = rc_Ctrl.key_V_flag;
//	rc_Ctrl.laste_key_F_flag = rc_Ctrl.key_F_flag;
//	rc_Ctrl.laste_key_G_flag = rc_Ctrl.key_G_flag;
//	rc_Ctrl.laste_key_C_flag = rc_Ctrl.key_C_flag;
//	rc_Ctrl.laste_key_R_flag = rc_Ctrl.key_R_flag;
//	rc_Ctrl.laste_key_B_flag = rc_Ctrl.key_B_flag;
//	rc_Ctrl.laste_key_Z_flag = rc_Ctrl.key_Z_flag;
//	rc_Ctrl.laste_key_X_flag = rc_Ctrl.key_X_flag;

//	
//	
//	
//		 
//	if(Hero_Holder.Pitch.Target_Angle>42.0f)
//	{
//		Hero_Holder.Pitch.Target_Angle = 42.0f;
//	}
//	else if(Hero_Holder.Pitch.Target_Angle< -10.0f)
//	{	
//		Hero_Holder.Pitch.Target_Angle = -10.0f;
//	}
//	
//}