#include "shoot.h"
//#include "offline_check.h"
//#include "pid.h"
//#include "referee.h"



//Hero_Shoot_t Hero_Shoot =
//{
//	.Mode =
//	{
//		.Shoot = 0  //单发
//	},
//	.Flag =
//	{
//		.Fric_On = 1,
//		.First_Shoot = 1,
//	//	.Fric_slow_flag = 1
//	},
//	.Load =
//	{
//		.Load_finish_flag = 1,
//	},
//	.Fric =
//	{
//		 .Target_speed_log[0]=-5500,
//		 .Target_speed_log[1]= 5500,
//	}
//	
//};

///* -----------------------拨弹盘开启判断---------------------------- */
//void Load_start_judge(Hero_Shoot_t * shoot)
//{
//	if((referee2022.power_heat_data.shooter_id1_42mm_cooling_heat+100) <= referee2022.game_robot_status.shooter_id1_42mm_cooling_limit)   //热量不超限
//	{
//			if(shoot->Flag.Fric_Ready==1&&shoot->Flag.Fire_Hit==1&&shoot->Flag.Load_start==0) //可能判断条件需要修改
//			{
//				shoot->Flag.Load_start=1;
//			}
//	}
//	if(shoot->Flag.Fire_Hit==0||shoot->Flag.Fric_Ready==0)
//	{
//		shoot->Flag.Load_start=0;	
//	}

//}
///* -----------------------拨弹盘打弹结束判断---------------------------- */
//void Load_finish_judge(Hero_Shoot_t * shoot)
//{
//	//看角速度积分的角度    （（每分钟转的圈数/60）*360*0.005）/27
//	shoot->Load.angle = shoot->Load.angle + shoot->Load.Feedback_Speed*0.000666;	
//	//首先判断摩擦轮的转速转变是不是大于100
//	shoot->Fric.Speed_delta= shoot->Fric.This_Speed - shoot->Fric.Last_Speed;
//	
//	if(shoot->Fric.Speed_delta>60||shoot->Fric.Speed_delta<-60)
//	{
//		
//		shoot->Flag.Load_start=0;
//		shoot->Flag.Fire_Hit =0;
//		shoot->Load.angle =0;

//	}
//	//然后看拨弹盘是不是转过了65°

//	if(shoot->Load.angle>360||shoot->Load.angle<-360)
//	{
//	  shoot->Flag.Load_start=0;
//		shoot->Flag.Fire_Hit =0;
//		shoot->Load.angle =0;	

//	}
//	
//	if(Offline_Check.Off_Flag.Dbus==1)
//	{
//		
//	 shoot->Flag.Load_start=0;
//		shoot->Flag.Fire_Hit =0;
//		shoot->Load.angle =0;	
//	
//	}
//	
//	if(	referee2022.game_robot_status.mains_power_shooter_output == 0)
//	{
//		shoot->Flag.Load_start=0;
//		shoot->Flag.Fire_Hit =0;
//		shoot->Load.angle =0;	
//	
//	
//	}

//}

///* -----------------------堵转判断---------------------------- */
//void Jam_judge(Hero_Shoot_t * shoot)
//{
//	static int32_t clock = 0;	
//	//因为定时器1的周期是0.005s 拨弹盘速度低，电流大且持续一段时间则认为是堵转
//	if(abs(shoot->Load.Feedback_Speed)<100)
//	{	if(abs(shoot->Load.Output)>13000)
//		{	clock++;	}
//	}
//	else
//	{	clock--;}
//	
//	if (clock<0) 
//	{ clock=0;}
//	
//	if(clock>500)
//	{
//		clock = 0;
//		shoot->Flag.Jam =1;		
//	}
//	else 
//	{shoot->Flag.Jam =0;}
//}


///* -----------------------摩擦轮缓启动---------------------------- */
//void Fric_slow_start(Hero_Shoot_t * shoot,Off_Check_t * Offline)
//{
//	if(Offline->Off_Flag.Dbus==0&&shoot->Flag.Fric_On==1)
//	{
//		if(shoot->Fric.Slow_open_time<300)
//		{
//			shoot->Fric.Target_Speed[0] =  (int16_t)shoot->Fric.Target_speed_log[0]*0.0033f*shoot->Fric.Slow_open_time;
//			shoot->Fric.Target_Speed[1] =  (int16_t)shoot->Fric.Target_speed_log[1]*0.0033f*shoot->Fric.Slow_open_time;
//		}
//		else if(shoot->Fric.Slow_open_time>=300)
//		{
//			shoot->Fric.Target_Speed[0] =  (int16_t)shoot->Fric.Target_speed_log[0];
//			shoot->Fric.Target_Speed[1] =  (int16_t)shoot->Fric.Target_speed_log[1];
//		}
//		shoot->Fric.Slow_open_time++;
//		shoot->Fric.Slow_close_time=0;	
//	}
//		
//	if(Offline_Check.Off_Flag.Dbus==1||Hero_Shoot.Flag.Fric_On==0) 
//	{
//		if(shoot->Fric.Slow_close_time<300)
//		{
//			shoot->Fric.Target_Speed[0] =  (int16_t)shoot->Fric.Target_speed_log[0]*0.0033f*(301-shoot->Fric.Slow_close_time);
//			shoot->Fric.Target_Speed[1] =  (int16_t)shoot->Fric.Target_speed_log[1]*0.0033f*(301-shoot->Fric.Slow_close_time);		
//		}
//		else if(shoot->Fric.Slow_close_time>=300)
//		{
//			shoot->Fric.Target_Speed[0] =  0;
//			shoot->Fric.Target_Speed[1] =  0;		
//		}
//			shoot->Fric.Slow_close_time++;
//			shoot->Fric.Slow_open_time=0;
//	}		
//}
///* -----------------------摩擦轮准备就绪判断---------------------------- */
//void Fric_ready_ok_judge(Hero_Shoot_t * shoot)
//{
//	//标志位开启
//	if(shoot->Flag.Fric_On==1)
//		{
//		if((shoot->Fric.Feedback_Speed[0] - shoot->Fric.Feedback_Speed[1])<500)
//			{
//				if(abs(shoot->Fric.Feedback_Speed[0])>5000)
//				{
//						if(shoot->Fric.Slow_open_time>300)
//						{
//							shoot->Flag.Fric_Ready = 1;
//						 }
//			 }
//		 }
//		
//		}
//	else 
//	{ shoot->Flag.Fric_Ready = 0; }

//}

///* -----------------------拨弹盘速度调整---------------------------- */
//void Load_speed_change(Hero_Shoot_t * shoot)
//{
//	//卡弹反转时间
//	if(shoot->Load.Turn_Back_Time >0)
//	{
//	shoot->Load.Turn_Back_Time--;
//	}
//	else 
//	{
//	shoot->Load.Turn_Back_Time = 0;
//	}
//	
//	
//	
//	if(shoot->Flag.Load_start ==1&&shoot->Flag.Jam==0)
//	{
//		shoot->Load.Target_Speed = -2000;
//	}
//	else if(shoot->Flag.Load_start ==1&&shoot->Flag.Jam==1) //卡弹
//	{
//		shoot->Load.Turn_Back_Time  = 100;
//		
//	}
//	else if(shoot->Flag.Load_start ==0)
//	{
//	   shoot->Load.Target_Speed = 0;
//	}


//}


///* -----------------------火力升级机制---------------------------- */
//void Fric_speed_upgrade(Hero_Shoot_t * shoot)
//{
//	 if(shoot->Flag.change_speed==0)
//	 {
//		 shoot->Fric.Target_speed_log[0] = -5500;
//     shoot->Fric.Target_speed_log[1] = 5500;
//	 }
//	 else if(shoot->Flag.change_speed==1)
//	 {
//	   shoot->Fric.Target_speed_log[0] = -5650;
//     shoot->Fric.Target_speed_log[1] = 5650;
//	 
//	 }






//}

//void shoot_ctrl(void)
//{
//	if(Hero_Shoot.Mode.Shoot==0)
//	{
//	 //火力升级机制（改变摩擦轮转速）
//	Fric_speed_upgrade(&Hero_Shoot);
//	//摩擦轮缓启动
//	Fric_slow_start(&Hero_Shoot,&Offline_Check);
//	//检查摩擦轮是否准备就绪
//	Fric_ready_ok_judge(&Hero_Shoot);
//	
//	//第一次数据初始化 (必须在摩擦轮转速稳定之后)
//	if(Hero_Shoot.Flag.Fric_Ready==1)
//	{
//		Hero_Shoot.Fric.This_Speed = Hero_Shoot.Fric.Feedback_Speed[1];
//		if(Hero_Shoot.Flag.First_Shoot==1)
//		{
//			Hero_Shoot.Fric.Last_Speed = Hero_Shoot.Fric.This_Speed;
//			Hero_Shoot.Flag.First_Shoot =0;	
//		}
//  }
//	else 
//	{
//		Hero_Shoot.Fric.This_Speed =0;
//		Hero_Shoot.Flag.First_Shoot=1;
//	}
//	
//	//拨弹盘开启判断
//	Load_start_judge(&Hero_Shoot);
//	//拨弹结束判断
//	//if(Hero_Shoot.Flag.Load_start==1&&Hero_Shoot.Load.Load_finish_flag==0)
//	{Load_finish_judge(&Hero_Shoot);}
//	
//	//减少下一次拨弹延迟  修改  因为测试的时候就算没有这个东西打弹效果也是很好的，所以就注掉了，但是加上效果有可能更好，谁知道呢，有时间再测试去
////  if(Hero_Shoot.Load.Load_finish_flag==1)
////	{
//////		Load_continue_time++;
//////		if(Load_continue_time>=Hero_Shoot.Load.Load_continue_max)
//////		{
//////			Hero_Shoot.Flag.Load_start=0;
//////			Hero_Shoot.Flag.Fire_Hit=0;
//////			Load_continue_time=0;
//////			Hero_Shoot.Load.Load_finish_flag=0;	
//////		}
////	}
//	
//	//堵转检测
//	Jam_judge(&Hero_Shoot);

//	Load_speed_change(&Hero_Shoot);
//	Hero_Shoot.Fric.Last_Speed = Hero_Shoot.Fric.This_Speed;

//	
//  }
//	else if(Hero_Shoot.Mode.Shoot==1) //连发自爆
//	{
//		Hero_Shoot.Fric.Target_Speed[0]=Hero_Shoot.Fric.Target_speed_log[0];
//		Hero_Shoot.Fric.Target_Speed[1]=Hero_Shoot.Fric.Target_speed_log[1];
//		Hero_Shoot.Load.Target_Speed = -2000;
//	}
//	
//	//PID计算
//	if(Hero_Shoot.Load.Turn_Back_Time > 5)
//	{ Hero_Shoot.Load.Output = 500; }
//  else 
//	{Hero_Shoot.Load.Output = One_Pid_Ctrl(Hero_Shoot.Load.Target_Speed,Hero_Shoot.Load.Feedback_Speed,&Load_pid);}
//	Hero_Shoot.Fric.Output[0] = One_Pid_Ctrl(Hero_Shoot.Fric.Target_Speed[0],Hero_Shoot.Fric.Feedback_Speed[0],&Fric_pid_l);
//	Hero_Shoot.Fric.Output[1] = One_Pid_Ctrl(Hero_Shoot.Fric.Target_Speed[1],Hero_Shoot.Fric.Feedback_Speed[1],&Fric_pid_r);
//	
//	//电机离线
//	if(Offline_Check.Off_Flag.Load == 1)
//	{ Hero_Shoot.Load.Output = 0;	}
//	else if(Offline_Check.Off_Flag.Fric_L == 1 || Offline_Check.Off_Flag.Fric_R == 1 )
//	{	Hero_Shoot.Fric.Output[0] = 0;
//	  Hero_Shoot.Fric.Output[1] = 0;}
//	



//}


