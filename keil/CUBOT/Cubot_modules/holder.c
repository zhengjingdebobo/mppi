#include "holder.h"
//#include "pid.h"
//#include "offline_check.h"
//#include "mpu6050.h"
//#include "gpio.h"
//#include "vision.h"
//#include "referee.h"
//#include "dr16.h"

//Hero_Holder_t Hero_Holder =
//{
//	   .Yaw =
//  {
//	  .Can_Ori =  553,  //-1223 
//	},
//	
//   .Pitch =
//  {
//		 .Can_Ori = 3900,
//	  .Gyro_or_Can = 1, //pitch 轴我还是想用picth角度
//	},
//		.Flag=
//	{
//		.Enable=1,	
//	}
//	
//};
///* -----------------------云台复位成功判断---------------------------- */
//static void Yaw_reset_ok_judge(Hero_Holder_t * holder)  // 这个函数应该只进行一次
//{  
//		static int16_t reset_cnt = 0; //云台复位时间	
//		if(abs(holder->Yaw.Can_Angle)<5)
//		{
//			reset_cnt++;
//			if(reset_cnt>300)
//		{
//			holder->Flag.Reset_OK = 1;
//			reset_cnt = 0;
//		}
//		}
//		else 
//		holder->Flag.Reset_OK = 0;
//}
///* -----------------------云台复位函数---------------------------- */
//static void Hoder_reset(Hero_Holder_t * holder)
//{
//				//if(Offline_Check.Off_Flag.Dbus==0&&Offline_Check.Off_Flag.Yaw==0)  //云台上电复位
//	     if(Offline_Check.Off_Flag.Yaw==0 )    //云台不上电复位
//				{	
//					holder->Yaw.Output = One_Pid_Ctrl(0,(holder->Yaw.Can_Angle),&Yaw_pid_reset1);
//					holder->Yaw.Output = Double_Pid_Ctrl(0,(holder->Yaw.Can_Angle),(holder->Yaw.Can_Speed),&Yaw_pid_reset);
//					Yaw_reset_ok_judge(holder); //复位判断
//				}
//				else 
//				{
//					holder->Yaw.Output = 0;
//				}
//}
///* -----------------------确定使用陀螺仪角度还是编码器角度---------------------------- */

//static void Use_data_type_validation(Hero_Holder_t * holder)
//{  
//	//>pitch 
//	if(holder->Pitch.Gyro_or_Can == 1)
//	{
//		holder->Pitch.Angle = holder->Pitch.Can_Angle;
//		//holder->Pitch.Speed = holder->Pitch.Can_Speed * 0.05;
//		holder->Pitch.Speed = -(gyro_data.pitch_speed*75);
//	}
//	else if(holder->Pitch.Gyro_or_Can == 0)
//	{
//	
//		holder->Pitch.Angle = mpuAngle.pitch;
//		//holder->Pitch.Speed = holder->Pitch.Can_Speed;
//		holder->Pitch.Speed = -(gyro_data.pitch_speed*75);
//	}
//	//>yaw
//	holder->Yaw.Angle = gyro_data.yaw_angle;
//	holder->Yaw.Speed = gyro_data.yaw_speed*500;
//}



//void Yaw_and_pitch_angle_change(void)
//{
//	//> Yaw
//	if(Hero_Holder.Yaw.Can_Ori<4096)
//	{
//		if(Hero_Holder.Yaw.Can_Angle_Raw>Hero_Holder.Yaw.Can_Ori + 4096)
//		{
//			Hero_Holder.Yaw.Can_Angle_Raw = Hero_Holder.Yaw.Can_Angle_Raw - 8192;
//		}
//	}
//	else
//	{
//		if(Hero_Holder.Yaw.Can_Angle_Raw>Hero_Holder.Yaw.Can_Ori - 4096)
//		{
//			Hero_Holder.Yaw.Can_Angle_Raw = Hero_Holder.Yaw.Can_Angle_Raw + 8192;
//		}
//	}
//	//>Pitch
//	if(Hero_Holder.Pitch.Can_Ori<4096)
//	{
//		if(Hero_Holder.Pitch.Can_Angle_Raw>Hero_Holder.Pitch.Can_Ori + 4096)
//		{
//			Hero_Holder.Pitch.Can_Angle_Raw = Hero_Holder.Pitch.Can_Angle_Raw - 8192;
//		}
//	}
//	else
//	{
//		if(Hero_Holder.Yaw.Can_Angle_Raw>Hero_Holder.Pitch.Can_Ori - 4096)
//		{
//			Hero_Holder.Yaw.Can_Angle_Raw = Hero_Holder.Pitch.Can_Angle_Raw + 8192;
//		}
//	}	
//	//>角度换算	
//	Hero_Holder.Yaw.Can_Angle   = K_Code_Yaw   * (Hero_Holder.Yaw.Can_Angle_Raw - Hero_Holder.Yaw.Can_Ori) ;	
//	Hero_Holder.Pitch.Can_Angle = K_Code_Picth * (Hero_Holder.Pitch.Can_Angle_Raw - Hero_Holder.Pitch.Can_Ori) ;	
//	
//}


//void holder_poweron(void)
//{
//	//< MPU6050 电源开启
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//}

//void holder_poweroff(void)
//{
//	//< MPU6050 电源开启
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//}

//void holder_judge_power(void)
//{

//			if(referee2022.game_robot_status.mains_power_chassis_output ==1)//== 1 && last_open_flag==0))//||rc_Ctrl.key_V_flag==1)
//			{
//				holder_poweron();
//			}
//			else if(referee2022.game_robot_status.mains_power_chassis_output==0)// == 0 && last_open_flag==1)
//			{
//				holder_poweroff();
//			
//			}


//}

//void holder_ctrl(void)  //云台控制函数
//{
//	
//	holder_judge_power();      //是不是要加上时间判断

//	if(Hero_Holder.Flag.Reset_OK==0)  //
//	{
//		Hoder_reset(&Hero_Holder);
//	}		
//	
//	else if(Hero_Holder.Flag.Reset_OK==1)
//	{
//  //>获得 yaw轴和pitch轴  角速度 角加速度
//		 Use_data_type_validation(&Hero_Holder);		
//  //> PID 计算 yaw pitch 输出
//		if(Hero_Holder.Flag.Enable==1&&Offline_Check.Off_Flag.Dbus==0)
//			{
//				if( Hero_Holder.Flag.Long_distance_shoot_model ==0)
//				{
//						Hero_Holder.Yaw.Output = Double_Pid_Ctrl(Hero_Holder.Yaw.Target_Angle, Hero_Holder.Yaw.Angle, Hero_Holder.Yaw.Speed, &Yaw_pid);
//						Hero_Holder.Pitch.Output = Double_Pid_Ctrl(Hero_Holder.Pitch.Target_Angle, Hero_Holder.Pitch.Angle, Hero_Holder.Pitch.Speed, &Pitch_pid);
//				}
//		
//				else if(Hero_Holder.Flag.Long_distance_shoot_model == 1)
//				{
//					  Hero_Holder.Yaw.Output = Double_Pid_Ctrl(Hero_Holder.Yaw.Target_Angle, Hero_Holder.Yaw.Angle, Hero_Holder.Yaw.Speed, &Yaw_pid_long_distance);
//						Hero_Holder.Pitch.Output = Double_Pid_Ctrl(Hero_Holder.Pitch.Target_Angle, Hero_Holder.Pitch.Angle, Hero_Holder.Pitch.Speed, &Pitch_pid_long_distance);
//	      }

//				
//	
//			}
//		else
//		 {
//				Hero_Holder.Pitch.Output=0;
//				Hero_Holder.Yaw.Output=0;	
//		 }
//  }
//}
