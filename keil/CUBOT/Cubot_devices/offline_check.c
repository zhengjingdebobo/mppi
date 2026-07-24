#include "offline_check.h"
//#include "iwdg.h"
//Off_Check_t Offline_Check;

//void Offline_Check_Init(void)
//{
//	Offline_Check.Off_Flag.Dbus=1;
//	Offline_Check.Off_Flag.Fric_L=1;
//	Offline_Check.Off_Flag.Fric_R=1;
//	Offline_Check.Off_Flag.Load=1;
//	Offline_Check.Off_Flag.Pitch=1;
//	Offline_Check.Off_Flag.Referee=1;
//	Offline_Check.Off_Flag.Super_Cap=1;
//	Offline_Check.Off_Flag.Vision=1;
//	Offline_Check.Off_Flag.Wheel =1;
//	Offline_Check.Off_Flag.Yaw=1;


//}
//void Run_Offline_Check(void)
//{
////PITCH
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Pitch>OFF_TIME)	
//		Offline_Check.Off_Flag.Pitch=1;
//	else	Offline_Check.Off_Flag.Pitch=0;
////Ä¦²ÁÂÖ×ó
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Fric_L>OFF_TIME)
//		Offline_Check.Off_Flag.Fric_L=1;
//	else	Offline_Check.Off_Flag.Fric_L=0;
////Ä¦²ÁÂÖÓÒ
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Fric_R>OFF_TIME)
//		Offline_Check.Off_Flag.Fric_R=1;
//	else	Offline_Check.Off_Flag.Fric_R=0;
////ÊÓ¾õ
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Vision>OFF_TIME)
//		Offline_Check.Off_Flag.Vision=1;
//	else	Offline_Check.Off_Flag.Vision=0;
////Ò£¿ØÆ÷	
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Dbus>OFF_TIME)
//		Offline_Check.Off_Flag.Dbus=1;
//	else	Offline_Check.Off_Flag.Dbus=0;
////YAW
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Yaw>OFF_TIME)
//		Offline_Check.Off_Flag.Yaw=1;
//	else	Offline_Check.Off_Flag.Yaw=0;
////ÂÖ×Ó	
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Wheel>OFF_TIME)
//		Offline_Check.Off_Flag.Wheel=1;
//	else	Offline_Check.Off_Flag.Wheel=0;
////²¦µ¯ÅÌ	
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Load>OFF_TIME)
//		Offline_Check.Off_Flag.Load=1;
//	else	Offline_Check.Off_Flag.Load=0;
////³¬¼¶µçÈÝ
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Super_Cap>OFF_TIME)
//		Offline_Check.Off_Flag.Super_Cap=1;
//	else	Offline_Check.Off_Flag.Super_Cap=0;
////²ÃÅÐÏµÍ³
//	if(Offline_Check.Sys_time-Offline_Check.Last_Update.Referee>OFF_TIME)
//		Offline_Check.Off_Flag.Referee=1;
//	else	Offline_Check.Off_Flag.Referee=0;
//}

//void Run_Security_System(void)
//{
//	Run_Offline_Check();
// // HAL_IWDG_Refresh(&hiwdg1);//Î¹¹·
//}
