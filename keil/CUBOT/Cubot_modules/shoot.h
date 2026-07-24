#ifndef SHOOT_H
#define SHOOT_H
////#include "stm32h7xx.h"

//#define abs(x) ((x)>0? (x):(-(x)))//绝对值宏定义
//typedef struct
//{
//	struct
//	{
//		uint8_t Shoot; // 0单发 1自爆
//	}Mode;//射击模式
//	
//	struct
//	{
//		uint8_t Jam;         //堵弹  	
//		uint8_t Load_start;  //1开始拨弹  0拨弹结束
//		uint8_t Fric_On;     //开启摩擦轮
//		uint8_t Fric_Ready;  //摩擦轮准备好
//		uint16_t Heat_Updata;//热量信息更新
//		uint8_t Heat_Over_Limit;    //超热量	
//		uint8_t First_Shoot;    //第一次数据初始化标志位（因为打弹完成的判断依据之一是 摩擦轮转速改变量）
//	//	uint8_t Fric_slow_flag; //缓启动标志位
//		uint8_t Fire_Hit; 
//    uint8_t change_speed;
//		
//	}Flag;
//	
//	struct
//	{
//		int16_t Load_continue_max; //拨弹完成之后继续一段时间，可以减少延迟
//		uint8_t Load_finish_flag;   //拨弹完成标志位
//		int16_t Feedback_Speed;     //拨弹盘反馈速度
//		int16_t Target_Speed;       //拨弹盘目标速度
//		int16_t Turn_Back_Time;     //拨弹盘反转时间
//		float angle;       //拨弹盘角度，由速度进行积分而来
//		int16_t Output;		 //输出值
//	}Load;
//	
//	struct
//	{ 
//    int16_t Target_speed_log[2];    //目标速度的记录值
//		int16_t Target_Speed[2];        //目标速度
//		int16_t Feedback_Speed[2];      //反馈速度
//		int16_t This_Speed;
//		int16_t Last_Speed; //上次的反馈速度
//		int16_t Speed_delta;
//		int16_t Output[2];              //输出
//		int32_t Slow_open_time;         //缓启动时间
//		int32_t Slow_close_time;        //缓关闭时间
//	}Fric;
//	
//}Hero_Shoot_t;


//extern Hero_Shoot_t Hero_Shoot;
///* -----------------------拨弹盘打弹结束判断---------------------------- */
//void Load_finish_judge(Hero_Shoot_t * shoot);
//void shoot_ctrl(void);

#endif  
