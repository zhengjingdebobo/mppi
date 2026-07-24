#ifndef HOLDER_H
#define HOLDER_H
////#include "stm32h7xx.h"

//#define abs(x) ((x)>0? (x):(-(x)))

//#define K_Code_Yaw      0.043945f  //360/8192
//#define K_Code_Picth    -0.012925f  //360/8192/3.24
//typedef struct
//{
//	struct
//	{
//		float Target_Angle;
//		int16_t Can_Ori;   //云台正中的时候的编码器角度
//		int16_t Can_Angle_Raw;  //can角度未经过处理
//		float Can_Angle;        //can角度经过处理  -180到180
//		int16_t Can_Speed;      //电机速度
//		float Angle;       //角度   进行pid运算时的角度   可以是陀螺仪角度 也可以是 can角度
//		float Speed;       //角速度 进行pid运算时的角速度 		
//		int16_t Output;    //输出
//	}Yaw;
//	
//	struct
//	{
//		float Target_Angle;
//		int16_t Can_Ori;   //云台正中的时候的编码器角度
//		int16_t Can_Angle_Raw;  //can角度未经过处理
//		float Can_Angle;        //can角度经过处理
//		int16_t Can_Speed;      //电机速度
//		float Angle;       //角度   进行pid运算时的角度   可以是陀螺仪角度 也可以是 can角度
//		float Speed;       //角速度 进行pid运算时的角速度 		
//		int16_t Output;    //输出
//		uint8_t Gyro_or_Can;   //0 是陀螺仪  --- --- 1  是can
//	}Pitch;
//	
//	struct
//	{
//		uint8_t Enable;  //使能 一般直接给1
//		uint8_t Reset_OK;//云台复位成功，一般之进行yaw轴复位
//		uint8_t Back;    //云台转180° 可有可无
//		uint8_t Long_distance_shoot_model; //吊射模式 可有可无
//		uint8_t Holder_static;  //云台固定 可有可无
//		uint8_t Holder_power_ON;
//	}Flag;
//	
//}Hero_Holder_t;


//extern Hero_Holder_t Hero_Holder; //声明结构体
//void Yaw_and_pitch_angle_change(void);//角度转换
//void holder_ctrl(void);  //云台控制函数

//void holder_poweron(void);
//void holder_poweroff(void);





#endif
