#ifndef BRAIN_H__
#define BRAIN_H__
#include "stm32h7xx.h"
#include "usart.h"

#include "holder.h"
#include "dr16.h"
#include "drv_usart.h"

#define ROBOT_TO_BRAIN_HeratbeatPacket 1
#define ROBOT_CMD_TO_BRAIN 5
#define ROBOT_TO_BRAIN_NUC 6
/**
  * @brief    内核的回调函数，用于在结算数据后实时执行指令
  * @param[in]  
  */
typedef uint8_t (*Brain_CoreCallback)(Hero_Holder_t* holder, RC_Ctrl* rc_ctrl); 


/**
  * @brief  上位机发送数据给下位机汇总
  */
typedef enum
{
	BRAIN_TO_ROBOT_CMD   = 1,	 //< 0b0001
	BRAIN_TO_ROBOT_HINT  = 2,	 //< 0b0010
}BraintToRobotCmdCode;


/**
  * @brief  下位机控制上位机汇总
  */
typedef enum
{
	REBOOT_ALL_CORE   = 1,	 //< 0b0001
	REBOOT_BRAIN  = 2,	 //< 0b0010
	REBOOT_NUC = 3,  //< 0b0011
}RobotToBrainCmdCode;



/**
  * @brief  内核模式汇总
  */
typedef enum
{
	MANUAL   = 0,
	SHOOTONE = 1,
	SHOOTTWO = 2,
	SHOOTTHREE = 3,
	SHOOTFOUR = 4,
	SHOOTFIVE = 5,
	SHOOTSENTRY = 6,
	SHOOTOUTPOST = 7,
	SHOOTBASE = 8,
	AUTOMATICHIT = 9,
	CURVEDFIREOUTPOST = 10,
	CURVEDFIREBASE = 11,
	SHOOTSMALLBUFF = 12,
	SHOOTLARGEBUFF = 13,
}BrainModeHint;    


/**
  * @brief  上位机大脑内核结构体
  */
typedef struct
{
	uint8_t CoreID;					//< 内核固定ID
	uint8_t BrainMode;  		//< 请求机器人大脑内核切换的工作模式。
	uint8_t BrainVelocity;  //< 请求机器人大脑内核切换的子弹初速度。
	
	struct 
	{
		float YawDeflectionAngle;
		float PitchDeflectionAngle;
		float Distance;
		int16_t IsFire;
	}CoreInstruction; 

	struct 
	{
		uint8_t Init;
		uint8_t Open;
		uint8_t Connect;  
		uint8_t Working;
	}CoreFlag;
	
		struct 
	{
		uint8_t Restart;   //<关于重启视觉和NUC标志位 1->brain  2->NUC
    uint8_t change_work_model;   //<关于切换工作模式请求(从0x01-->0x0B)
	}Robot_Send_Quest;

	Brain_CoreCallback CoreCallback;	
	
}BrainCore_t;         


/**
  * @brief  上位机大脑结构体
  */
typedef struct
{ 
	uint8_t FrameType;            //数据帧的类型 
	uint8_t FrameCoreID;          //机器人大脑内核编号
	BrainCore_t BrainCore;
}CubotBrain_t;           


/**
  * @brief      上位机数据拆分解算函数
	* @param[in]  brain      上位机数据结构体
	* @param[in]  recBuffer  缓存区数组数据
  */
void Brain_DataUnpack(CubotBrain_t* brain, uint8_t* recBuffer);




extern int16_t dpc_test_working_model;
extern CubotBrain_t Brain;
extern void Brain_RobotToBrainQuest(uint8_t Type);
extern void  Brain_RobotToBrainCmd(uint8_t Type);
extern void Brain_RobotToNuc(uint8_t Type);
extern void Robot_To_Brain_Working(void);
#endif

