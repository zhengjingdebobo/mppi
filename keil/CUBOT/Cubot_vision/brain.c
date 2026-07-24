#include "brain.h"
#include "vision.h"
#include "string.h"



CubotBrain_t Brain;



/**
  * @brief  上位机数据解算
  */
void Brain_DataUnpack(CubotBrain_t* brain, uint8_t* recBuffer)
{
	if(recBuffer[0] == 0xAA)
	{
		brain->FrameType   = recBuffer[1] ;
		brain->FrameCoreID = recBuffer[2] ;
		if((brain->FrameType == BRAIN_TO_ROBOT_CMD) && recBuffer[9] == 0xDD)  //< 解算偏转角
		{
			
			if((recBuffer[3] >> 6) == 0) 
				brain->BrainCore.CoreInstruction.YawDeflectionAngle = ((float)((recBuffer[3]&0x3f)*100 + recBuffer[4])/100);
			else if((recBuffer[3] >> 6) == 1) 
				brain->BrainCore.CoreInstruction.YawDeflectionAngle = (-1) * ((float)((recBuffer[3]&0x3f)*100 + recBuffer[4])/100);
		
			//< pitch偏转角 在（-63.99 ~ +63.99）范围内
			if((recBuffer[5] >> 6) == 0) 
				brain->BrainCore.CoreInstruction.PitchDeflectionAngle = ((float)((recBuffer[5]&0x3f)*100 + recBuffer[6])/100);
			else if((recBuffer[4] >> 6) == 1) 
				brain->BrainCore.CoreInstruction.PitchDeflectionAngle = (-1) * ((float)((recBuffer[5]&0x3f)*100 + recBuffer[6])/100);
		

			//< 距离信息 在（0 ~ 12.7）范围内
			brain->BrainCore.CoreInstruction.Distance = ((float)(recBuffer[7]))/10;
			brain->BrainCore.CoreInstruction.IsFire   =  recBuffer[8];   //<机器人本体开火建议
		}
		else if((brain->FrameType == BRAIN_TO_ROBOT_HINT) && recBuffer[6] == 0xDD)  //< 解算brain状态
		{
 
      brain->BrainCore.CoreID        = recBuffer[2] ;
			brain->BrainCore.BrainMode     = recBuffer[3];
			brain->BrainCore.BrainVelocity = recBuffer[4];
			brain->BrainCore.CoreFlag.Working  = (recBuffer[5] & 0x08) >> 3;
			brain->BrainCore.CoreFlag.Connect  = (recBuffer[5] & 0x04) >> 2;
		  brain->BrainCore.CoreFlag.Open     = (recBuffer[5] & 0x02) >> 1;		
		  brain->BrainCore.CoreFlag.Init     = (recBuffer[5] & 0x01);	
			
		}

		
	}
}


void Brain_RobotToBrainQuest(uint8_t Type)   //<心跳包保持发送
{
	HeratbeatPacket[0] = 0xAA;
	HeratbeatPacket[1] = ROBOT_TO_BRAIN_HeratbeatPacket & 0x0F;   
	HeratbeatPacket[2] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, HeratbeatPacket, 3);
}



/**
  * @brief  下位机向上位机发送视觉代码重启请求
  */
void Brain_RobotToBrainCmd(uint8_t Type)
{
	CMDBuffer[0] = 0xAA;
	CMDBuffer[1] = ROBOT_CMD_TO_BRAIN;   //<固定为0x05    
	CMDBuffer[2] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, CMDBuffer, 3);
}

/**
  * @brief  下位机向上位机发送视觉代码重启请求
  */
void Brain_RobotToNuc(uint8_t Type)
{
	RobotToNuc[0] = 0xAA;
	RobotToNuc[1] = ROBOT_TO_BRAIN_NUC;   //<固定为0x06
	RobotToNuc[2] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, RobotToNuc, 3);
}



/**
  * @brief  下位机向上位机发送工作模式请求
  */
void Robot_To_Brain_Working(void)
{
	Work_Mode[0] = 0xAA;
	Work_Mode[1] = 0x02;   
	Work_Mode[2] = 0x01;
	Work_Mode[3] = Brain.BrainCore.Robot_Send_Quest.change_work_model;
	Work_Mode[4] = 0xDD;
	HAL_UART_Transmit_DMA(&huart2, Work_Mode, 5);
}
