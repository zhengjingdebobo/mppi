#include "vision.h"
#include "kalman.h" 

int16_t test1028 = 0;
uint8_t Vision_to_NUC[6];//__attribute__((at(0x24016000)));
//uint8_t Vision_meta[6];//__attribute__((at(0x24018000)));//nuc缓存数组

Hero_Vision_t  Hero_Vision =  
{
	 .Sensitivity=
	 {
		 .Yaw_K = -0.9f, 
		 .Pitch_K =-0.8f  //-0.9f

	 } 
};


void Run_User_Vision_Task_1(Hero_Holder_t* holder)
{
	uint8_t Vision_Ready = 0;
	Brain_DataUnpack(&Brain,BrainToRobotBuffer);
	
  if(Brain.BrainCore.CoreFlag.Connect==1)
	Vision_Ready++;
	if(Brain.BrainCore.CoreFlag.Open==1)
	Vision_Ready++;		
	if(Brain.BrainCore.CoreFlag.Connect==1)
	Vision_Ready++;	
  if(Brain.BrainCore.CoreFlag.Working ==1)
 	Vision_Ready++;	
	

	if(Hero_Vision.Flag.open == 1)//&& Vision_Ready==4 不是手动射击 并且 是上位机到下位机的指令数据帧
	{
		
		if(abs(Brain.BrainCore.CoreInstruction.PitchDeflectionAngle<30)&&abs(Brain.BrainCore.CoreInstruction.YawDeflectionAngle<30))
		{
			
			holder->Yaw.Target_Angle =holder->Yaw.Angle+ (Brain.BrainCore.CoreInstruction.YawDeflectionAngle * Hero_Vision.Sensitivity.Yaw_K);
		//	Brain.BrainCore.CoreInstruction.PitchDeflectionAngle = KalmanFilter_pitch(Brain.BrainCore.CoreInstruction.PitchDeflectionAngle,IMU_KALMAN_Q,IMU_KALMAN_R);
			holder->Pitch.Target_Angle=holder->Pitch.Angle + (Brain.BrainCore.CoreInstruction.PitchDeflectionAngle * Hero_Vision.Sensitivity.Pitch_K);
			

		}
		else 
		{ 					
			Brain.BrainCore.CoreInstruction.YawDeflectionAngle=0;
			Brain.BrainCore.CoreInstruction.PitchDeflectionAngle=0;
	  }

		
	}
	else
		{ 					
			Brain.BrainCore.CoreInstruction.YawDeflectionAngle=0;
			Brain.BrainCore.CoreInstruction.PitchDeflectionAngle=0;
	  }
			
		 Hero_Vision.cnt_vision_ALL++ ;
		 Hero_Vision.cnt_vision++;


	}
