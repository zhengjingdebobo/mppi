#ifndef DRV_TIMER_H
#define DRV_TIMER_H
//#include "stm32h7xx_hal.h"
#include "tim.h"


typedef void (*TIM_ElapsedCallback)(void); 


typedef struct
{
	TIM_HandleTypeDef* 	Handle;
	uint32_t ClockTime;             			//< 任务定时器的计数变量
	uint32_t UI_ClockTime;             			//< 任务定时器的计数变量
	TIM_ElapsedCallback		ElapCallback;		//< 定时器溢出中断
	uint16_t HolderTime;
	uint16_t ErrorTime;
	uint8_t qiangzhi_reset_flag;
}TIM_Object;

/**
  * @brief 初始化定时器的用户回调
  */
void TIMx_Init(TIM_HandleTypeDef* handle, TIM_ElapsedCallback callback);


/**
  * @brief     开启定时器溢出中断
  */
void TIM_Open(TIM_Object* tim);



extern TIM_Object tim1;;
extern TIM_Object tim15;;

#endif
