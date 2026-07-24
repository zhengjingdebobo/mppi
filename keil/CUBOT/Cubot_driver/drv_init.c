/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  串口bsp层的实现
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "drv_init.h"
#include "drv_log.h"
#include "stdlib.h"
//#include "memory.h"


void BSPInit()
{
    DWT_Init(168);
//  BSPLogInit();
}
