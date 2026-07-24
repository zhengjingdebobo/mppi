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
#include "drv_usart.h"
#include "drv_log.h"
#include "stdlib.h"
//#include "memory.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT)
	{ // 超过最大实例数
//        while (1)
//				{
//        //    LOGERROR("[bsp_usart] USART exceed max instance count!");
//				}
			}

    for (uint8_t i = 0; i < idx; i++) 
	{
			// 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
				{
//            while (1)
//						{
//							//LOGERROR("[bsp_usart] USART instance already registered!");
//							
//						}
				}
    }
    
    // 1. 申请结构体内存
    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));
    
    // 2. 赋值
    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;
    
    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/* @todo 当前仅进行了形式上的封装,后续要进一步考虑是否将module的行为与bsp完全分离 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数居量,暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            // 模块回调
            if (usart_instance[i]->module_callback != NULL)
            {
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            // 初始化并启动 “DMA + 空闲中断” 模式的串口接收 -> DMA 控制器会自动把 RDR 的字节搬到内存 recv_buff[]
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            // 禁用这个 DMA 的半传输中断
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            //  LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
            return;
        }
    }
}

// 调试不受控制
///**
// * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
// *
// * @note  最常见的错误:奇偶校验/溢出/帧错误
// *
// * @param huart 发生错误的串口
// */
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    // ============================================================
//    // 【新增 1】: 必须显式清除错误标志位，否则串口可能会锁死
//    // 特别是 ORE (溢出错误)，这是调试时最容易遇到的
//    // ============================================================
//    uint32_t isr_flags = READ_REG(huart->Instance->SR); // 读取状态寄存器(F4系列是SR, H7/G4是ISR)
//    
//    if ((HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE) || (isr_flags & UART_FLAG_ORE))
//    {
//        __HAL_UART_CLEAR_OREFLAG(huart); // 清除溢出错误
//    }
//    
//    if ((HAL_UART_GetError(huart) & HAL_UART_ERROR_NE) || (isr_flags & UART_FLAG_NE))
//    {
//        __HAL_UART_CLEAR_NEFLAG(huart); // 清除噪声错误
//    }
//    
//    if ((HAL_UART_GetError(huart) & HAL_UART_ERROR_FE) || (isr_flags & UART_FLAG_FE))
//    {
//        __HAL_UART_CLEAR_FEFLAG(huart); // 清除帧错误
//    }

//    if ((HAL_UART_GetError(huart) & HAL_UART_ERROR_PE) || (isr_flags & UART_FLAG_PE))
//    {
//        __HAL_UART_CLEAR_PEFLAG(huart); // 清除奇偶校验错误
//    }

//    // ============================================================
//    // 【原有逻辑】: 遍历实例，重启 DMA 接收
//    // ============================================================
//    for (uint8_t i = 0; i < idx; ++i)
//    {
//        if (huart == usart_instance[i]->usart_handle)
//        {
//            // 尝试重启接收 (使用 Idle Line 检测模式)
//            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, 
//                                         usart_instance[i]->recv_buff, 
//                                         usart_instance[i]->recv_buff_size);
//            
//            // 关闭 DMA 半传输中断 (防止频繁进入中断，通常我们只关心传完或空闲)
//            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
//            
//            // 可以加一句 Log 方便调试 (可选)
//            // LOGWARNING("[bsp_usart] Error recovered on instance idx [%d], ORE cleared.", i);
//            
//            return;
//        }
//    }
//}
