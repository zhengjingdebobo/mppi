#include "flysky_sbus.h"
#include "string.h"
#include "drv_usart.h"
//#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "drv_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 25u // 遥控器接收的buffer大小

// 遥控器数据
 RC_ctrl_t rc_ctrl;     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

// 发射机连接状态（通过 SBUS failsafe 标志位判断遥控器是否真正开机）
// 接收机上电就会发帧，但不能说明遥控器开着
static uint8_t  rc_tx_connected = 0u;
static uint16_t rc_failsafe_count = 0u;
#define RC_TX_LOST_DEBOUNCE  10u   // 连续收到 10 帧 failsafe 才判定遥控器关机 (~140ms @70Hz)

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
//    for (uint8_t i = 0; i < 5; ++i)
//        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
//            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
	
	static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 1. 定义协议常量（替代宏）
    const uint8_t  SBUS_START_BYTE  = 0x0F;
    const uint8_t  SBUS_END_BYTE  = 0x00;
    // SBUS 第 23 字节标志位
    const uint8_t  SBUS_FLAG_FAILSAFE = 0x04u;  // bit2: failsafe 激活
    const uint8_t  SBUS_FLAG_FRAME_LOST = 0x08u; // bit3: 帧丢失
    uint8_t flags;
    uint8_t is_failsafe;


    // 2. 检查帧头有效性
    if(sbus_buf[0] == SBUS_START_BYTE && sbus_buf[24] == SBUS_END_BYTE)
    {
	rc_ctrl.rc_channels[0] = ((uint16_t)sbus_buf[ 1] >> 0 | ((int16_t)sbus_buf[ 2] << 8 )) & 0x07FF;
	rc_ctrl.rc_channels[1] = ((uint16_t)sbus_buf[ 2] >> 3 | ((int16_t)sbus_buf[ 3] << 5 )) & 0x07FF;
	rc_ctrl.rc_channels[2] = ((uint16_t)sbus_buf[ 3] >> 6 | ((int16_t)sbus_buf[ 4] << 2 )  | (int16_t)sbus_buf[ 5] << 10 ) & 0x07FF;
	rc_ctrl.rc_channels[3] = ((uint16_t)sbus_buf[ 5] >> 1 | ((int16_t)sbus_buf[ 6] << 7 )) & 0x07FF;
	rc_ctrl.rc_channels[4] = ((uint16_t)sbus_buf[ 6] >> 4 | ((int16_t)sbus_buf[ 7] << 4 )) & 0x07FF;
	rc_ctrl.rc_channels[5] = ((uint16_t)sbus_buf[ 7] >> 7 | ((int16_t)sbus_buf[ 8] << 1 )  | (int16_t)sbus_buf[9] <<  9 ) & 0x07FF;
	rc_ctrl.rc_channels[6] = ((uint16_t)sbus_buf[ 9] >> 2 | ((int16_t)sbus_buf[10] << 6 )) & 0x07FF;
	rc_ctrl.rc_channels[7] = ((uint16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[11] << 3 )) & 0x07FF;

	rc_ctrl.rc_channels[8] = ((uint16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF;
	rc_ctrl.rc_channels[9] = ((uint16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF;
	rc_ctrl.rc_channels[10] = ((uint16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 )  | (int16_t)sbus_buf[16] << 10 ) & 0x07FF;
	rc_ctrl.rc_channels[11] = ((uint16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF;
	rc_ctrl.rc_channels[12] = ((uint16_t)sbus_buf[17] >> 4 | ((int16_t)sbus_buf[18] << 4 )) & 0x07FF;
	rc_ctrl.rc_channels[13] = ((uint16_t)sbus_buf[18] >> 7 | ((int16_t)sbus_buf[19] << 1 )  | (int16_t)sbus_buf[20] <<  9 ) & 0x07FF;
	rc_ctrl.rc_channels[14] = ((uint16_t)sbus_buf[20] >> 2 | ((int16_t)sbus_buf[21] << 6 )) & 0x07FF;
	rc_ctrl.rc_channels[15] = ((uint16_t)sbus_buf[21] >> 5 | ((int16_t)sbus_buf[22] << 3 )) & 0x07FF;

        // 3. 读取第 23 字节标志位，判断遥控器是否真正开机
        flags = sbus_buf[23];
        is_failsafe = ((flags & SBUS_FLAG_FAILSAFE) != 0u) ||
                       ((flags & SBUS_FLAG_FRAME_LOST) != 0u);

        if (is_failsafe)
        {
            // 连续 failsafe 帧 → 累计
            if (rc_failsafe_count < RC_TX_LOST_DEBOUNCE)
                rc_failsafe_count++;

            if (rc_failsafe_count >= RC_TX_LOST_DEBOUNCE && rc_tx_connected != 0u)
            {
                rc_tx_connected = 0u;
                // 清零通道值，防止遥控器关机时残留数据被误读
                // channel[2] 是使能开关，清零确保 rc_manual_enabled = 0
                memset(rc_ctrl.rc_channels, 0, sizeof(rc_ctrl.rc_channels));
            }
        }
        else
        {
            // 正常帧 → 递减计数器，降到 0 表示遥控器已恢复
            if (rc_failsafe_count > 0u)
                rc_failsafe_count--;

            if (rc_failsafe_count == 0u && rc_tx_connected == 0u)
                rc_tx_connected = 1u;
        }
    }
}

	
	

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(&rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    rc_tx_connected = 0u;
    rc_failsafe_count = 0u;
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
//    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 20, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return &rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return (DaemonIsOnline(rc_daemon_instance) && rc_tx_connected) ? 1u : 0u;
    return 0;
}
