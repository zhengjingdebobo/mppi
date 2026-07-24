//#ifndef HWT101CT_H
//#define HWT101CT_H

//#include "FreeRTOS.h"

//#include <stdint.h>
//#include "main.h"
//#include "usart.h"

///**
// * @brief HWT101CT 陀螺仪数据结构
// *
// * 命名说明：
// *  - yaw_raw        : 原始角度 short（协议中的 Yaw）
// *  - yaw_deg        : 单圈角度 [-180, 180)
// *  - yaw_total_deg  : 展开后的角度 (-inf, +inf)
// *  - turn_count     : 累计圈数（正：逆时针，负：顺时针）
// *  - rwz_raw        : 原始角速度 short（协议中的 RWz）
// *  - rwz_deg_s      : 原始角速度，单位 deg/s
// *  - wz_raw         : 校准后角速度 short（协议中的 Wz）
// *  - wz_deg_s       : 校准后角速度，单位 deg/s
// */
//typedef struct
//{
//    /* --- 角度相关 --- */
//    int16_t yaw_raw;
//    float   yaw_deg; //范围-180到+180
//    float   yaw_total_deg; //范围负无穷到正无穷
//	float   yaw_zxj;  //增加零点,范围-180到+180
//    float   yaw_total_zxj; //增加零点,范围负无穷到正无穷
//	
//    int32_t turn_count;

//    /* --- 角速度相关 --- */
//    int16_t rwz_raw;
//    float   rwz_deg_s;

//    int16_t wz_raw;
//    float   wz_deg_s;
//} HWT101CT_t;

///* 全局数据实例 */
//extern HWT101CT_t hwt101ct_ctrl;
//extern uint8_t g_hwt_rx_buf[11];

///**
// * @brief 初始化 HWT101CT 串口模块，并注册到 drv_usart / Daemon
// *
// * @param hwt_usart_handle  陀螺仪使用的 UART 句柄（例如 &huart1）
// * @return HWT101CT_t*      返回全局数据指针（&hwt101ct_ctrl）
// */
//HWT101CT_t *HWT101CTInit(UART_HandleTypeDef *hwt_usart_handle);

///**
// * @brief 查询陀螺仪是否在线（基于 Daemon）
// *
// * @return 1：在线；0：离线或未初始化
// */
//uint8_t HWT101CTIsOnline(void);


//void HWT101_RxCallback(void);




//#endif /* HWT101CT_H */

#ifndef HWT101CT_H
#define HWT101CT_H

#include "FreeRTOS.h"

#include <stdint.h>
#include "main.h"
#include "usart.h"

/**
 * @brief HWT101CT 陀螺仪数据结构
 *
 * 命名说明：
 *  - yaw_raw        : 原始角度 short（协议中的 Yaw）
 *  - yaw_deg        : 单圈角度 [-180, 180)
 *  - yaw_total_deg  : 展开后的角度 (-inf, +inf)
 *  - yaw_zxj        : 以“软件零点”为参考的单圈角度 [-180, 180)
 *  - yaw_total_zxj  : 以“软件零点”为参考的展开角度 (-inf, +inf)
 *  - turn_count     : 累计圈数（正：顺时针，负：逆时针）
 *  - rwz_raw        : 原始角速度 short（协议中的 RWz）
 *  - rwz_deg_s      : 原始角速度，单位 deg/s
 *  - wz_raw         : 校准后角速度 short（协议中的 Wz）
 *  - wz_deg_s       : 校准后角速度，单位 deg/s
 */
typedef struct
{
    /* --- 角度相关 --- */
    int16_t yaw_raw;
    float   yaw_deg;         // 范围 -180 ~ +180（传感器原始单圈）
    float   yaw_total_deg;   // 展开角度 (-inf ~ +inf)

    float   yaw_zxj;         // 增加零点后的单圈角度，范围 -180 ~ +180
    float   yaw_total_zxj;   // 增加零点后的展开角度，范围 (-inf ~ +inf)

    int32_t turn_count;      // 圈数（与 yaw_total_deg 对应）

    /* --- 角速度相关 --- */
    int16_t rwz_raw;
    float   rwz_deg_s;

    int16_t wz_raw;
    float   wz_deg_s;
} HWT101CT_t;

/* 全局数据实例 */
extern HWT101CT_t hwt101ct_ctrl;
extern uint8_t g_hwt_rx_buf[11];

/**
 * @brief 初始化 HWT101CT 串口模块，并注册到 drv_usart / Daemon
 *
 * @param hwt_usart_handle  陀螺仪使用的 UART 句柄（例如 &huart1）
 * @return HWT101CT_t*      返回全局数据指针（&hwt101ct_ctrl）
 *
 * @note  初始化后会自动设置一次“软件零点”（在收到第一帧角度数据时生效），
 *        即上电后当前姿态视为 0 度。
 */
HWT101CT_t *HWT101CTInit(UART_HandleTypeDef *hwt_usart_handle);

/**
 * @brief 查询陀螺仪是否在线（基于 Daemon）
 *
 * @return 1：在线；0：离线或未初始化
 */
uint8_t HWT101CTIsOnline(void);

/**
 * @brief 串口接收回调（注册给 drv_usart）
 */
void HWT101_RxCallback(void);

/**
 * @brief 软件清零接口
 *
 * 调用后：
 *  - 当前展开角度 yaw_total_deg 被记为零点；
 *  - 之后 yaw_total_zxj = yaw_total_deg - 当前值；
 *  - 当前帧立刻视为 0 度，即 yaw_zxj = 0, yaw_total_zxj = 0。
 */
void HWT101CTSetYawZero(void);

#endif /* HWT101CT_H */






