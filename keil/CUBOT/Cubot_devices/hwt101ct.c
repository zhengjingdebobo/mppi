//#include "hwt101ct.h"

//#include <string.h>

//#include "drv_usart.h"
//#include "daemon.h"
//#include "drv_log.h"    // 若不需要日志，可删除这一行


///* ========================= 协议相关宏 ========================= */

//// 一帧长度固定 11 字节：0x55 + TYPE + 8 字节数据 + SUM
//#define HWT101_FRAME_SIZE           11u

//// 协议常量
//#define HWT101_FRAME_HEADER         0x55
//#define HWT101_FRAME_TYPE_GYRO      0x52   // 角速度帧
//#define HWT101_FRAME_TYPE_ANGLE     0x53   // 角度帧

///* ========================= 全局输出数据 ========================= */

//HWT101CT_t hwt101ct_ctrl;

///* ====================== 文件内静态状态变量 ====================== */

//// 串口实例 & 守护进程实例（只在本文件内可见）

//uint8_t g_hwt_rx_buf[11]={0};   // 全局可见，调试器里能直接看
//static USARTInstance  *hwt_usart_instance;
//static DaemonInstance *hwt_daemon_instance;

//// 初始化标志位
//static uint8_t hwt101ct_init_flag = 0;

//// 展开角度内部状态（上一帧单圈角度 & 是否已初始化）
//static float   s_yaw_prev_deg   = 0.0f;
//static uint8_t s_yaw_prev_valid = 0;

///* ====================== 内部函数声明（静态） ====================== */

//static uint8_t HWT101_CheckSum(const uint8_t *buf);
//static void    HWT101_UpdateYawUnwrap(void);
//static void    HWT101_DecodeAngleFrame(const uint8_t *buf);
//static void    HWT101_DecodeGyroFrame(const uint8_t *buf);

//static void    HWT101_LostCallback(void *id);

///* ========================= 内部函数实现 ========================= */

///**
// * @brief 校验和检查，适用于 11 字节帧（TYPE = 0x52 / 0x53 等）
// */
//static uint8_t HWT101_CheckSum(const uint8_t *buf)
//{
//    uint8_t sum = 0;
//    for (uint8_t i = 0; i < HWT101_FRAME_SIZE - 1; ++i)
//    {
//        sum += buf[i];
//    }
//    return (sum == buf[HWT101_FRAME_SIZE - 1]);
//}

///**
// * @brief 根据当前 yaw_deg 和上一帧 yaw_deg，把 [-180,180) 展开到 (-inf,+inf)
// *
// * 前提：采样频率约 500 Hz，两帧之间真实旋转量不会超过 180°。
// *
// * 设：
// *   prev = 上一帧单圈角度（[-180, 180)）
// *   curr = 当前帧单圈角度（[-180, 180)）
// *
// *   delta_raw = curr - prev
// *
// *   若 delta_raw ∈ [-180, 180]，则直接视为真实增量；
// *   若 delta_raw >  180，则从负端跨到正端（例如 -175 -> 155），
// *      真实增量 = delta_raw - 360，并 turn_count--；
// *   若 delta_raw < -180，则从正端跨到负端（例如 175 -> -155），
// *      真实增量 = delta_raw + 360，并 turn_count++。
// */
//static void HWT101_UpdateYawUnwrap(void)
//{
//    float curr = hwt101ct_ctrl.yaw_deg;

//    // 第一次有有效角度，直接初始化
//    if (!s_yaw_prev_valid)
//    {
//        s_yaw_prev_valid       = 1;
//        s_yaw_prev_deg         = curr;
//        hwt101ct_ctrl.turn_count    = 0;
//        hwt101ct_ctrl.yaw_total_deg = curr;
//        return;
//    }

//    float delta_raw = curr - s_yaw_prev_deg;
//    float delta     = delta_raw;

//    // 正向跨界：例如 -175 -> 155，delta_raw = 330
//    if (delta_raw > 180.0f)
//    {
//        delta = delta_raw - 360.0f;   // 330 -> -30
//        hwt101ct_ctrl.turn_count--;   // 累计逆时针一圈
//    }
//    // 反向跨界：例如 175 -> -155，delta_raw = -330
//    else if (delta_raw < -180.0f)
//    {
//        delta = delta_raw + 360.0f;   // -330 -> +30
//        hwt101ct_ctrl.turn_count++;   // 累计顺时针一圈
//    }

//    // 累加展开角度
//    hwt101ct_ctrl.yaw_total_deg += delta;

//    // 记录当前角度，供下一帧使用
//    s_yaw_prev_deg = curr;
//}

///**
// * @brief 解析角度帧（TYPE = 0x53）
// *
// * 帧格式：
// *   0x55 0x53 0x00 0x00 0x00 0x00 YawL YawH VL VH SUM
// *
// * 协议约定：
// *   Yaw = ((YawH << 8) | YawL) / 32768 * 180 (deg)
// */
//static void HWT101_DecodeAngleFrame(const uint8_t *buf)
//{
//    // 头 + 校验
//    if (buf[0] != HWT101_FRAME_HEADER)
//    {
//        return;
//    }
//    if (!HWT101_CheckSum(buf))
//    {
//        return;
//    }

//    int16_t yaw_raw = (int16_t)(((int16_t)buf[7] << 8) | buf[6]);

//    hwt101ct_ctrl.yaw_raw = yaw_raw;
//    hwt101ct_ctrl.yaw_deg =
//        (float)yaw_raw * 180.0f / 32768.0f;

//    // 更新展开角度和圈数
//    HWT101_UpdateYawUnwrap();
//}

///**
// * @brief 解析角速度帧（TYPE = 0x52）
// *
// * 典型帧格式（Z 轴示意）：
// *   0x55 0x52 0x00 0x00 RWzL RWzH WzL WzH 0x00 0x00 SUM
// *
// * 协议约定：
// *   RWz = ((RWzH << 8) | RWzL) / 32768 * 2000 (deg/s)
// *   Wz  = ((WzH << 8) | WzL)   / 32768 * 2000 (deg/s)
// */
//static void HWT101_DecodeGyroFrame(const uint8_t *buf)
//{
//    // 头 + 校验
//    if (buf[0] != HWT101_FRAME_HEADER)
//    {
//        return;
//    }
//    if (!HWT101_CheckSum(buf))
//    {
//        return;
//    }

//    int16_t rwz_raw =
//        (int16_t)(((int16_t)buf[5] << 8) | buf[4]);   // RWzH, RWzL
//    int16_t wz_raw  =
//        (int16_t)(((int16_t)buf[7] << 8) | buf[6]);   // WzH,  WzL

//    hwt101ct_ctrl.rwz_raw   = rwz_raw;
//    hwt101ct_ctrl.rwz_deg_s =
//        (float)rwz_raw * 2000.0f / 32768.0f;

//    hwt101ct_ctrl.wz_raw    = wz_raw;
//    hwt101ct_ctrl.wz_deg_s  =
//        (float)wz_raw * 2000.0f / 32768.0f;
//}

///**
// * @brief 串口接收完成回调（注册给 drv_usart）
// *
// * 由 drv_usart 在接收到一帧完整数据时调用
// */


//void HWT101_RxCallback(void)
//{
//	
//	DaemonReload(hwt_daemon_instance);
//	
//	
//    if (!hwt101ct_init_flag)
//    {
//        return;
//    }

//    // 喂看门狗，表示本周期有新数据
//    

//    memcpy(g_hwt_rx_buf, hwt_usart_instance->recv_buff, HWT101_FRAME_SIZE);
//	
//    uint8_t type = g_hwt_rx_buf[1];

//    switch (type)
//    {
//    case HWT101_FRAME_TYPE_ANGLE:
//        HWT101_DecodeAngleFrame(g_hwt_rx_buf);
//        break;

//    case HWT101_FRAME_TYPE_GYRO:
//        HWT101_DecodeGyroFrame(g_hwt_rx_buf);
//        break;

//    default:
//        // 其它类型暂时不关心
//        break;
//    }
//}






///**
// * @brief 守护进程回调：认为陀螺仪掉线时调用
// */
//static void HWT101_LostCallback(void *id)
//{
////    (void)id;

//    // 清空输出数据
////    memset(&hwt101ct_ctrl, 0, sizeof(hwt101ct_ctrl));

////    // 清空展开角度内部状态
////    s_yaw_prev_valid = 0;
////    s_yaw_prev_deg   = 0.0f;

//    // 尝试重新启动串口接收
//    USARTServiceInit(hwt_usart_instance);

////    LOGWARNING("[hwt101] imu offline");
//}

///* ========================= 对外接口实现 ========================= */

///**
// * @brief 初始化 HWT101 串口模块
// */
//HWT101CT_t *HWT101CTInit(UART_HandleTypeDef *hwt_usart_handle)
//{
//    // 清零输出数据
//    memset(&hwt101ct_ctrl, 0, sizeof(hwt101ct_ctrl));
//    s_yaw_prev_valid = 0;
//    s_yaw_prev_deg   = 0.0f;

//    // 1. 注册到串口服务
//    USART_Init_Config_s usart_conf;
//    usart_conf.module_callback = HWT101_RxCallback;
//    usart_conf.usart_handle    = hwt_usart_handle;
//    usart_conf.recv_buff_size  = HWT101_FRAME_SIZE;   // 一帧 11 字节

//    hwt_usart_instance = USARTRegister(&usart_conf);

//    // 2. 注册到守护进程，用来判断在线/掉线
//    Daemon_Init_Config_s daemon_conf = {
//        .reload_count = 200,       // 约 100ms 超时，可按实际输出频率调整
//        .callback     = HWT101_LostCallback,
//        .owner_id     = NULL,
//    };
//    hwt_daemon_instance = DaemonRegister(&daemon_conf);

//    hwt101ct_init_flag = 1;

//    return &hwt101ct_ctrl;
//}



///**
// * @brief 查询陀螺仪是否在线
// */
//uint8_t HWT101CTIsOnline(void)
//{
//    if (!hwt101ct_init_flag)
//    {
//        return 0;
//    }
//    return DaemonIsOnline(hwt_daemon_instance);
//}






#include "hwt101ct.h"

#include <string.h>

#include "drv_usart.h"
#include "daemon.h"
#include "drv_log.h"    // 若不需要日志，可删除这一行

/* ========================= 协议相关宏 ========================= */

// 一帧长度固定 11 字节：0x55 + TYPE + 8 字节数据 + SUM
#define HWT101_FRAME_SIZE           11u

// 协议常量
#define HWT101_FRAME_HEADER         0x55
#define HWT101_FRAME_TYPE_GYRO      0x52   // 角速度帧
#define HWT101_FRAME_TYPE_ANGLE     0x53   // 角度帧

/* ========================= 全局输出数据 ========================= */

HWT101CT_t hwt101ct_ctrl;

/* ====================== 文件内静态状态变量 ====================== */

// 串口实例 & 守护进程实例（只在本文件内可见）
uint8_t g_hwt_rx_buf[11] = {0};      // 全局可见，调试器里能直接看
static USARTInstance  *hwt_usart_instance;
static DaemonInstance *hwt_daemon_instance;

// 初始化标志位
static uint8_t hwt101ct_init_flag = 0;

// 展开角度内部状态（上一帧单圈角度 & 是否已初始化）
static float   s_yaw_prev_deg    = 0.0f;
static uint8_t s_yaw_prev_valid  = 0;

// 软件零点：以展开角 yaw_total_deg 为基准的零点偏置
static float   s_yaw_zero_total  = 0.0f;
static uint8_t s_yaw_zero_valid  = 0;

/* ====================== 内部函数声明（静态） ====================== */

static uint8_t HWT101_CheckSum(const uint8_t *buf);
static void    HWT101_UpdateYawUnwrap(void);
static void    HWT101_UpdateYawZeroBased(void);
static void    HWT101_DecodeAngleFrame(const uint8_t *buf);
static void    HWT101_DecodeGyroFrame(const uint8_t *buf);

static void    HWT101_LostCallback(void *id);

/* ========================= 内部函数实现 ========================= */

/**
 * @brief 校验和检查，适用于 11 字节帧（TYPE = 0x52 / 0x53 等）
 */
static uint8_t HWT101_CheckSum(const uint8_t *buf)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < HWT101_FRAME_SIZE - 1; ++i)
    {
        sum += buf[i];
    }
    return (sum == buf[HWT101_FRAME_SIZE - 1]);
}

/**
 * @brief 根据当前 yaw_deg 和上一帧 yaw_deg，把 [-180,180) 展开到 (-inf,+inf)
 *
 * 前提：采样频率约 500 Hz，两帧之间真实旋转量不会超过 180°。
 *
 * 设：
 *   prev = 上一帧单圈角度（[-180, 180)）
 *   curr = 当前帧单圈角度（[-180, 180)）
 *
 *   delta_raw = curr - prev
 *
 *   若 delta_raw ∈ [-180, 180]，则直接视为真实增量；
 *   若 delta_raw >  180，则从负端跨到正端（例如 -175 -> 155），
 *      真实增量 = delta_raw - 360，并 turn_count--；
 *   若 delta_raw < -180，则从正端跨到负端（例如 175 -> -155），
 *      真实增量 = delta_raw + 360，并 turn_count++。
 */
static void HWT101_UpdateYawUnwrap(void)
{
    float curr = hwt101ct_ctrl.yaw_deg;

    // 第一次有有效角度，直接初始化
    if (!s_yaw_prev_valid)
    {
        s_yaw_prev_valid        = 1;
        s_yaw_prev_deg          = curr;
        hwt101ct_ctrl.turn_count    = 0;
        hwt101ct_ctrl.yaw_total_deg = curr;  // 展开角从第一帧开始
        return;
    }

    float delta_raw = curr - s_yaw_prev_deg;
    float delta     = delta_raw;

    // 正向跨界：例如 -175 -> 155，delta_raw = 330
    if (delta_raw > 180.0f)
    {
        delta = delta_raw - 360.0f;   // 330 -> -30
        hwt101ct_ctrl.turn_count--;   // 视为顺时针转了一圈（具体正负按你实际约定）
    }
    // 反向跨界：例如 175 -> -155，delta_raw = -330
    else if (delta_raw < -180.0f)
    {
        delta = delta_raw + 360.0f;   // -330 -> +30
        hwt101ct_ctrl.turn_count++;   // 视为逆时针转了一圈
    }

    // 累加展开角度
    hwt101ct_ctrl.yaw_total_deg += delta;

    // 记录当前角度，供下一帧使用
    s_yaw_prev_deg = curr;
}

/**
 * @brief 根据当前展开角度 yaw_total_deg，更新“零点后”的角度
 *
 *  - s_yaw_zero_total：软件零点在 yaw_total_deg 上的值
 *  - yaw_total_zxj   = yaw_total_deg - s_yaw_zero_total
 *  - yaw_zxj         = yaw_total_zxj 折叠到 [-180, 180]
 *
 * 初始化行为：
 *  - 在收到第一帧有效角度后，如果从未设置过零点，会自动把
 *    当前 yaw_total_deg 作为零点，使得 yaw_total_zxj 从 0 开始。
 */
static void HWT101_UpdateYawZeroBased(void)
{
    // 如果还没有任何展开角，可直接返回
    // （理论上只在初始化前几帧可能触发）
    float total = hwt101ct_ctrl.yaw_total_deg;

    // 初始化阶段：自动把第一次的 yaw_total_deg 当成零点
    if (!s_yaw_zero_valid)
    {
        s_yaw_zero_total = total;
        s_yaw_zero_valid = 1;
    }

    // 计算相对零点的展开角
    float total_rel = total - s_yaw_zero_total;
    hwt101ct_ctrl.yaw_total_zxj = total_rel;

    // 折叠到 [-180, 180] 作为单圈角 yaw_zxj
    float yaw_rel = total_rel;
    while (yaw_rel > 180.0f)
    {
        yaw_rel -= 360.0f;
    }
    while (yaw_rel < -180.0f)
    {
        yaw_rel += 360.0f;
    }
    hwt101ct_ctrl.yaw_zxj = yaw_rel;
}

/**
 * @brief 解析角度帧（TYPE = 0x53）
 *
 * 帧格式：
 *   0x55 0x53 0x00 0x00 0x00 0x00 YawL YawH VL VH SUM
 *
 * 协议约定：
 *   Yaw = ((YawH << 8) | YawL) / 32768 * 180 (deg)
 */
static void HWT101_DecodeAngleFrame(const uint8_t *buf)
{
    // 头 + 校验
    if (buf[0] != HWT101_FRAME_HEADER)
    {
        return;
    }
    if (!HWT101_CheckSum(buf))
    {
        return;
    }

    int16_t yaw_raw = (int16_t)(((int16_t)buf[7] << 8) | buf[6]);

    hwt101ct_ctrl.yaw_raw = yaw_raw;
    hwt101ct_ctrl.yaw_deg =
        (float)yaw_raw * 180.0f / 32768.0f;
    // 类似归一化 解包数据
    // 传输数据，浮点数传输成本高 -> 映射到二字节（16 位整数）传输角度信息

    // 更新展开角度和圈数（yaw_total_deg / turn_count）
    HWT101_UpdateYawUnwrap();

    // 基于展开角度更新零点相关的 yaw_zxj / yaw_total_zxj
    HWT101_UpdateYawZeroBased();
}

/**
 * @brief 解析角速度帧（TYPE = 0x52）
 *
 * 典型帧格式（Z 轴示意）：
 *   0x55 0x52 0x00 0x00 RWzL RWzH WzL WzH 0x00 0x00 SUM
 *
 * 协议约定：
 *   RWz = ((RWzH << 8) | RWzL) / 32768 * 2000 (deg/s)
 *   Wz  = ((WzH << 8) | WzL)   / 32768 * 2000 (deg/s)
 */
static void HWT101_DecodeGyroFrame(const uint8_t *buf)
{
    // 头 + 校验
    if (buf[0] != HWT101_FRAME_HEADER)
    {
        return;
    }
    if (!HWT101_CheckSum(buf))
    {
        return;
    }

    int16_t rwz_raw =
        (int16_t)(((int16_t)buf[5] << 8) | buf[4]);   // RWzH, RWzL
    int16_t wz_raw  =
        (int16_t)(((int16_t)buf[7] << 8) | buf[6]);   // WzH,  WzL

    hwt101ct_ctrl.rwz_raw   = rwz_raw;
    hwt101ct_ctrl.rwz_deg_s =
        (float)rwz_raw * 2000.0f / 32768.0f;

    hwt101ct_ctrl.wz_raw    = wz_raw;
    hwt101ct_ctrl.wz_deg_s  =
        (float)wz_raw * 2000.0f / 32768.0f;
}

/**
 * @brief 串口接收完成回调（注册给 drv_usart）
 *
 * 由 drv_usart 在接收到一帧完整数据时调用
 */
void HWT101_RxCallback(void)
{
    // 喂看门狗，表示本周期有新数据
    DaemonReload(hwt_daemon_instance);

    if (!hwt101ct_init_flag)
    {
        return;
    }

    // 将 drv_usart 的 recv_buff 拷贝到调试用缓冲区
    memcpy(g_hwt_rx_buf, hwt_usart_instance->recv_buff, HWT101_FRAME_SIZE);

    uint8_t type = g_hwt_rx_buf[1];

    switch (type)
    {
    case HWT101_FRAME_TYPE_ANGLE:
        HWT101_DecodeAngleFrame(g_hwt_rx_buf);
        break;

    case HWT101_FRAME_TYPE_GYRO:
        HWT101_DecodeGyroFrame(g_hwt_rx_buf);
        break;

    default:
        // 其它类型暂时不关心
        break;
    }
}

/**
 * @brief 守护进程回调：认为陀螺仪掉线时调用
 */
static void HWT101_LostCallback(void *id)
{
    (void)id;

    // 掉线时可以选择是否清空数据，这里只尝试重启串口服务
    USARTServiceInit(hwt_usart_instance);
    // LOGWARNING("[hwt101] imu offline");
}

/* ========================= 对外接口实现 ========================= */

/**
 * @brief 初始化 HWT101 串口模块
 */
HWT101CT_t *HWT101CTInit(UART_HandleTypeDef *hwt_usart_handle)
{
    // 清零输出数据
    memset(&hwt101ct_ctrl, 0, sizeof(hwt101ct_ctrl));

    // 清空展开角度内部状态
    s_yaw_prev_valid = 0;
    s_yaw_prev_deg   = 0.0f;

    // 清空软件零点状态：等待第一帧角度自动设零
    s_yaw_zero_valid = 0;
    s_yaw_zero_total = 0.0f;

    // 1. 注册到串口服务
    USART_Init_Config_s usart_conf;
    usart_conf.module_callback = HWT101_RxCallback;
    usart_conf.usart_handle    = hwt_usart_handle;
    usart_conf.recv_buff_size  = HWT101_FRAME_SIZE;   // 一帧 11 字节

    hwt_usart_instance = USARTRegister(&usart_conf);

    // 2. 注册到守护进程，用来判断在线/掉线
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 200,       // 约 100ms 超时，可按实际输出频率调整
        .callback     = HWT101_LostCallback,
        .owner_id     = NULL,
    };
    hwt_daemon_instance = DaemonRegister(&daemon_conf);

    hwt101ct_init_flag = 1;

    return &hwt101ct_ctrl;
}

/**
 * @brief 查询陀螺仪是否在线
 */
uint8_t HWT101CTIsOnline(void)
{
    if (!hwt101ct_init_flag)
    {
        return 0;
    }
    return DaemonIsOnline(hwt_daemon_instance);
}

/**
 * @brief 软件零点清零接口
 *
 * 调用后：
 *  - 以当前展开角 yaw_total_deg 为新的零点；
 *  - 立刻更新 yaw_total_zxj = 0，yaw_zxj = 0。
 */
void HWT101CTSetYawZero(void)
{
    if (!hwt101ct_init_flag)
    {
        return;
    }

    // 如果还没有任何角度数据，就等下一帧自动设零
    if (!s_yaw_prev_valid)
    {
        s_yaw_zero_valid = 0;
        s_yaw_zero_total = 0.0f;
        hwt101ct_ctrl.yaw_zxj        = 0.0f;
        hwt101ct_ctrl.yaw_total_zxj  = 0.0f;
        return;
    }

    // 以当前展开角度为新的零点
    s_yaw_zero_total = hwt101ct_ctrl.yaw_total_deg;
    s_yaw_zero_valid = 1;

    // 当前帧立即视为 0 度
    hwt101ct_ctrl.yaw_total_zxj = 0.0f;
    hwt101ct_ctrl.yaw_zxj       = 0.0f;
}

