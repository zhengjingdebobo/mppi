#ifndef NX16_H
#define NX16_H
//#include "stm32h7xx.h"
#include "FreeRTOS.h"
#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "stdbool.h"

// --- [新增] 命令ID常量 (Agent -> 小车), 与 car_control.py 保持一致 ---
#define CMD_MOVE_FORWARD  0x01
#define CMD_MOVE_BACKWARD 0x02
#define CMD_ROTATE_CCW    0x03 // 逆时针, 左转
#define CMD_ROTATE_CW     0x04 // 顺时针, 右转
#define CMD_STOP          0x05
#define CMD_INIT          0x06 //初始化
#define CMD_PATH_TRACKING	0x07 // 路径跟踪
#define CMD_SWITCH_PATH 0x08 // 切换路径 --用于避障或者增加新的路径

// --- [新增] V2 命令ID常量 (Agent -> 小车), 16字节扩展协议 ---
#define CMD2_MOVE_POLAR_SPEED     0x21 // 按“方向角 + 速度”持续平移
#define CMD2_MOVE_POLAR_DISTANCE  0x22 // 按“方向角 + 距离”执行一次定距离位移
#define CMD2_ROTATE_IN_PLACE      0x23 // 原地旋转，param1=方向，param2=角度
#define CMD2_STOP                 0x24 // 停止当前 API 覆盖或任务
#define CMD2_INIT                 0x25 // 复位任务/路径接收状态
#define CMD2_HEARTBEAT            0x26 // 心跳保活，当前仅记账不触发动作
#define CMD2_ROTATE_SPEED         0x27 // 按给定车体角速度持续原地旋转


// --- [新增] 状态码常量 (小车 -> Agent) ---
#define STATUS_IDLE         0x00
#define STATUS_EXECUTING    0x01
#define STATUS_CMD_SUCCESS  0x02
#define STATUS_CMD_FAILED   0x03

typedef enum {
    NX16_SWITCH_INVALID = -1, // 表明无需切换，适用于首次跟踪
    NX16_SWITCH_IMMEDIATE = 0, // 收到切换请求就立刻换
    NX16_SWITCH_WHEN_SAFE = 1, // // 等底盘满足“安全条件”
} Nx16SwitchMode_t;

/**
 * @brief 获取当前 active 路径指针与长度（只读）
 */
void Nx16_GetDynamicPath(void **points_ptr, size_t *count);

/**
 * @brief 上位机发 CMD_SWITCH_PATH 时调用：登记“要切换”的请求
 * @param mode 切换模式：立即/等停稳
 */
void Nx16_RequestSwitch(Nx16SwitchMode_t mode);

/**
 * @brief 在控制循环里尝试切换 pending->active（成功则 swap）
 * @param safe_to_switch 由 chassis.c 判断（例如 stop_like）
 * @return true=本次成功切换；false=未切换
 */
bool Nx16_TrySwitchActive(bool safe_to_switch);

Nx16SwitchMode_t GetNx16_Switch_mode(void);
static inline bool Nx16_TrySwitchActive_DefaultSafe(void)
{
    return Nx16_TrySwitchActive(true);
}


/* ----------------------- PC Key Definition-------------------------------- */


// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
	uint8_t ModeID;	  //前进，后退，专项
//    uint8_t RxFlag;   //接受信息
//	uint8_t InTask;   //在任务中
	uint8_t TestFlag;
	int16_t TaskTime;
	int16_t TaskTime_zxjtest;
	struct 
	{
		float YawAngle;
		float Distance;
		int16_t IsFire;
	}CoreInstruction; 
	
	
	
    // --- Agent指令相关 ---
    uint8_t CommandID;      // 当前待执行的指令ID
    float   CommandParam;   // 指令参数 (距离或弧度)
    uint8_t RxFlag;         // 收到新指令的标志位
	uint8_t CommandID_test_zxj;

    // --- 任务与状态反馈 ---
    uint8_t InTask;         // 正在执行的任务ID (0表示空闲)
    uint8_t Status;         // 当前状态码 (STATUS_IDLE, etc.)
    uint8_t LastCommandID;  // 最近完成的指令ID

    uint32_t rx_callback_count;
    uint32_t rx_valid_count;
    uint32_t rx_bad_head_count;
    uint32_t rx_bad_checksum_count;
    uint32_t tx_status_count;
    uint32_t tx_status_error_count;
    uint32_t last_tx_status;

    // --- 里程计/位姿数据 ---
    float current_x;        // X坐标 (米)
    float current_y;        // Y坐标 (米)
    float current_yaw;      // 当前累计 Yaw 角 (度)


}BrainCore_t;
  

/*-------------------------------匿名上位机---------------------------------*/
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
// void ANO_V6_Send_Up_Computer(UART_HandleTypeDef* UART_X,int16_t user1,int16_t user2,int16_t user3,int16_t user4,int16_t user5,int16_t user6);
void ANO_V8_Send_UserData(UART_HandleTypeDef* UART_X, uint8_t Frame_ID, 
                          int16_t user1, int16_t user2, int16_t user3, 
                          int16_t user4, int16_t user5, int16_t user6);

extern void Nx16_GetDynamicPath(void **points_ptr, size_t *count);
extern uint8_t data_to_send_V6[18];


extern  BrainCore_t nx16_ctrl;



///* ------------------------- Internal Data ----------------------------------- */
///**
// * @brief 初始化Agent通讯模块，并启动串口DMA接收。
// * @param nx16_usart_handle 指向用于与Agent通讯的UART句柄 (例如 &huart6)。
// * @return 返回指向全局nx16_ctrl结构体的指针。
// */
// */
BrainCore_t *Nx16ControlInit(UART_HandleTypeDef *nx16_usart_handle);

/* Execute validated V2 commands from the chassis task, never from UART IRQ context. */
void Nx16ProcessPendingCommand(void);

/* Clear command de-duplication and path receive state during an explicit reset. */
void Nx16ResetProtocolState(void);

/* 持续速度模式使用的 V2 链路看门狗。 */
uint8_t Nx16V2LinkIsAlive(uint32_t timeout_ms);

/**
 * @brief 发送状态与里程计反馈帧给Agent。
 */
void SendStatusAndOdometryToAgent(UART_HandleTypeDef* UART_X);
void SendVESCFeedbackToAgent(UART_HandleTypeDef* UART_X);
void SendIMUDataToAgent(UART_HandleTypeDef* UART_X);



#endif



