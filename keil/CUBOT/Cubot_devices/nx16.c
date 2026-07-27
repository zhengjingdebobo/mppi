#include "nx16.h"
#include "string.h"
#include "drv_usart.h"
#include "drv_can.h"
#include "chassis.h"
#include <stdbool.h>
#include <math.h>
#include "task.h"
#include "hwt9053_can.h"
#include "drv_dwt.h"
#include "hardware_config.h"
#include "vesc_motor.h"

#define NX16_CRITICAL_ENTER()  taskENTER_CRITICAL()
#define NX16_CRITICAL_EXIT()   taskEXIT_CRITICAL()

#define AGENT_FRAME_LENGTH_LEGACY  8u
#define AGENT_FRAME_LENGTH_V2      16u
#define AGENT_RX_FRAME_LENGTH_MAX  AGENT_FRAME_LENGTH_V2
#define MAX_POINTS 1000
#define DEG_TO_RAD 0.0174532925f
#define IMU_FRAME_PAYLOAD_LEN 160u
#define IMU_FRAME_LEN         166u
#define VESC_FRAME_TYPE       0x56u
#define VESC_FRAME_PAYLOAD_LEN 72u
#define VESC_FRAME_LEN        78u
#define AGENT_TX_QUEUE_SIZE   4096u
#define AGENT_TX_DMA_CHUNK    256u

typedef struct {
    float x;
    float y;
} SendPoint_t;
static SendPoint_t PathBuf_active[MAX_POINTS];
static SendPoint_t PathBuf_pending[MAX_POINTS];

static SendPoint_t *g_active_path  = PathBuf_active;
static uint16_t     g_active_count = 0;

static SendPoint_t *g_pending_path  = PathBuf_pending;
static uint16_t     g_pending_count = 0;

static volatile Nx16SwitchMode_t switch_mode = NX16_SWITCH_INVALID;
static volatile bool g_pending_ready = false; // 切换路径序列
static volatile bool g_switch_req = false;
static bool g_upload_only = false;

static uint8_t command_id = 0;
/* V2 frames may be delivered more than once by the UART receive path.
 * Remember the most recently dispatched command/sequence pair so a relative
 * distance or rotation command cannot be applied twice.
 */
static uint8_t agent_v2_last_cmd = 0u;
static uint8_t agent_v2_last_seq = 0u;
static uint8_t agent_v2_last_valid = 0u;

typedef struct
{
    uint8_t command_id;
    uint8_t sequence;
    float param1;
    float param2;
    volatile uint8_t valid;
} AgentV2PendingCommand_t;

static AgentV2PendingCommand_t agent_v2_pending;

// 上位机数据
//uint8_t data_to_send_V6[18];
// --- 全局变量 ---
BrainCore_t nx16_ctrl = {
    .TaskTime_zxjtest = 800 
};

// --- 用于将4字节数组安全转换为float的联合体 ---
typedef union {
    float f;
    uint8_t bytes[4];
} FloatUnion;

static float FrameReadFloatLE(const uint8_t *buf)
{
    FloatUnion value;
    memcpy(value.bytes, buf, 4u);
    return value.f;
}

static uint8_t FrameChecksumU8(const uint8_t *buf, uint8_t start, uint8_t end)
{
    uint8_t checksum = 0u;
    uint8_t i;

    for (i = start; i <= end; i++)
    {
        checksum += buf[i];
    }

    return checksum;
}

static uint8_t agent_tx_queue[AGENT_TX_QUEUE_SIZE];
static uint16_t agent_tx_head = 0u;
static uint16_t agent_tx_tail = 0u;
static uint8_t agent_tx_dma_buf[AGENT_TX_DMA_CHUNK];
static volatile uint8_t agent_tx_dma_busy = 0u;
static volatile uint32_t agent_tx_drop_count = 0u;

static uint16_t AgentTxQueueFree(void)
{
    uint16_t head = agent_tx_head;
    uint16_t tail = agent_tx_tail;

    if (head >= tail)
    {
        return (uint16_t)(AGENT_TX_QUEUE_SIZE - (head - tail) - 1u);
    }
    return (uint16_t)(tail - head - 1u);
}

static void AgentTxKick(UART_HandleTypeDef *uart)
{
    uint16_t len = 0u;

    if (uart == NULL || agent_tx_dma_busy || agent_tx_head == agent_tx_tail)
    {
        return;
    }

    while (agent_tx_tail != agent_tx_head && len < AGENT_TX_DMA_CHUNK)
    {
        agent_tx_dma_buf[len++] = agent_tx_queue[agent_tx_tail];
        agent_tx_tail++;
        if (agent_tx_tail >= AGENT_TX_QUEUE_SIZE)
        {
            agent_tx_tail = 0u;
        }
    }

    agent_tx_dma_busy = 1u;
    if (HAL_UART_Transmit_DMA(uart, agent_tx_dma_buf, len) != HAL_OK)
    {
        agent_tx_dma_busy = 0u;
        agent_tx_drop_count++;
    }
}

static uint8_t AgentTxWrite(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (uart == NULL || data == NULL || len == 0u)
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    if (AgentTxQueueFree() < len)
    {
        agent_tx_drop_count++;
        taskEXIT_CRITICAL();
        return 0u;
    }

    for (i = 0u; i < len; i++)
    {
        agent_tx_queue[agent_tx_head] = data[i];
        agent_tx_head++;
        if (agent_tx_head >= AGENT_TX_QUEUE_SIZE)
        {
            agent_tx_head = 0u;
        }
    }
    taskEXIT_CRITICAL();

    AgentTxKick(uart);
    return 1u;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &AGENT_UART_HANDLE)
    {
        agent_tx_dma_busy = 0u;
        AgentTxKick(huart);
    }
}


typedef enum {
    PATH_RX_IDLE = 0,
    PATH_RX_WAIT_X,
    PATH_RX_WAIT_Y
} PathRxState_t;// 上位机发送离散路径点

static bool g_path_rx_active = false;
static uint16_t g_path_expect_points = 0;
static uint16_t g_path_got_points = 0;
static PathRxState_t g_path_state = PATH_RX_IDLE;
static float g_tmp_x = 0.0f;

//static void PathRx_Reset(void)
//{
//    g_path_rx_active = false;
//    g_path_expect_points = 0;
//    g_path_got_points = 0;
//    g_path_state = PATH_RX_IDLE;
//    
//    // ? 不要置NULL，恢复到静态缓冲区
////    g_active_path  = PathBuf_active;
////    g_pending_path = PathBuf_pending;

////    g_pending_ready = false;
////    g_switch_req = false;
////    g_upload_only = false;

////    g_active_count  = 0;
////    g_pending_count = 0;
//    
//    g_tmp_x = 0.0f;
//    // g_active_count = 0;
//}

static void PathRx_Reset(void)
{
    g_path_rx_active = false;
    g_path_expect_points = 0;
    g_path_got_points = 0;
    g_path_state = PATH_RX_IDLE;
    g_tmp_x = 0.0f;
    // g_active_count = 0;
}


// =======================================================
// 获取动态路径数据的指针和长度
void Nx16_GetDynamicPath(void **points_ptr, size_t *count)
{
    // NX16_CRITICAL_ENTER(); // 临界区
    *points_ptr = (void*)g_active_path;
    *count = g_active_count;
    // NX16_CRITICAL_EXIT();
}

void Nx16_RequestSwitch(Nx16SwitchMode_t mode)
{
    switch_mode = mode; 
    g_switch_req = true; // 请求切换
}

Nx16SwitchMode_t GetNx16_Switch_mode(void) {
    return switch_mode;
}

// 全局函数
bool Nx16_TrySwitchActive(bool safe_to_switch)
{
    // 只要 pending 没准备好，任何模式都不能切
    if (!g_pending_ready) {
        return false;
    }

    // 不是首次(INVALID)时，必须有切换请求
    if (switch_mode != NX16_SWITCH_INVALID && !g_switch_req) {
        return false;
    }

    // 不满足SAFE条件则切换
    if (switch_mode == NX16_SWITCH_WHEN_SAFE && !safe_to_switch) {
        return false;
    }
    // NX16_CRITICAL_ENTER();
    {
        // 交换指针
        SendPoint_t *tmp_p = g_active_path;
        g_active_path  = g_pending_path;
        g_pending_path = tmp_p;

        // 交换长度
        uint16_t tmp_n = g_active_count;
        g_active_count  = g_pending_count;
        g_pending_count = tmp_n;

        g_pending_ready = false;
        g_switch_req    = false;
    }
    // NX16_CRITICAL_EXIT();
    return true;
}

//帧格式:
//[0] 0xAA
//[1] cmd_id
//[2..5] float参数（4字节）
//[6] checksum
//[7] 0xDD
/**
 * @brief (内部函数) 解析Agent指令帧
 */
static void ParseAgentCommand(const uint8_t *frame_buf)
{
    // 1. 基本校验: 帧头和帧尾
    if (frame_buf[0] != 0xAA || frame_buf[7] != 0xDD) {
        nx16_ctrl.rx_bad_head_count++;
        return;
    }
    
    // 2. 校验和验证
    uint8_t checksum = 0;
    for (int i = 1; i < 6; i++) {
        checksum += frame_buf[i];
    }
    if (checksum != frame_buf[6]) {
        nx16_ctrl.rx_bad_checksum_count++;
        return;
    }
    nx16_ctrl.rx_valid_count++;
    command_id = frame_buf[1];
    nx16_ctrl.CommandID_test_zxj = frame_buf[1];
    
    
    // 3. 逻辑判断：空闲时接受指令 OR 收到急停指令时无条件接受
    if (command_id == CMD_STOP && g_path_rx_active) { // stop 且 true
        PathRx_Reset(); // STOP 到来时，如果正在收路径点就复位接收状态
    }
    bool allow =
        (nx16_ctrl.InTask == 0) ||
        (command_id == CMD_STOP) ||
        (command_id == CMD_PATH_TRACKING) ||
        (command_id == CMD_SWITCH_PATH); 
    if (allow)
    {
        nx16_ctrl.CommandID = command_id;

        // 4. 参数解析
        FloatUnion param_converter;
        memcpy(param_converter.bytes, &frame_buf[2], 4);
        nx16_ctrl.CommandParam = param_converter.f;

        // 5. 预计算任务参数
        if(nx16_ctrl.CommandID == CMD_MOVE_FORWARD || nx16_ctrl.CommandID == CMD_MOVE_BACKWARD)
        {
            nx16_ctrl.CoreInstruction.Distance = nx16_ctrl.CommandParam;
            float temp = (nx16_ctrl.CoreInstruction.Distance * 500.0f) + 300.0f;
            
            if (temp > 32767.0f) nx16_ctrl.TaskTime_zxjtest = 32767;
            else if (temp < -32768.0f) nx16_ctrl.TaskTime_zxjtest = -32768;
            else nx16_ctrl.TaskTime_zxjtest = (int16_t)temp;
        }
        else if(nx16_ctrl.CommandID == CMD_ROTATE_CCW)
        {
            nx16_ctrl.CoreInstruction.YawAngle = nx16_ctrl.CommandParam;
        }
        else if(nx16_ctrl.CommandID == CMD_ROTATE_CW)
        {
            nx16_ctrl.CoreInstruction.YawAngle = -nx16_ctrl.CommandParam;
        }
        //  轨迹点上传状态机
        else if(nx16_ctrl.CommandID == CMD_PATH_TRACKING)
        {
            /*
             * 该分支实现：上位机发送离散路径点的“流式接收”状态机
             *
             * 上位机协议（每帧固定8字节）：
             *   1) Header帧：CMD_PATH_TRACKING + param = 点数n(浮点，但取整)
             *   2) Body帧：  CMD_PATH_TRACKING + param = x1
             *               CMD_PATH_TRACKING + param = y1
             *               CMD_PATH_TRACKING + param = x2
             *               CMD_PATH_TRACKING + param = y2 ...
             *   3) 下位机收满 2*n 个 float 后，自动触发轨迹跟踪任务（置 RxFlag）
             *
             * 注意：
             * - 只在“收齐点”那一刻置 RxFlag=1，防止底盘提前启动导致后续点无法接收。
             * 上位机协议（每帧固定8字节）：
             *   1) Header帧：CMD_PATH_TRACKING + param = 点数n(浮点，但取整)
             *   2) Body帧：  CMD_PATH_TRACKING + param = x1
             *               CMD_PATH_TRACKING + param = y1
             *               CMD_PATH_TRACKING + param = x2
             *               CMD_PATH_TRACKING + param = y2 ...
             *   3) 下位机收满 2*n 个 float 后，自动触发轨迹跟踪任务（置 RxFlag）
             *
             * 注意：
             * - 只在“收齐点”那一刻置 RxFlag=1，防止底盘提前启动导致后续点无法接收。
             * - 上传过程中如收到 STOP，复位接收状态，避免下一次上传错位。
             */

            float p = nx16_ctrl.CommandParam;  // 点数/坐标

            // 不在接收模式：把第一帧当作 Header（点数）
            if (!g_path_rx_active)
            {
                if (fabsf(p) > 1.0f)
                {
                    // upload-only：header为负数 仅上传 返回true
                    g_upload_only = (p < 0.0f);
                    uint16_t n = (uint16_t)(fabsf(p) + 0.5f); // float -> uint16，四舍五入
                    
                    if (n > MAX_POINTS) n = MAX_POINTS;
                    if (n < 2) return;

                    // 进入接收模式：只缓存，不启动任务
                    g_path_rx_active     = true;
                    g_path_expect_points = n;
                    g_path_got_points    = 0;
                    g_path_state         = PATH_RX_WAIT_X;
                    g_tmp_x              = 0.0f;
                    g_pending_count = 0;
                    g_pending_ready = false;

                    return;
                }
                // 两个条件：CMD_PATH_TRACKING指令 + p = 0
                else if (fabsf(p) < 1e-6f)
                {
                    nx16_ctrl.RxFlag = 1; // 通知底盘启动跟踪 active *************这里合理?*************
                }
                return;
            }
            // 接收模式：把后续帧当作 x/y 数据流
            else
            {
//                if (fabsf(p) < 1e-6f) return;
                if (g_path_state == PATH_RX_WAIT_X)
                {
                    // 收到一个 x，暂存
                    g_tmp_x = p;
                    g_path_state = PATH_RX_WAIT_Y; // 状态切换
                    return;
                }
                else // PATH_RX_WAIT_Y
                {
                    // 超出期望点数或数组上限，直接复位
                    if (g_path_got_points >= g_path_expect_points || g_path_got_points >= MAX_POINTS)
                    {
                        PathRx_Reset();
                        return;
                    }
                    // NX16_CRITICAL_ENTER();
                    // 收到一个 y，与上一次的 x 组成一个点
                    if (g_path_got_points < MAX_POINTS)
                    {
                        g_pending_path[g_path_got_points].x = g_tmp_x;
                        g_pending_path[g_path_got_points].y = p;
                        g_path_got_points++;
                    }
                    // NX16_CRITICAL_EXIT();
                    
                    g_path_state = PATH_RX_WAIT_X;

                    // 收齐：更新点数并判断upload-only
                    if (g_path_got_points >= g_path_expect_points)
                    {
                        g_pending_count = g_path_expect_points;
                        g_pending_ready = true;
                        g_path_rx_active = false;
                        
                        // 不是 upload-only/首次路径跟踪，则上传完成立即开始
                        if (!g_upload_only || GetNx16_Switch_mode() == NX16_SWITCH_INVALID) // 路径切换 0x01 -> g_upload_only = true
                        {
                            // nx16_ctrl.CommandID = CMD_PATH_TRACKING;
                            // nx16_ctrl.CommandParam = 0.0f;
                            nx16_ctrl.RxFlag = 1;
                            return;
//                            // =========================后续优化=========================
//                            // 根据当前是否正在执行任务来决定切换模式
//                            if (nx16_ctrl.InTask == 4)
//                            {
//                                // 已经在路径跟踪任务中 -> 安全切换保证平滑
//                                switch_mode = NX16_SWITCH_WHEN_SAFE; 
//                            }
//                            else
//                            {
//                                // 空闲/首次启动 -> 立刻响应
//                                switch_mode = NX16_SWITCH_IMMEDIATE; 
//                            }
//                            g_switch_req = true; // 触发切换请求
                            // =========================后续优化=========================
                        }
                        // upload-only：只更新 pending，不触发执行
                        return;
                    }
                }
                return;
            }
        }
        else if (nx16_ctrl.CommandID == CMD_SWITCH_PATH)
        {
            float p = nx16_ctrl.CommandParam;
            // param: = 1 NX16_SWITCH_WHEN_SAFE
            Nx16SwitchMode_t mode = (p > 0.5f) ? NX16_SWITCH_WHEN_SAFE : NX16_SWITCH_IMMEDIATE;
            Nx16_RequestSwitch(mode);
        }
        else if(nx16_ctrl.CommandID == CMD_INIT)
        {
             /* Defer reset to OmniCalculate(); this parser runs in UART IRQ context. */
        }
        nx16_ctrl.RxFlag = 1; // 通知ChassisTask处理
    }
}

static void ParseAgentCommandV2(const uint8_t *frame_buf)
{
    uint8_t cmd_id;
    uint8_t sequence;
    float param1;
    float param2;

    if (frame_buf[0] != 0xABu || frame_buf[1] != 0xCDu || frame_buf[15] != 0xDDu)
    {
        nx16_ctrl.rx_bad_head_count++;
        return;
    }

    if (FrameChecksumU8(frame_buf, 2u, 13u) != frame_buf[14])
    {
        nx16_ctrl.rx_bad_checksum_count++;
        return;
    }

    nx16_ctrl.rx_valid_count++;

    cmd_id = frame_buf[2];
    sequence = frame_buf[12];

    if (cmd_id != CMD2_MOVE_POLAR_SPEED &&
        cmd_id != CMD2_MOVE_POLAR_DISTANCE &&
        cmd_id != CMD2_ROTATE_IN_PLACE &&
        cmd_id != CMD2_STOP &&
        cmd_id != CMD2_INIT &&
        cmd_id != CMD2_HEARTBEAT)
    {
        return;
    }

    if (agent_v2_last_valid != 0u &&
        agent_v2_last_cmd == cmd_id &&
        agent_v2_last_seq == sequence)
    {
        return;
    }

    param1 = FrameReadFloatLE(&frame_buf[4]);
    param2 = FrameReadFloatLE(&frame_buf[8]);

    nx16_ctrl.CommandID_test_zxj = cmd_id;

    /* STOP/INIT may replace an unprocessed motion command. Other commands wait
     * for the single-slot mailbox to be consumed by the chassis task. */
    if (agent_v2_pending.valid != 0u &&
        cmd_id != CMD2_STOP && cmd_id != CMD2_INIT)
    {
        return;
    }

    agent_v2_pending.command_id = cmd_id;
    agent_v2_pending.sequence = sequence;
    agent_v2_pending.param1 = param1;
    agent_v2_pending.param2 = param2;
    __DMB();
    agent_v2_pending.valid = 1u;

    agent_v2_last_cmd = cmd_id;
    agent_v2_last_seq = sequence;
    agent_v2_last_valid = 1u;
}

void Nx16ResetProtocolState(void)
{
    NX16_CRITICAL_ENTER();
    PathRx_Reset();
    agent_v2_pending.valid = 0u;
    agent_v2_last_cmd = 0u;
    agent_v2_last_seq = 0u;
    agent_v2_last_valid = 0u;
    NX16_CRITICAL_EXIT();
}

void Nx16ProcessPendingCommand(void)
{
    AgentV2PendingCommand_t pending;
    ChassisApiResult_e api_result = CHASSIS_API_BAD_PARAM;
    uint8_t legacy_cmd = 0u;

    NX16_CRITICAL_ENTER();
    if (agent_v2_pending.valid == 0u)
    {
        NX16_CRITICAL_EXIT();
        return;
    }
    pending.command_id = agent_v2_pending.command_id;
    pending.sequence = agent_v2_pending.sequence;
    pending.param1 = agent_v2_pending.param1;
    pending.param2 = agent_v2_pending.param2;
    agent_v2_pending.valid = 0u;
    NX16_CRITICAL_EXIT();

    switch (pending.command_id)
    {
    case CMD2_MOVE_POLAR_SPEED:
        legacy_cmd = (pending.param2 >= 0.0f) ? CMD_MOVE_FORWARD : CMD_MOVE_BACKWARD;
        api_result = ChassisMoveByAngleAndSpeed(pending.param1, pending.param2);
        break;

    case CMD2_MOVE_POLAR_DISTANCE:
        legacy_cmd = (pending.param2 >= 0.0f) ? CMD_MOVE_FORWARD : CMD_MOVE_BACKWARD;
        api_result = ChassisMoveByAngleAndDistance(pending.param1, pending.param2);
        if (api_result == CHASSIS_API_OK)
        {
            nx16_ctrl.CoreInstruction.Distance = pending.param2;
        }
        break;

    case CMD2_ROTATE_IN_PLACE:
    {
        ChassisRotateDir_e dir = (((int32_t)(pending.param1 + 0.5f)) == 0) ?
                                 CHASSIS_ROTATE_LEFT : CHASSIS_ROTATE_RIGHT;
        legacy_cmd = (dir == CHASSIS_ROTATE_LEFT) ? CMD_ROTATE_CCW : CMD_ROTATE_CW;
        api_result = ChassisRotateInPlace(dir, pending.param2);
        break;
    }

    case CMD2_STOP:
        ChassisStopCommand();
        Nx16ResetProtocolState();
        return;

    case CMD2_INIT:
        legacy_cmd = CMD_INIT;
        ChassisClearApiCommand();
        TaskInit();
        Nx16ResetProtocolState();
        nx16_ctrl.CommandID = legacy_cmd;
        nx16_ctrl.CommandParam = 0.0f;
        nx16_ctrl.LastCommandID = legacy_cmd;
        nx16_ctrl.Status = STATUS_IDLE;
        nx16_ctrl.RxFlag = 0u;
        return;

    case CMD2_HEARTBEAT:
        nx16_ctrl.CommandID_test_zxj = CMD2_HEARTBEAT;
        return;

    default:
        return;
    }

    nx16_ctrl.CommandID = legacy_cmd;
    nx16_ctrl.CommandParam = pending.param2;
    if (api_result == CHASSIS_API_OK)
    {
        nx16_ctrl.Status = STATUS_EXECUTING;
        nx16_ctrl.RxFlag = 0u;
    }
    else if (api_result != CHASSIS_API_BUSY)
    {
        nx16_ctrl.LastCommandID = legacy_cmd;
        nx16_ctrl.Status = STATUS_CMD_FAILED;
    }
}

static void ParseAgentCommandAuto(const uint8_t *frame_buf)
{
    if (frame_buf[0] == 0xABu && frame_buf[1] == 0xCDu)
    {
        ParseAgentCommandV2(frame_buf);
        return;
    }

    if (frame_buf[0] == 0xAAu)
    {
        ParseAgentCommand(frame_buf);
    }
}


static USARTInstance *nx16_usart_instance;

///**
// *向匿名上位机（ANO Ground Station）发送调试数据
// *构建符合 V6 协议的数据帧，通过 DMA 发送。
// *通常用于调试 PID 波形或姿态显示。
//*/
//void ANO_V6_Send_Up_Computer(UART_HandleTypeDef* UART_X,int16_t user1,int16_t user2,int16_t user3,int16_t user4,int16_t user5,int16_t user6)
//{
//    uint8_t _cnt=0; 
//    data_to_send_V6[_cnt++]=0xAA;
//    data_to_send_V6[_cnt++]=0xFF;
//    data_to_send_V6[_cnt++]=0x01;
//    data_to_send_V6[_cnt++]=0x00;
//    data_to_send_V6[_cnt++]=0x00;
//    data_to_send_V6[_cnt++]=0x01;
//    data_to_send_V6[_cnt++]=0x00;
//    data_to_send_V6[_cnt++]=0xDD;
//    
//    HAL_UART_Transmit_DMA(UART_X,data_to_send_V6,_cnt);
//}

#define FRAME_LEN_EXT 86
// 向 Python 上位机反馈机器人的实时状态
void SendStatusAndOdometryToAgent(UART_HandleTypeDef* UART_X)
{
    static uint8_t frame[FRAME_LEN_EXT];

    frame[0] = 0xAA;
    frame[1] = 0xAA;
    frame[2] = nx16_ctrl.Status;
    frame[3] = nx16_ctrl.LastCommandID;

    FloatUnion x_converter, y_converter, yaw_converter,
               tar_x_converter, tar_y_converter, tar_yaw_converter,
               cmd_vx_converter, cmd_vy_converter, cmd_wz_converter,
               lf_ref_converter, rf_ref_converter, lb_ref_converter, rb_ref_converter,
               lf_fdb_converter, rf_fdb_converter, lb_fdb_converter, rb_fdb_converter,
               cmd_vx_true_converter, cmd_vy_true_converter, cmd_wz_true_converter;
    
    x_converter.f = nx16_ctrl.current_x;
    y_converter.f = nx16_ctrl.current_y;
    yaw_converter.f = nx16_ctrl.current_yaw;

    tar_x_converter.f = (float)CANGetHcan1Fifo0CallbackCount();
    tar_y_converter.f = (float)CANGetHcan1Fifo1DispatchCount();
    tar_yaw_converter.f = (float)CANGetLastExtId();
    
    cmd_vx_converter.f = g_dbg.cmd_vx;
    cmd_vy_converter.f = g_dbg.cmd_vy;
    cmd_wz_converter.f = g_dbg.cmd_wz;
    
    lf_ref_converter.f = rc_ctrl.vt_lf;
    lf_fdb_converter.f = (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LF);
    rf_ref_converter.f = rc_ctrl.vt_rf;
    rf_fdb_converter.f = (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RF);

    lb_ref_converter.f = rc_ctrl.vt_lb;
    lb_fdb_converter.f = (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LB);
    rb_ref_converter.f = rc_ctrl.vt_rb;
    rb_fdb_converter.f = (float)VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RB);

    cmd_vx_true_converter.f = (float)CANGetHcan1RxMsgCount();
    cmd_vy_true_converter.f = (float)VESCMotorGetRxAnyCount();
    cmd_wz_true_converter.f = (float)nx16_ctrl.InTask;

    
    memcpy(&frame[4], x_converter.bytes, 4);
    memcpy(&frame[8], y_converter.bytes, 4);
    memcpy(&frame[12], yaw_converter.bytes, 4);
    
    memcpy(&frame[16], tar_x_converter.bytes, 4);
    memcpy(&frame[20], tar_y_converter.bytes, 4);
    memcpy(&frame[24], tar_yaw_converter.bytes, 4);

    memcpy(&frame[28], cmd_vx_converter.bytes, 4);
    memcpy(&frame[32], cmd_vy_converter.bytes, 4);
    memcpy(&frame[36], cmd_wz_converter.bytes, 4);
    
    memcpy(&frame[40], lf_ref_converter.bytes, 4); // LF Ref
    memcpy(&frame[44], lf_fdb_converter.bytes, 4); // LF Fdb
    
    memcpy(&frame[48], rf_ref_converter.bytes, 4); // RF Ref
    memcpy(&frame[52], rf_fdb_converter.bytes, 4); // RF Fdb
    
    memcpy(&frame[56], lb_ref_converter.bytes, 4); // LB Ref
    memcpy(&frame[60], lb_fdb_converter.bytes, 4); // LB Fdb
    
    memcpy(&frame[64], rb_ref_converter.bytes, 4); // RB Ref
    memcpy(&frame[68], rb_fdb_converter.bytes, 4); // RB Fdb (结束于79)
    
    memcpy(&frame[72], cmd_vx_true_converter.bytes, 4);
    memcpy(&frame[76], cmd_vy_true_converter.bytes, 4);
    memcpy(&frame[80], cmd_wz_true_converter.bytes, 4);


    uint8_t checksum = 0;
    for(int i = 2; i < FRAME_LEN_EXT - 2; i++) {
        checksum += frame[i];
    }
    frame[FRAME_LEN_EXT - 2] = checksum;
    frame[FRAME_LEN_EXT - 1] = 0xDD;
    
    nx16_ctrl.last_tx_status = AgentTxWrite(UART_X, frame, FRAME_LEN_EXT) ? HAL_OK : HAL_ERROR;
    if (nx16_ctrl.last_tx_status == HAL_OK) {
        nx16_ctrl.tx_status_count++;
    } else {
        nx16_ctrl.tx_status_error_count++;
    }
}

// 串口接收回调函数


static void IMUFramePutU64(uint8_t *buf, uint8_t *idx, uint64_t value)
{
    buf[(*idx)++] = (uint8_t)(value & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 16) & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 24) & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 32) & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 40) & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 48) & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 56) & 0xFFu);
}
static void IMUFramePutU32(uint8_t *buf, uint8_t *idx, uint32_t value)
{
    buf[(*idx)++] = (uint8_t)(value & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 16) & 0xFFu);
    buf[(*idx)++] = (uint8_t)((value >> 24) & 0xFFu);
}

static void IMUFramePutFloat(uint8_t *buf, uint8_t *idx, float value)
{
    FloatUnion converter;
    converter.f = value;
    buf[(*idx)++] = converter.bytes[0];
    buf[(*idx)++] = converter.bytes[1];
    buf[(*idx)++] = converter.bytes[2];
    buf[(*idx)++] = converter.bytes[3];
}

static void IMUFramePutI32(uint8_t *buf, uint8_t *idx, int32_t value)
{
    buf[(*idx)++] = (uint8_t)(value & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 16) & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 24) & 0xFF);
}

void SendVESCFeedbackToAgent(UART_HandleTypeDef* UART_X)
{
    static uint8_t frame[VESC_FRAME_LEN];
    uint8_t idx = 4u;
    uint8_t checksum = 0u;
    uint8_t i;

    frame[0] = 0xAAu;
    frame[1] = 0x55u;
    frame[2] = VESC_FRAME_TYPE;
    frame[3] = VESC_FRAME_PAYLOAD_LEN;

    IMUFramePutU64(frame, &idx, DWT_GetTimeline_us());

    IMUFramePutI32(frame, &idx, VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LF));
    IMUFramePutFloat(frame, &idx, VESCMotorGetFeedbackCurrent(VESC_LF_OUTPUT_ID));
    IMUFramePutFloat(frame, &idx, VESCMotorGetFeedbackDuty(VESC_LF_OUTPUT_ID));
    IMUFramePutU32(frame, &idx, (uint32_t)VESCMotorFeedbackIsOnline(VESC_LF_OUTPUT_ID));

    IMUFramePutI32(frame, &idx, VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RF));
    IMUFramePutFloat(frame, &idx, VESCMotorGetFeedbackCurrent(VESC_RF_OUTPUT_ID));
    IMUFramePutFloat(frame, &idx, VESCMotorGetFeedbackDuty(VESC_RF_OUTPUT_ID));
    IMUFramePutU32(frame, &idx, (uint32_t)VESCMotorFeedbackIsOnline(VESC_RF_OUTPUT_ID));

    IMUFramePutI32(frame, &idx, VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_LB));
    IMUFramePutFloat(frame, &idx, VESCMotorGetFeedbackCurrent(VESC_LB_OUTPUT_ID));
    IMUFramePutFloat(frame, &idx, VESCMotorGetFeedbackDuty(VESC_LB_OUTPUT_ID));
    IMUFramePutU32(frame, &idx, (uint32_t)VESCMotorFeedbackIsOnline(VESC_LB_OUTPUT_ID));

    IMUFramePutI32(frame, &idx, VESCMotorGetLogicalFeedbackERPM(VESC_WHEEL_RB));
    IMUFramePutFloat(frame, &idx, VESCMotorGetFeedbackCurrent(VESC_RB_OUTPUT_ID));
    IMUFramePutFloat(frame, &idx, VESCMotorGetFeedbackDuty(VESC_RB_OUTPUT_ID));
    IMUFramePutU32(frame, &idx, (uint32_t)VESCMotorFeedbackIsOnline(VESC_RB_OUTPUT_ID));

    for (i = 2u; i < (VESC_FRAME_LEN - 2u); i++)
    {
        checksum += frame[i];
    }
    frame[VESC_FRAME_LEN - 2u] = checksum;
    frame[VESC_FRAME_LEN - 1u] = 0xDDu;

    AgentTxWrite(UART_X, frame, VESC_FRAME_LEN);
}

void SendIMUDataToAgent(UART_HandleTypeDef* UART_X)
{
    static uint8_t frame[IMU_FRAME_LEN];
    uint8_t idx = 4u;
    uint8_t checksum = 0u;
    uint8_t i;

    frame[0] = 0xAAu;
    frame[1] = 0x55u;
    frame[2] = 0x49u;
    frame[3] = IMU_FRAME_PAYLOAD_LEN;

    IMUFramePutU64(frame, &idx, DWT_GetTimeline_us());
    IMUFramePutFloat(frame, &idx, hwt9053_can.angle_deg[0]);
    IMUFramePutFloat(frame, &idx, hwt9053_can.angle_deg[1]);
    IMUFramePutFloat(frame, &idx, hwt9053_can.yaw_zxj * HWT9053_CONTROL_YAW_SIGN);
    IMUFramePutFloat(frame, &idx, hwt9053_can.yaw_total_zxj * HWT9053_CONTROL_YAW_SIGN);
    IMUFramePutFloat(frame, &idx, hwt9053_can.gyro_dps[0]);
    IMUFramePutFloat(frame, &idx, hwt9053_can.gyro_dps[1]);
    IMUFramePutFloat(frame, &idx, hwt9053_can.gyro_dps[2]);
    IMUFramePutFloat(frame, &idx, hwt9053_can.acc_g[0]);
    IMUFramePutFloat(frame, &idx, hwt9053_can.acc_g[1]);
    IMUFramePutFloat(frame, &idx, hwt9053_can.acc_g[2]);
    IMUFramePutU32(frame, &idx, hwt9053_can.state);
    IMUFramePutU32(frame, &idx, hwt9053_can.rx_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.valid_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.last_error);
    IMUFramePutU32(frame, &idx, hwt9053_can.last_std_id);
    IMUFramePutU32(frame, &idx, hwt9053_can.last_ext_id);
    IMUFramePutU32(frame, &idx, hwt9053_can.last_ide);
    IMUFramePutU32(frame, &idx, hwt9053_can.last_rtr);
    IMUFramePutU32(frame, &idx, hwt9053_can.last_dlc);
    IMUFramePutU32(frame, &idx, hwt9053_can.last_type);
    IMUFramePutU32(frame, &idx, hwt9053_can.error_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.hal_error_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.acc_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.gyro_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.angle_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.yaw_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.config_attempt_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.config_done);
    IMUFramePutU32(frame, &idx, hwt9053_can.config_tx_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.config_tx_error_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.last_config_status);
    IMUFramePutU32(frame, &idx, hwt9053_can.init_count);
    IMUFramePutU32(frame, &idx, hwt9053_can.start_status);
    IMUFramePutU32(frame, &idx, hwt9053_can.notify_status);
    IMUFramePutU32(frame, &idx, hwt9053_can.last_tick);
    IMUFramePutU32(frame, &idx, (uint32_t)HWT9053CAN_IsOnline());
    IMUFramePutU32(frame, &idx, hwt9053_can.fifo0_level);
    IMUFramePutU32(frame, &idx, hwt9053_can.fifo1_level);

    for (i = 2u; i < (IMU_FRAME_LEN - 2u); i++)
    {
        checksum += frame[i];
    }
    frame[IMU_FRAME_LEN - 2u] = checksum;
    frame[IMU_FRAME_LEN - 1u] = 0xDDu;

    AgentTxWrite(UART_X, frame, IMU_FRAME_LEN);
}
static void Nx16BufferRxCallback(void)
{
    nx16_ctrl.rx_callback_count++;
    ParseAgentCommandAuto(nx16_usart_instance->recv_buff);
}

// 模块初始化函数
BrainCore_t *Nx16ControlInit(UART_HandleTypeDef *nx16_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = Nx16BufferRxCallback;
    conf.usart_handle = nx16_usart_handle;
    conf.recv_buff_size = AGENT_RX_FRAME_LENGTH_MAX;
    nx16_usart_instance = USARTRegister(&conf); // 启动串口服务

    return &nx16_ctrl;
}




