# IMU 与上位机通信交接记录

日期：2026-05-25

## 当前硬件拓扑

当前确认的连接逻辑：

```text
C 型板 CAN1  -> 底盘电机
C 型板 CAN2  -> HWT9053-CAN IMU
C 型板 UART1 -> CP2102 USB-TTL -> Ubuntu 上位机
```

UART1 引脚：

```text
USART1_TX = PA9
USART1_RX = PB7
```

CP2102 接线应为：

```text
CP2102 RXD -> C 板 PA9 / USART1_TX
CP2102 TXD -> C 板 PB7 / USART1_RX
CP2102 GND -> C 板 GND
```

注意：CP2102 必须与 C 板共地。确认 CP2102 为 TTL 电平，优先使用 3.3V 逻辑。

## 已完成的下位机改动

### 1. HWT9053-CAN 使用 CAN2

文件：

```text
CUBOT/Cubot_devices/hwt9053_can.c
CUBOT/Cubot_devices/hwt9053_can.h
```

当前 HWT9053 驱动使用 `hcan2`，符合当前硬件连接：

```c
HAL_CAN_ConfigFilter(&hcan2, &filter);
HAL_CAN_Start(&hcan2);
HAL_CAN_ActivateNotification(&hcan2, ...);
```

IMU 数据被解析到全局结构体：

```c
volatile HWT9053CAN_t hwt9053_can;
```

主要字段：

```c
hwt9053_can.acc_g[3];
hwt9053_can.gyro_dps[3];
hwt9053_can.angle_deg[3];
hwt9053_can.yaw_zxj;
hwt9053_can.yaw_total_zxj;
hwt9053_can.rx_count;
hwt9053_can.valid_count;
hwt9053_can.last_error;
```

### 2. 上位机串口切到 UART1

文件：

```text
CUBOT/Cubot_User_Config/hardware_config.h
```

已将上位机通信串口统一宏改为：

```c
#define AGENT_UART_HANDLE huart1
```

这样 `hardware_config.c` 中：

```c
nx16_data = Nx16ControlInit(&AGENT_UART_HANDLE);
```

实际使用的是 UART1。

### 3. UART1 波特率改为 460800

文件：

```text
Src/usart.c
```

已将：

```c
huart1.Init.BaudRate = 460800;
```

原因：300Hz IMU 上传时，原 `115200` 带宽不足。

当前估算数据量：

```text
IMU 帧 70 字节 * 300Hz = 21000 B/s
状态帧 86 字节 * 20Hz = 1720 B/s
总计约 22720 B/s
460800 bps 理论有效约 46080 B/s
```

所以 460800 理论上足够。

### 4. 新增 IMU 独立上报帧

文件：

```text
CUBOT/Cubot_devices/nx16.c
CUBOT/Cubot_devices/nx16.h
```

新增函数：

```c
void SendIMUDataToAgent(UART_HandleTypeDef* UART_X);
```

帧格式：

```text
AA 55 49 LEN PAYLOAD CHECKSUM DD
```

其中：

```text
AA 55      IMU 帧头
49         帧类型，ASCII 'I'
LEN        payload 长度，目前为 64
PAYLOAD    IMU 数据
CHECKSUM   从 frame[2] 到 payload 结束的 uint8 累加和
DD         帧尾
```

当前 IMU 帧总长度：

```c
#define IMU_FRAME_PAYLOAD_LEN 64u
#define IMU_FRAME_LEN         70u
```

payload 内容顺序：

```text
uint64 stm32_us
float  roll_deg
float  pitch_deg
float  yaw_deg
float  yaw_total_deg
float  gyro_x_dps
float  gyro_y_dps
float  gyro_z_dps
float  acc_x_g
float  acc_y_g
float  acc_z_g
uint32 state
uint32 rx_count
uint32 valid_count
uint32 last_error
```

上位机 Python 解包格式：

```python
struct.unpack("<Q10f4I", payload)
```

### 5. 微秒级真实时间戳

文件：

```text
CUBOT/Cubot_devices/nx16.c
CUBOT/Cubot_modules/chassis.c
```

已使用 DWT 微秒时间：

```c
DWT_GetTimeline_us()
```

IMU 帧中的 `stm32_us` 为下位机启动后的微秒时间戳。

相关 DWT 初始化在：

```text
CUBOT/Cubot_driver/drv_init.c
```

其中：

```c
DWT_Init(168);
```

### 6. IMU 上传频率约 300Hz

文件：

```text
CUBOT/Cubot_modules/chassis.c
```

在 `ChassisTask()` 中加入：

```c
static uint64_t imu_send_us = 0u;
uint64_t now_us = DWT_GetTimeline_us();

if (imu_send_us == 0u || (now_us - imu_send_us) >= 3333u)
{
    imu_send_us = now_us;
    SendIMUDataToAgent(&AGENT_UART_HANDLE);
}
```

理论目标：

```text
3333 us -> 约 300 Hz
```

状态帧仍为约 20Hz：

```c
if (now_tick - agent_send_tick >= 50u)
{
    agent_send_tick = now_tick;
    SendStatusAndOdometryToAgent(&AGENT_UART_HANDLE);
}
```

## 已完成的上位机改动

### 1. `car_controlst.py`

文件：

```text
car_controlst.py
```

曾经对 `car_controlst.py` 做过 IMU 解析能力增强：

- 支持识别原状态帧 `AA AA ... DD`
- 支持识别新增 IMU 帧 `AA 55 49 ... DD`
- 支持保存 IMU CSV
- 默认 baudrate 改为 `460800`
- IMU 帧长度同步为 `70`
- 时间戳字段同步为 `stm32_us`

后来用户要求“不要在这个上面改，新写一个”，因此又撤回了临时加入的：

```text
--imu-only
--duration
--no-init
```

这些纯 IMU 采集模式不再依赖 `car_controlst.py`。

### 2. 新增独立 IMU 采集脚本

文件：

```text
imu_logger.py
```

用途：

```text
只监听串口
只解析 IMU 帧 AA 55 49
不发送任何控制命令
保存 CSV
每秒打印接收频率和最后一帧 yaw
```

运行命令：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_log.csv
```

采集指定时长，例如 10 秒：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_log.csv --duration 10
```

诊断原始字节：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_log.csv --duration 5 --dump-raw 128
```

CSV 字段：

```text
pc_time
stm32_us
roll_deg
pitch_deg
yaw_deg
yaw_total_deg
gyro_x_dps
gyro_y_dps
gyro_z_dps
acc_x_g
acc_y_g
acc_z_g
state
rx_count
valid_count
last_error
```

## 当前测试结果

在 Ubuntu 上运行：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_log.csv --duration 5 --dump-raw 128
```

输出：

```text
Connected /dev/ttyUSB0@460800
Writing imu_log.csv
Press Ctrl+C to stop.
waiting for IMU frames... bytes=0 status=0 bad=0
waiting for IMU frames... bytes=0 status=0 bad=0
waiting for IMU frames... bytes=0 status=0 bad=0
waiting for IMU frames... bytes=0 status=0 bad=0
waiting for IMU frames... bytes=0 status=0 bad=0
Done. bytes=0 imu=0 status=0 bad=0
```

结论：

```text
CP2102 端口可以打开
但是上位机完全没有收到任何串口字节
不是解析问题
当前问题在下位机 UART1 是否发出数据、固件是否下载、或硬件接线
```

## 当前最可能的问题

优先级从高到低：

### 1. 下位机没有下载最新固件

如果 C 板仍在跑旧固件，则：

```text
UART1 可能没有被作为上位机口
UART1 可能仍是 115200
IMU 帧函数不存在
```

需要用 Keil 重新编译下载。

### 2. UART1 接线不对

当前应该是：

```text
C 板 PA9 / USART1_TX -> CP2102 RXD
C 板 PB7 / USART1_RX -> CP2102 TXD
C 板 GND             -> CP2102 GND
```

如果接反或没有共地，上位机会 `bytes=0`。

### 3. CP2102 接到的不是 UART1

如果 CP2102 实际接的是 USART6 或其它串口，而代码已经切到 `huart1`，也会 `bytes=0`。

### 4. UART1 没有实际输出

虽然代码里调用：

```c
SendStatusAndOdometryToAgent(&AGENT_UART_HANDLE);
SendIMUDataToAgent(&AGENT_UART_HANDLE);
```

但如果 `ChassisTask()` 没有运行，或任务没启动，也不会有数据。

## 下一步排查建议

### 第一步：Keil 编译下载

先确保最新代码已下载进 C 板。

如果 Keil 报错，重点看：

```text
AGENT_UART_HANDLE 未定义
DWT_GetTimeline_us 未定义
SendIMUDataToAgent 未定义
```

### 第二步：确认 UART1 是否有输出

上位机继续运行：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_log.csv --duration 5 --dump-raw 128
```

判断：

```text
bytes=0  -> 完全没收到串口数据
bytes>0, status>0, imu=0 -> 有状态帧，IMU 帧没发或格式错
bytes>0, imu>0 -> 正常
```

### 第三步：尝试旧波特率

如果怀疑新固件没下载或 UART1 仍为 115200：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 115200 --output imu_log.csv --duration 5 --dump-raw 128
```

如果 115200 有数据，说明板子不是最新 460800 固件。

### 第四步：Keil Watch 看 IMU 是否在 CAN2 正常工作

观察：

```c
hwt9053_can.rx_count
hwt9053_can.valid_count
hwt9053_can.last_type
hwt9053_can.yaw_total_zxj
hwt9053_can.last_error
```

正常表现：

```text
rx_count 持续增加
valid_count 持续增加
last_type 在 0x51/0x52/0x53/0x54 间变化
yaw_total_zxj 转动 IMU 会变化
```

如果这些不变，问题在 CAN2/IMU 侧，不在 USB-TTL。

## 关键文件列表

下位机：

```text
CUBOT/Cubot_devices/hwt9053_can.c
CUBOT/Cubot_devices/hwt9053_can.h
CUBOT/Cubot_devices/nx16.c
CUBOT/Cubot_devices/nx16.h
CUBOT/Cubot_modules/chassis.c
CUBOT/Cubot_User_Config/hardware_config.h
Src/usart.c
```

上位机：

```text
car_controlst.py
imu_logger.py
```

## 当前推荐运行命令

只采集 IMU：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_log.csv
```

诊断模式：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_log.csv --duration 5 --dump-raw 128
```

如果没有数据，试旧波特率：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 115200 --output imu_log.csv --duration 5 --dump-raw 128
```

## 备注

当前 `bytes=0` 是非常明确的信号：上位机没有收到任何字节。此时不要继续调 Python 解析逻辑，优先检查：

```text
固件是否最新
UART1 TX/RX/GND 接线
CP2102 实际连接的是不是 UART1
下位机任务是否在运行
```

---

# 完整扩展交接记录

以下内容用于把本次项目中已经讨论、分析、修改、验证和遗留的问题完整交接出去。内容不仅包括最后的 IMU 高速上传方案，也包括此前围绕 HWT9053-CAN、底盘位置控制、编码器/IMU 融合、上位机 Python 脚本、Keil 下载、串口调试、CAN 调试和当前未解决问题的全部上下文。后续接手的人应先通读本文，再进行 Keil 下载、硬件接线检查和上位机测试，避免重复走已经走过的弯路。

## 一、项目目标总览

本项目当前目标可以分成两个层级。

第一层级是底盘控制目标：C 型板作为下位机，负责读取遥控器、接收上位机指令、驱动 CAN1 上的底盘电机、读取 CAN2 上的 HWT9053-CAN IMU，并利用编码器和 IMU 的信息完成小车运动控制。此前重点需求包括：

```text
1. 小车每次启动时的当前朝向作为软件 0 朝向；
2. 上位机发送前进 1m、后退、左转 90 度等命令时，下位机执行闭环控制；
3. 编码器负责估计已经走过的距离；
4. HWT9053-CAN 的 yaw 负责航向保持；
5. 尽量减少前进过程中的横向偏移；
6. 遥控器控制优先级必须高于上位机任务；
7. 上位机任务执行过程中，如果遥控器有明显输入，应打断上位机任务，恢复人工控制；
8. 上位机可以收到下位机回传的位置、控制状态、调试量；
9. 后续需要用 IMU 信息辅助判断打滑、侧滑和姿态异常。
```

第二层级是数据记录目标：现在需要把底层 IMU 信息以较高频率上传到上位机保存，便于后续分析控制效果、打滑、漂移、加速度响应和 yaw 稳定性。最新明确需求是：

```text
1. IMU 与 C 型板 CAN2 相连；
2. C 型板与底盘电机通过 CAN1 相连；
3. C 型板 UART1 与 CP2102 USB-TTL 相连，上位机通过 /dev/ttyUSB0 读取；
4. IMU 信息需要以接近 300Hz 的频率上传；
5. 数据帧需要带真实时间戳；
6. 上位机需要保存这些信息；
7. IMU 采集脚本不要继续改 car_controlst.py，需要独立写一个。
```

当前已经为第二层级新增了独立脚本 `imu_logger.py`，并在下位机中新增了 `SendIMUDataToAgent()`。但是当前实测结果为：

```text
bytes=0
imu=0
status=0
bad=0
```

这说明 Ubuntu 上位机打开了 CP2102 端口，但是没有收到任何串口字节。这个结果不是 Python 解析错误，而是串口链路没有数据进来。后续应优先检查固件是否下载、UART1 接线、CP2102 是否接到了正确串口，以及下位机任务是否运行。

## 二、硬件拓扑最终确认

用户最后明确的硬件连接如下：

```text
C 型板 CAN1 连接底盘电机；
C 型板 CAN2 连接 HWT9053-CAN IMU；
C 型板 UART1 通过 CP2102 USB-TTL 连接 Ubuntu 上位机。
```

这是当前所有代码修改的依据。后续不要再把 IMU 改到 CAN1，也不要再默认上位机连接 USART6。

### 2.1 CAN1 与电机

底盘电机沿用 CAN1。当前 `chassis.c` 中电机初始化仍然使用：

```c
.can_init_config.can_handle = &hcan1
```

这说明底盘电机驱动仍走 CAN1。此前曾经担心如果 IMU 也接 CAN1，会和电机波特率冲突；现在用户已经明确 IMU 在 CAN2，因此这个冲突不存在。

### 2.2 CAN2 与 HWT9053-CAN

HWT9053-CAN 使用 CAN2。`hwt9053_can.c` 中当前写死读取 `hcan2`：

```c
HAL_CAN_ConfigFilter(&hcan2, &filter);
hwt9053_can.start_status = HAL_CAN_Start(&hcan2);
hwt9053_can.notify_status = HAL_CAN_ActivateNotification(&hcan2, ...);
```

记录原始数据时也限制：

```c
if (hcan != &hcan2 || rx_header == NULL || data == NULL)
{
    return;
}
```

因此只要 IMU 硬件确实接到 CAN2，且 CAN2 波特率与 HWT9053 匹配，下位机就应该能在 `hwt9053_can` 结构体中看到数据变化。

此前用户说明 HWT9053-CAN 的波特率是 `250k`。当前 `Src/can.c` 里 CAN2 配置是：

```c
hcan2.Init.Prescaler = 12;
hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;
hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
```

这个配置此前就是为 CAN2/IMU 使用而保留的。后续如果 CAN2 收不到 IMU 数据，应首先在 Keil Watch 中观察：

```c
hwt9053_can.rx_count
hwt9053_can.valid_count
hwt9053_can.last_type
hwt9053_can.yaw_total_zxj
hwt9053_can.last_error
```

如果这些不变化，问题在 CAN2 硬件、波特率、终端电阻、IMU 供电或 IMU 输出模式，而不是 UART 上传和 Python。

### 2.3 UART1 与 CP2102

当前上位机通过 CP2102 USB-TTL 接 C 型板 UART1。STM32 工程里 UART1 引脚为：

```text
USART1_TX = PA9
USART1_RX = PB7
```

CP2102 正确接线应为：

```text
CP2102 RXD -> C 型板 PA9 / USART1_TX
CP2102 TXD -> C 型板 PB7 / USART1_RX
CP2102 GND -> C 型板 GND
```

注意 TX/RX 要交叉连接。当前 `imu_logger.py` 诊断结果为 `bytes=0`，因此后续应重点确认 `PA9 -> CP2102 RXD` 这条线是否正确连接。若只需要下位机上传 IMU 到上位机，理论上只接 C 板 TX 到 CP2102 RXD 和 GND 也可以收到数据；如果后续要上位机发控制命令，则 CP2102 TXD 到 C 板 RX 也必须接。

CP2102 模块通常支持 460800 波特率。当前代码已经把 UART1 和上位机默认波特率都改为 460800。如果 CP2102 模块、线材或驱动不稳定，可以进一步改到 921600；但当前 `bytes=0` 不是高波特率乱码，而是完全没有数据，因此不应优先调波特率，应该优先确认串口是否真的发出。

## 三、下位机 IMU 驱动现状

### 3.1 HWT9053-CAN 数据结构

文件：

```text
CUBOT/Cubot_devices/hwt9053_can.h
```

结构体：

```c
typedef struct
{
    uint32_t init_count;
    uint32_t start_status;
    uint32_t notify_status;
    uint32_t state;
    uint32_t fifo0_level;
    uint32_t fifo1_level;
    uint32_t rx_count;
    uint32_t valid_count;
    uint32_t error_count;
    uint32_t hal_error_count;
    uint32_t last_tick;
    uint32_t last_std_id;
    uint32_t last_ext_id;
    uint32_t last_ide;
    uint32_t last_rtr;
    uint8_t last_dlc;
    uint8_t last_type;
    uint8_t last_data[8];

    uint32_t acc_count;
    uint32_t gyro_count;
    uint32_t angle_count;
    uint32_t roll_count;
    uint32_t pitch_count;
    uint32_t yaw_count;
    uint32_t mag_count;

    int16_t acc_raw[3];
    float acc_g[3];

    int16_t gyro_raw[3];
    float gyro_dps[3];

    int32_t angle_raw[3];
    float angle_deg[3];
    float yaw_deg;
    float yaw_total_deg;
    float yaw_zxj;
    float yaw_total_zxj;
    int32_t turn_count;

    int16_t mag_raw[3];

    uint32_t last_error;
} HWT9053CAN_t;
```

这个结构体已经能记录：

```text
CAN 初始化状态；
CAN 接收计数；
有效帧计数；
最后一帧类型；
最后一帧数据；
加速度；
角速度；
欧拉角；
软件置零后的 yaw；
展开 yaw；
错误码。
```

### 3.2 HWT9053-CAN 解析逻辑

文件：

```text
CUBOT/Cubot_devices/hwt9053_can.c
```

当前识别的数据类型：

```c
#define HWT9053_CAN_FRAME_HEAD 0x55u
#define HWT9053_CAN_TYPE_ACC   0x51u
#define HWT9053_CAN_TYPE_GYRO  0x52u
#define HWT9053_CAN_TYPE_ANGLE 0x53u
#define HWT9053_CAN_TYPE_MAG   0x54u
```

处理加速度帧 `0x51`：

```c
hwt9053_can.acc_raw[0] = HWT9053_GetInt16(data, 2);
hwt9053_can.acc_raw[1] = HWT9053_GetInt16(data, 4);
hwt9053_can.acc_raw[2] = HWT9053_GetInt16(data, 6);
hwt9053_can.acc_g[0] = (float)hwt9053_can.acc_raw[0] / 32768.0f * 16.0f;
hwt9053_can.acc_g[1] = (float)hwt9053_can.acc_raw[1] / 32768.0f * 16.0f;
hwt9053_can.acc_g[2] = (float)hwt9053_can.acc_raw[2] / 32768.0f * 16.0f;
```

处理角速度帧 `0x52`：

```c
hwt9053_can.gyro_raw[0] = HWT9053_GetInt16(data, 2);
hwt9053_can.gyro_raw[1] = HWT9053_GetInt16(data, 4);
hwt9053_can.gyro_raw[2] = HWT9053_GetInt16(data, 6);
hwt9053_can.gyro_dps[0] = (float)hwt9053_can.gyro_raw[0] / 32768.0f * 2000.0f;
hwt9053_can.gyro_dps[1] = (float)hwt9053_can.gyro_raw[1] / 32768.0f * 2000.0f;
hwt9053_can.gyro_dps[2] = (float)hwt9053_can.gyro_raw[2] / 32768.0f * 2000.0f;
```

处理角度帧 `0x53`：

```c
if (data[2] >= 1u && data[2] <= 3u)
{
    uint8_t axis = (uint8_t)(data[2] - 1u);
    hwt9053_can.angle_raw[axis] = HWT9053_GetInt32(data, 4);
    hwt9053_can.angle_deg[axis] = (float)hwt9053_can.angle_raw[axis] / 1000.0f;

    if (axis == 2u)
    {
        hwt9053_can.yaw_count++;
        HWT9053_UpdateYaw(hwt9053_can.angle_deg[2]);
    }
}
```

其中 `HWT9053_UpdateYaw()` 实现了 yaw 展开和软件零点：

```c
hwt9053_can.yaw_total_deg += delta;
hwt9053_can.yaw_total_zxj = hwt9053_can.yaw_total_deg - hwt9053_yaw_zero_total;
hwt9053_can.yaw_zxj = HWT9053_WrapDeg180(hwt9053_can.yaw_total_zxj);
```

这使得下位机中可以同时使用：

```text
yaw_zxj         折叠到 [-180, 180] 的相对 yaw；
yaw_total_zxj   连续展开的相对 yaw，用于转多圈或持续航向控制。
```

### 3.3 IMU 在线判断

函数：

```c
uint8_t HWT9053CAN_IsOnline(void)
{
    uint32_t now = HAL_GetTick();
    return (hwt9053_can.valid_count > 0u && (now - hwt9053_can.last_tick) < 200u) ? 1u : 0u;
}
```

如果后续需要在上传帧里增加在线标志，可以直接使用这个函数。当前上传帧里没有单独发送在线标志，但发送了：

```text
state
rx_count
valid_count
last_error
```

上位机可以根据 `valid_count` 是否增长来判断 IMU 是否正常。

## 四、底盘控制相关历史记录

虽然当前用户最新需求是 IMU 数据上传，但此前已经对底盘控制逻辑做过较多修改和分析。这部分也需要完整交接，因为后续 IMU 数据记录的目的之一就是辅助分析底盘控制。

### 4.1 起初的问题

用户最初希望验证 CAN2 上连接的 HWT9053-CAN 版 IMU 是否有效。通过 Keil Watch 观察过：

```text
hwt9053_can.state = 2
hwt9053_can.last_error 先为 0x80018，后变为 0x80000
hwt9053_can.rx_count = 0x2a4
hwt9053_can.valid_count = 0x2a4
hwt9053_can.last_data = "UR"
hwt9053_can.last_type = 0x52 'R'
```

这些现象表明 CAN2 能收到 IMU 数据，并且数据类型在变化。后来进一步确定 HWT9053-CAN 的安装方向：

```text
Y 轴朝小车正前方；
X 轴朝小车右方。
```

用户曾经反馈左转 90 度后角度从大约：

```text
-1, 1, 4
```

变成：

```text
-1, 1, 102
```

这说明 yaw 的方向和尺度基本符合预期。

### 4.2 旧 HWT101 串口 IMU 被替换

项目原来有 HWT101 串口 IMU 相关代码。后来用户明确要求：

```text
注释掉 101 部分，直接使用 9053 的适配。
```

当前 `hardware_config.c` 中已经有说明：

```c
/* HWT101 serial IMU disabled; HWT9053-CAN is initialized in main.c. */
```

`chassis.c` 中也已经使用：

```c
#include "hwt9053_can.h"
```

航向反馈来源是：

```c
rc_ctrl.feedback_angle_class = hwt9053_can.yaw_total_zxj;
```

初始化时调用：

```c
HWT9053CAN_SetYawZero();
```

### 4.3 启动朝向作为 0 朝向

用户明确提出：

```text
小车每次启动时的朝向就是初始化 0 朝向。
```

当前实现中，`TaskInit()` 调用：

```c
HWT9053CAN_SetYawZero();
OdomXDrive_ResetAllWithImuZero(&g_odom);
```

这会把当前 IMU yaw 设置为软件零点，并清零里程计。后续 yaw 控制使用 `yaw_total_zxj`，因此小车上电/初始化时的方向就是 0 度。

### 4.4 遥控器优先级

用户曾反馈：

```text
遥控器控制布料小车了；
遥控器又好用了；
运行过程中要优先保证遥控器控制。
```

因此 `OmniCalculate()` 里加入了遥控器优先逻辑：

```c
uint8_t rc_move_active = (...);

if (rc_ctrl.rc_channels[0] < 200 || rc_ctrl.rc_channels[2] < 300)
{
    move_direct_wheel_mode = 0u;
    nx16_ctrl.InTask = 0;
    nx16_ctrl.RxFlag = 0;
    nx16_ctrl.Status = STATUS_IDLE;
    ...
}
else if (rc_move_active)
{
    move_direct_wheel_mode = 0u;
    nx16_ctrl.InTask = 0;
    nx16_ctrl.RxFlag = 0;
    nx16_ctrl.Status = STATUS_IDLE;
}
```

含义：

```text
只要遥控器有明显前后、左右或旋转输入，就打断上位机任务；
停止 direct wheel mode；
清除上位机任务状态；
恢复遥控器控制。
```

### 4.5 上位机前进/后退距离闭环

用户曾要求：

```text
我需要前进一米这种指令准确；
用底层电机编码器里程计计算已经走过的距离；
用 HWT9053-CAN 的 yaw 做航向保持；
两者做走过距离计算后进行位置闭环。
```

最终形成的控制思路是：

```text
编码器作为主距离反馈；
HWT9053 yaw 作为主航向反馈；
融合位姿小权重辅助修正；
横向偏移单独闭环；
完成条件同时检查距离、横向偏移、yaw 误差和速度稳定；
疑似打滑时限制前进速度。
```

当前相关函数：

```text
Chassis_StartMoveTask()
Chassis_UpdateMoveTask()
Chassis_SetMoveWheelSpeed()
MoveTaskClamp()
```

前进任务启动时记录：

```c
move_start_fused_x = g_odom.pose.x_m;
move_start_fused_y = g_odom.pose.y_m;
move_start_x = g_odom.pose.encoder_x_m;
move_start_y = g_odom.pose.encoder_y_m;
move_start_yaw_rad = g_odom.pose.yaw_total_rad;
move_start_yaw_deg = hwt9053_can.yaw_total_zxj;
move_start_encoder_x = move_start_x;
move_start_encoder_y = move_start_y;
move_start_imu_x = g_odom.pose.imu_x_m;
move_start_imu_y = g_odom.pose.imu_y_m;
```

每周期计算：

```c
fused_forward = (-dx_world * s0 + dy_world * c0) * (float)move_dir;
fused_lateral = dx_world * c0 + dy_world * s0;
encoder_traveled = (-dx_encoder_world * s0 + dy_encoder_world * c0) * (float)move_dir;
imu_traveled = (-dx_imu_world * s0 + dy_imu_world * c0) * (float)move_dir;
encoder_lateral = dx_encoder_world * c0 + dy_encoder_world * s0;
imu_lateral = dx_imu_world * c0 + dy_imu_world * s0;
```

控制使用：

```c
control_forward = encoder_traveled + (fused_forward - encoder_traveled) * MOVE_FUSED_POS_WEIGHT;
control_lateral = encoder_lateral + (fused_lateral - encoder_lateral) * MOVE_FUSED_LAT_WEIGHT;
```

当前权重：

```c
#define MOVE_FUSED_POS_WEIGHT 0.25f
#define MOVE_FUSED_LAT_WEIGHT 0.35f
```

这个设计不是完全相信 IMU 双积分，因为 IMU 加速度双积分长期漂移很快；而是编码器主导、融合位姿辅助。

### 4.6 横向偏移和打滑判断

横向控制：

```c
target_lateral_mps = -control_lateral * MOVE_LATERAL_KP_MPS - lateral_vel * MOVE_LATERAL_KD_MPS;
```

限幅：

```c
MOVE_LATERAL_MAX_SPEED_MPS
```

疑似打滑逻辑：

```c
if (fabsf(forward_vel) > MOVE_SLIP_ENC_SPEED_MPS &&
    imu_speed_abs < MOVE_SLIP_IMU_SPEED_MPS &&
    world_acc_abs < MOVE_SLIP_ACCEL_MPS2)
{
    if (move_slip_hold_count < MOVE_SLIP_HOLD_TICK) move_slip_hold_count++;
}
```

如果打滑标志成立：

```c
if (slip_detected && target_forward_mps > MOVE_SLIP_LIMIT_SPEED_MPS)
{
    target_forward_mps = MOVE_SLIP_LIMIT_SPEED_MPS;
}
```

这部分还没有经过充分实车验证。后续可以利用 300Hz IMU 日志分析是否真的能稳定识别打滑。

### 4.7 位置回传仍使用编码器

当前 `App_TaskLoop()` 中：

```c
float x = g_odom.pose.encoder_x_m;
float y = g_odom.pose.encoder_y_m;
float yaw = g_odom.pose.yaw_rad;

nx16_ctrl.current_x = x;
nx16_ctrl.current_y = y;
nx16_ctrl.current_yaw = yaw * 57.2957795f;
```

这意味着上位机状态帧里的位置是编码器里程计位置，不是纯 IMU 积分位置，也不是完全融合位置。此前曾尝试过更相信 IMU，但用户反馈距离与偏移效果变差，因此当前选择是编码器回传更稳定。

## 五、上位机控制脚本历史

### 5.1 `ssss.py`

此前用户要求写一个传输命令的脚本，发送：

```text
前进 1m
等待 30 秒
左转 90 度
```

曾经用 `ssss.py` 发送类似：

```text
AA 06 00 00 00 00 06 DD
AA 01 00 00 80 3F C0 DD
AA 03 00 00 B4 42 F9 DD
```

后来发现没有效果，主要原因是上位机协议、串口、任务状态回传和下位机控制逻辑之间不一致。后续更多参考了旧版本 `car_control_v3.0+1.28.py` 和 `car_control.py`。

### 5.2 `car_controlst.py`

当前 `car_controlst.py` 是上位机控制脚本。主要能力：

```text
打开串口；
发送 INIT/MOVE/ROTATE/STOP/PATH_TRACKING 命令；
监听下位机状态帧；
维护 odom_position、position、rotation；
支持 IMU 帧解析；
支持把 IMU 帧保存 CSV；
```

曾经遇到的问题：

```text
1. 串口断开时报 device reports readiness to read but returned no data；
2. listener 线程里调用 disconnect 导致 cannot join current thread；
3. 中文输出乱码；
4. 上位机 pos 映射和底层 current_x/current_y 理解混乱；
5. 控制命令超时但状态帧没有返回；
6. 前进命令实际方向错；
7. 前进距离比例错；
8. 回传位置与现实运动不一致。
```

已经做过的处理：

```text
1. 串口读写加 serial_lock；
2. disconnect 避免 join 当前 listener 线程；
3. cv2 和 OrbbecCamera 延迟导入；
4. Python 语法检查通过；
5. 新增 IMU 帧解析和 CSV 保存；
6. 默认 baudrate 改为 460800；
7. IMU 帧长度改为 70；
8. IMU 时间戳字段改为 stm32_us。
```

后来用户明确要求：

```text
不要在 car_controlst.py 上做纯 IMU 采集，新写一个。
```

因此新建了 `imu_logger.py`。

### 5.3 `imu_logger.py`

`imu_logger.py` 是当前推荐用于纯 IMU 采集的脚本。它不会发送任何控制命令，也不会初始化小车，不会让小车移动。

它只做：

```text
打开串口；
读取原始字节；
寻找 AA 55 49 IMU 帧；
校验 checksum；
按 <Q10f4I 解包；
写入 CSV；
每秒打印统计；
支持 dump 原始字节帮助诊断。
```

关键常量：

```python
IMU_HEADER = b"\xAA\x55\x49"
STATUS_HEADER = b"\xAA\xAA"
IMU_FRAME_LEN = 70
IMU_PAYLOAD_LEN = 64
STATUS_FRAME_LEN = 86
```

解析函数：

```python
values = struct.unpack("<Q10f4I", payload)
```

诊断输出含义：

```text
bytes   收到的原始串口字节数；
imu     解析成功的 IMU 帧数；
status  收到的原状态帧数；
bad     找到疑似 IMU 帧但校验或长度错误的数量；
rate    上位机实际接收 IMU 帧频率。
```

当前测试为：

```text
bytes=0
imu=0
status=0
bad=0
```

因此问题在串口数据源侧，而不是 Python 解析侧。

## 六、IMU 高速上传设计细节

### 6.1 为什么不用字符串

300Hz 上传时不应该使用 ASCII 文本或 `printf`。原因：

```text
1. 字符串帧长度不固定；
2. 浮点数转字符串耗费 CPU；
3. 串口带宽浪费；
4. 容易受乱码、编码、换行影响；
5. Python 解析复杂且容易丢字段。
```

因此采用固定长度二进制帧。

### 6.2 为什么时间戳用 STM32 微秒

上位机 `pc_time` 只能表示 PC 收到数据的时间，会受到：

```text
串口缓冲；
USB 驱动；
Linux 调度；
Python GIL；
CSV 写盘；
线程调度；
```

等因素影响。要分析 IMU 原始采样间隔，应该用下位机时间戳。当前采用：

```c
DWT_GetTimeline_us()
```

这个时间戳由 Cortex-M DWT cycle counter 换算得到，分辨率比 `HAL_GetTick()` 高很多，适合记录 300Hz 数据间隔。

### 6.3 带宽估算

当前 IMU 帧：

```text
70 字节/帧
300 帧/秒
约 21000 字节/秒
```

UART 8N1 每字节约 10 bit，因此：

```text
21000 B/s * 10 = 210000 bit/s
```

状态帧：

```text
86 字节/帧
20 帧/秒
约 1720 字节/秒
约 17200 bit/s
```

总计约：

```text
227200 bit/s
```

UART1 当前为：

```text
460800 bit/s
```

理论余量约 2 倍。但由于当前发送函数使用阻塞：

```c
HAL_UART_Transmit(UART_X, frame, IMU_FRAME_LEN, 20);
```

实际会占用任务时间。300Hz 下每秒需要发送约 21000 字节，阻塞时间约 0.45 秒/秒，负载较高但可以先验证。若后续影响底盘控制，应改为 DMA 环形队列或降低频率。

### 6.4 后续更稳的方案

如果后续需要严格稳定 300Hz，建议：

```text
1. UART1 波特率提高到 921600；
2. IMU 帧使用 HAL_UART_Transmit_DMA；
3. 建立发送环形缓冲区；
4. 状态帧与 IMU 帧分别排队；
5. 如果发送队列满，丢弃旧 IMU 帧而不阻塞控制；
6. 用单独通信任务处理 UART 发送；
7. 在 CSV 中记录 seq 序号，以便识别丢帧。
```

当前为了快速可用，先用阻塞发送。

## 七、当前故障的明确判断

当前 Ubuntu 输出：

```text
Connected /dev/ttyUSB0@460800
Writing imu_log.csv
Press Ctrl+C to stop.
waiting for IMU frames... bytes=0 status=0 bad=0
...
Done. bytes=0 imu=0 status=0 bad=0
```

这说明：

```text
1. /dev/ttyUSB0 能打开；
2. Python 脚本运行正常；
3. 没有收到任何原始串口字节；
4. 不存在 checksum 错误，因为压根没数据；
5. 不存在 IMU 帧格式不匹配，因为压根没数据；
6. 不存在只收到状态帧的问题，因为 status=0；
7. 不应该继续调 Python 解包格式；
8. 应该检查下位机 UART1 是否真的发出数据。
```

因此下一步排查必须围绕下位机和硬件连接。

## 八、推荐排查流程

### 8.1 确认 Keil 下载的是最新固件

必须重新编译并下载。需要确认下载后板子运行的确实是最新固件。

如果 Keil 编译报错，重点看：

```text
AGENT_UART_HANDLE 未定义；
DWT_GetTimeline_us 未定义；
SendIMUDataToAgent 未定义；
IMUFramePutU64 未定义；
huart1 未定义；
drv_dwt.h 找不到；
```

如果这些错误出现，说明 include 链或文件加入工程有问题。

### 8.2 确认 `nx16.c` 是否加入 Keil 工程

`SendIMUDataToAgent()` 写在 `CUBOT/Cubot_devices/nx16.c`。如果 Keil 工程里没有编译这个文件，或者使用的是另一个路径下的旧 `nx16.c`，则下位机不会包含新函数。

同理，`imu_logger.py` 运行在 Ubuntu 上，它只验证上位机；不能证明下位机已经使用新固件。

### 8.3 确认 `chassis.c` 里的发送调用

应有：

```c
SendStatusAndOdometryToAgent(&AGENT_UART_HANDLE);
SendIMUDataToAgent(&AGENT_UART_HANDLE);
```

并且 `AGENT_UART_HANDLE` 应为：

```c
#define AGENT_UART_HANDLE huart1
```

如果 `chassis.c` 没有包含 `hardware_config.h` 或 include 链断了，Keil 会报宏未定义。如果没有报错，说明宏已能解析。

### 8.4 确认 `ChassisTask()` 是否运行

当前 IMU 上传调用放在 `ChassisTask()` 内。如果底盘任务没有创建、没有调度、被卡死、或者进入某个提前 return，就不会发串口。

可临时在 Keil Watch 中观察：

```c
chassis_times
```

如果这个变量持续变化，说明 `ChassisTask()` 在运行。若不变化，则上传函数不会被调用。

### 8.5 确认 UART1 引脚

根据 `Src/usart.c`：

```text
USART1_TX = PA9
USART1_RX = PB7
```

当前 CP2102 应接：

```text
CP2102 RXD -> PA9
CP2102 TXD -> PB7
GND -> GND
```

如果 CP2102 实际接到了 PG14/PG9，也就是 USART6，那么当前代码不会从那边发数据，Ubuntu 就会 `bytes=0`。

### 8.6 用示波器或逻辑分析仪验证 PA9

如果条件允许，直接测 PA9。正常情况下，以 300Hz 上传 70 字节，PA9 应持续有密集串口波形。若 PA9 没波形：

```text
固件没运行；
任务没运行；
发送函数没被调用；
UART1 没初始化；
板子接线/引脚不对。
```

如果 PA9 有波形而 Ubuntu `bytes=0`：

```text
CP2102 RXD 没接到 PA9；
GND 没共地；
CP2102 模块异常；
Linux 端口不是这个模块；
USB 权限或设备被其它进程占用。
```

### 8.7 尝试旧波特率

如果怀疑固件没下载，或者 UART1 仍然是 115200，可运行：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 115200 --output imu_log.csv --duration 5 --dump-raw 128
```

如果 115200 有数据，说明下位机不是最新 460800 固件。

如果 115200 和 460800 都 `bytes=0`，更可能是接线或下位机没有发。

## 九、可能需要进一步修改的地方

### 9.1 如果 UART1 接收上位机命令也要稳定

当前 UART1 已有 DMA RX/TX 和中断配置：

```text
DMA2_Stream7 USART1_TX
DMA2_Stream5 USART1_RX
USART1_IRQn
```

如果未来上位机控制命令也通过 UART1 发送，`Nx16ControlInit(&AGENT_UART_HANDLE)` 已经会注册到 UART1。需要确认 `drv_usart.c` 对 UART1 的 DMA 空闲中断接收是否兼容 460800。

### 9.2 如果 300Hz 影响底盘控制

当前 `SendIMUDataToAgent()` 使用阻塞发送，可能影响控制任务实时性。若实车发现底盘控制卡顿，应优先改为 DMA 队列发送。设计建议：

```text
1. 定义 UART TX ring buffer；
2. SendIMUDataToAgent 只写 buffer，不阻塞；
3. DMA 空闲时启动发送；
4. DMA 完成回调继续发送下一段；
5. 队列满时丢弃 IMU 旧帧；
6. 状态帧优先级高于 IMU 帧或分开队列。
```

### 9.3 如果需要绝对真实采样时间

当前时间戳是“发送时刻”的微秒时间，而不是 CAN 接收 IMU 数据那一刻。如果要更精确，应在 `HWT9053CAN_RecordRaw()` 中每次收到 CAN 帧时记录：

```c
hwt9053_can.last_acc_us
hwt9053_can.last_gyro_us
hwt9053_can.last_angle_us
```

然后 IMU 上传帧中发送每类数据的最近更新时间。这样可以区分：

```text
CAN 接收时间
UART 发送时间
上位机接收时间
```

当前版本只发送一个 `stm32_us`，代表下位机打包发送时间。

### 9.4 如果要判断丢帧

建议后续在 IMU 帧中增加：

```text
uint32 imu_tx_seq
```

每发送一帧自增。上位机可以检查 seq 是否连续，准确统计丢帧。

当前上位机只能通过接收频率和 `stm32_us` 间隔估算是否丢帧。

## 十、历史上踩过的坑

### 10.1 不要把 IMU 和电机放同一条 CAN，除非波特率一致

曾经误以为 IMU 接 CAN1。如果 IMU 是 250k，而电机 CAN1 是另一个波特率，二者不能共线。现在已确认：

```text
CAN1 电机
CAN2 IMU
```

因此不要把 HWT9053 驱动迁移到 CAN1。

### 10.2 不要用上位机 `pos` 判断下位机所有问题

曾经多次围绕上位机 `position` 映射产生误解。当前：

```python
self.position[0] = curr_x
self.position[1] = 0.0
self.position[2] = curr_y
```

只是上位机渲染或状态维护，不代表底层全部控制逻辑。底层距离控制和回传位置主要来自 `g_odom.pose.encoder_x_m/y_m`。

### 10.3 不要完全相信 IMU 双积分位置

用户曾希望“更相信 IMU 计算出的位置信息，用这个信息更新编码器得到的位置”。实际测试效果不佳，出现距离不到、横向偏移更大等问题。原因是低成本 IMU 加速度双积分在低速底盘上很容易被噪声、震动、姿态误差和零偏影响。

当前合理方案是：

```text
编码器主导距离；
HWT9053 yaw 主导航向；
IMU 加速度用于辅助检测打滑和短时趋势；
不要用 IMU 双积分直接替代编码器位置。
```

### 10.4 不要继续在 car_controlst.py 上堆纯 IMU 采集逻辑

用户已经明确要求“不要在这个上面改，新写一个”。因此纯 IMU 日志采集应使用：

```text
imu_logger.py
```

`car_controlst.py` 保留作为运动控制脚本。

### 10.5 当前 bytes=0 时不要继续调 Python 解析

当前最重要判断：

```text
bytes=0 表示串口没有任何数据进入 Python。
```

此时无论怎么改 `struct.unpack`、checksum、帧头、CSV，都不会有用。必须查：

```text
下位机是否发；
接线是否对；
固件是否最新；
端口是否正确。
```

## 十一、建议给下一位接手者的最短操作清单

如果有人从零接手，按下面顺序做：

```text
1. 打开 IMU_UART_HANDOFF.md，确认硬件拓扑；
2. 用 Keil 编译当前工程；
3. 下载最新固件到 C 型板；
4. 确认 CP2102 接 PA9/PB7/GND；
5. Ubuntu 运行 imu_logger.py 诊断命令；
6. 如果 bytes=0，用示波器测 PA9；
7. 如果 PA9 有波形但 Ubuntu bytes=0，查 CP2102/接线/端口；
8. 如果 PA9 没波形，查 ChassisTask 是否运行和固件是否最新；
9. 如果 bytes>0 status>0 imu=0，查 SendIMUDataToAgent 是否被调用；
10. 如果 imu>0，看 rate 是否接近 300Hz；
11. 如果 rate 不稳，考虑 921600 或 DMA 队列。
```

## 十二、当前文件状态摘要

### 下位机文件

```text
CUBOT/Cubot_devices/hwt9053_can.c
```

用途：

```text
CAN2 接收 HWT9053 数据；
解析 acc、gyro、angle、mag；
维护 yaw 软件零点和展开 yaw。
```

```text
CUBOT/Cubot_devices/nx16.c
```

用途：

```text
上位机协议解析；
状态帧回传；
新增 IMU 帧回传；
新增 70 字节 AA 55 49 IMU 帧。
```

```text
CUBOT/Cubot_modules/chassis.c
```

用途：

```text
底盘控制；
状态帧 20Hz；
IMU 帧约 300Hz；
遥控器优先；
上位机前进/后退距离闭环。
```

```text
CUBOT/Cubot_User_Config/hardware_config.h
```

用途：

```text
定义 AGENT_UART_HANDLE 为 huart1。
```

```text
Src/usart.c
```

用途：

```text
UART1 波特率 460800；
UART1 TX/RX DMA 和中断配置。
```

### 上位机文件

```text
car_controlst.py
```

用途：

```text
控制小车；
解析状态帧；
保留 IMU 解析能力；
不作为纯 IMU 采集主脚本。
```

```text
imu_logger.py
```

用途：

```text
纯 IMU 日志采集；
不发控制命令；
保存 CSV；
打印接收频率；
支持 dump raw 诊断。
```

```text
IMU_UART_HANDOFF.md
```

用途：

```text
当前交接文档；
记录硬件、代码、协议、测试结果和排查路径。
```

## 十三、后续建议的数据分析方式

一旦 `imu_logger.py` 正常收到数据，建议先采集静止 30 秒：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_static.csv --duration 30
```

分析：

```text
1. stm32_us 间隔是否接近 3333us；
2. yaw_total_deg 静止漂移量；
3. gyro_z_dps 静止均值；
4. acc_x/acc_y/acc_z 静止噪声；
5. valid_count 是否持续增加；
6. last_error 是否长期为 0 或稳定值。
```

再采集手动旋转 IMU：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_rotate.csv --duration 30
```

分析：

```text
yaw_deg 是否折叠；
yaw_total_deg 是否连续；
gyro_z_dps 与 yaw_total_deg 微分是否方向一致；
转 90 度时 yaw_total_deg 是否接近 90。
```

最后采集小车前进：

```bash
python imu_logger.py --port /dev/ttyUSB0 --baudrate 460800 --output imu_move.csv --duration 30
```

配合状态帧或上位机控制脚本分析：

```text
前进时 yaw_total_deg 是否稳定；
acc_y 是否有明显启动/停止峰值；
侧滑时 acc_x 是否异常；
编码器速度与 IMU 加速度趋势是否一致。
```

## 十四、如果要进一步完善协议

当前 IMU 帧已经能使用，但为了后续工程化，建议最终升级为：

```text
AA 55 TYPE LEN SEQ TIMESTAMP PAYLOAD CRC16 DD
```

其中：

```text
TYPE = 0x49 表示 IMU
SEQ  = uint32 自增序号
TIMESTAMP = uint64 stm32_us
CRC16 替代 uint8 checksum
```

这样能更可靠地发现丢帧、错帧和串口噪声。当前 uint8 checksum 足够快速验证，但工程长期使用建议 CRC16。

## 十五、最终结论

当前代码层面已经完成：

```text
1. CAN2 读取 HWT9053；
2. UART1 作为上位机通信口；
3. UART1 波特率 460800；
4. 新增 IMU 独立二进制帧；
5. IMU 帧包含微秒时间戳；
6. 目标上传频率约 300Hz；
7. 新增独立 imu_logger.py；
8. Python 能诊断原始字节、状态帧和 IMU 帧。
```

当前实测未通过的点是：

```text
Ubuntu 通过 CP2102 打开 /dev/ttyUSB0 后，完全没有收到任何字节。
```

因此下一步不是修改 Python，而是验证下位机 UART1 是否真实输出。最有效手段是：

```text
1. 重新 Keil 编译下载；
2. 确认 CP2102 接 PA9/PB7/GND；
3. 用示波器或逻辑分析仪测 PA9；
4. 尝试 115200 判断是否旧固件；
5. Keil Watch 检查 ChassisTask 是否运行。
```

只要 `imu_logger.py` 中 `bytes` 开始增加，后续就可以继续判断是状态帧、IMU 帧还是格式问题；但在 `bytes=0` 前，所有协议解析层面的修改都没有意义。

