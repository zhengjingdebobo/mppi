#include "path_tracker.h"
#include "chassis.h"
#include <math.h>    
#include <stdio.h>    
#include <stdint.h>
#include <stdbool.h>  
extern BrainCore_t nx16_ctrl;
extern RC_ctrl_t rc_ctrl;

static bool is_ori_index_initialized = false;

// 物理限制
#define MAX_V               2.0f    // m/s
#define MAX_W               2.0f    // rad/s
#define MAX_ACCEL           0.25f
#define MAX_ALPHA           0.25f    

// 跟踪参数
#define KP_TR               2.5f    // 位置P                                                                               
#define KP_YAW              2.0f    // 航向P
#define KD_YAW              0.2f    // 航向D
#define LOOK_AHEAD_DIST     0.20f    // 前瞻距离

#define LOOKAHEAD_MIN_M   0.10f
#define LOOKAHEAD_MAX_M   0.30f
#define LOOKAHEAD_K_M_PER_MPS  0.08f   // 每 1 m/s 增加15cm


#define MIN_PASSING_SPEED   0.8f    // 最小通行速度
#define ERR_RADIUS          0.08f   // 路径点切换半径

// 路径终点
#define END_SLOW_DIST   0.08f
#define END_STOP_DIST   0.05f

// ================= 全局/静态变量 =================
static size_t current_index = 0;

static float last_valid_target_yaw = 0.0f;
static float last_yaw_error = 0.0f;
static float last_cmd_vx = 0.0f;
static float last_cmd_vy = 0.0f;
static float last_cmd_wz = 0.0f;
static bool  is_finished = false;

// 避障状态
static bool  obs_detected = false;

// 调试使用
volatile float kappa = 0.0f;
volatile float wz_feedforward = 0.0f;
volatile float wz_pid = 0.0f;

// ================= 辅助数学函数 =================
#define M_PI 3.1415926535f
#define RAD_TO_DEG 57.2957795f

// ================= 函数声明 =================
float CalculateCurvature(TrajectoryPoint_t* waypoints,
                         size_t path_size,
                         size_t current_idx,
                         float* dxdt, float* dydt,
                         float* d2xdt2, float* d2ydt2);

static float FloatClamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static float NormalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

static float calc_dist(float x1, float y1, float x2, float y2) {
    float dx = x1 - x2;
    float dy = y1 - y2;
    return sqrtf(dx*dx + dy*dy);
}

static TrajectoryPoint_t asPt(TrajectoryPoint_t* pts, size_t size, int idx) {
    if (idx < 0) idx = 0;
    if (idx >= size) idx = size - 1;
    return pts[idx];
}

void ResetPathIndex(void)
{
    current_index = 0;

    is_ori_index_initialized = false;
    obs_detected = false;
    is_finished = false;

    last_yaw_error = 0.0f;
    last_valid_target_yaw = 0.0f;;
    
    last_cmd_vx = 0.0f;
    last_cmd_vy = 0.0f;
    last_cmd_wz = 0.0f;
}

int32_t GetCurrentPathIndex(void)
{
    return current_index;
}

// 考虑航行&位置的代价函数 -> 遍历选择第一个目标点
void FindClosestIndexWithHeading(float rx, float ry, float ryaw, TrajectoryPoint_t* waypoints, size_t path_size) {
    // 权重可调
    float w_dist = 0.8f;
    float w_yaw = 0.2f;
    float min_cost = 1e9f;
    current_index = 0;

    for (size_t i = 0; i < path_size - 1; i++) {
        float dx = waypoints[i+1].x - waypoints[i].x;
        float dy = waypoints[i+1].y - waypoints[i].y;
        float path_yaw = -atan2f(dy, dx);
        
        float d_yaw = fabsf(NormalizeAngle(ryaw - path_yaw));
        float d_dist = calc_dist(rx, ry, waypoints[i].x, waypoints[i].y);
        
        // 代价函数
        float cost = w_dist * (d_dist * d_dist) + w_yaw * (d_yaw * d_yaw);
        
        if (cost < min_cost) {
            min_cost = cost;
            current_index = i;
        }
    }
}



ControlCmd_t OmniControl(float x, float y, float current_yaw_rad, TrajectoryPoint_t* waypoints, size_t path_size) {
    ControlCmd_t cmd = {0.0f};

    if (path_size == 0 || waypoints == NULL) return cmd;
    
    // 首个目标点搜索
    if (!is_ori_index_initialized) {
        FindClosestIndexWithHeading(x, y, current_yaw_rad, waypoints, path_size);
        is_ori_index_initialized = true;
    }

    // 1. 索引边界保护
    if (current_index >= path_size) current_index = path_size - 1;

    // 2. 预瞄点搜索 (Look Ahead)
    float speed_mps = sqrtf(last_cmd_vx*last_cmd_vx + last_cmd_vy*last_cmd_vy);
    float lookahead_dist_m = LOOKAHEAD_MIN_M + LOOKAHEAD_K_M_PER_MPS * speed_mps;
    lookahead_dist_m = FloatClamp(lookahead_dist_m, LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M);

    size_t look_ahead_idx = current_index;
    for (size_t i = current_index; i < path_size; i++) {
        float d = calc_dist(waypoints[i].x, waypoints[i].y, x, y);
        if (d > lookahead_dist_m) {
            look_ahead_idx = i;
            break;
        }
    }

    float dist = calc_dist(x, y, waypoints[current_index].x, waypoints[current_index].y);
    if (dist < ERR_RADIUS && current_index < path_size - 1) 
    {
        current_index++;
    }
        
    TrajectoryPoint_t target = waypoints[look_ahead_idx];
    float dx = target.x - x;
    float dy = target.y - y;
    
    // 3. 位置闭环 P 控制
    float vx_global = dx * KP_TR;
    float vy_global = dy * KP_TR;
    float v_norm = sqrtf(vx_global*vx_global + vy_global*vy_global);
    
    // 4. 最小通行速度 & 限幅
    bool is_near_end = (path_size - current_index) < 5;
    
    if (!is_near_end && v_norm < MIN_PASSING_SPEED && v_norm > 1e-7f) {
        float scale = MIN_PASSING_SPEED / v_norm;
        vx_global *= scale;
        vy_global *= scale;
    }
    
    v_norm = sqrtf(vx_global*vx_global + vy_global*vy_global);
    if (v_norm > MAX_V) {
        float scale = MAX_V / v_norm;
        vx_global *= scale; vy_global *= scale;
    }
    
    // 5. 坐标系转换 (Global -> Body)
    float c = cosf(current_yaw_rad);
    float s = sinf(current_yaw_rad);

    // 适配“右=Y”里程计 逆解公式：
    float vx_car_target = vx_global * c - vy_global * s;
    float vy_car_target = vx_global * s + vy_global * c; // m/s

    // 6. 姿态控制
    //   float tar_yaw = -atan2f(dy, dx);
    //   last_valid_target_yaw = tar_yaw;
    //   float yaw_error = NormalizeAngle(last_valid_target_yaw - current_yaw_rad);
    //   float wz_car_target = yaw_error * KP_YAW + (yaw_error - last_yaw_error) * KD_YAW;
    
    float dxdt, dydt, d2xdt2, d2ydt2;
    // 基于切线方向 更新目标角度
    kappa = CalculateCurvature(waypoints, path_size,current_index,&dxdt, &dydt, &d2xdt2, &d2ydt2);
    
    float tan_sq = dxdt*dxdt + dydt*dydt;
    // 防止计算角度乱跳
    if (tan_sq > 1e-6f) {
        float tangent_yaw = -atan2f(dydt, dxdt);
        last_valid_target_yaw = tangent_yaw;
    }
     // 先测试纯跟踪后续测试前馈
     if (current_index > 0 && current_index < path_size -1) 
     {
         float FF_GAIN = 0.12; // 前馈增益系数
         wz_feedforward = kappa * (-FF_GAIN * v_norm); // 0.15左右
        //  wz_feedforward = 0;
     } else
     {
         wz_feedforward = 0;
     }

    // 5. 计算反馈量 (PD)
    float yaw_error = NormalizeAngle(last_valid_target_yaw - current_yaw_rad);
    wz_pid = yaw_error * KP_YAW + (yaw_error - last_yaw_error) * KD_YAW;
    last_yaw_error = yaw_error;
    
     // 6. wz输出
    float wz_total_rad = wz_pid + wz_feedforward; // rad/s
    float wz_car_target = wz_total_rad;
   
    //  g_dbg.curr_yaw = current_yaw_rad * RAD_TO_DEG;
    g_dbg.tar_yaw  = last_valid_target_yaw * RAD_TO_DEG; // 输出期望的切线角度


    // 减速策略 -> 后续考虑优化
    float yaw_err_deg = fabsf(yaw_error * RAD_TO_DEG);
    if (yaw_err_deg > 120.0f) {
        vx_car_target = 0; vy_car_target = 0;
    } else if (yaw_err_deg >= 80.0f) {
        float scale = (120.0f - yaw_err_deg) / (120.0f - 80.0f); // 线性缩放
        vx_car_target *= scale; vy_car_target *= scale;
    }

    // 7. 终点停车
    if (is_near_end) {
        TrajectoryPoint_t end_pt = waypoints[path_size - 1];
        float end_dist = calc_dist(x, y, end_pt.x, end_pt.y);
        
        if (end_dist < END_SLOW_DIST) {
            float scale = (end_dist - END_STOP_DIST) / (END_SLOW_DIST - END_STOP_DIST);
            if (scale < 0) scale = 0;
            vx_car_target *= scale; vy_car_target *= scale; wz_car_target *= scale;
        }
        
        if (end_dist < END_STOP_DIST) {
            is_finished = true;
            last_cmd_vx = 0; last_cmd_vy = 0; last_cmd_wz = 0;
            cmd.vx_cmd = 0; cmd.vy_cmd = 0; cmd.wz_cmd = 0;
            return cmd;
        }
    }

    // 8. 斜坡限幅 (Ramp / Accel Limit)
    float diff_vx = vx_car_target - last_cmd_vx;
    last_cmd_vx += FloatClamp(diff_vx, -MAX_ACCEL, MAX_ACCEL);
    
    float diff_vy = vy_car_target - last_cmd_vy;
    last_cmd_vy += FloatClamp(diff_vy, -MAX_ACCEL, MAX_ACCEL);
    
    float diff_wz = wz_car_target - last_cmd_wz;
    last_cmd_wz += FloatClamp(diff_wz, -MAX_ALPHA, MAX_ALPHA);

    // 9. 输出限幅
    cmd.vx_cmd = FloatClamp(last_cmd_vx, -MAX_V, MAX_V) * 1e3;
    cmd.vy_cmd = FloatClamp(last_cmd_vy, -MAX_V, MAX_V) * 1e3;
    cmd.wz_cmd = FloatClamp(last_cmd_wz, -MAX_W, MAX_W) * 1e3;

    return cmd;
}


float CalculateCurvature(TrajectoryPoint_t* waypoints,
                         size_t path_size,
                         size_t current_idx,
                         float* dxdt, float* dydt,
                         float* d2xdt2, float* d2ydt2)
{
    if (waypoints == NULL || path_size < 3 ||
        dxdt == NULL || dydt == NULL ||
        d2xdt2 == NULL || d2ydt2 == NULL)
    {
        return 0.0f;
    }

    // 确保 current_idx 在 [1, path_size-2]
    size_t idx = current_idx;
    if (idx == 0) idx = 1;
    if (idx >= path_size - 1) idx = path_size - 2;

    // ---------- 1) 内部可调参数 ----------
    // 点太密/重复点 -> 需要“跨步取点”
    const int STEP = 6;              
    const float MIN_SEP = 0.005f;

    // ---------- 2) 选 prev/next（跨步 + 去重，避免曲率为0） ----------
    size_t ip = (idx > (size_t)STEP) ? (idx - (size_t)STEP) : 0;
    size_t in = (idx + (size_t)STEP < path_size) ? (idx + (size_t)STEP) : (path_size - 1);

    // 向前跳过“重复/极密点”
    while (ip > 0 && calc_dist(waypoints[ip].x, waypoints[ip].y,
                              waypoints[idx].x, waypoints[idx].y) < MIN_SEP)
    {
        ip--;
    }

    // 向后跳过“重复/极密点”
    while (in + 1 < path_size && calc_dist(waypoints[in].x, waypoints[in].y,
                                          waypoints[idx].x, waypoints[idx].y) < MIN_SEP)
    {
        in++;
    }

    // 如果仍然退化（例如整段点都重合），直接返回 0
    if (ip == idx || in == idx || ip == in)
    {
        *dxdt = 1.0f;
        *dydt = 0.0f;
        *d2xdt2 = 0.0f;
        *d2ydt2 = 0.0f;
        return 0.0f;
    }

    // ---------- 3) 切线方向（用于 target_yaw） ----------
    // 注意：这里输出的是“差值切线”，不是除以 dt；对 yaw 来说只看方向即可
    *dxdt = waypoints[in].x - waypoints[ip].x;
    *dydt = waypoints[in].y - waypoints[ip].y;

    float tan_norm2 = (*dxdt)*(*dxdt) + (*dydt)*(*dydt);
    if (tan_norm2 < 1e-15f)
    {
        *dxdt = 1.0f;
        *dydt = 0.0f;
        *d2xdt2 = 0.0f;
        *d2ydt2 = 0.0f;
        return 0.0f;
    }

    // ---------- 4) 三点外接圆法曲率（最稳，抗点密/不等距/重复） ----------
    TrajectoryPoint_t A = waypoints[ip];
    TrajectoryPoint_t B = waypoints[idx];
    TrajectoryPoint_t C = waypoints[in];

    float ABx = B.x - A.x, ABy = B.y - A.y;
    float ACx = C.x - A.x, ACy = C.y - A.y;
    float BCx = C.x - B.x, BCy = C.y - B.y;

    float AB = sqrtf(ABx*ABx + ABy*ABy);
    float AC = sqrtf(ACx*ACx + ACy*ACy);
    float BC = sqrtf(BCx*BCx + BCy*BCy);

    float denom = AB * AC * BC;
    if (denom < 1e-9f)
    {
        *d2xdt2 = 0.0f;
        *d2ydt2 = 0.0f;
        return 0.0f;
    }

    // 2倍有符号面积（决定左右转符号）
    float cross = ABx*ACy - ABy*ACx;

    // 曲率 kappa：单位 1/m（当 x,y 单位为 m）
    float kappa = 2.0f * cross / denom;

    // ---------- 5) 二阶差分输出 ----------
    *d2xdt2 = (C.x - 2.0f*B.x + A.x);
    *d2ydt2 = (C.y - 2.0f*B.y + A.y);

    // 数值保护
    if (!isfinite(kappa)) kappa = 0.0f;

    return kappa;
}


