#ifndef CHASSIS_VELOCITY_CONFIG_H
#define CHASSIS_VELOCITY_CONFIG_H

/*
 * 新底盘速度框架的集中参数表。
 *
 * 第一阶段仅给出保守初值，所有与实车相关的参数都需要在后续测试中标定。
 * 旧定距离、定角度控制继续使用原有参数，不受本文件影响。
 */

/* 控制周期与命令安全超时。 */
#define CHASSIS_VELOCITY_CONTROL_PERIOD_S       0.005f
/* 与上面的秒值必须保持一致；状态式补偿使用它累计毫秒计时。 */
#define CHASSIS_VELOCITY_CONTROL_PERIOD_MS          5u
#define CHASSIS_VELOCITY_COMMAND_TIMEOUT_MS     500u

/*
 * 新串口速度链的最大平移合速度，单位 m/s。
 *
 * 前进、后退、横移和任意斜向共用同一个上限；控制器按 vx/vy 矢量模长
 * 等比例缩放，因此达到上限时不会改变用户指定的运动角度。
 *
 * 增大：可以提高所有方向的最高速度，但需要同步确认单轮 ERPM 上限、
 *       VESC 电流、电机温度、麦轮打滑和制动距离。
 * 减小：可以统一降低所有方向的最高速度，不需要分别调整前后和横移参数。
 *
 * 当前 0.50 m/s 在最不利的 45° 方向补偿后约需 12080 ERPM，
 * 与下方 13000 ERPM 单轮保护上限配套使用。
 */
#define CHASSIS_VELOCITY_MAX_TRANSLATION_MPS    0.50f
/* 新速度入口的最高旋转速度；重载旋转不稳时降低该值。 */
#define CHASSIS_VELOCITY_MAX_WZ_RADPS           0.60f
/*
 * 最大平移加速度：重量增加后通常应降低，减少起步电流和四轮不同步。
 */
#define CHASSIS_VELOCITY_MAX_ACCEL_MPS2         0.35f
/*
 * 最大平移减速度：重量增加后适当降低，可减少急停打滑和姿态冲击。
 */
#define CHASSIS_VELOCITY_MAX_DECEL_MPS2         0.45f
/*
 * 最大角加速度：重量或转动惯量增加后应降低，减少旋转起步冲击。
 */
#define CHASSIS_VELOCITY_MAX_ANG_ACCEL_RADPS2   1.50f

/* 麦克纳姆/X-drive 几何与传动参数。 */
#define MECANUM_WHEEL_DIAMETER_M                0.1075f
#define MECANUM_CHASSIS_LENGTH_M                0.4000f
#define MECANUM_CHASSIS_WIDTH_M                 0.4000f
#define MECANUM_GEAR_RATIO                      19.0f
#define MECANUM_MOTOR_POLE_PAIRS                7.0f
#define MECANUM_INV_SQRT2                       0.70710678f

/*
 * 10 deg/s 实车标定得到稳态约 17.75 deg/s，因此旋转 ERPM 单独乘
 * 10 / 17.75 = 0.563。该比例不影响已经验证的平移换算。
 */
#define MECANUM_YAW_ERPM_SCALE                  0.563f

/*
 * 2026-07-29 实车复测：
 * 串口定角速度命令的左右方向与物理旋转方向相反，而定角度旧任务正确。
 * 因此只反向新速度运动学的 yaw 指令；该符号不影响前后/横向平移，
 * 也不修改定角度旧任务链。
 */
#define MECANUM_YAW_COMMAND_SIGN                (-1.0f)
#define IMU_STATE_YAW_SIGN                      (1.0f)

/*
 * VESC 全局反馈符号仍服务于旧里程计，不在驱动层改动。
 * 新速度链在 wheel_feedback 边界转换为新运动学的 LF/RF/LB/RB 约定。
 */
#define MECANUM_FEEDBACK_SIGN_LF                (1.0f)
#define MECANUM_FEEDBACK_SIGN_RF                (-1.0f)
#define MECANUM_FEEDBACK_SIGN_LB                (-1.0f)
#define MECANUM_FEEDBACK_SIGN_RB                (1.0f)

/*
 * ======================= RPM 死区与载荷配置区 =======================
 *
 * 2026-07-29 当前载荷四轮绝对 ERPM 标定结果：
 * 800 RPM 不转，900 RPM 可缓慢起转，1000 RPM 可可靠运行；
 * 900 RPM 稳定后的实际转速约为 600 RPM，1000 RPM 以上接近 1:1。
 *
 * 更换电池、增加上层机构或改变重心后，只调整本配置区，不修改
 * rpm_compensation.c 中的算法。建议重新执行 RPM 标定后再填写。
 */

/*
 * 每轮可靠起转指令：
 * 增大：更容易克服静摩擦，但起步冲击和电流会增加；
 * 减小：起步更柔和，但重载时可能无法起转或起转时间过长。
 */
#define RPM_COMP_START_RPM_LF                    1000.0f
#define RPM_COMP_START_RPM_RF                    1000.0f
#define RPM_COMP_START_RPM_LB                    1000.0f
#define RPM_COMP_START_RPM_RB                    1000.0f

/*
 * 每轮最低运行指令：
 * 电机已经转动后，用该值维持低速运动。通常应低于 START_RPM。
 * 增大可降低运行中再次停转的概率，但最低车速也会升高。
 */
#define RPM_COMP_RUN_CMD_MIN_RPM_LF               900.0f
#define RPM_COMP_RUN_CMD_MIN_RPM_RF               900.0f
#define RPM_COMP_RUN_CMD_MIN_RPM_LB               900.0f
#define RPM_COMP_RUN_CMD_MIN_RPM_RB               900.0f

/*
 * 每轮最低稳定实际转速：
 * 填写 RUN_CMD_MIN 指令稳定后测得的实际绝对 RPM，用于反向映射。
 * 数值越大，表示当前载荷下无法平稳实现的低速区越宽。
 */
#define RPM_COMP_RUN_ACTUAL_MIN_RPM_LF            600.0f
#define RPM_COMP_RUN_ACTUAL_MIN_RPM_RF            600.0f
#define RPM_COMP_RUN_ACTUAL_MIN_RPM_LB            600.0f
#define RPM_COMP_RUN_ACTUAL_MIN_RPM_RB            600.0f

/*
 * 每轮低速映射终点：
 * 目标低于该值时执行死区反向映射；达到该值后恢复线性增益输出。
 * 如果增重后 1000 RPM 附近仍明显非线性，可适当提高此值。
 */
#define RPM_COMP_LINEAR_END_RPM_LF                1000.0f
#define RPM_COMP_LINEAR_END_RPM_RF                1000.0f
#define RPM_COMP_LINEAR_END_RPM_LB                1000.0f
#define RPM_COMP_LINEAR_END_RPM_RB                1000.0f

/*
 * 每轮线性区增益：
 * 只修正 LINEAR_END 以上的输出斜率，不影响启动增强和最低运行指令，
 * 并保证分段连接点连续。大于 1 提高该轮高转速输出，小于 1 则降低；
 * 1.0 表示不修正。
 */
#define RPM_COMP_LINEAR_GAIN_LF                      1.004f
#define RPM_COMP_LINEAR_GAIN_RF                      0.999f
#define RPM_COMP_LINEAR_GAIN_LB                      1.024f
#define RPM_COMP_LINEAR_GAIN_RB                      1.015f

/*
 * 起转反馈阈值：
 * 实际绝对 RPM 达到该值才开始累计起转确认时间。
 * 过低容易把抖动误判为起转；过高会延长 1000 RPM 启动增强时间。
 */
#define RPM_COMP_START_FDB_THRESHOLD_RPM            300.0f

/*
 * 启动增强最短保持时间：
 * 即使进入启动态时仍读到旧的高速反馈，也至少输出 START_RPM 这么久。
 * 增大可增强突破静摩擦的可靠性，但会增加起步冲击。
 */
#define RPM_COMP_START_BOOST_MIN_MS                   50u

/*
 * 起转确认时间：
 * 实际 RPM 必须连续超过起转阈值这么久，才进入正常运行分段。
 * 增大可提高确认可靠性，但启动增强保持时间会更长。
 */
#define RPM_COMP_START_CONFIRM_MS                     25u

/*
 * 单次起转超时：
 * 持续输出 START_RPM 仍未确认起转时，按一次失败处理。
 * 重载起转较慢时可增大，但不宜过大，以免堵转大电流持续过久。
 */
#define RPM_COMP_START_TIMEOUT_MS                   1500u

/*
 * 堵转反馈阈值与确认时间：
 * 已运行且实际绝对 RPM 连续低于阈值达到确认时间，则重新起转。
 * 阈值过高会误判正常低速波动；确认时间过长会延迟堵转保护。
 */
#define RPM_COMP_STALL_FDB_THRESHOLD_RPM             100.0f
#define RPM_COMP_STALL_CONFIRM_MS                     200u

/*
 * 完全停止反馈阈值与确认时间：
 * 零目标后，实际绝对 RPM 连续低于该阈值达到确认时间，才认为已静止。
 * 阈值过高可能把仍在慢转的轮子误判为停止；时间过短容易受反馈抖动影响。
 */
#define RPM_COMP_STOP_FDB_THRESHOLD_RPM               50.0f
#define RPM_COMP_STOP_CONFIRM_MS                      100u

/*
 * 滑行状态保持时间：
 * 零目标后的这段时间内，如果同方向命令恢复且轮子仍在转，
 * 直接回到运行态，不重复施加 START_RPM 启动增强。
 */
#define RPM_COMP_COAST_HOLD_MS                        300u

/*
 * 反向释放阈值与等待超时：
 * 方向改变后先输出零，旧方向实际转速下降到该阈值以下才反向增强。
 * 阈值越低反向冲击越小；等待超过超时时间则锁存故障并停车。
 */
#define RPM_COMP_REVERSE_RELEASE_RPM                 100.0f
#define RPM_COMP_REVERSE_TIMEOUT_MS                 1000u

/*
 * 最大连续重新起转次数：
 * 超过次数后补偿模块锁存故障，速度控制链立即退出并停车。
 * 增大可容忍短时阻力，但会增加电机堵转发热风险。
 */
#define RPM_COMP_RESTART_MAX_COUNT                      2u

/*
 * 零目标阈值：
 * 目标绝对值不超过该值时直接停车并复位该轮状态。
 * 增大可过滤浮点残留，但会吞掉更多很小的有效目标。
 */
#define RPM_COMP_STOP_EPSILON_RPM                     1.0f

/*
 * 补偿输出绝对上限：
 * 用于拦截异常参数或增益导致的过大指令，不能删除该安全保护。
 * 当前 13000 ERPM 可覆盖任意方向 0.50 m/s：最不利 45° 方向的
 * 补偿需求约为 12080 ERPM，并保留约 7.6% 余量。
 *
 * 如果以后增大 CHASSIS_VELOCITY_MAX_TRANSLATION_MPS，必须重新计算最不利
 * 方向的单轮需求，并通过电流、温升、打滑和制动测试后再提高本值。
 */
#define RPM_COMP_MAX_OUTPUT_RPM                      13000.0f

/* 互补状态估计参数。 */
#define STATE_ESTIMATOR_GYRO_WEIGHT             0.80f
#define STATE_ESTIMATOR_MIN_DT_S                0.001f
#define STATE_ESTIMATOR_MAX_DT_S                0.020f

/* 旋转打滑检测初值，单位为 rad/s 和秒。 */
#define SLIP_DETECT_ENTER_THRESHOLD_RADPS       0.20f
#define SLIP_DETECT_EXIT_THRESHOLD_RADPS        0.10f
#define SLIP_DETECT_ENTER_HOLD_S                 0.10f
#define SLIP_DETECT_EXIT_HOLD_S                  0.30f
#define SLIP_DETECT_SCORE_FILTER_TAU_S           0.10f

/* 遥控器抢占新速度控制时使用的摇杆死区。 */
#define CHASSIS_VELOCITY_RC_MOVE_DEADBAND       35
#define CHASSIS_VELOCITY_RC_YAW_DEADBAND        80

#endif /* CHASSIS_VELOCITY_CONFIG_H */
