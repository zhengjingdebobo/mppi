#include "odom_xdrive.h"

#include "hwt9053_can.h"

#include <math.h>
#include <string.h>

#define ODOM_GRAVITY_MPS2        9.80665f
#define ODOM_ACCEL_BIAS_SAMPLE_N 200u
#define ODOM_STATIONARY_VEL_MPS  0.015f

static uint8_t odom_is_finite(float x)
{
    if (x != x) return 0u;
    if (x == INFINITY) return 0u;
    if (x == -INFINITY) return 0u;
    return 1u;
}

static float odom_wrap_pi(float a)
{
    while (a > ODOM_PI) a -= ODOM_2PI;
    while (a < -ODOM_PI) a += ODOM_2PI;
    return a;
}

static float odom_clampf(float x, float min_v, float max_v)
{
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return x;
}

static void odom_inverse_kinematics(const OdomXDrive_Config_t *cfg,
                                    float vt_lf, float vt_rf, float vt_lb, float vt_rb,
                                    float *vx_raw, float *vy_raw, float *wz_raw)
{
    const float a = cfg->inv_sqrt2;
    const float denom = 4.0f * a;
    float forward_raw = ((vt_rf - vt_lf) + (vt_rb - vt_lb)) / denom;
    float right_raw = ((vt_lb - vt_lf) + (vt_rb - vt_rf)) / denom;

    *vx_raw = right_raw;
    *vy_raw = forward_raw;
    *wz_raw = (vt_lf + vt_rf + vt_lb + vt_rb) * 0.25f;
}

static void odom_reset_filter(OdomXDrive_t *odom)
{
    memset(odom->state_x, 0, sizeof(odom->state_x));
    memset(odom->state_y, 0, sizeof(odom->state_y));
    memset(odom->cov_x, 0, sizeof(odom->cov_x));
    memset(odom->cov_y, 0, sizeof(odom->cov_y));

    odom->state_x[0] = odom->pose.x_m;
    odom->state_y[0] = odom->pose.y_m;
    odom->cov_x[0][0] = 1.0e-4f;
    odom->cov_x[1][1] = 1.0e-2f;
    odom->cov_y[0][0] = 1.0e-4f;
    odom->cov_y[1][1] = 1.0e-2f;
    odom->filter_ready = 1u;
}

static void odom_kf_predict_axis(float state[2], float cov[2][2],
                                 float accel, float dt,
                                 float q_pos, float q_vel)
{
    float p00 = cov[0][0];
    float p01 = cov[0][1];
    float p10 = cov[1][0];
    float p11 = cov[1][1];

    state[0] += state[1] * dt + 0.5f * accel * dt * dt;
    state[1] += accel * dt;

    cov[0][0] = p00 + dt * (p10 + p01) + dt * dt * p11 + q_pos;
    cov[0][1] = p01 + dt * p11;
    cov[1][0] = p10 + dt * p11;
    cov[1][1] = p11 + q_vel;
}

static void odom_kf_update_pos(float state[2], float cov[2][2], float z_pos, float r_pos)
{
    float p00 = cov[0][0];
    float p01 = cov[0][1];
    float p10 = cov[1][0];
    float p11 = cov[1][1];
    float innovation = z_pos - state[0];
    float s = p00 + r_pos;
    float k0;
    float k1;

    if (s <= 1.0e-9f) return;

    k0 = p00 / s;
    k1 = p10 / s;

    state[0] += k0 * innovation;
    state[1] += k1 * innovation;

    cov[0][0] = (1.0f - k0) * p00;
    cov[0][1] = (1.0f - k0) * p01;
    cov[1][0] = p10 - k1 * p00;
    cov[1][1] = p11 - k1 * p01;
}

static void odom_kf_update_vel(float state[2], float cov[2][2], float z_vel, float r_vel)
{
    float p00 = cov[0][0];
    float p01 = cov[0][1];
    float p10 = cov[1][0];
    float p11 = cov[1][1];
    float innovation = z_vel - state[1];
    float s = p11 + r_vel;
    float k0;
    float k1;

    if (s <= 1.0e-9f) return;

    k0 = p01 / s;
    k1 = p11 / s;

    state[0] += k0 * innovation;
    state[1] += k1 * innovation;

    cov[0][0] = p00 - k0 * p10;
    cov[0][1] = p01 - k0 * p11;
    cov[1][0] = (1.0f - k1) * p10;
    cov[1][1] = (1.0f - k1) * p11;
}

static void odom_update_yaw(OdomXDrive_t *odom)
{
    OdomXDrive_Pose2D_t *pose = &odom->pose;
    const OdomXDrive_Config_t *cfg = &odom->cfg;
    HWT9053Heading_t heading;
    float yaw_deg_src;

    if (!HWT9053CAN_GetHeading(&heading))
    {
        pose->valid = 0u;
        return;
    }
    yaw_deg_src = heading.yaw_total_deg;
    if (!odom_is_finite(yaw_deg_src))
    {
        pose->valid = 0u;
        return;
    }

    {
        float cand = (yaw_deg_src * cfg->yaw_sign) * ODOM_DEG2RAD + cfg->yaw_offset_rad;

        if (pose->update_cnt > 0u)
        {
            float dy = odom_wrap_pi(cand - pose->yaw_total_rad);
            float jump_rad = cfg->yaw_jump_deg * ODOM_DEG2RAD;

            if (fabsf(dy) > jump_rad)
            {
                pose->reject_cnt_yaw++;
                pose->valid = 0u;
                return;
            }
        }

        pose->yaw_total_rad = cand;
        pose->yaw_rad = odom_wrap_pi(pose->yaw_total_rad);
        pose->wz_dps = heading.gyro_z_dps;
    }
}

OdomXDrive_Config_t OdomXDrive_GetDefaultConfig(void)
{
    OdomXDrive_Config_t cfg;

    cfg.length_m = 0.4f;
    cfg.width_m = 0.4f;
    cfg.inv_sqrt2 = 0.70710678f;
    cfg.lf_sign = 1.0f;
    cfg.rf_sign = 1.0f;
    cfg.lb_sign = 1.0f;
    cfg.rb_sign = 1.0f;
    cfg.yaw_sign = 1.0f;
    cfg.yaw_offset_rad = 0.0f;
    cfg.yaw_jump_deg = 90.0f;
    cfg.k_pos_m_per_unit = VESC_WHEEL_M_PER_OUTPUT_DEG;
    cfg.nominal_dt_s = 0.001f;
    cfg.imu_accel_deadband_mps2 = 0.08f;
    cfg.process_noise_pos = 5.0e-6f;
    cfg.process_noise_vel = 2.0e-3f;
    cfg.meas_noise_pos = 4.0e-4f;
    cfg.meas_noise_vel = 2.5e-2f;
    cfg.encoder_feedback_gain = 0.0f;
    cfg.encoder_feedback_max_mps = 0.0f;

    return cfg;
}

void OdomXDrive_InitOnce(OdomXDrive_t *odom, const OdomXDrive_Config_t *cfg)
{
    if (!odom || !cfg) return;
    if (odom->inited) return;

    memset(odom, 0, sizeof(*odom));
    odom->cfg = *cfg;
    odom->pose.valid = 1u;
    odom->last_update_tick = HAL_GetTick();
    odom->inited = 1u;
    odom->first_run = 1u;
    odom->accel_bias_ready = 0u;
    odom->acc_bias_sample_count = 0u;
    odom->acc_bias_x_g = 0.0f;
    odom->acc_bias_y_g = 0.0f;
    odom->acc_bias_sum_x_g = 0.0f;
    odom->acc_bias_sum_y_g = 0.0f;
    odom_reset_filter(odom);
}

void OdomXDrive_BindVESC(OdomXDrive_t *odom,
                         uint8_t vesc_id_lf,
                         uint8_t vesc_id_rf,
                         uint8_t vesc_id_lb,
                         uint8_t vesc_id_rb)
{
    if (!odom) return;

    odom->vesc_id_lf = vesc_id_lf;
    odom->vesc_id_rf = vesc_id_rf;
    odom->vesc_id_lb = vesc_id_lb;
    odom->vesc_id_rb = vesc_id_rb;
}

void OdomXDrive_ResetAllWithImuZero(OdomXDrive_t *odom)
{
    if (!odom || !odom->inited) return;

    HWT9053CAN_SetYawZero();

    memset(&odom->pose, 0, sizeof(odom->pose));
    odom->pose.yaw_total_rad = odom->cfg.yaw_offset_rad;
    odom->pose.yaw_rad = odom_wrap_pi(odom->pose.yaw_total_rad);
    odom->pose.valid = 1u;
    odom->last_update_tick = HAL_GetTick();
    odom->first_run = 1u;
    odom->accel_bias_ready = 0u;
    odom->acc_bias_sample_count = 0u;
    odom->acc_bias_x_g = 0.0f;
    odom->acc_bias_y_g = 0.0f;
    odom->acc_bias_sum_x_g = 0.0f;
    odom->acc_bias_sum_y_g = 0.0f;
    odom_reset_filter(odom);
}

void OdomXDrive_Update(OdomXDrive_t *odom)
{
    OdomXDrive_Pose2D_t *pose;
    const OdomXDrive_Config_t *cfg;
    float v_lf;
    float v_rf;
    float v_lb;
    float v_rb;
    float a_lf;
    float a_rf;
    float a_lb;
    float a_rb;
    float da_lf;
    float da_rf;
    float da_lb;
    float da_rb;
    float vt_lf;
    float vt_rf;
    float vt_lb;
    float vt_rb;
    float dx_body_m;
    float dy_body_m;
    float dx_world_m;
    float dy_world_m;
    float enc_vx_world;
    float enc_vy_world;
    float imu_pred_x_m;
    float imu_pred_y_m;
    float imu_pred_vx_mps;
    float imu_pred_vy_mps;
    float feedback_gain;
    float max_feedback_step;
    float corr_x;
    float corr_y;
    float body_ax_mps2;
    float body_ay_mps2;
    float world_ax_mps2;
    float world_ay_mps2;
    float c;
    float s;
    uint32_t now_tick;
    uint32_t delta_tick;
    float dt;

    if (!odom || !odom->inited) return;

    pose = &odom->pose;
    cfg = &odom->cfg;

    if (!VESCMotorAllFeedbackOnline())
    {
        pose->valid = 0u;
        pose->update_cnt++;
        return;
    }

    now_tick = HAL_GetTick();
    delta_tick = now_tick - odom->last_update_tick;
    odom->last_update_tick = now_tick;
    dt = (delta_tick > 0u) ? ((float)delta_tick * 0.001f) : cfg->nominal_dt_s;
    if (dt <= 0.0f || dt > 0.1f) dt = cfg->nominal_dt_s;

    pose->valid = 1u;

    v_lf = VESCMotorGetLogicalOutputSpeedDPS(VESC_WHEEL_LF);
    v_rf = VESCMotorGetLogicalOutputSpeedDPS(VESC_WHEEL_RF);
    v_lb = VESCMotorGetLogicalOutputSpeedDPS(VESC_WHEEL_LB);
    v_rb = VESCMotorGetLogicalOutputSpeedDPS(VESC_WHEEL_RB);
    a_lf = VESCMotorGetLogicalOutputAngleDeg(VESC_WHEEL_LF);
    a_rf = VESCMotorGetLogicalOutputAngleDeg(VESC_WHEEL_RF);
    a_lb = VESCMotorGetLogicalOutputAngleDeg(VESC_WHEEL_LB);
    a_rb = VESCMotorGetLogicalOutputAngleDeg(VESC_WHEEL_RB);

    if (!odom_is_finite(v_lf) || !odom_is_finite(v_rf) || !odom_is_finite(v_lb) || !odom_is_finite(v_rb) ||
        !odom_is_finite(a_lf) || !odom_is_finite(a_rf) || !odom_is_finite(a_lb) || !odom_is_finite(a_rb))
    {
        pose->reject_cnt_wheel++;
        pose->valid = 0u;
        pose->update_cnt++;
        return;
    }

    vt_lf = cfg->lf_sign * v_lf;
    vt_rf = cfg->rf_sign * v_rf;
    vt_lb = cfg->lb_sign * v_lb;
    vt_rb = cfg->rb_sign * v_rb;

    odom_inverse_kinematics(cfg, vt_lf, vt_rf, vt_lb, vt_rb,
                            &pose->vx_raw, &pose->vy_raw, &pose->wz_raw);

    if (odom->first_run)
    {
        odom->last_angle_lf = a_lf;
        odom->last_angle_rf = a_rf;
        odom->last_angle_lb = a_lb;
        odom->last_angle_rb = a_rb;
        odom->first_run = 0u;
    }

    da_lf = cfg->lf_sign * (a_lf - odom->last_angle_lf);
    da_rf = cfg->rf_sign * (a_rf - odom->last_angle_rf);
    da_lb = cfg->lb_sign * (a_lb - odom->last_angle_lb);
    da_rb = cfg->rb_sign * (a_rb - odom->last_angle_rb);

    odom->last_angle_lf = a_lf;
    odom->last_angle_rf = a_rf;
    odom->last_angle_lb = a_lb;
    odom->last_angle_rb = a_rb;

    odom_update_yaw(odom);
    if (!pose->valid)
    {
        pose->update_cnt++;
        return;
    }

    c = cosf(pose->yaw_total_rad);
    s = sinf(pose->yaw_total_rad);

    odom_inverse_kinematics(cfg, da_lf, da_rf, da_lb, da_rb,
                            &dx_body_m, &dy_body_m, &pose->wz_raw);
    dx_body_m *= cfg->k_pos_m_per_unit;
    dy_body_m *= cfg->k_pos_m_per_unit;

    dx_world_m = c * dx_body_m - s * dy_body_m;
    dy_world_m = s * dx_body_m + c * dy_body_m;

    pose->encoder_x_m += dx_world_m;
    pose->encoder_y_m += dy_world_m;

    enc_vx_world = dx_world_m / dt;
    enc_vy_world = dy_world_m / dt;

    if (!odom->accel_bias_ready)
    {
        if (hwt9053_can.acc_count > 0u)
        {
            odom->acc_bias_sum_x_g += hwt9053_can.acc_g[0];
            odom->acc_bias_sum_y_g += hwt9053_can.acc_g[1];
            odom->acc_bias_sample_count++;

            if (odom->acc_bias_sample_count >= ODOM_ACCEL_BIAS_SAMPLE_N)
            {
                odom->acc_bias_x_g = odom->acc_bias_sum_x_g / (float)odom->acc_bias_sample_count;
                odom->acc_bias_y_g = odom->acc_bias_sum_y_g / (float)odom->acc_bias_sample_count;
                odom->accel_bias_ready = 1u;
            }
        }

        body_ax_mps2 = 0.0f;
        body_ay_mps2 = 0.0f;
    }
    else
    {
        body_ax_mps2 = (hwt9053_can.acc_g[0] - odom->acc_bias_x_g) * ODOM_GRAVITY_MPS2;
        body_ay_mps2 = (hwt9053_can.acc_g[1] - odom->acc_bias_y_g) * ODOM_GRAVITY_MPS2;
    }

    if (fabsf(body_ax_mps2) < cfg->imu_accel_deadband_mps2) body_ax_mps2 = 0.0f;
    if (fabsf(body_ay_mps2) < cfg->imu_accel_deadband_mps2) body_ay_mps2 = 0.0f;

    world_ax_mps2 = c * body_ax_mps2 - s * body_ay_mps2;
    world_ay_mps2 = s * body_ax_mps2 + c * body_ay_mps2;

    if (!odom->filter_ready) odom_reset_filter(odom);

    odom_kf_predict_axis(odom->state_x, odom->cov_x,
                         world_ax_mps2, dt,
                         cfg->process_noise_pos * dt,
                         cfg->process_noise_vel * dt);
    odom_kf_predict_axis(odom->state_y, odom->cov_y,
                         world_ay_mps2, dt,
                         cfg->process_noise_pos * dt,
                         cfg->process_noise_vel * dt);

    imu_pred_x_m = odom->state_x[0];
    imu_pred_y_m = odom->state_y[0];
    imu_pred_vx_mps = odom->state_x[1];
    imu_pred_vy_mps = odom->state_y[1];
    pose->imu_x_m = imu_pred_x_m;
    pose->imu_y_m = imu_pred_y_m;
    pose->imu_vx_mps = imu_pred_vx_mps;
    pose->imu_vy_mps = imu_pred_vy_mps;

    feedback_gain = cfg->encoder_feedback_gain * dt;
    if (feedback_gain < 0.0f) feedback_gain = 0.0f;
    if (feedback_gain > 0.20f) feedback_gain = 0.20f;
    max_feedback_step = cfg->encoder_feedback_max_mps * dt;
    if (max_feedback_step < 0.0f) max_feedback_step = 0.0f;

    corr_x = (imu_pred_x_m - pose->encoder_x_m) * feedback_gain;
    corr_y = (imu_pred_y_m - pose->encoder_y_m) * feedback_gain;
    corr_x = odom_clampf(corr_x, -max_feedback_step, max_feedback_step);
    corr_y = odom_clampf(corr_y, -max_feedback_step, max_feedback_step);
    pose->encoder_x_m += corr_x;
    pose->encoder_y_m += corr_y;
    enc_vx_world += (imu_pred_vx_mps - enc_vx_world) * feedback_gain;
    enc_vy_world += (imu_pred_vy_mps - enc_vy_world) * feedback_gain;

    odom_kf_update_pos(odom->state_x, odom->cov_x, pose->encoder_x_m, cfg->meas_noise_pos);
    odom_kf_update_pos(odom->state_y, odom->cov_y, pose->encoder_y_m, cfg->meas_noise_pos);
    odom_kf_update_vel(odom->state_x, odom->cov_x, enc_vx_world, cfg->meas_noise_vel);
    odom_kf_update_vel(odom->state_y, odom->cov_y, enc_vy_world, cfg->meas_noise_vel);

    pose->x_m = odom->state_x[0];
    pose->y_m = odom->state_y[0];
    if (fabsf(enc_vx_world) < ODOM_STATIONARY_VEL_MPS &&
        fabsf(enc_vy_world) < ODOM_STATIONARY_VEL_MPS &&
        world_ax_mps2 == 0.0f && world_ay_mps2 == 0.0f)
    {
        odom->state_x[1] = 0.0f;
        odom->state_y[1] = 0.0f;
    }
    pose->vx_mps = odom->state_x[1];
    pose->vy_mps = odom->state_y[1];
    pose->ax_mps2 = world_ax_mps2;
    pose->ay_mps2 = world_ay_mps2;
    pose->update_cnt++;
}
