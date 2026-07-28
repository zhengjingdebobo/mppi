#ifndef SLIP_DETECTOR_H
#define SLIP_DETECTOR_H

#include "imu_state.h"
#include "mecanum_kinematics.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool slipping;
    float slip_score;
} SlipState_t;

typedef struct
{
    float enter_threshold_radps;
    float exit_threshold_radps;
    float enter_hold_s;
    float exit_hold_s;
    float score_filter_tau_s;
} SlipDetectorParam_t;

void SlipDetector_Init(const SlipDetectorParam_t *param);
void SlipDetector_Reset(void);

uint8_t SlipDetector_Update(const WheelVelocity_t *wheel_velocity,
                            const IMUState_t *imu,
                            float dt_s);

void SlipDetector_GetState(SlipState_t *state);

#endif /* SLIP_DETECTOR_H */
