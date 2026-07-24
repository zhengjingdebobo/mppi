#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include "nx16.h" 
#include <math.h> 


typedef struct {
    float x;
    float y;
} TrajectoryPoint_t;

typedef struct {
    float vx_cmd;
    float vy_cmd;
    float wz_cmd;
} ControlCmd_t;


// float PurePursuitControl(float current_x, float current_y,
// 													float current_yaw_rad, const TrajectoryPoint_t* path_points, size_t path_size);


ControlCmd_t OmniControl(float x, float y, float yaw, TrajectoryPoint_t* waypoints, size_t path_size);

static float NormalizeAngle(float angle);
static float FloatClamp(float val, float min_val, float max_val);
int32_t GetCurrentPathIndex(void);
void ResetPathIndex(void);

#endif // PATH_TRACKER_H
