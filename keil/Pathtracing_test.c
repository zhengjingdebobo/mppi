#include <stdio.h>
typedef struct {
    float x;
    float y;
} TrajectoryPoint_t;

// S 形曲线的离散点
static const TrajectoryPoint_t pathPoints[] = {
{0.0f, 0.0f}, {0.5f, 0.1f}, {1.0f, 0.3f}, {1.5f, 0.5f},
{2.0f, 0.5f}, {2.5f, 0.3f}, {3.0f, 0.1f}, {3.5f, 0.0f}};

int main() {
    // 计算数组元素个数（正确）
    size_t current_path_len = sizeof(pathPoints) / sizeof(pathPoints[0]);
    
    // 使用 printf 输出，注意 size_t 对应的格式符 %zu
    printf("current_path_len = %zu\n", current_path_len);  // 输出结果为 8
    
    return 0;
}