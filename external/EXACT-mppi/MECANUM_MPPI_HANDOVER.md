# EXACT-MPPI 麦克纳姆轮仿真交接说明

## 1. 功能概述

本项目使用 EXACT-MPPI 控制三自由度麦克纳姆轮机器人在二维环境中导航。算法每个控制周期根据机器人当前状态、目标点和局部障碍物感知，输出底盘速度指令 `[vx, vy, wz]`。当前导航只要求到达目标二维位置，不要求最终航向角对齐。

机器人 footprint 为 `0.5 m × 0.5 m` 的正方形，四角采用半径 `0.1 m` 的圆弧。

## 2. 代码接口位置

直接调用接口位于 `EXACT_MPPI_core/exact_mppi/mppi_jax/controller.py`：

```python
controller.computeVelocityCommands(
    robot_pose, robot_speed, plan, goal, lidar_points
)
```

当前仿真调用位于 `EXACT_MPPI_core/example/mecanum_robot/mecanum_simulation.py` 的 `run_simulation()`。运动学模型位于 `mecanum_model.py`。

## 3. 算法输入

### 3.1 当前机器人位姿

```python
robot_pose = np.array([x, y, theta], dtype=np.float32)
```

`x、y` 为世界坐标系位置，单位 m；`theta` 为相对世界坐标系的航向角，单位 rad。

### 3.2 当前机器人速度

```python
robot_speed = np.array([vx, vy, wz], dtype=np.float32)
```

`vx`、`vy` 为机器人自身坐标系的前向/横向速度（m/s），`wz` 为绕 z 轴角速度（rad/s）。

### 3.3 目标点

```python
goal = np.array([x_goal, y_goal, theta_goal], dtype=np.float32)
```

当前实际使用 `x_goal、y_goal`。`theta_goal` 不参与目标代价和到达判断，可固定为 `0.0`。

### 3.4 参考路径

最简单的输入是当前点和目标点：

```python
plan = np.vstack((robot_pose, goal)).astype(np.float32)
```

如果外部仿真已有全局路径，也可以传入多个 `[x, y, theta]` 路径点。

### 3.5 局部障碍物点

```python
lidar_points.shape == (N, 2)
```

当前仿真使用世界坐标系、单位 m 的障碍物点。无障碍物时使用：

```python
lidar_points = np.empty((0, 2), dtype=np.float32)
```

外部仿真若输出机器人坐标系点，应在适配层转换为项目要求的坐标系。

## 4. MPPI 输出

调用：

```python
command = controller.computeVelocityCommands(
    robot_pose=robot_pose,
    robot_speed=robot_speed,
    plan=plan,
    goal=goal,
    lidar_points=lidar_points,
)
vx_cmd, vy_cmd, wz_cmd = command
```

输出是当前控制周期的速度指令，而不是完整路径：

```text
vx_cmd：前向速度，m/s，机器人坐标系
vy_cmd：横向速度，m/s，机器人坐标系
wz_cmd：角速度，rad/s
```

注意变量名是 `wz`，不是 `vz`。

## 5. 外部仿真控制循环

```python
while not finished:
    robot_pose = simulator.get_pose()
    robot_speed = simulator.get_velocity()
    lidar_points = simulator.get_local_obstacles()

    goal = np.array([goal_x, goal_y, 0.0], dtype=np.float32)
    plan = np.vstack((robot_pose, goal)).astype(np.float32)

    command = controller.computeVelocityCommands(
        robot_pose=robot_pose,
        robot_speed=robot_speed,
        plan=plan,
        goal=goal,
        lidar_points=lidar_points,
    )
    command = model.maximum_velocity_constraint(command)
    simulator.set_velocity(command)
    simulator.step(dt)
```

当前配置 `dt=0.1`，控制频率约为 10 Hz。外部仿真执行指令并推进一个时间步后，需要返回新的位姿和速度。

## 6. 外部仿真适配器接口

```python
class MecanumSimulationInterface:
    def get_pose(self) -> np.ndarray:
        """返回 [x, y, theta]。"""

    def get_velocity(self) -> np.ndarray:
        """返回 [vx, vy, wz]。"""

    def get_local_obstacles(self) -> np.ndarray:
        """返回 shape=(N, 2) 的障碍物点。"""

    def set_velocity(self, command: np.ndarray) -> None:
        """接收 [vx, vy, wz] 速度指令。"""

    def step(self, dt: float) -> None:
        """推进仿真时间。"""
```

如果启用障碍物避障，还必须实现 `get_local_obstacles()`。

## 7. 麦克纳姆运动学模型

```text
state   = [x, y, theta]
control = [vx, vy, wz]
```

```text
x_next     = x + (vx*cos(theta) - vy*sin(theta))*dt
y_next     = y + (vx*sin(theta) + vy*cos(theta))*dt
theta_next = theta + wz*dt
```

模型接口：

```python
command = model.maximum_velocity_constraint(command)
next_state = model.state_transition(state, command)
```

## 8. 仿真轨迹输出

`run_simulation()` 返回 `SimulationResult`：

```text
states, controls, clearances, perception_counts, scan_history, status
```

`save_trajectory()` 写出的 CSV 字段为：

```text
step, time, x, y, theta, vx, vy, wz,
local_clearance, perception_count
```

其中 `vx、vy、wz` 是实际执行的控制量，`x、y、theta` 是机器人状态轨迹。

## 9. 运行示例

```bash
cd /home/bobo/workspace/mppi/external/EXACT-mppi
python EXACT_MPPI_core/example/mecanum_robot/mecanum_simulation.py \
  --seed 12 --animate
```

无窗口运行并保存结果：

```bash
MPLBACKEND=Agg \
python EXACT_MPPI_core/example/mecanum_robot/mecanum_simulation.py \
  --seed 12 \
  --trajectory-file /tmp/mecanum_trajectory.csv \
  --figure-file /tmp/mecanum_result.png
```
