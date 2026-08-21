"""使用 MPPI 控制运动学自行车模型完成目标点避障。

本文件是当前实验的“决策层”，主要负责三件事：

1. 根据车辆当前状态生成大量带随机控制扰动的候选轨迹；
2. 用目标距离、前进进度、行驶距离、控制量和障碍物安全距离评价轨迹；
3. 按照 MPPI 的信息论权重融合扰动，得到当前时刻要执行的控制量。

车辆真正的状态更新和动画绘制位于 ``pathtracking_kbm_obav.py``。
这里的 ``_F`` 是控制器内部用于预测的动力学模型，车辆类中的更新公式
是仿真环境实际执行的动力学模型；两者应保持一致。
"""

import math
import numpy as np
from typing import Tuple
from pathtracking_kbm_obav import Vehicle


def generate_random_obstacles(
        number_of_obstacles: int = 5,
        x_range: Tuple[float, float] = (5.0, 30.0),
        y_range: Tuple[float, float] = (-8.0, 8.0),
        radius_range: Tuple[float, float] = (1.0, 2.0),
        start_xy: np.ndarray = np.array([0.0, 0.0]),
        goal_xy: np.ndarray = np.array([30.0, 0.0]),
        seed: int | None = None,
    ) -> np.ndarray:
    """在起点和目标点之间的区域随机生成互不重叠的圆形障碍物。

    参数：
        number_of_obstacles: 障碍物数量。
        x_range: 障碍物中心 x 坐标的随机范围。
        y_range: 障碍物中心 y 坐标的随机范围。
        radius_range: 障碍物半径的随机范围，单位为米。
        start_xy: 起点坐标，起点附近不会放置障碍物。
        goal_xy: 目标点坐标，目标点附近不会放置障碍物。
        seed: 随机种子。传入整数可复现实验，传入 None 则每次运行都不同。

    返回：
        形状为 ``(number_of_obstacles, 3)`` 的数组，每行是
        ``[obs_x, obs_y, obs_radius]``。
    """
    # 使用独立随机数生成器，避免影响程序中其他地方的随机数状态。
    rng = np.random.default_rng(seed)
    obstacles = []
    max_attempts = 1000

    # 逐个尝试放置障碍物；每个障碍物最多尝试 max_attempts 次。
    for _ in range(number_of_obstacles):
        for _ in range(max_attempts):
            # 在给定矩形区域内随机采样障碍物圆心和半径。
            center = np.array([
                rng.uniform(*x_range),
                rng.uniform(*y_range),
            ])
            radius = rng.uniform(*radius_range)

            # 起点附近不能放障碍物，否则车辆一开始就可能被判定碰撞。
            if np.linalg.norm(center - start_xy) < radius + 5.0:
                continue
            # 目标点附近不能放障碍物，否则车辆到达目标时会同时碰撞。
            if np.linalg.norm(center - goal_xy) < radius + 5.0:
                continue

            # 障碍物之间留出安全间距，避免互相重叠成一堵墙。
            is_overlapping = any(
                np.linalg.norm(center - obstacle[:2])
                < radius + obstacle[2] + 2.0
                for obstacle in obstacles
            )
            if is_overlapping:
                continue

            obstacles.append([center[0], center[1], radius])
            break
        else:
            raise RuntimeError("Failed to generate non-overlapping obstacles.")

    return np.asarray(obstacles, dtype=float)


class MPPIControllerForPathTracking():
    """基于 MPPI 的目标点导航控制器。

    状态向量定义为 ``x = [x, y, yaw, v]``：

    - ``x, y``：车辆在全局坐标系中的位置，单位 m；
    - ``yaw``：车头朝向，单位 rad；
    - ``v``：纵向速度，单位 m/s。

    控制向量定义为 ``u = [steer, accel]``：

    - ``steer``：前轮转角，单位 rad；
    - ``accel``：纵向加速度，单位 m/s²。

    每次调用 :meth:`calc_control_input` 时，控制器都会：

    ``当前状态 -> K 条候选控制序列 -> K 条预测轨迹 -> 计算代价 ->
    MPPI 加权融合 -> 返回第一个控制量``。

    只执行返回序列的第一个控制量，然后下一时刻重新观测并优化，
    这就是“滚动时域”或“模型预测控制”的基本工作方式。
    """

    def __init__(
            self,
            delta_t: float = 0.05,
            wheel_base: float = 2.5, # 轴距 [m]
            vehicle_width: float = 3.0, # 车宽 [m]
            vehicle_length: float = 4.0, # 车长 [m]
            max_steer_abs: float = 0.523, # 最大转角 [rad]
            max_accel_abs: float = 2.000, # 最大加速度 [m/s^2]
            goal_xy: np.ndarray = np.array([30.0, 0.0]),
            horizon_step_T: int = 30,
            number_of_samples_K: int = 1000,
            param_exploration: float = 0.0,
            param_lambda: float = 50.0,
            param_alpha: float = 1.0,
            sigma: np.ndarray = np.array([[0.5, 0.0], [0.0, 0.1]]), 
            goal_distance_weight: float = 5.0,
            terminal_goal_weight: float = 100.0,
            progress_weight: float = 30.0,
            distance_weight: float = 0.1,
            time_weight: float = 1.0,
            control_weight: np.ndarray = np.array([0.01, 0.01]),
            goal_speed_weight: float = 2.0,
            unreached_goal_penalty: float = 1000.0,
            obstacle_clearance_margin: float = 2.0,
            obstacle_clearance_weight: float = 500.0,
            collision_cost_weight: float = 1.0e6,
            visualize_optimal_traj = True,  # 是否绘制名义最优预测轨迹
            visualze_sampled_trajs = False, # 是否绘制所有采样候选轨迹
            obstacle_circles: np.ndarray = np.array([[-2.0, 1.0, 1.0], [2.0, -1.0, 1.0]]), # [障碍物x, 障碍物y, 半径]
            collision_safety_margin_rate: float = 1.2, # 碰撞检测时的车辆外形膨胀系数
    ) -> None:
        """初始化 MPPI 控制器及车辆、障碍物参数。"""
        # ==================== MPPI 的核心参数 ====================
        self.dim_x = 4 # 状态向量维度
        self.dim_u = 2 # 控制向量维度
        self.T = horizon_step_T  # 每条候选轨迹向未来预测的时间步数
        self.K = number_of_samples_K  # 每次优化生成的候选轨迹数量
        self.param_exploration = param_exploration  # 纯探索样本所占比例
        self.param_lambda = param_lambda  # MPPI 温度参数，控制权重的集中程度
        self.param_alpha = param_alpha  # 控制序列平滑/记忆相关参数
        # gamma 是控制代价修正项中的系数。
        self.param_gamma = self.param_lambda * (1.0 - (self.param_alpha))
        # Sigma 是控制噪声的协方差矩阵，不是直接的标准差。
        # 本项目中控制维度为 2，因此 Sigma 必须是 2x2 矩阵。
        self.Sigma = sigma
        self.goal_xy = np.asarray(goal_xy, dtype=float)
        if self.goal_xy.shape != (2,):
            raise ValueError("goal_xy must have shape (2,).")
        self.goal_distance_weight = goal_distance_weight
        self.terminal_goal_weight = terminal_goal_weight
        self.progress_weight = progress_weight
        self.distance_weight = distance_weight
        self.time_weight = time_weight
        self.control_weight = np.asarray(control_weight, dtype=float)
        if self.control_weight.shape != (self.dim_u,):
            raise ValueError("control_weight must have shape (2,).")
        self.goal_speed_weight = goal_speed_weight
        self.unreached_goal_penalty = unreached_goal_penalty
        # 障碍物代价分为两层：
        # 1. clearance 代价：还没碰撞但进入安全范围时就开始增加；
        # 2. collision 代价：真正碰撞后施加更大的惩罚。
        self.obstacle_clearance_margin = obstacle_clearance_margin
        self.obstacle_clearance_weight = obstacle_clearance_weight
        self.collision_cost_weight = collision_cost_weight
        self.visualize_optimal_traj = visualize_optimal_traj
        self.visualze_sampled_trajs = visualze_sampled_trajs

        # ==================== 车辆模型参数 ====================
        self.delta_t = delta_t # 预测步长 [s]
        self.wheel_base = wheel_base # 轴距 [m]
        self.vehicle_w = vehicle_width # 车宽 [m]
        self.vehicle_l = vehicle_length # 车长 [m]
        self.max_steer_abs = max_steer_abs # 最大转角 [rad]
        self.max_accel_abs = max_accel_abs # 最大加速度 [m/s^2]

        # ==================== 障碍物参数 ====================
        self.obstacle_circles = obstacle_circles
        self.collision_safety_margin_rate = collision_safety_margin_rate

        # 名义控制序列。第一次为全零，之后每次调用会向左滚动一格，
        # 让上一次的优化结果成为下一次优化的初始猜测。
        self.u_prev = np.zeros((self.T, self.dim_u))

    def calc_control_input(self, observed_x: np.ndarray) -> Tuple[float, np.ndarray]:
        """根据当前观测状态计算驶向目标点的控制输入。

        参数 ``observed_x`` 是当前实际车辆状态。返回值中的第一个元素
        是当前要执行的 ``[steer, accel]``，后面的序列和轨迹主要用于
        可视化和调试。
        """
        # 取出上一轮得到的名义控制序列，作为本轮采样中心。
        u = self.u_prev

        # 预测从当前观测状态开始，而不是从上一轮预测状态开始。
        x0 = observed_x.copy()

        # S[k] 保存第 k 条候选轨迹的总代价。
        S = np.zeros((self.K))

        # 采样控制噪声 epsilon，形状为 [K, T, dim_u]。
        # epsilon[k, t] 就是第 k 条轨迹在第 t 步加入的控制扰动。
        epsilon = self._calc_epsilon(self.Sigma, self.K, self.T, self.dim_u)

        # v[k, t] 是第 k 条候选轨迹在第 t 步实际用于预测的控制量。
        v = np.zeros((self.K, self.T, self.dim_u))

        # ==================== 遍历 K 条候选轨迹 ====================
        for k in range(self.K):

            # 每一条候选轨迹都从同一个当前状态 x0 出发。
            x = x0.copy()
            travel_distance = 0.0
            arrival_step = self.T + 1
            arrival_state = None
            previous_goal_distance = np.linalg.norm(x[:2] - self.goal_xy)

            # ==================== 向前预测 T 步 ====================
            for t in range(1, self.T+1):

                # exploitation：在上一轮名义控制附近采样；
                # exploration：完全用噪声采样，帮助跳出局部最优。
                if k < (1.0-self.param_exploration)*self.K:
                    v[k, t-1] = u[t-1] + epsilon[k, t-1] # 利用：围绕名义控制采样
                else:
                    v[k, t-1] = epsilon[k, t-1] # 探索：完全由噪声生成控制

                # 使用运动学自行车模型预测下一状态，并累计该轨迹的行驶距离。
                x_next = self._F(x, self._g(v[k, t-1]))
                step_distance = np.linalg.norm(x_next[:2] - x[:2])
                travel_distance += step_distance
                current_goal_distance = np.linalg.norm(x_next[:2] - self.goal_xy)
                progress = previous_goal_distance - current_goal_distance

                # 阶段代价评价“这一步走得好不好”。
                S[k] += self._c(x_next, v[k, t-1], step_distance, progress)
                S[k] += self.param_gamma * u[t-1].T @ np.linalg.inv(self.Sigma) @ v[k, t-1]

                x = x_next
                previous_goal_distance = current_goal_distance
                if arrival_step == self.T + 1 and self._is_goal_reached(x):
                    arrival_step = t
                    arrival_state = x.copy()

            # 优先使用第一次到达目标时的状态计算终端代价，
            # 避免候选轨迹到达目标后又驶离目标而被误判。
            terminal_state = x if arrival_state is None else arrival_state
            S[k] += self._phi(terminal_state, travel_distance, arrival_step)

        # ==================== 根据代价计算 MPPI 权重 ====================
        # 代价越低，权重越大；所有权重之和为 1。
        w = self._compute_weights(S)

        # 对每个时间步的噪声进行加权平均，得到控制序列修正量。
        w_epsilon = np.zeros((self.T, self.dim_u))
        for t in range(0, self.T): # 遍历预测时间步 t = 0 ~ T-1
            for k in range(self.K):
                w_epsilon[t] += w[k] * epsilon[k, t]

        # 对修正量做滑动平均，降低随机采样导致的控制抖动。
        w_epsilon = self._moving_average_filter(xx=w_epsilon, window_size=10)

        # MPPI 更新：新控制序列 = 名义控制序列 + 加权噪声修正量。
        u += w_epsilon

        # 用更新后的名义控制序列再次预测，作为“当前最优预测轨迹”。
        optimal_traj = np.zeros((self.T, self.dim_x))
        if self.visualize_optimal_traj:
            x = x0
            for t in range(0, self.T): # 遍历预测时间步 t = 0 ~ T-1
                x = self._F(x, self._g(u[t]))
                optimal_traj[t] = x

        # 如果打开采样轨迹可视化，则保存所有候选轨迹。
        sampled_traj_list = np.zeros((self.K, self.T, self.dim_x))
        sorted_idx = np.argsort(S) # 按总代价排序，索引 0 对应最低代价样本
        if self.visualze_sampled_trajs:
            for k in sorted_idx:
                x = x0
                for t in range(0, self.T): # 遍历预测时间步 t = 0 ~ T-1
                    x = self._F(x, self._g(v[k, t]))
                    sampled_traj_list[k, t] = x

        # 滚动时域：丢弃已经执行的第 0 步，其余控制左移一格。
        # 最后一步沿用原来的末端控制，保证数组长度仍为 T。
        self.u_prev[:-1] = u[1:]
        self.u_prev[-1] = u[-1]

        # 只执行 u[0]；其余控制量留给下一轮重新优化。
        return u[0], u, optimal_traj, sampled_traj_list

    def _calc_epsilon(self, sigma: np.ndarray, size_sample: int, size_time_step: int, size_dim_u: int) -> np.ndarray:
        """从零均值高斯分布采样控制噪声。"""
        # Sigma 必须是 dim_u x dim_u 的方阵。
        if sigma.shape[0] != sigma.shape[1] or sigma.shape[0] != size_dim_u or size_dim_u < 1:
            print("[ERROR] sigma must be a square matrix with the size of size_dim_u.")
            raise ValueError

        # 均值为零，Sigma 决定各控制通道的噪声方差和相关性。
        mu = np.zeros((size_dim_u))
        epsilon = np.random.multivariate_normal(mu, sigma, (size_sample, size_time_step))
        return epsilon

    def _g(self, v: np.ndarray) -> float:
        """将候选控制量限制在车辆执行器允许的范围内。"""
        v[0] = np.clip(v[0], -self.max_steer_abs, self.max_steer_abs) # 限制转向输入
        v[1] = np.clip(v[1], -self.max_accel_abs, self.max_accel_abs) # 限制加速度输入
        return v

    def _c(
            self,
            x_t: np.ndarray,
            u_t: np.ndarray,
            step_distance: float,
            progress: float,
        ) -> float:
        """计算单个预测时间步的代价。

        这里不只惩罚“离目标还有多远”，还奖励本时间步的目标方向进度。
        这样车辆为了绕开障碍物暂时偏离目标直线时，不会立即被目标距离
        项压回障碍物方向。
        """
        # 目标距离：保留一个较温和的全局目标引导。
        # 只使用平方距离会强烈偏好直线，因此它不能设置得过大。
        goal_distance = np.linalg.norm(x_t[:2] - self.goal_xy)
        stage_cost = self.goal_distance_weight * goal_distance**2

        # 进度奖励：progress > 0 表示本步更接近目标，因此降低代价；
        # progress < 0 表示暂时远离目标，但绕障碍物时这是允许的。
        stage_cost -= self.progress_weight * progress

        # 行驶距离：在可行路线中偏好更短的路线。
        stage_cost += self.distance_weight * step_distance

        # 控制代价：避免长期使用过大的转向和加速度。
        stage_cost += np.sum(self.control_weight * u_t**2)

        # 障碍物代价具有连续的安全距离梯度：
        # 车辆越靠近障碍物，代价越大；真正碰撞时再加硬惩罚。
        stage_cost += self._obstacle_cost(x_t)
        return stage_cost

    def _phi(
            self,
            x_T: np.ndarray,
            travel_distance: float,
            arrival_step: int,
        ) -> float:
        """计算目标点导航的终端代价。"""
        goal_distance = np.linalg.norm(x_T[:2] - self.goal_xy)
        terminal_cost = self.terminal_goal_weight * goal_distance**2
        terminal_cost += self.goal_speed_weight * x_T[3]**2

        # 候选轨迹越早到达目标，时间代价越小。
        if arrival_step <= self.T:
            terminal_cost += self.time_weight * arrival_step * self.delta_t
        else:
            # 没有在预测窗口内到达目标的轨迹增加额外惩罚。
            terminal_cost += self.time_weight * self.T * self.delta_t
            terminal_cost += self.unreached_goal_penalty

        # 当前版本暂未直接使用 travel_distance 的终端项；
        # 保留该参数是为了后续扩展更复杂的路径长度代价。
        _ = travel_distance
        terminal_cost += self._obstacle_cost(x_T)
        return terminal_cost

    def _is_goal_reached(self, x_t: np.ndarray, tolerance: float = 1.0) -> bool:
        """判断车辆位置是否进入目标区域。"""
        return np.linalg.norm(x_t[:2] - self.goal_xy) <= tolerance

    def _obstacle_cost(self, x_t: np.ndarray) -> float:
        """计算连续安全距离代价和碰撞硬惩罚。"""
        clearance = self._min_obstacle_clearance(x_t)
        obstacle_cost = 0.0

        # clearance 小于安全余量时，使用平方惩罚形成平滑的排斥趋势。
        if clearance < self.obstacle_clearance_margin:
            obstacle_cost += self.obstacle_clearance_weight * (
                self.obstacle_clearance_margin - clearance
            ) ** 2

        # clearance <= 0 表示车辆外形与障碍物圆发生重叠。
        if clearance <= 0.0:
            obstacle_cost += self.collision_cost_weight

        return obstacle_cost

    def _vehicle_polygon(self, x_t: np.ndarray) -> np.ndarray:
        """返回当前车辆外形在全局坐标系下的多边形顶点。

        车辆不是一个质点，因此碰撞检测不能只比较车辆中心与圆心的距离。
        这里用带有安全膨胀系数的多边形近似车辆外形。
        """
        vw = self.vehicle_w * self.collision_safety_margin_rate
        vl = self.vehicle_l * self.collision_safety_margin_rate
        x, y, yaw, _ = x_t

        vehicle_shape_x = [
            -0.5 * vl, -0.5 * vl, 0.0, 0.5 * vl,
            0.5 * vl, 0.5 * vl, 0.0, -0.5 * vl, -0.5 * vl,
        ]
        vehicle_shape_y = [
            0.0, 0.5 * vw, 0.5 * vw, 0.5 * vw,
            0.0, -0.5 * vw, -0.5 * vw, -0.5 * vw, 0.0,
        ]
        polygon_x, polygon_y = self._affine_transform(
            vehicle_shape_x,
            vehicle_shape_y,
            yaw,
            [x, y],
        )
        return np.column_stack([polygon_x, polygon_y])

    @staticmethod
    def _point_to_segment_distance(
            point: np.ndarray,
            segment_start: np.ndarray,
            segment_end: np.ndarray,
        ) -> float:
        """计算二维点到线段的最短距离。

        将点投影到线段所在直线，再把投影比例限制在 [0, 1]，
        就可以得到点到有限线段的最近点。
        """
        segment = segment_end - segment_start
        segment_length_squared = np.dot(segment, segment)
        if segment_length_squared < 1.0e-12:
            return np.linalg.norm(point - segment_start)

        projection = np.dot(point - segment_start, segment) / segment_length_squared
        projection = np.clip(projection, 0.0, 1.0)
        closest_point = segment_start + projection * segment
        return np.linalg.norm(point - closest_point)

    @staticmethod
    def _point_inside_polygon(point: np.ndarray, polygon: np.ndarray) -> bool:
        """使用射线法判断点是否位于多边形内部。"""
        inside = False
        x, y = point
        for i in range(len(polygon) - 1):
            x1, y1 = polygon[i]
            x2, y2 = polygon[i + 1]
            crosses_horizontal_ray = (y1 > y) != (y2 > y)
            if crosses_horizontal_ray:
                intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
                if x < intersection_x:
                    inside = not inside
        return inside

    def _min_obstacle_clearance(self, x_t: np.ndarray) -> float:
        """计算车辆外形到所有障碍物的最小净空距离。

        净空距离定义为：

            车辆边界到障碍物圆心的距离 - 障碍物半径

        小于 0 表示发生重叠，小于安全余量表示需要提前避让。

        这里采用“车辆多边形边界到障碍物圆心的最短距离减去半径”
        作为近似净空。若圆心落在车辆多边形内部，则净空取负值。
        """
        if len(self.obstacle_circles) == 0:
            return np.inf

        polygon = self._vehicle_polygon(x_t)
        min_clearance = np.inf

        for obs_x, obs_y, obs_radius in self.obstacle_circles:
            obstacle_center = np.array([obs_x, obs_y])
            boundary_distance = min(
                self._point_to_segment_distance(
                    obstacle_center,
                    polygon[i],
                    polygon[i + 1],
                )
                for i in range(len(polygon) - 1)
            )

            # 障碍物圆心若在车辆多边形内部，使用负的有符号距离。
            if self._point_inside_polygon(obstacle_center, polygon):
                signed_distance = -boundary_distance
            else:
                signed_distance = boundary_distance

            clearance = signed_distance - obs_radius
            min_clearance = min(min_clearance, clearance)

        return min_clearance

    def _is_collided(self,  x_t: np.ndarray) -> bool:
        """根据净空距离判断车辆是否与障碍物发生碰撞。"""
        return self._min_obstacle_clearance(x_t) <= 0.0

    # 将图形旋转并平移到 x-y 平面中的目标位置。
    def _affine_transform(self, xlist: list, ylist: list, angle: float, translation: list=[0.0, 0.0]) -> Tuple[list, list]:
        """将局部坐标点旋转 ``angle`` 并平移到全局坐标系。"""
        transformed_x = []
        transformed_y = []
        if len(xlist) != len(ylist):
            print("[ERROR] xlist and ylist must have the same size.")
            raise AttributeError

        for i, xval in enumerate(xlist):
            transformed_x.append((xlist[i])*np.cos(angle)-(ylist[i])*np.sin(angle)+translation[0])
            transformed_y.append((xlist[i])*np.sin(angle)+(ylist[i])*np.cos(angle)+translation[1])
        transformed_x.append(transformed_x[0])
        transformed_y.append(transformed_y[0])
        return transformed_x, transformed_y

    def _F(self, x_t: np.ndarray, v_t: np.ndarray) -> np.ndarray:
        """用运动学自行车模型预测一个时间步后的状态。

        离散模型为：

        ``x_next = x + v*cos(yaw)*dt``

        ``y_next = y + v*sin(yaw)*dt``

        ``yaw_next = yaw + v/wheel_base*tan(steer)*dt``

        ``v_next = v + accel*dt``

        这里的 ``_F`` 必须和 ``Vehicle.update`` 中的实际仿真模型一致，
        否则控制器预测的轨迹会与真实车辆运动逐渐偏离。
        """
        # 取出当前状态和候选控制量。
        x, y, yaw, v = x_t
        steer, accel = v_t

        # 读取车辆轴距和控制器预测步长。
        l = self.wheel_base
        dt = self.delta_t

        # 根据离散运动学自行车模型推进一个时间步。
        new_x = x + v * np.cos(yaw) * dt
        new_y = y + v * np.sin(yaw) * dt
        new_yaw = yaw + v / l * np.tan(steer) * dt
        new_v = v + accel * dt

        # 返回 [x, y, yaw, v]。
        x_t_plus_1 = np.array([new_x, new_y, new_yaw, new_v])
        return x_t_plus_1

    def _compute_weights(self, S: np.ndarray) -> np.ndarray:
        """根据候选轨迹代价计算归一化权重。"""
        w = np.zeros((self.K))

        # 减去最小代价 rho，避免指数计算下溢，同时不改变权重比例。
        rho = S.min()

        # eta 是归一化因子。
        eta = 0.0
        for k in range(self.K):
            eta += np.exp( (-1.0/self.param_lambda) * (S[k]-rho) )

        # 代价越小，exp(-代价/lambda) 越大，获得的权重越大。
        for k in range(self.K):
            w[k] = (1.0 / eta) * np.exp( (-1.0/self.param_lambda) * (S[k]-rho) )
        return w

    def _moving_average_filter(self, xx: np.ndarray, window_size: int) -> np.ndarray:
        """用滑动平均平滑控制序列。

        这里使用的是简单移动平均；原始 MPPI 论文还讨论过
        Savitzky-Golay 滤波。注意 ``window_size`` 不能大于预测时域 ``T``。
        """
        b = np.ones(window_size)/window_size
        dim = xx.shape[1]
        xx_mean = np.zeros(xx.shape)

        for d in range(dim):
            xx_mean[:,d] = np.convolve(xx[:,d], b, mode="same")
            n_conv = math.ceil(window_size/2)
            xx_mean[0,d] *= window_size/n_conv
            for i in range(1, n_conv):
                xx_mean[i,d] *= window_size/(i+n_conv)
                xx_mean[-i,d] *= window_size/(i + n_conv - (window_size % 2)) 
        return xx_mean


def run_simulation_mppi_pathtracking() -> None:
    """运行从起点到目标点的 MPPI 避障仿真。

    注意：函数名保留了旧版的 ``pathtracking`` 名称，方便兼容原工程；
    当前逻辑已经不是跟踪 CSV 参考路径，而是从 ``start_state`` 导航到
    ``goal_xy``，同时避开随机生成的圆形障碍物。
    """
    print("[INFO] Start goal navigation with MPPI controller")

    # ==================== 仿真总设置 ====================
    delta_t = 0.05 # 仿真步长 [s]
    sim_steps = 300 # 最大仿真步数
    print(f"[INFO] delta_t : {delta_t:.2f}[s] , sim_steps : {sim_steps}[steps], total_sim_time : {delta_t*sim_steps:.2f}[s]")

    # ---------- 起点和目标点 ----------
    start_state = np.array([0.0, 0.0, 0.0, 0.0]) # [x[m], y[m], yaw[rad], v[m/s]]
    goal_xy = np.array([30.0, 0.0]) # 目标位置 [x[m], y[m]]

    # ---------- 随机障碍物参数 ----------
    # seed=None：每次运行生成不同障碍物；改成整数（例如 0）可复现实验。
    RANDOM_SEED = None
    OBSTACLE_CIRCLES = generate_random_obstacles(
        x_range=(5.0, 28.0),
        y_range=(-8.0, 8.0),
        radius_range=(1.0, 2.0),
        start_xy=start_state[:2],
        goal_xy=goal_xy,
        seed=RANDOM_SEED,
    )
    print(f"[INFO] Random obstacle circles:\n{OBSTACLE_CIRCLES}")

    # ==================== 创建实际仿真车辆 ====================
    # Vehicle.update() 会真正推进状态，并负责生成动画帧。
    vehicle = Vehicle(
        wheel_base=2.5,
        max_steer_abs=0.523, # 最大转角 [rad]
        max_accel_abs=2.000, # 最大加速度 [m/s^2]
        ref_path = np.empty((0, 2)), # 目标点导航不使用参考路径
        goal_xy = goal_xy,
        obstacle_circles = OBSTACLE_CIRCLES, # [obs_x, obs_y, obs_radius]
    )
    vehicle.reset(
        init_state = start_state,
    )

    # ==================== 创建 MPPI 控制器 ====================
    # 下面这些参数是最值得做实验的部分：
    # K 越大，候选轨迹越多但 CPU 越慢；T 越大，单条轨迹看得越远但计算越慢；
    # Sigma 越大，探索范围越大，但控制可能更抖；lambda 越大，权重越平均。
    mppi = MPPIControllerForPathTracking(
        delta_t = delta_t*2.0, # 控制器预测步长 [s]
        wheel_base = 2.5, # 轴距 [m]
        max_steer_abs = 0.523, # 最大转角 [rad]
        max_accel_abs = 2.000, # 最大加速度 [m/s^2]
        goal_xy = goal_xy,
        # 预测时域扩大到 70 步，给车辆更多时间看到并绕过障碍物。
        horizon_step_T = 70, # 预测步数
        # 增加样本数，提高找到安全绕行轨迹的概率。
        number_of_samples_K = 800, # 候选轨迹数量
        # 增大纯探索样本比例，避免所有候选轨迹都沿着当前名义控制前进。
        param_exploration = 0.15,
        param_lambda = 100.0,
        param_alpha = 0.98,
        # Sigma 是控制噪声协方差矩阵，不是标准差：
        # 转向噪声标准差约为 sqrt(0.15)，加速度噪声标准差约为 sqrt(3.0)。
        sigma = np.array([[0.15, 0.0], [0.0, 3.0]]),
        # 降低逐步目标距离代价，允许车辆为了绕障碍物暂时远离目标。
        goal_distance_weight = 0.5,
        terminal_goal_weight = 100.0,
        # 增加实际行驶距离的影响，避免绕行路线过长。
        distance_weight = 1.0,
        time_weight = 2.0,
        # 进度奖励允许车辆在整体上持续接近目标，同时保留绕行自由度。
        progress_weight = 30.0,
        control_weight = np.array([0.01, 0.01]),
        goal_speed_weight = 2.0,
        unreached_goal_penalty = 1000.0,
        # 车辆进入障碍物附近的安全范围时就开始受到惩罚。
        obstacle_clearance_margin = 2.0,
        obstacle_clearance_weight = 500.0,
        # 碰撞仍然是最严重的情况，但不再使用过大的 1e10 造成权重完全饱和。
        collision_cost_weight = 1.0e6,
        visualze_sampled_trajs = True, # 绘制候选轨迹
        obstacle_circles = OBSTACLE_CIRCLES, # [障碍物x, 障碍物y, 半径]
        collision_safety_margin_rate = 1.2, # 碰撞检测时的车辆外形膨胀系数
    )

    # 连续满足目标条件若干步后才结束，避免车辆高速擦过目标点就提前停止。
    goal_stable_count = 0
    goal_stable_required = 5

    # ==================== 滚动时域仿真循环 ====================
    # 每一次循环只执行当前最优控制序列的第一个控制量，随后重新观测、重算。
    for i in range(sim_steps):

        # 读取真实仿真车辆的当前状态。
        current_state = vehicle.get_state()

        # MPPI 在内部预测 K*T 个状态转移，并返回当前时刻控制量。
        optimal_input, optimal_input_sequence, optimal_traj, sampled_traj_list = mppi.calc_control_input(
            observed_x = current_state
        )

        goal_distance = np.linalg.norm(current_state[:2] - goal_xy)
        print(f"Time: {i*delta_t:>2.2f}[s], x={current_state[0]:>+3.3f}[m], y={current_state[1]:>+3.3f}[m], goal_dist={goal_distance:>3.3f}[m], yaw={current_state[2]:>+3.3f}[rad], v={current_state[3]:>+3.3f}[m/s], steer={optimal_input[0]:>+6.2f}[rad], accel={optimal_input[1]:>+6.2f}[m/s]")

        # 将 MPPI 的第一个控制量交给实际车辆模型推进一个 delta_t。
        vehicle.update(u=optimal_input, delta_t=delta_t, optimal_traj=optimal_traj[:, 0:2], sampled_traj_list=sampled_traj_list[:, :, 0:2])

        next_state = vehicle.get_state()
        next_goal_distance = np.linalg.norm(next_state[:2] - goal_xy)
        # 停止条件：车辆进入目标半径 1m 内且速度足够小，并连续满足 5 帧。
        if next_goal_distance <= 1.0 and abs(next_state[3]) <= 0.3:
            goal_stable_count += 1
        else:
            goal_stable_count = 0

        if goal_stable_count >= goal_stable_required:
            print("[INFO] Goal reached and vehicle stopped.")
            break

    # 保存动画；需要系统中可用 ffmpeg。
    vehicle.save_animation("mppi_goal_navigation_obav_demo.mp4", interval=int(delta_t * 1000), movie_writer="ffmpeg") # 保存 mp4 需要 ffmpeg

if __name__ == "__main__":
    run_simulation_mppi_pathtracking()
