import math
import numpy as np
from pendulum import Pendulum
from typing import Tuple


class MPPIControllerForPendulum():
    """使用 MPPI 控制器让单摆摆起并保持竖直。

    单摆状态为 ``x = [theta, theta_dot]``，控制输入为关节力矩。
    每次调用 ``calc_control_input`` 时，控制器都会：

    1. 在上一轮控制序列附近采样 K 条候选控制序列；
    2. 使用单摆动力学预测每条序列对应的未来轨迹；
    3. 根据轨迹代价计算权重；
    4. 用加权噪声修正控制序列；
    5. 只返回并执行当前时刻的第一个控制量。

    最后一步体现了 MPC 的滚动时域思想：下一时刻重新观测状态，再重复
    上述过程。
    """
    def __init__(
            self,
            delta_t: float = 0.05,
            mass_of_pole: float = 1.0,
            length_of_pole: float = 1.0,
            max_torque_abs: float = 2.0,
            max_speed_abs: float = 8.0,
            horizon_step_T: int = 30,
            number_of_samples_K: int = 1000,
            param_exploration: float = 0.01,
            param_lambda: float = 1.0,
            param_alpha: float = 0.1,
            sigma: float = 1.0,
            stage_cost_weight: np.ndarray = np.array([1.0, 0.1]),
            terminal_cost_weight: np.ndarray = np.array([1.0, 0.1]),
    ) -> None:
        """初始化单摆 MPPI 控制器。"""
        # ---------- MPPI 参数 ----------
        self.dim_u = 1  # 控制输入维度：单摆只有一个力矩
        self.T = horizon_step_T  # 预测时域长度，即每条轨迹包含多少个未来动作
        self.K = number_of_samples_K  # 每次迭代采样的候选轨迹数量
        # 从名义控制序列附近采样时，使用纯噪声序列进行探索的比例
        self.param_exploration = param_exploration
        # soft-min 权重中的温度参数；越大时各样本权重越平均
        self.param_lambda = param_lambda
        # 控制代价修正系数中的混合参数
        self.param_alpha = param_alpha
        self.param_gamma = self.param_lambda * (1.0 - self.param_alpha)
        # 噪声方差（单输入情形下 Sigma 是一个标量）
        self.Sigma = sigma
        self.stage_cost_weight = stage_cost_weight
        self.terminal_cost_weight = terminal_cost_weight

        # ---------- 单摆物理参数 ----------
        self.g = 9.81
        self.delta_t = delta_t
        self.mass_of_pole = mass_of_pole
        self.length_of_pole = length_of_pole
        self.max_torque = max_torque_abs
        self.max_speed = max_speed_abs

        # ---------- MPPI 内部变量 ----------
        # 上一轮得到的控制序列，同时也是下一轮采样的名义控制序列。
        # 形状为 (T,)，并在每个控制周期结束时向左滚动一格。
        self.u_prev = np.zeros((self.T))

    def calc_control_input(self, observed_x: np.ndarray) -> Tuple[float, np.ndarray]:
        """根据当前观测状态计算控制输入。

        参数：
            observed_x: 当前单摆状态 ``[theta, theta_dot]``。

        返回：
            当前时刻实际执行的力矩，以及完整的预测控制序列。
        """
        # 取出上一轮控制序列作为本轮的名义控制序列。
        # 这里不复制数组，是为了直接更新 self.u_prev，保持原有实现逻辑。
        u = self.u_prev

        # 每条候选轨迹都从当前观测状态开始预测。
        x0 = observed_x

        # S[k] 保存第 k 条候选轨迹的累计代价，形状为 (K,)。
        S = np.zeros((self.K))

        # 采样控制噪声，形状为 (K, T)：
        # epsilon[k, t] 表示第 k 条轨迹在第 t 个时刻的扰动。
        epsilon = self._calc_epsilon(self.Sigma, self.K, self.T)

        # ---------- 逐条滚动预测候选轨迹 ----------
        for k in range(self.K):
            # v 是第 k 条带噪声的候选控制序列，形状为 (T,)。
            v = np.zeros((self.T))

            # 第 k 条轨迹从当前状态 x0 开始。
            x = x0

            # 依次预测未来 T 个时间步。
            for t in range(1, self.T+1):

                # 大多数样本围绕名义控制序列探索；少部分样本使用纯噪声，
                # 避免所有候选轨迹过于相似。
                if k < (1.0-self.param_exploration)*self.K:
                    v[t-1] = u[t-1] + epsilon[k, t-1]
                else:
                    v[t-1] = epsilon[k, t-1]

                # 使用模型预测下一状态，并对候选力矩施加物理限制。
                x = self._F(x, self._g(v[t-1]))

                # 累加阶段代价和 MPPI 中的控制修正项。
                S[k] += self._c(x) + self.param_gamma * u[t-1] * (1.0/self.Sigma) * v[t-1]

            # 预测结束后再加入终端代价，强调轨迹末端的状态质量。
            S[k] += self._phi(x)

        # ---------- 根据代价计算样本权重 ----------
        # 代价越小，权重越大，所有权重之和为 1。
        w = self._compute_weights(S)

        # 对各条轨迹的噪声进行加权平均，得到控制序列的修正量。
        w_epsilon = np.zeros((self.T))
        for t in range(0, self.T):
            for k in range(self.K):
                w_epsilon[t] += w[k] * epsilon[k, t]

        # 对修正量做滑动平均，减少随机采样带来的高频抖动。
        w_epsilon = self._moving_average_filter(xx=w_epsilon, window_size=5)

        # 用加权噪声修正名义控制序列。
        u += w_epsilon

        # 滚动控制序列：本轮执行 u[0]，下一轮从原来的 u[1] 开始 warm start。
        self.u_prev[:-1] = u[1:]
        self.u_prev[-1] = u[-1]

        # MPC 只执行当前时刻的第一个控制量，后续动作下个周期重新计算。
        return u[0], u 

    def _calc_epsilon(self, sigma: float, size_sample: int, size_time_step: int) -> np.ndarray:
        """采样高斯控制噪声，返回形状为 ``(K, T)`` 的数组。"""
        # 当前实现使用控制器自身的 K、T；形参保留是为了与通用接口一致。
        epsilon = np.random.normal(0.0, sigma, (self.K, self.T)) # size is self.K x self.T
        return epsilon

    def _g(self, v: np.ndarray) -> float:
        """限制候选力矩，保证其满足执行器约束。"""
        v = np.clip(v, -self.max_torque, self.max_torque)
        return v

    def _c(self, x_t: np.ndarray) -> float:
        """计算单个预测状态的阶段代价。"""
        # 状态顺序为 [角度 theta, 角速度 theta_dot]。
        theta, theta_dot = x_t[0], x_t[1]
        # 将角度归一化到 [-pi, pi]，避免角度不断累积导致代价失真。
        theta = ((theta + np.pi) % (2 * np.pi)) - np.pi # normalize theta to [-pi, pi]

        # 目标是 theta=0、theta_dot=0，即保持单摆竖直且静止。
        stage_cost = self.stage_cost_weight[0]*theta**2 + self.stage_cost_weight[1]*theta_dot**2
        return stage_cost

    def _phi(self, x_T: np.ndarray) -> float:
        """计算预测终点的终端代价。"""
        theta, theta_dot = x_T[0], x_T[1]
        theta = ((theta + np.pi) % (2 * np.pi)) - np.pi # normalize theta to [-pi, pi]

        # 终端代价通常可以设置得比阶段代价更大，鼓励轨迹在预测末端收敛。
        terminal_cost = self.terminal_cost_weight[0]*theta**2 + self.terminal_cost_weight[1]*theta_dot**2
        return terminal_cost

    def _F(self, x_t: np.ndarray, v_t: np.ndarray) -> np.ndarray:
        """根据离散化动力学计算单摆下一时刻状态。"""
        # 当前状态：角度 theta、角速度 theta_dot。
        theta, theta_dot = x_t[0], x_t[1]

        # 读取物理参数和离散仿真步长。
        g = self.g
        m = self.mass_of_pole
        l = self.length_of_pole
        dt = self.delta_t

        # 先根据角加速度更新角速度，再用新的角速度更新角度。
        torque = v_t
        new_theta_dot = theta_dot + (3 * g / (2 * l) * np.sin(theta) + 3.0 / (m * l**2) * torque) * dt
        new_theta_dot = np.clip(new_theta_dot, -self.max_speed, self.max_speed)
        new_theta = theta + new_theta_dot * dt

        # 返回 x_(t+1) = [theta_(t+1), theta_dot_(t+1)]。
        x_t_plus_1 = np.array([new_theta, new_theta_dot])
        return x_t_plus_1

    def _compute_weights(self, S: np.ndarray) -> np.ndarray:
        """根据候选轨迹代价计算归一化权重。"""
        w = np.zeros((self.K))

        # rho 用于数值稳定性：避免直接计算很小的指数导致下溢。
        rho = S.min()

        # eta 是所有未归一化权重之和。
        eta = 0.0
        for k in range(self.K):
            eta += np.exp( (-1.0/self.param_lambda) * (S[k]-rho) )

        # MPPI 使用代价的 soft-min：低代价样本的指数权重更大。
        for k in range(self.K):
            w[k] = (1.0 / eta) * np.exp( (-1.0/self.param_lambda) * (S[k]-rho) )
        return w

    def _moving_average_filter(self, xx: np.ndarray, window_size: int) -> np.ndarray:
        """对控制修正量做滑动平均滤波，减少输入抖动。

        原始 MPPI 论文使用 Savitzky-Golay 滤波器；本项目为了简单，
        使用移动平均滤波器。
        """
        b = np.ones(window_size)/window_size
        xx_mean = np.convolve(xx, b, mode="same")
        n_conv = math.ceil(window_size/2)
        xx_mean[0] *= window_size/n_conv
        for i in range(1, n_conv):
            xx_mean[i] *= window_size/(i+n_conv)
            xx_mean[-i] *= window_size/(i + n_conv - (window_size % 2)) 
        return xx_mean


def run_simulation_mppi_pendulum() -> None:
    """运行单摆摆起仿真并保存动画。"""
    print("[INFO] Start simulation of swinging up a pendulum with MPPI controller")

    # ---------- 仿真设置 ----------
    delta_t = 0.05 # 仿真步长 [s]
    sim_steps = 150 # 仿真总步数
    print(f"[INFO] delta_t : {delta_t:.2f}[s] , sim_steps : {sim_steps}[steps], total_sim_time : {delta_t*sim_steps:.2f}[s]")

    # ---------- 创建被控对象 ----------
    pendulum = Pendulum(
        mass_of_pole = 1.0,
        length_of_pole = 1.0,
        max_torque_abs = 2.0,
        max_speed_abs = 8.0,
        delta_t = delta_t,
        visualize = True,
    )
    pendulum.reset(
        # 从倒置位置开始，目标是摆到 theta=0 并稳定下来。
        init_state = np.array([np.pi, 0.0]), # [theta(rad), theta_dot(rad/s)]
    )

    # ---------- 创建 MPPI 控制器 ----------
    mppi = MPPIControllerForPendulum(
        delta_t = delta_t,
        mass_of_pole = 1.0,
        length_of_pole = 1.0,
        max_torque_abs = 2.0,
        max_speed_abs = 8.0,
        horizon_step_T = 20,
        number_of_samples_K = 2000,
        param_exploration = 0.05,
        param_lambda = 0.5,
        param_alpha = 0.8,
        sigma = 1.0,
        stage_cost_weight    = np.array([1.0, 0.1]), # [theta, theta_dot] 的阶段代价权重
        terminal_cost_weight = 5.0 * np.array([1.0, 0.1]), # 终端代价权重
    )

    # ---------- 滚动时域仿真循环 ----------
    for i in range(sim_steps):

        # 读取当前真实状态。
        current_state = pendulum.get_state()

        # MPPI 预测未来，但这里只执行返回序列中的第一个力矩。
        input_torque, input_torque_sequence = mppi.calc_control_input(
            observed_x = current_state
        )

        # 输出状态和当前控制量，便于观察控制过程。
        print(f"Time: {i*delta_t:>2.2f}[s], theta={current_state[0]:>+3.3f}[rad], theta_dot={current_state[1]:>+3.3f}[rad/s], input torque={input_torque:>+3.2f}[Nm]", end="")
        print(", # currently staying upright #" if abs(current_state[0]) < 0.1 and abs(current_state[1] < 0.1) else "")

        # 将控制量施加到真实仿真环境，推进一个时间步。
        pendulum.update(u=[input_torque], delta_t=delta_t)

    # 保存动画；需要系统安装 ffmpeg。
    pendulum.save_animation("mppi_pendulum.mp4", interval=int(delta_t * 1000), movie_writer="ffmpeg") # ffmpeg is required to write mp4 file


if __name__ == "__main__":
    run_simulation_mppi_pendulum()
