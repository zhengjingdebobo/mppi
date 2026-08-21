"""运动学自行车模型车辆与可视化环境。

本文件是仿真系统的“执行层”：

1. 保存车辆状态 ``[x, y, yaw, v]``；
2. 接收 MPPI 输出的 ``[steer, accel]`` 并推进真实仿真状态；
3. 绘制车辆、障碍物、目标点、预测轨迹和控制量仪表盘；
4. 将收集到的帧保存为 mp4 动画。

控制器文件中的 ``MPPIControllerForPathTracking._F`` 会复制这里的
运动学模型，用于预测候选轨迹。因此修改车辆动力学时，两个文件必须同步。
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple
from matplotlib import patches
from matplotlib.animation import ArtistAnimation

class Vehicle():
    """使用运动学自行车模型的二维车辆仿真环境。"""

    def __init__(
            self,
            wheel_base: float = 2.5, # 轴距 [m]
            vehicle_width = 3.0, # 车宽 [m]
            vehicle_length = 4.0, # 车长 [m]
            max_steer_abs: float = 0.523, # 最大转角 [rad]
            max_accel_abs: float = 2.000, # 最大加速度 [m/s^2]
            ref_path: np.ndarray = np.array([[-100.0, 0.0], [100.0, 0.0]]),
            goal_xy: np.ndarray = np.empty(0),
            obstacle_circles: np.ndarray = np.array([[-2.0, 1.0, 1.0], [2.0, -1.0, 1.0]]), # [障碍物x, 障碍物y, 半径]
            delta_t: float = 0.05, # 仿真步长 [s]
            visualize: bool = True,
        ) -> None:
        """初始化车辆参数、地图信息和可视化窗口配置。

        ``ref_path`` 用于旧版路径跟踪实验；目标点导航实验传入空数组。
        ``goal_xy`` 如果存在，会在主视图和小地图中画出绿色目标点。
        """
        # ==================== 车辆与仿真参数 ====================
        self.wheel_base = wheel_base#[m]
        self.vehicle_w = vehicle_width
        self.vehicle_l = vehicle_length
        self.max_steer_abs = max_steer_abs # [rad]
        self.max_accel_abs = max_accel_abs # [m/s^2]
        self.delta_t = delta_t #[s]
        self.ref_path = ref_path
        self.goal_xy = np.asarray(goal_xy, dtype=float)
        if self.goal_xy.size not in (0, 2):
            raise ValueError("goal_xy must be empty or have shape (2,).")

        # 障碍物数组的每一行为 [圆心 x, 圆心 y, 半径]。
        self.obstacle_circles = obstacle_circles

        # ==================== 可视化坐标范围 ====================
        self.view_x_lim_min, self.view_x_lim_max = -20.0, 20.0
        self.view_y_lim_min, self.view_y_lim_max = -25.0, 25.0
        self.minimap_view_x_lim_min, self.minimap_view_x_lim_max = -40.0, 40.0
        self.minimap_view_y_lim_min, self.minimap_view_y_lim_max = -10.0, 40.0

        # 创建/清空状态和动画帧。
        self.visualize_flag = visualize
        self.reset()

    def reset(
            self, 
            init_state: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0]), # [x, y, yaw, v]
        ) -> None:
        """将车辆状态和动画帧重置到初始状态。"""

        # 状态顺序为 [x, y, yaw, v]。
        self.state = init_state

        # 每次 reset 都重新开始记录动画。
        self.frames = []

        if self.visualize_flag:
            # ==================== 创建四个绘图区 ====================
            # 左侧是车辆主视图，右上角是全局小地图，
            # 右下两块显示当前转向角和加速度。
            self.fig = plt.figure(figsize=(9,9))
            self.main_ax = plt.subplot2grid((3,4), (0,0), rowspan=3, colspan=3)
            self.minimap_ax = plt.subplot2grid((3,4), (0,3))
            self.steer_ax = plt.subplot2grid((3,4), (1,3))
            self.accel_ax = plt.subplot2grid((3,4), (2,3))

            # ==================== 设置坐标轴和显示样式 ====================
            ## 主视图
            self.main_ax.set_aspect('equal')
            self.main_ax.set_xlim(self.view_x_lim_min, self.view_x_lim_max)
            self.main_ax.set_ylim(self.view_y_lim_min, self.view_y_lim_max)
            self.main_ax.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False)
            self.main_ax.tick_params(bottom=False, left=False, right=False, top=False)
            ## 小地图
            self.minimap_ax.set_aspect('equal')
            self.minimap_ax.axis('off')
            self.minimap_ax.set_xlim(self.minimap_view_x_lim_min, self.minimap_view_x_lim_max)
            self.minimap_ax.set_ylim(self.minimap_view_y_lim_min, self.minimap_view_y_lim_max)
            ## 转向角显示
            self.steer_ax.set_title("Steering Angle", fontsize="12")
            self.steer_ax.axis('off')
            ## 加速度显示
            self.accel_ax.set_title("Acceleration", fontsize="12")
            self.accel_ax.axis('off')
            
            # 自动调整子图间距，避免标题或文字重叠。
            self.fig.tight_layout()

    def update(
            self, 
            u: np.ndarray, 
            delta_t: float = 0.0, 
            append_frame: bool = True, 
            optimal_traj: np.ndarray = np.empty(0), # MPPI 预测的名义最优轨迹
            sampled_traj_list: np.ndarray = np.empty(0), # MPPI 采样得到的候选轨迹
        ) -> None:
        """接收一个控制量，并将车辆推进一个仿真时间步。

        这里是真实环境执行控制的地方。MPPI 只负责预测和选择控制量，
        最终由本方法修改 ``self.state``。
        """
        # 保存当前状态，稍后使用离散动力学计算下一状态。
        x, y, yaw, v = self.state

        # delta_t 为 0 时使用车辆默认步长，否则使用调用者显式传入的步长。
        l = self.wheel_base
        dt = self.delta_t if delta_t == 0.0 else delta_t

        # 对控制器输出再次限幅，保证仿真车辆不会收到越界输入。
        steer = np.clip(u[0], -self.max_steer_abs, self.max_steer_abs)
        accel = np.clip(u[1], -self.max_accel_abs, self.max_accel_abs)

        # 运动学自行车模型的离散状态更新。
        new_x = x + v * np.cos(yaw) * dt
        new_y = y + v * np.sin(yaw) * dt
        new_yaw = yaw + v / l * np.tan(steer) * dt
        new_v = v + accel * dt
        self.state = np.array([new_x, new_y, new_yaw, new_v])

        # 更新后再记录帧，因此画面显示的是新状态。
        if append_frame:
            self.append_frame(steer, accel, optimal_traj, sampled_traj_list)

    def get_state(self) -> np.ndarray:
        """返回当前状态的副本，避免外部直接修改环境内部状态。"""
        return self.state.copy()

    def append_frame(self, steer: float, accel: float, optimal_traj: np.ndarray, sampled_traj_list: np.ndarray) -> list:
        """根据当前状态和控制量绘制并保存一帧动画。

        主视图采用“车辆固定在画面中心”的相对坐标；小地图采用全局坐标，
        便于同时观察局部细节和全局路线。
        """
        # 读取当前状态，车辆外形和目标点都基于它进行绘制。
        x, y, yaw, v = self.state

        # ==================== 主视图：车辆外形 ====================
        # 先在车辆局部坐标系画车，再旋转到 yaw 对应方向。
        vw, vl = self.vehicle_w, self.vehicle_l
        vehicle_shape_x = [-0.5*vl, -0.5*vl, +0.5*vl, +0.5*vl, -0.5*vl, -0.5*vl]
        vehicle_shape_y = [0.0, +0.5*vw, +0.5*vw, -0.5*vw, -0.5*vw, 0.0]
        rotated_vehicle_shape_x, rotated_vehicle_shape_y = \
            self._affine_transform(vehicle_shape_x, vehicle_shape_y, yaw, [0, 0]) # 让车辆位于主视图中心
        frame = self.main_ax.plot(rotated_vehicle_shape_x, rotated_vehicle_shape_y, color='black', linewidth=2.0, zorder=3)

        # 绘制四个车轮。前轮额外旋转 steer，表现转向方向。
        ww, wl = 0.4, 0.7 # 车轮宽度和长度 [m]
        wheel_shape_x = np.array([-0.5*wl, -0.5*wl, +0.5*wl, +0.5*wl, -0.5*wl, -0.5*wl])
        wheel_shape_y = np.array([0.0, +0.5*ww, +0.5*ww, -0.5*ww, -0.5*ww, 0.0])

        # 后左轮
        wheel_shape_rl_x, wheel_shape_rl_y = \
            self._affine_transform(wheel_shape_x, wheel_shape_y, 0.0, [-0.3*vl, 0.3*vw])
        wheel_rl_x, wheel_rl_y = \
            self._affine_transform(wheel_shape_rl_x, wheel_shape_rl_y, yaw, [0, 0])
        frame += self.main_ax.fill(wheel_rl_x, wheel_rl_y, color='black', zorder=3)

        # 后右轮
        wheel_shape_rr_x, wheel_shape_rr_y = \
            self._affine_transform(wheel_shape_x, wheel_shape_y, 0.0, [-0.3*vl, -0.3*vw])
        wheel_rr_x, wheel_rr_y = \
            self._affine_transform(wheel_shape_rr_x, wheel_shape_rr_y, yaw, [0, 0])
        frame += self.main_ax.fill(wheel_rr_x, wheel_rr_y, color='black', zorder=3)

        # 前左轮
        wheel_shape_fl_x, wheel_shape_fl_y = \
            self._affine_transform(wheel_shape_x, wheel_shape_y, steer, [0.3*vl, 0.3*vw])
        wheel_fl_x, wheel_fl_y = \
            self._affine_transform(wheel_shape_fl_x, wheel_shape_fl_y, yaw, [0, 0])
        frame += self.main_ax.fill(wheel_fl_x, wheel_fl_y, color='black', zorder=3)

        # 前右轮
        wheel_shape_fr_x, wheel_shape_fr_y = \
            self._affine_transform(wheel_shape_x, wheel_shape_y, steer, [0.3*vl, -0.3*vw])
        wheel_fr_x, wheel_fr_y = \
            self._affine_transform(wheel_shape_fr_x, wheel_shape_fr_y, yaw, [0, 0])
        frame += self.main_ax.fill(wheel_fr_x, wheel_fr_y, color='black', zorder=3)

        # 车辆中心小圆点用于观察车辆参考点位置。
        vehicle_center = patches.Circle([0, 0], radius=vw/20.0, fc='white', ec='black', linewidth=2.0, zorder=6)
        frame += [self.main_ax.add_artist(vehicle_center)]

        # 如果提供了参考路径就绘制参考路径；目标点导航模式下可以为空。
        if self.ref_path.size > 0:
            ref_path_x = self.ref_path[:, 0] - np.full(self.ref_path.shape[0], x)
            ref_path_y = self.ref_path[:, 1] - np.full(self.ref_path.shape[0], y)
            frame += self.main_ax.plot(ref_path_x, ref_path_y, color='black', linestyle="dashed", linewidth=1.5)

        # 绘制目标点。主视图使用相对坐标，车辆始终位于画面中心。
        if self.goal_xy.size == 2:
            goal_circle = patches.Circle(
                [self.goal_xy[0] - x, self.goal_xy[1] - y],
                radius=1.0,
                fc='lightgreen',
                ec='green',
                linewidth=2.0,
                zorder=1,
            )
            frame += [self.main_ax.add_artist(goal_circle)]

        # 显示当前速度；主视图中车辆位置固定，所以只需更新文字。
        text = "vehicle velocity = {v:>+6.1f} [m/s]".format(pos_e=x, head_e=np.rad2deg(yaw), v=v)
        frame += [self.main_ax.text(0.5, 0.02, text, ha='center', transform=self.main_ax.transAxes, fontsize=14, fontfamily='monospace')]

        # 紫色线：MPPI 用更新后名义控制序列预测的轨迹。
        if optimal_traj.any():
            optimal_traj_x_offset = np.ravel(optimal_traj[:, 0]) - np.full(optimal_traj.shape[0], x)
            optimal_traj_y_offset = np.ravel(optimal_traj[:, 1]) - np.full(optimal_traj.shape[0], y)
            frame += self.main_ax.plot(optimal_traj_x_offset, optimal_traj_y_offset, color='#990099', linestyle="solid", linewidth=1.5, zorder=5)

        # 灰色线：MPPI 采样得到的候选轨迹。
        # 这些线可以帮助观察 K 条候选轨迹是否覆盖了绕障碍物的方向。
        if sampled_traj_list.any():
            min_alpha_value = 0.25
            max_alpha_value = 0.35
            for idx, sampled_traj in enumerate(sampled_traj_list):
                # 排名越靠前的候选轨迹颜色越深，便于观察低代价样本。
                alpha_value = (1.0 - (idx+1)/len(sampled_traj_list)) * (max_alpha_value - min_alpha_value) + min_alpha_value
                sampled_traj_x_offset = np.ravel(sampled_traj[:, 0]) - np.full(sampled_traj.shape[0], x)
                sampled_traj_y_offset = np.ravel(sampled_traj[:, 1]) - np.full(sampled_traj.shape[0], y)
                frame += self.main_ax.plot(sampled_traj_x_offset, sampled_traj_y_offset, color='gray', linestyle="solid", linewidth=0.2, zorder=4, alpha=alpha_value)

        # 主视图中的障碍物使用相对坐标，因此随车辆移动而平移画面。
        for obs in self.obstacle_circles:
            obs_x, obs_y, obs_r = obs
            obs_circle = patches.Circle([obs_x-x, obs_y-y], radius=obs_r, fc='white', ec='black', linewidth=2.0, zorder=0)
            frame += [self.main_ax.add_artist(obs_circle)]

        # ==================== 小地图：全局坐标 ====================
        if self.ref_path.size > 0:
            frame += self.minimap_ax.plot(self.ref_path[:, 0], self.ref_path[:,1], color='black', linestyle='dashed')
        if self.goal_xy.size == 2:
            goal_circle_minimap = patches.Circle(
                self.goal_xy,
                radius=1.0,
                fc='lightgreen',
                ec='green',
                linewidth=2.0,
                zorder=1,
            )
            frame += [self.minimap_ax.add_artist(goal_circle_minimap)]
        rotated_vehicle_shape_x_minimap, rotated_vehicle_shape_y_minimap = \
            self._affine_transform(vehicle_shape_x, vehicle_shape_y, yaw, [x, y]) # 将车辆放到全局坐标 [x, y]
        frame += self.minimap_ax.plot(rotated_vehicle_shape_x_minimap, rotated_vehicle_shape_y_minimap, color='black', linewidth=2.0, zorder=3)
        frame += self.minimap_ax.fill(rotated_vehicle_shape_x_minimap, rotated_vehicle_shape_y_minimap, color='white', zorder=2)

        # 小地图中的障碍物和目标点使用全局坐标。
        for obs in self.obstacle_circles:
            obs_x, obs_y, obs_r = obs
            obs_circle = patches.Circle([obs_x, obs_y], radius=obs_r, fc='white', ec='black', linewidth=2.0, zorder=0)
            frame += [self.minimap_ax.add_artist(obs_circle)]

        # ==================== 控制量仪表盘 ====================
        # 用环形饼图显示转向角的绝对值和方向。
        MAX_STEER = self.max_steer_abs
        PIE_RATE = 3.0/4.0
        PIE_STARTANGLE = 225 # 仪表盘起始角度 [deg]
        s_abs = np.abs(steer)
        if steer < 0.0: # steer 为负时向右转
            steer_pie_obj, _ = self.steer_ax.pie([MAX_STEER*PIE_RATE, s_abs*PIE_RATE, (MAX_STEER-s_abs)*PIE_RATE, 2*MAX_STEER*(1-PIE_RATE)], startangle=PIE_STARTANGLE, counterclock=False, colors=["lightgray", "black", "lightgray", "white"], wedgeprops={'linewidth': 0, "edgecolor":"white", "width":0.4})
        else: # steer 为正时向左转
            steer_pie_obj, _ = self.steer_ax.pie([(MAX_STEER-s_abs)*PIE_RATE, s_abs*PIE_RATE, MAX_STEER*PIE_RATE, 2*MAX_STEER*(1-PIE_RATE)], startangle=PIE_STARTANGLE, counterclock=False, colors=["lightgray", "black", "lightgray", "white"], wedgeprops={'linewidth': 0, "edgecolor":"white", "width":0.4})
        frame += steer_pie_obj
        frame += [self.steer_ax.text(0, -1, f"{np.rad2deg(steer):+.2f} " + r"$ \rm{[deg]}$", size = 14, horizontalalignment='center', verticalalignment='center', fontfamily='monospace')]

        # 用环形饼图显示加速度的绝对值和方向。
        MAX_ACCEL = self.max_accel_abs
        PIE_RATE = 3.0/4.0
        PIE_STARTANGLE = 225 # 仪表盘起始角度 [deg]
        a_abs = np.abs(accel)
        if accel > 0.0:
            accel_pie_obj, _ = self.accel_ax.pie([MAX_ACCEL*PIE_RATE, a_abs*PIE_RATE, (MAX_ACCEL-a_abs)*PIE_RATE, 2*MAX_ACCEL*(1-PIE_RATE)], startangle=PIE_STARTANGLE, counterclock=False, colors=["lightgray", "black", "lightgray", "white"], wedgeprops={'linewidth': 0, "edgecolor":"white", "width":0.4})
        else:
            accel_pie_obj, _ = self.accel_ax.pie([(MAX_ACCEL-a_abs)*PIE_RATE, a_abs*PIE_RATE, MAX_ACCEL*PIE_RATE, 2*MAX_ACCEL*(1-PIE_RATE)], startangle=PIE_STARTANGLE, counterclock=False, colors=["lightgray", "black", "lightgray", "white"], wedgeprops={'linewidth': 0, "edgecolor":"white", "width":0.4})
        frame += accel_pie_obj
        frame += [self.accel_ax.text(0, -1, f"{accel:+.2f} " + r"$ \rm{[m/s^2]}$", size = 14, horizontalalignment='center', verticalalignment='center', fontfamily='monospace')]

        # ArtistAnimation 要求每一帧是 artist 对象列表。
        self.frames.append(frame)

    # 将图形旋转并平移到 x-y 平面中的目标位置。
    def _affine_transform(self, xlist: list, ylist: list, angle: float, translation: list=[0.0, 0.0]) -> Tuple[list, list]:
        """将局部平面图形旋转并平移到目标坐标系。"""
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

    def save_animation(self, filename: str, interval: int, movie_writer: str="ffmpeg") -> None:
        """将已经记录的帧保存为动画文件。

        ``interval`` 是相邻帧之间的毫秒数；保存 mp4 时通常需要 ffmpeg。
        """
        ani = ArtistAnimation(self.fig, self.frames, interval=interval)
        ani.save(filename, writer=movie_writer)

if __name__ == "__main__":
    # 直接运行本文件时执行一个简单的正弦控制测试。
    # 这部分不是 MPPI 实验，只用于检查车辆模型和动画绘制是否正常。
    sim_step = 100
    delta_t = 0.05
    ref_path = np.genfromtxt('./data/ovalpath.csv', delimiter=',', skip_header=1)
    vehicle = Vehicle(ref_path=ref_path[:, 0:2])
    for i in range(sim_step):
        vehicle.update(u=[0.6 * np.sin(i/3.0), 2.2 * np.sin(i/10.0)], delta_t=delta_t) # 控制量为 [转角[rad], 加速度[m/s^2]]
    # 保存车辆模型自检动画。
    vehicle.save_animation("vehicle.mp4", interval=int(delta_t * 1000), movie_writer="ffmpeg") # 保存 mp4 需要 ffmpeg
