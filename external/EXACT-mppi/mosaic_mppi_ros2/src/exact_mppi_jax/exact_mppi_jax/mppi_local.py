#!/usr/bin/env python

import cmd
import os
import threading
import time
import traceback
import struct
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np
import numpy.typing as npt
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from rclpy.time import Time
from visualization_msgs.msg import Marker
import yaml

from exact_mppi_jax.utils import quat_to_yaw, yaw_to_quat

from exact_mppi_jax.visualization_manager import VisualizationManager

try:
    from exact_mppi.mppi_jax.controller import MPPIController
except ImportError as e:
    raise ImportError(
        f"Failed to import the installed 'exact_mppi' package: {e}. "
        "Please install exact_mppi into the same Python environment used by this ROS2 node."
    ) from e


@dataclass
class RobotShapeConfig:
    vertices_list: list
    wheelbase: float


class MPPILocalNode(Node):
    """Local-input MPPI node.

    Subscribes:
      - /odom (nav_msgs/Odometry)
      - /scan (sensor_msgs/LaserScan)
      - /mppi_ref_traj_local (nav_msgs/Path)   (local ref trajectory in robot frame)
      - /mppi_goal_local (geometry_msgs/PoseStamped) (goal in robot frame)

    Publishes:
      - /mppi_cmd_vel (geometry_msgs/Twist) (remap to /cmd_vel in launch)

    Core behavior:
      - Converts odom twist (v, w) into (v, steer) if kinematics == 'acker'
      - Consumes per-tick local-frame ref_traj_local + goal_local from an external publisher
            (e.g., global_ref_path or a high-level policy)
      - Calls `MPPIController.computeVelocityCommands(...)` with explicit local inputs
    """

    def __init__(self) -> None:
        # Allow launch files to pass parameters without us having to declare every single one.
        # We intentionally treat planner YAML as the source of truth for runtime behavior.
        super().__init__("mppi_local", automatically_declare_parameters_from_overrides=True)

        self._state_lock = threading.Lock()
        self._mppi_lock = threading.Lock()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.callback_group = ReentrantCallbackGroup()

        self.pkg_dir = get_package_share_directory("exact_mppi_jax")

        # When launched with `--params-file`, parameters may already be declared
        # (because of automatically_declare_parameters_from_overrides=True).
        if not self.has_parameter("mppi_config_file"):
            self.declare_parameter("mppi_config_file", "corridor_planner.yaml")

        # Resolve planner config YAML
        cfg_name = self.get_parameter("mppi_config_file").get_parameter_value().string_value
        planner_config_file = os.path.join(self.pkg_dir, "config", "mppi_config", cfg_name)
        if not os.path.exists(planner_config_file):
            raise FileNotFoundError(
                f"Planner config file not found: {planner_config_file}. "
                "Check exact_mppi_jax/config/mppi_config."
            )
        self.planner_cfg = self._load_yaml(planner_config_file)

        # Load ROS2-facing settings from planner YAML (source of truth).
        planner_ref_enabled = bool((self.planner_cfg.get("reference_path", {}) or {}).get("enabled", False))
        ros2_overrides = {}
        if isinstance(self.planner_cfg, dict):
            ros2_overrides = self.planner_cfg.get("ros2", None) or {}

        default_ros2_cfg: Dict[str, Any] = {
            "control_frequency": 50.0,
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "ref_traj_local_topic": "/mppi_ref_traj_local",
            "goal_local_topic": "/mppi_goal_local",
            "cmd_vel_topic": "/mppi_cmd_vel",
            "publish_raw_cmd_vel": True,
            "cmd_vel_raw_topic": "/mppi_cmd_vel_raw",
            "scan_angle_min": -3.14,
            "scan_angle_max": 3.14,
            "scan_downsample": 1,
            "scan_range_min": 0.00,
            "scan_range_max": 50.0,
            "ref_enabled": planner_ref_enabled,
            "goal_enabled": True,
            "stop_on_arrival": True,
            "map_frame": "odom",
            "base_frame": "base_link",
            "enable_visualization": True,
            "enable_robot_marker": True,
            "enable_mppi_marker": True,
            "enable_rollout_markers": False,
            "enable_optimal_path_markers": False,
            "marker_size": 0.15,
            "marker_z": 0.01,
            "robot_marker_topic": "/robot_marker",
            "mppi_markers_topic": "/mppi_markers",
            "rollout_markers_topic": "/rollout_markers",
            "optimal_path_markers_topic": "/mppi_plan",
            "path_follow_marker_topic": "/mppi_path_follow_marker",
            "rollout_downsample": 10,
            "enable_timing_log": True,
            "timing_log_every_n": 50,
            "cmd_timeout_s": 0.2,
            "cmd_vel_lpf_enable": True,
            "cmd_vel_lpf_tau_linear": 0.0,
            "cmd_vel_lpf_tau_angular": 0.0,
            "cmd_vel_lpf_tau_vx": 0.0,
            "cmd_vel_lpf_tau_vy": 0.0,
            "publish_cost_breakdown": True,
            "cost_breakdown_topic": "/mppi_cost_breakdown",
            "cost_breakdown_every_n": 1,
        }

        if not isinstance(ros2_overrides, dict):
            ros2_overrides = {}
        self.ros2_cfg: Dict[str, Any] = {**default_ros2_cfg, **ros2_overrides}

        # Normalize types
        try:
            self.control_frequency = float(self.ros2_cfg.get("control_frequency", 50.0))
        except Exception:
            self.control_frequency = 50.0
        if self.control_frequency <= 0:
            raise ValueError(f"Invalid control_frequency: {self.control_frequency}")

        self.scan_topic = str(self.ros2_cfg["scan_topic"])
        self.odom_topic = str(self.ros2_cfg["odom_topic"])
        self.ref_traj_local_topic = str(self.ros2_cfg["ref_traj_local_topic"])
        self.goal_local_topic = str(self.ros2_cfg["goal_local_topic"])
        self.cmd_vel_topic = str(self.ros2_cfg["cmd_vel_topic"])
        self.publish_raw_cmd_vel = bool(self.ros2_cfg.get("publish_raw_cmd_vel", True))
        self.cmd_vel_raw_topic = str(self.ros2_cfg.get("cmd_vel_raw_topic", "/mppi_cmd_vel_raw"))

        self.scan_angle_min = float(self.ros2_cfg.get("scan_angle_min", -3.14))
        self.scan_angle_max = float(self.ros2_cfg.get("scan_angle_max", 3.14))
        self.scan_downsample = max(1, int(self.ros2_cfg.get("scan_downsample", 1)))
        self.scan_range_min = float(self.ros2_cfg.get("scan_range_min", 0.05))
        self.scan_range_max = float(self.ros2_cfg.get("scan_range_max", 50.0))

        self.ref_enabled = bool(self.ros2_cfg.get("ref_enabled", planner_ref_enabled))
        self.goal_enabled = bool(self.ros2_cfg.get("goal_enabled", True))
        self.stop_on_arrival = bool(self.ros2_cfg.get("stop_on_arrival", True))

        self.map_frame = str(self.ros2_cfg.get("map_frame", "map"))
        self.base_frame = str(self.ros2_cfg.get("base_frame", "base_link"))
        self.enable_timing_log = bool(self.ros2_cfg.get("enable_timing_log", True))
        self.timing_log_every_n = max(1, int(self.ros2_cfg.get("timing_log_every_n", 50)))
        self.cmd_timeout_s = float(self.ros2_cfg.get("cmd_timeout_s", 0.2))
        self.cmd_vel_lpf_enable = bool(self.ros2_cfg.get("cmd_vel_lpf_enable", False))
        legacy_tau = self.ros2_cfg.get("cmd_vel_lpf_tau", 0.0)
        self.cmd_vel_lpf_tau_linear = float(
            self.ros2_cfg.get("cmd_vel_lpf_tau_linear", legacy_tau)
        )
        self.cmd_vel_lpf_tau_vx = float(
            self.ros2_cfg.get("cmd_vel_lpf_tau_vx", self.cmd_vel_lpf_tau_linear)
        )
        self.cmd_vel_lpf_tau_vy = float(
            self.ros2_cfg.get("cmd_vel_lpf_tau_vy", self.cmd_vel_lpf_tau_linear)
        )
        self.cmd_vel_lpf_tau_angular = float(
            self.ros2_cfg.get("cmd_vel_lpf_tau_angular", legacy_tau)
        )
        self.cmd_vel_lpf_alpha_vx = 1.0
        self.cmd_vel_lpf_alpha_vy = 1.0
        self.cmd_vel_lpf_alpha_angular = 1.0
        if self.cmd_vel_lpf_enable:
            dt = 1.0 / self.control_frequency
            if self.cmd_vel_lpf_tau_vx > 0.0:
                self.cmd_vel_lpf_alpha_vx = max(
                    0.0, min(1.0, dt / (self.cmd_vel_lpf_tau_vx + dt))
                )
            if self.cmd_vel_lpf_tau_vy > 0.0:
                self.cmd_vel_lpf_alpha_vy = max(
                    0.0, min(1.0, dt / (self.cmd_vel_lpf_tau_vy + dt))
                )
            if self.cmd_vel_lpf_tau_angular > 0.0:
                self.cmd_vel_lpf_alpha_angular = max(
                    0.0, min(1.0, dt / (self.cmd_vel_lpf_tau_angular + dt))
                )
        self.enable_optimal_path_markers = bool(self.ros2_cfg.get("enable_optimal_path_markers", False))
        self.optimal_path_topic = str(self.ros2_cfg.get("optimal_path_markers_topic", "/mppi_plan"))
        self.path_follow_marker_topic = str(
            self.ros2_cfg.get("path_follow_marker_topic", "/mppi_path_follow_marker")
        )
        self.marker_size = float(self.ros2_cfg.get("marker_size", 0.15))
        self.publish_cost_breakdown = bool(self.ros2_cfg.get("publish_cost_breakdown", True))
        self.cost_breakdown_topic = str(self.ros2_cfg.get("cost_breakdown_topic", "/mppi_cost_breakdown"))
        self.cost_breakdown_every_n = max(1, int(self.ros2_cfg.get("cost_breakdown_every_n", 1)))
        self.lidar_debug_every_n = max(0, int(self.ros2_cfg.get("lidar_debug_every_n", 0)))
        self.lidar_debug_points_topic = str(self.ros2_cfg.get("lidar_debug_points_topic", "/mppi_lidar_points"))

        self.robot, self.mppi_controller = self._build_stack(self.planner_cfg)

        if isinstance(self.planner_cfg, dict):
            mppi_cfg = self.planner_cfg.get("MPPI", {}) or {}
        else:
            mppi_cfg = {}
        self.kinematics = str(mppi_cfg.get("motion_model_name", mppi_cfg.get("motion_model", "diff")))
        # print the kinematics model
        rclpy.logging.get_logger("MPPILocalNode").info(f"Using kinematics model: {self.kinematics}")
        self.arrival_position_threshold = float(self.planner_cfg.get("arrival_position_threshold", 0.3))
        self.arrival_yaw_threshold = float(self.planner_cfg.get("arrival_yaw_threshold", 0.3))

        # Runtime state
        self.robot_state_world: Optional[npt.NDArray] = None  # (3,) [x,y,yaw]
        self.robot_vel: Optional[npt.NDArray] = None          # (2,) [v,delta] or [v,w]
        self.lidar_points_local: Optional[npt.NDArray] = None # (N,2)

        # External local inputs (robot frame)
        self.goal_local: Optional[npt.NDArray] = None          # (3,)
        self.ref_traj_local: Optional[npt.NDArray] = None      # (N,3)
        self._cmd_lpf_prev: npt.NDArray = np.zeros((3,), dtype=np.float32)
        self._cmd_lpf_has_prev = False




        # Publishers/subscribers
        scan_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_cb,
            scan_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_cb,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            Path,
            self.ref_traj_local_topic,
            self._ref_traj_local_cb,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            PoseStamped,
            self.goal_local_topic,
            self._goal_local_cb,
            10,
            callback_group=self.callback_group,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.cmd_raw_pub = None
        if self.publish_raw_cmd_vel:
            self.cmd_raw_pub = self.create_publisher(
                Twist,
                self.cmd_vel_raw_topic,
                10,
            )
        self.optimal_path_pub = None
        self.path_follow_marker_pub = None
        if self.enable_optimal_path_markers:
            self.optimal_path_pub = self.create_publisher(Path, self.optimal_path_topic, 10)
            self.path_follow_marker_pub = self.create_publisher(
                Marker, self.path_follow_marker_topic, 10
            )
        self.cost_breakdown_pub = None
        self._cost_breakdown_keys: Optional[list[str]] = None
        if self.publish_cost_breakdown:
            self.cost_breakdown_pub = self.create_publisher(
                Float32MultiArray, self.cost_breakdown_topic, 10
            )
        self.lidar_debug_pub = None
        if self.lidar_debug_every_n > 0:
            self.lidar_debug_pub = self.create_publisher(
                PointCloud2, self.lidar_debug_points_topic, 10
            )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._warned_tf_missing = False

        self._tick = 0

        viz_config = {
            "enable_visualization": bool(self.ros2_cfg.get("enable_visualization", True)),
            "enable_dune_markers": False,
            "enable_robot_marker": bool(self.ros2_cfg.get("enable_robot_marker", False)),
            "enable_mppi_marker": bool(self.ros2_cfg.get("enable_mppi_marker", True)),
            "enable_rollout_markers": bool(self.ros2_cfg.get("enable_rollout_markers", False)),
            "map_frame": self.map_frame,
            "marker_size": float(self.ros2_cfg.get("marker_size", 0.15)),
            "marker_z": float(self.ros2_cfg.get("marker_z", 0.01)),
            "dune_markers_topic": "/dune_point_markers",
            "robot_marker_topic": str(self.ros2_cfg.get("robot_marker_topic", "/robot_marker")),
            "mppi_markers_topic": str(self.ros2_cfg.get("mppi_markers_topic", "/mppi_markers")),
            "rollout_markers_topic": str(self.ros2_cfg.get("rollout_markers_topic", "/rollout_markers")),
            "rollout_downsample": int(self.ros2_cfg.get("rollout_downsample", 10)),
            "state_lock": self._state_lock,
        }
        self.viz_manager = VisualizationManager(self, viz_config)

        self.get_logger().info(
            f"MPPILocalNode ready. planner={cfg_name} kinematics={self.kinematics} control={self.control_frequency}Hz"
        )

        self.create_timer(1.0 / self.control_frequency, self._run, callback_group=self.control_group)

    def _load_yaml(self, path: str) -> dict:
        with open(path, "r") as f:
            return yaml.safe_load(f)

    def _build_stack(self, planner_cfg: dict) -> Tuple[RobotShapeConfig, MPPIController]:
        mppi_cfg = dict(planner_cfg.get("MPPI", {}) or {})
        motion_model_name = mppi_cfg.get("motion_model_name", None)
        if "motion_model" not in mppi_cfg and motion_model_name:
            mppi_cfg["motion_model"] = motion_model_name
        # print motion model applied
        rclpy.logging.get_logger("MPPILocalNode").info(
            f"Building MPPIController with motion_model: {mppi_cfg.get('motion_model', 'diff')}"
        )

        kinematics = str(mppi_cfg.get("motion_model", "diff"))
        wheelbase = float(mppi_cfg.get("wheelbase", 1.6))

        vertices_list = mppi_cfg.get("vertices", None)
        mppi_unit_vertices = mppi_cfg.get("mppi_unit_vertices", [])
        if not vertices_list and mppi_unit_vertices:
            vertices_list = [mppi_unit_vertices]

        if not vertices_list:
            vertices_list = [[[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]]
            mppi_unit_vertices = vertices_list[0]

        max_speed = [
            float(mppi_cfg.get("vx_max", 2.0)),
            float(mppi_cfg.get("wz_max", 0.55)),
        ]
        max_accel = [
            float(mppi_cfg.get("ax_max", 1.0)),
            float(mppi_cfg.get("az_max", 1.0)),
        ]
        min_turning_r = float(mppi_cfg.get("min_turning_r", 0.2))

        robot = RobotShapeConfig(
            vertices_list=vertices_list,
            wheelbase=wheelbase,
        )

        if "motion_model" not in mppi_cfg:
            if kinematics == "acker":
                mppi_cfg["motion_model"] = "acker"
            else:
                mppi_cfg["motion_model"] = "diff"

        if "time_steps" not in mppi_cfg:
            mppi_cfg["time_steps"] = 30
        if "model_dt" not in mppi_cfg:
            mppi_cfg["model_dt"] = 0.1
        if "max_obs_num" not in mppi_cfg:
            mppi_cfg["max_obs_num"] = 100
        if "wheelbase" not in mppi_cfg:
            mppi_cfg["wheelbase"] = wheelbase

        if kinematics == "acker":
            ack_cfg = dict(mppi_cfg.get("AckermannConstraints", {}) or {})
            ack_cfg.setdefault("min_turning_r", min_turning_r)
            mppi_cfg["AckermannConstraints"] = ack_cfg

        self.mppi_horizon = int(mppi_cfg.get("time_steps", 30))

        controller = MPPIController(**mppi_cfg)
        footprint_vertices = mppi_cfg.get("vertices", None)
        if footprint_vertices is None:
            footprint_vertices = vertices_list
        if footprint_vertices:
            controller.setRectangleFootprint(footprint_vertices)

        return robot, controller

    def _scan_cb(self, scan_msg: LaserScan) -> None:
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            with self._state_lock:
                self.lidar_points_local = None
            return

        scan_frame = str(scan_msg.header.frame_id) if scan_msg.header.frame_id else self.base_frame
        self._last_scan_frame = scan_frame
        if scan_frame != self.base_frame:
            try:
                at_time = Time.from_msg(scan_msg.header.stamp) if (
                    scan_msg.header.stamp.sec != 0 or scan_msg.header.stamp.nanosec != 0
                ) else Time()
                tf = self._tf_buffer.lookup_transform(self.base_frame, scan_frame, at_time)
            except (LookupException, ConnectivityException, ExtrapolationException):
                try:
                    tf = self._tf_buffer.lookup_transform(self.base_frame, scan_frame, Time())
                except (LookupException, ConnectivityException, ExtrapolationException):
                    self.get_logger().warn(
                        f"LaserScan frame '{scan_frame}' cannot transform to base_frame='{self.base_frame}'. "
                        "Skipping scan update.",
                        throttle_duration_sec=2.0,
                    )
                    with self._state_lock:
                        self.lidar_points_local = None
                    return
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            q = tf.transform.rotation
            yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            c, s = float(np.cos(yaw)), float(np.sin(yaw))
        else:
            tx = ty = 0.0
            c, s = 1.0, 0.0

        angles = np.linspace(float(scan_msg.angle_min), float(scan_msg.angle_max), len(ranges)).astype(np.float32)

        down = self.scan_downsample
        min_r = max(float(scan_msg.range_min), float(self.scan_range_min))
        max_r = min(float(scan_msg.range_max), float(self.scan_range_max))

        idx = np.arange(len(ranges))
        valid = (
            (idx % down == 0)
            & np.isfinite(ranges)
            & (ranges >= min_r)
            & (ranges <= max_r)
        )

        rr = ranges[valid]
        aa = angles[valid]
        if rr.size == 0:
            with self._state_lock:
                self.lidar_points_local = None
            return

        x = rr * np.cos(aa)
        y = rr * np.sin(aa)
        pts = np.stack([x, y], axis=1).astype(np.float32)  # (N,2) in scan_frame
        if scan_frame != self.base_frame:
            x_b = c * pts[:, 0] - s * pts[:, 1] + tx
            y_b = s * pts[:, 0] + c * pts[:, 1] + ty
            pts = np.stack([x_b, y_b], axis=1).astype(np.float32)

        with self._state_lock:
            self.lidar_points_local = pts

    def _odom_cb(self, msg: Odometry) -> None:
        # Pose
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        yaw = float(quat_to_yaw(msg.pose.pose.orientation))

        # Twist
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        w = float(msg.twist.twist.angular.z)
        vel = np.array([vx, vy, w], dtype=np.float32)

        with self._state_lock:
            self.robot_state_world = np.array([px, py, yaw], dtype=np.float32)
            self.robot_vel = vel

    def _lookup_robot_pose_map(self, at_time: Optional[Time] = None) -> Optional[npt.NDArray]:
        """Return base_frame pose in map_frame using TF for visualization."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                at_time if at_time is not None else Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            try:
                tf = self._tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                if not self._warned_tf_missing:
                    self.get_logger().warn(
                        f"TF '{self.map_frame}' -> '{self.base_frame}' is missing; "
                        "skipping map-frame visualization.",
                        throttle_duration_sec=2.0,
                    )
                    self._warned_tf_missing = True
                return None

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        yaw = float(quat_to_yaw(tf.transform.rotation))
        return np.array([tx, ty, yaw], dtype=np.float32)

    def _goal_local_cb(self, msg: PoseStamped) -> None:
        goal_local = np.array(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(quat_to_yaw(msg.pose.orientation)),
            ],
            dtype=np.float32,
        )
        with self._state_lock:
            self.goal_local = goal_local

    def _ref_traj_local_cb(self, msg: Path) -> None:
        n = len(msg.poses)
        if n < 2:
            with self._state_lock:
                self.ref_traj_local = None
            return

        xs = np.array([p.pose.position.x for p in msg.poses], dtype=np.float32)
        ys = np.array([p.pose.position.y for p in msg.poses], dtype=np.float32)

        try:
            thetas = np.array([float(quat_to_yaw(p.pose.orientation)) for p in msg.poses], dtype=np.float32)
        except Exception:
            dx = np.diff(xs, append=xs[-1])
            dy = np.diff(ys, append=ys[-1])
            thetas = np.arctan2(dy, dx).astype(np.float32)
            if n > 1:
                thetas[-1] = thetas[-2]

        pts = np.stack([xs, ys, thetas], axis=1).astype(np.float32)
        with self._state_lock:
            self.ref_traj_local = pts

    def _fit_ref_traj_to_T(self, ref_traj_local: npt.NDArray) -> npt.NDArray:
        """Pad/truncate ref_traj_local to exactly MPPI horizon T."""
        T = int(getattr(self, "mppi_horizon", 0))
        if T <= 0:
            return ref_traj_local

        rt = np.asarray(ref_traj_local, dtype=np.float32)
        if rt.ndim != 2 or rt.shape[1] < 3:
            raise ValueError(f"ref_traj_local must be (N,3+), got {rt.shape}")

        if rt.shape[0] >= T:
            return rt[:T, :3]

        if rt.shape[0] == 0:
            return np.zeros((T, 3), dtype=np.float32)

        pad_n = T - rt.shape[0]
        last = rt[-1:, :3]
        return np.concatenate([rt[:, :3], np.repeat(last, pad_n, axis=0)], axis=0)

    def _points_to_pointcloud(self, points: npt.NDArray, stamp_msg) -> PointCloud2:
        msg = PointCloud2()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = stamp_msg

        if points is None:
            msg.height = 1
            msg.width = 0
            msg.fields = []
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = 0
            msg.data = b""
            msg.is_dense = False
            return msg

        pts = np.asarray(points, dtype=np.float32).reshape((-1, 2))
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False

        if msg.width == 0:
            msg.data = b""
            return msg

        data = bytearray(msg.row_step)
        offset = 0
        for p in pts:
            struct.pack_into("fff", data, offset, float(p[0]), float(p[1]), 0.0)
            offset += msg.point_step
        msg.data = bytes(data)
        return msg
        
    def _action_to_cmd_array(self, action: npt.NDArray) -> Optional[npt.NDArray]:
        if action is None:
            return None

        vel = np.asarray(action).reshape(-1)
        if vel.size < 2:
            return None

        if self.kinematics == "acker":
            vx = float(vel[0])
            steer = float(vel[1])
            wheelbase = float(getattr(self.robot, "wheelbase", 0.0) or 0.0)
            if wheelbase <= 0.0:
                self.get_logger().error("Wheelbase is not set for Ackermann kinematics.")
                return np.zeros((3,), dtype=np.float32)
            return np.array(
                [vx, 0.0, (vx / wheelbase) * float(np.tan(steer))],
                dtype=np.float32,
            )

        if self.kinematics == "diff":
            return np.array([float(vel[0]), 0.0, float(vel[1])], dtype=np.float32)

        if self.kinematics == "omni_xy":
            return np.array([float(vel[0]), float(vel[1]), 0.0], dtype=np.float32)

        if self.kinematics == "omni" or self.kinematics == "rangerminiv3":
            if vel.size < 3:
                self.get_logger().error(
                    f"Kinematics '{self.kinematics}' expects a 3D command, got shape {vel.shape}."
                )
                return np.zeros((3,), dtype=np.float32)
            return np.array([float(vel[0]), float(vel[1]), float(vel[2])], dtype=np.float32)

        self.get_logger().error(
            f"Unsupported kinematics model: {self.kinematics}. "
            "Expected 'diff', 'acker', 'omni_xy', 'omni', or 'rangerminiv3'."
        )
        return np.zeros((3,), dtype=np.float32)

    def _generate_twist_msg_direct(
        self,
        action: npt.NDArray,
        stop: bool,
        arrive: bool,
    ) -> Twist:
        msg = Twist()
        if stop or arrive:
            return msg

        cmd = self._action_to_cmd_array(action)
        if cmd is None or not np.all(np.isfinite(cmd)):
            return msg

        msg.linear.x = float(cmd[0])
        msg.linear.y = float(cmd[1])
        msg.angular.z = float(cmd[2])
        return msg
    
    def _generate_twist_msg(
        self,
        action: npt.NDArray,
        stop: bool,
        arrive: bool,
        *,
        apply_lpf: bool = True,
    ) -> Twist:
        msg = Twist()
        if stop or arrive:
            self._reset_cmd_lpf()
            return msg

        cmd = self._action_to_cmd_array(action)
        if cmd is None:
            return msg

        if not np.all(np.isfinite(cmd)):
            self._reset_cmd_lpf()
            return msg

        if self.cmd_vel_lpf_enable and apply_lpf:
            cmd = self._apply_cmd_lpf(cmd)

        msg.linear.x = float(cmd[0])
        msg.linear.y = float(cmd[1])
        msg.angular.z = float(cmd[2])
        return msg

    def _apply_cmd_lpf(self, cmd: npt.NDArray) -> npt.NDArray:
        cmd_arr = np.asarray(cmd, dtype=np.float32).reshape((3,))
        if not self._cmd_lpf_has_prev:
            self._cmd_lpf_prev[:] = cmd_arr
            self._cmd_lpf_has_prev = True
            return cmd_arr

        alpha_vx = float(self.cmd_vel_lpf_alpha_vx)
        alpha_vy = float(self.cmd_vel_lpf_alpha_vy)
        alpha_ang = float(self.cmd_vel_lpf_alpha_angular)
        prev = self._cmd_lpf_prev
        filt = np.empty_like(cmd_arr)
        filt[0] = alpha_vx * cmd_arr[0] + (1.0 - alpha_vx) * prev[0]
        filt[1] = alpha_vy * cmd_arr[1] + (1.0 - alpha_vy) * prev[1]
        filt[2] = alpha_ang * cmd_arr[2] + (1.0 - alpha_ang) * prev[2]
        self._cmd_lpf_prev[:] = filt
        return filt

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _path_msg_from_world(self, pts_world: npt.NDArray, stamp_msg) -> Path:
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = stamp_msg

        pts = np.asarray(pts_world, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] < 2:
            return msg

        has_yaw = pts.shape[1] >= 3
        for i in range(pts.shape[0]):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = stamp_msg
            ps.pose.position.x = float(pts[i, 0])
            ps.pose.position.y = float(pts[i, 1])
            ps.pose.position.z = 0.0
            yaw = float(pts[i, 2]) if has_yaw else 0.0
            ps.pose.orientation = yaw_to_quat(yaw)
            msg.poses.append(ps)

        return msg

    def _path_follow_marker(self, point_world: npt.NDArray, stamp_msg, action: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp_msg
        marker.ns = "path_follow_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = action
        marker.scale.x = self.marker_size * 1.2
        marker.scale.y = self.marker_size * 1.2
        marker.scale.z = self.marker_size * 1.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        if point_world is not None:
            marker.pose.position.x = float(point_world[0])
            marker.pose.position.y = float(point_world[1])
            marker.pose.position.z = 0.08
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def _local_traj_to_world(
        traj_local: npt.NDArray, robot_pose_world: npt.NDArray
    ) -> Optional[npt.NDArray]:
        if traj_local is None or robot_pose_world is None:
            return None
        traj = np.asarray(traj_local, dtype=np.float32)
        if traj.ndim != 2 or traj.shape[1] < 2:
            return None
        rx, ry, ryaw = robot_pose_world[:3]
        c = float(np.cos(ryaw))
        s = float(np.sin(ryaw))
        xw = rx + c * traj[:, 0] - s * traj[:, 1]
        yw = ry + s * traj[:, 0] + c * traj[:, 1]
        if traj.shape[1] >= 3:
            yaw = (traj[:, 2] + ryaw + np.pi) % (2.0 * np.pi) - np.pi
            return np.stack([xw, yw, yaw.astype(np.float32)], axis=1)
        return np.stack([xw, yw], axis=1)

    @staticmethod
    def _local_point_to_world(
        point_local: npt.NDArray, robot_pose_world: npt.NDArray
    ) -> Optional[npt.NDArray]:
        if point_local is None or robot_pose_world is None:
            return None
        pt = np.asarray(point_local, dtype=np.float32).reshape(-1)
        if pt.size < 2:
            return None
        rx, ry, ryaw = robot_pose_world[:3]
        c = float(np.cos(ryaw))
        s = float(np.sin(ryaw))
        xw = rx + c * pt[0] - s * pt[1]
        yw = ry + s * pt[0] + c * pt[1]
        return np.array([xw, yw], dtype=np.float32)

    def _build_goal_plan(self, goal_local: Optional[npt.NDArray], plan_len: int) -> npt.NDArray:
        if goal_local is None:
            return np.zeros((max(2, plan_len), 3), dtype=np.float32)
        start = np.zeros((3,), dtype=np.float32)
        goal = np.asarray(goal_local, dtype=np.float32).reshape((3,))
        n = max(2, int(plan_len))
        return np.linspace(start, goal, n).astype(np.float32)

    def _publish_zero_cmd(self) -> None:
        self._reset_cmd_lpf()
        self._publish_cmd(Twist())

    def _reset_cmd_lpf(self) -> None:
        if self.cmd_vel_lpf_enable:
            self._cmd_lpf_prev[0] = 0.0
            self._cmd_lpf_prev[1] = 0.0
            self._cmd_lpf_prev[2] = 0.0
            self._cmd_lpf_has_prev = False

    def _publish_cmd(self, msg: Twist) -> None:
        try:
            self.cmd_pub.publish(msg)
        except Exception:
            pass

    def _publish_raw_cmd(self, msg: Twist) -> None:
        if self.cmd_raw_pub is None:
            return
        try:
            self.cmd_raw_pub.publish(msg)
        except Exception:
            pass

    def _publish_cost_breakdown(
        self,
        costs_debug: Optional[dict],
    ) -> None:
        if not self.publish_cost_breakdown or self.cost_breakdown_pub is None:
            return

        payload: Dict[str, float] = {}
        if costs_debug:
            for name, values in costs_debug.items():
                arr = np.asarray(values, dtype=np.float32)
                if arr.size == 0:
                    continue
                payload[str(name)] = float(np.mean(arr))

        if not payload:
            return

        keys = sorted(payload.keys())
        self._cost_breakdown_keys = keys

        msg = Float32MultiArray()
        msg.data = [float(payload[k]) for k in keys]
        msg.layout.dim = [
            MultiArrayDimension(
                label="keys:" + ",".join(keys),
                size=len(msg.data),
                stride=len(msg.data),
            )
        ]
        try:
            self.cost_breakdown_pub.publish(msg)
        except Exception:
            pass

    def _run(self) -> None:
        self._tick += 1
        t0 = time.perf_counter()

        with self._state_lock:
            rs = None if self.robot_state_world is None else self.robot_state_world.copy()
            rv = None if self.robot_vel is None else self.robot_vel.copy()
            lidar_local = None if self.lidar_points_local is None else self.lidar_points_local.copy()
            goal_local_in = None if self.goal_local is None else self.goal_local.copy()
            ref_traj_local_in = None if self.ref_traj_local is None else self.ref_traj_local.copy()
            has_ok = rv is not None

        if self.lidar_debug_every_n > 0 and (self._tick % self.lidar_debug_every_n) == 0:
            if lidar_local is None or lidar_local.size == 0:
                self.get_logger().info(
                    f"lidar_debug: 0 points (scan_frame={getattr(self, '_last_scan_frame', 'unknown')} "
                    f"base_frame={self.base_frame})"
                )
                if self.lidar_debug_pub is not None:
                    try:
                        self.lidar_debug_pub.publish(
                            self._points_to_pointcloud(np.zeros((0, 2), dtype=np.float32), self.get_clock().now().to_msg())
                        )
                    except Exception:
                        pass
            else:
                max_obs = int(getattr(self.mppi_controller, "max_obs_num_", 0) or 0)
                if max_obs > 0 and lidar_local.shape[0] > max_obs:
                    r2 = np.einsum("ij,ij->i", lidar_local, lidar_local)
                    idx = np.argpartition(r2, max_obs - 1)[:max_obs]
                    selected_pts = lidar_local[idx]
                else:
                    selected_pts = lidar_local
                selected = int(selected_pts.shape[0])
                d2 = np.einsum("ij,ij->i", lidar_local, lidar_local)
                d = np.sqrt(np.maximum(d2, 0.0))
                self.get_logger().info(
                    f"lidar_debug: raw={lidar_local.shape[0]} selected={selected} "
                    f"max_obs={max_obs} range=[{float(d.min()):.3f}, {float(d.max()):.3f}] m"
                )
                if self.lidar_debug_pub is not None:
                    try:
                        self.lidar_debug_pub.publish(
                            self._points_to_pointcloud(selected_pts, self.get_clock().now().to_msg())
                        )
                    except Exception:
                        pass

        if not has_ok:
            self._publish_zero_cmd()
            return

        # Require goal unless explicitly disabled
        goal_enabled = bool(self.goal_enabled)
        if goal_enabled and (goal_local_in is None):
            self._publish_zero_cmd()
            return

        ref_enabled = bool(self.ref_enabled)
        stop_on_arrival = bool(self.stop_on_arrival)

        # Controller expects robot-frame inputs. Use a local frame with pose = (0,0,0).
        robot_pose_local = np.zeros((3,), dtype=np.float32)

        # External local inputs take priority.
        # If ref is enabled, require an externally provided local trajectory.
        if ref_enabled and ref_traj_local_in is None:
            self._publish_zero_cmd()
            return

        # If goal disabled, push a far dummy goal to avoid arrival stopping inside costs
        if not goal_enabled:
            goal_local = np.array([1e6, 0.0, 0.0], dtype=np.float32)
        else:
            goal_local = np.asarray(goal_local_in, dtype=np.float32).reshape((3,))

        # Build local plan (robot frame).
        t_ref0 = time.perf_counter()
        if ref_enabled and ref_traj_local_in is not None:
            ref_traj_local = self._fit_ref_traj_to_T(ref_traj_local_in)
            plan_local = ref_traj_local
        else:
            ref_traj_local = None
            plan_len = max(2, int(getattr(self, "mppi_horizon", 30)))
            plan_local = self._build_goal_plan(goal_local, plan_len)
        t_ref_ms = (time.perf_counter() - t_ref0) * 1000.0

        # Build robot speed vector (vx, vy, wz) in robot frame.
        vx = float(rv[0]) #if rv.size > 0 else 0.0
        w = float(rv[2]) #if rv.size > 1 else 0.0
        vy = float(rv[1]) #if rv.size > 2 else 0.0
        robot_speed = np.array([vx, vy, w], dtype=np.float32)

        # Run MPPI-JAX controller (robot-frame inputs).
        t_mppi0 = time.perf_counter()
        costs_debug = None
        optimal_traj_local = None
        want_optimal_path = bool(self.enable_optimal_path_markers)
        try:
            with self._mppi_lock:
                u = self.mppi_controller.computeVelocityCommands(
                    robot_pose=robot_pose_local,
                    robot_speed=robot_speed,
                    plan=plan_local,
                    goal=goal_local,
                    lidar_points=lidar_local,
                )
                if want_optimal_path:
                    optimal_traj_local = self.mppi_controller.getOptimalTrajectory()
                if self.publish_cost_breakdown and (self._tick % self.cost_breakdown_every_n) == 0:
                    costs_debug = self.mppi_controller.getCostsDebug()
        except Exception as e:
            self.get_logger().error(f"MPPI-JAX planning exception: {e}\n{traceback.format_exc()}")
            self._publish_zero_cmd()
            return
        t_mppi_ms = (time.perf_counter() - t_mppi0) * 1000.0

        if u is None:
            self._publish_zero_cmd()
            return

        u = np.asarray(u).reshape(-1)
        if u.size < 2 or not np.all(np.isfinite(u[:2])):
            self._publish_zero_cmd()
            return

        arrive = False
        goal_dist = None
        goal_yaw_err = None
        if goal_local is not None:
            goal_dist = float(np.linalg.norm(goal_local[:2]))
            goal_yaw_err = float(self._wrap_pi(goal_local[2]))
            arrive = bool(
                (goal_dist <= self.arrival_position_threshold)
                and (abs(goal_yaw_err) <= self.arrival_yaw_threshold)
            )
        if goal_dist is None:
            goal_dist = float("nan")
        if goal_yaw_err is None:
            goal_yaw_err = float("nan")

        stop = bool(arrive and stop_on_arrival)
        if self.cmd_raw_pub is not None:
            self._publish_raw_cmd(
                self._generate_twist_msg_direct(u, stop=stop, arrive=arrive)
            )
        self._publish_cmd(self._generate_twist_msg(u, stop=stop, arrive=arrive))
        if self.publish_cost_breakdown and (self._tick % self.cost_breakdown_every_n) == 0:
            self._publish_cost_breakdown(costs_debug)

        # Prefer TF map pose for visualization to keep markers in map frame.
        rs_map = self._lookup_robot_pose_map()

        if want_optimal_path and rs_map is not None and optimal_traj_local is not None:
            try:
                optimal_traj_world = self._local_traj_to_world(optimal_traj_local, rs_map)
                if optimal_traj_world is not None and self.optimal_path_pub is not None:
                    now_msg = self.get_clock().now().to_msg()
                    self.optimal_path_pub.publish(
                        self._path_msg_from_world(optimal_traj_world, now_msg)
                    )

                    path_follow_local = self.mppi_controller.getPathFollowPoint()
                    path_follow_point_world = self._local_point_to_world(path_follow_local, rs_map)
                    if self.path_follow_marker_pub is not None:
                        if path_follow_point_world is not None:
                            self.path_follow_marker_pub.publish(
                                self._path_follow_marker(
                                    path_follow_point_world, now_msg, Marker.ADD
                                )
                            )
                        else:
                            self.path_follow_marker_pub.publish(
                                self._path_follow_marker(
                                    np.zeros((2,), dtype=np.float32), now_msg, Marker.DELETE
                                )
                            )
            except Exception:
                pass

        if self.viz_manager.enable_visualization:
            try:
                robot_state = rs_map[:3].reshape((3, 1)) if rs_map is not None else None
                self.viz_manager.publish_visualization(
                    planner=self.mppi_controller,
                    robot_state=robot_state,
                    robot_config=self.robot,
                )
            except Exception:
                pass

        if self.enable_timing_log and (self._tick % self.timing_log_every_n) == 0:
            total_ms = (time.perf_counter() - t0) * 1000.0
            self.get_logger().info(
                f"[mppi_local timing] total={total_ms:.2f}ms ref={t_ref_ms:.2f}ms mppi={t_mppi_ms:.2f}ms "
                f"ref_enabled={ref_enabled} goal_enabled={goal_enabled} stop={stop} arrive={arrive} "
                f"goal_dist={goal_dist:.3f} goal_yaw_err={goal_yaw_err:.3f}"
            )


def main(args=None) -> None:
    # do not preallocate GPU memory in JAX to allow sharing with other processes (e.g. RViz)
    os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")

    rclpy.init(args=args)

    node = None
    executor = None
    try:
        node = MPPILocalNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
