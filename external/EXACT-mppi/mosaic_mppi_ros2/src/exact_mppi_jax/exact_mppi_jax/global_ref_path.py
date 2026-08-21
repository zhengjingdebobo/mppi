#!/usr/bin/env python

import math
import os
import threading
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

import yaml

from exact_mppi_jax.utils import yaw_to_quat
from exact_mppi_jax import pgm_conversion
from exact_mppi_jax.visualization_manager import VisualizationManager

from ament_index_python.packages import get_package_share_directory

try:
    from exact_mppi.path.path_search import PathSearch
    from exact_mppi.utils import env_config_to_grid
    from exact_mppi_jax.utils import ddr_map_to_irsim_map
except Exception:
    # Keep node importable even if exact_mppi is missing; line mode will still work.
    PathSearch = None  # type: ignore
    env_config_to_grid = None  # type: ignore
    ddr_map_to_irsim_map = None  # type: ignore
    # ament_index_python is still available; only exact_mppi bits may be missing.


def _yaw_from_xy(dx: float, dy: float, fallback: float = 0.0) -> float:
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return float(fallback)
    return float(math.atan2(dy, dx))


class GlobalRefPathNode(Node):
    """Publishes a global reference path (/plan) driven by an external goal pose.

    Behavior matches the previous corridor demo node:
      - The controller should not move until a /goal_pose is provided (e.g., RViz 2D Goal).
      - Once a goal is set, publish /plan at a fixed rate.
      - Optionally use /odom to set the start pose near the robot.
    """

    def __init__(self) -> None:
        super().__init__("global_ref_path", automatically_declare_parameters_from_overrides=True)

        # Planner config file selection stays as a ROS parameter (so launch can pick which YAML).
        if not self.has_parameter("mppi_config_file"):
            self.declare_parameter("mppi_config_file", "corridor_planner.yaml")
        if not self.has_parameter("map_yaml"):
            self.declare_parameter("map_yaml", "")
        if not self.has_parameter("base_frame"):
            self.declare_parameter("base_frame", "")
        if not self.has_parameter("enable_robot_marker"):
            self.declare_parameter("enable_robot_marker", False)

        cfg_name = self.get_parameter("mppi_config_file").get_parameter_value().string_value
        pkg_dir = get_package_share_directory("exact_mppi_jax")
        self._pkg_dir = pkg_dir
        planner_config_file = os.path.join(pkg_dir, "config", "mppi_config", cfg_name)
        if not os.path.exists(planner_config_file):
            raise FileNotFoundError(
                f"Planner config file not found: {planner_config_file}. "
                "Check exact_mppi_jax/config/mppi_config."
            )

        with open(planner_config_file, "r") as f:
            self.planner_cfg = yaml.safe_load(f) or {}

        ros2_cfg = {}
        if isinstance(self.planner_cfg, dict):
            ros2_cfg = self.planner_cfg.get("ros2", None) or self.planner_cfg.get("ros", None) or {}
        if not isinstance(ros2_cfg, dict):
            ros2_cfg = {}

        ref_cfg = (self.planner_cfg.get("reference_path", {}) or {}) if isinstance(self.planner_cfg, dict) else {}
        if not isinstance(ref_cfg, dict):
            ref_cfg = {}

        gr_cfg = {}
        if isinstance(self.planner_cfg, dict):
            gr_cfg = (
                self.planner_cfg.get("global_ref_path", None)
                or self.planner_cfg.get("global_ref_path_real_robot", None)
                or {}
            )
        if not isinstance(gr_cfg, dict):
            gr_cfg = {}

        # Shared ROS2 topics/frames live under planner YAML `ros2:` (no duplication).
        self.map_frame = str(ros2_cfg.get("map_frame", "map"))
        base_frame_param = self.get_parameter("base_frame").get_parameter_value().string_value
        self.base_frame = str(base_frame_param or ros2_cfg.get("base_frame", "base_link"))
        self.plan_topic = str(ros2_cfg.get("plan_topic", "/mppi_initial_path"))
        self.goal_topic = str(ros2_cfg.get("goal_topic", "/goal_pose"))
        self.odom_topic = str(ros2_cfg.get("odom_topic", "/odom"))
        self.goal_local_topic = str(ros2_cfg.get("goal_local_topic", "/mppi_goal_local"))
        self.ref_traj_local_topic = str(ros2_cfg.get("ref_traj_local_topic", "/mppi_ref_traj_local"))
        self.use_geometry_center_pose = bool(
            gr_cfg.get("use_geometry_center_pose", ros2_cfg.get("use_geometry_center_pose", False))
        )
        self.geometry_center_offset_x = float(ros2_cfg.get("geometry_center_offset_x", 0.0))
        self.geometry_center_offset_y = float(ros2_cfg.get("geometry_center_offset_y", 0.0))

        # Node-specific settings live under planner YAML `global_ref_path:`.
        self.use_tf_pose = bool(gr_cfg.get("use_tf_pose", True))
        self.publish_frequency = float(gr_cfg.get("publish_frequency", 30.0))
        if self.publish_frequency <= 0:
            self.publish_frequency = 1.0

        self.local_horizon = max(2, int(gr_cfg.get("local_horizon", 40)))
        self.local_stride = max(1, int(gr_cfg.get("local_stride", 1)))
        self.path_resolution = float(gr_cfg.get("path_resolution", 0.0))

        start = gr_cfg.get("start", [0.0, 20.0, 0.0])
        try:
            sx = float(start[0])
            sy = float(start[1])
            syaw = float(start[2]) if len(start) > 2 else 0.0
            self.start = (sx, sy, syaw)
        except Exception:
            self.start = (0.0, 20.0, 0.0)

        # Planning style comes from reference_path.path_type (no duplication).
        self.path_type = str(ref_cfg.get("path_type", "astar"))
        self.grid_resolution = float(ref_cfg.get("grid_resolution", 0.2))
        self.grid_inflation = float(ref_cfg.get("grid_inflation", 1.5))

        self.map_config_file = str(gr_cfg.get("map_config_file", "corridor_mppi.yaml"))
        map_yaml_param = self.get_parameter("map_yaml").get_parameter_value().string_value
        map_yaml_cfg = str(gr_cfg.get("map_yaml", ""))
        map_yaml = str(map_yaml_param or map_yaml_cfg)
        if map_yaml and not os.path.isabs(map_yaml):
            map_yaml = os.path.join(self._pkg_dir, map_yaml)
        self.map_yaml = map_yaml
        self.treat_unknown_as_occupied = bool(gr_cfg.get("treat_unknown_as_occupied", True))
        self.pgm_inflation_radius = float(gr_cfg.get("inflation_radius", self.grid_inflation))
        self.num_points = max(2, int(gr_cfg.get("num_points", 400)))
        self.waypoints = gr_cfg.get("waypoints", [])
        self._configured_waypoints = self._parse_waypoints(self.waypoints)

        # Optional robot marker visualization (map frame).
        self._viz_lock = threading.Lock()
        self._viz_manager: Optional[VisualizationManager] = None
        self._viz_robot_cfg = self._build_viz_robot_cfg()
        enable_robot_marker_param = self.get_parameter("enable_robot_marker").get_parameter_value().bool_value
        viz_cfg = {
            "enable_visualization": bool(ros2_cfg.get("enable_visualization", True)),
            "enable_dune_markers": False,
            "enable_robot_marker": bool(enable_robot_marker_param),
            "enable_mosaic_marker": bool(ros2_cfg.get("enable_mosaic_marker", True)),
            "enable_rollout_markers": False,
            "map_frame": self.map_frame,
            "marker_size": float(ros2_cfg.get("marker_size", 0.15)),
            "marker_z": float(ros2_cfg.get("marker_z", 0.01)),
            "dune_markers_topic": "/dune_point_markers",
            "robot_marker_topic": str(ros2_cfg.get("robot_marker_topic", "/robot_marker")),
            "mosaic_markers_topic": str(ros2_cfg.get("mosaic_markers_topic", "/mosaic_markers")),
            "rollout_markers_topic": str(ros2_cfg.get("rollout_markers_topic", "/rollout_markers")),
            "rollout_downsample": int(ros2_cfg.get("rollout_downsample", 10)),
            "state_lock": self._viz_lock,
        }
        self._viz_manager = VisualizationManager(self, viz_cfg)

        self.plan_pub = self.create_publisher(Path, self.plan_topic, 10)
        self.goal_local_pub = self.create_publisher(PoseStamped, self.goal_local_topic, 10)
        self.ref_traj_local_pub = self.create_publisher(Path, self.ref_traj_local_topic, 10)

        self.create_subscription(PoseStamped, self.goal_topic, self._goal_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self.timer = self.create_timer(1.0 / float(self.publish_frequency), self._on_timer)

        self._cached_path_msg: Optional[Path] = None
        self._cached_global_pts: Optional[np.ndarray] = None
        self._goal_world: Optional[Tuple[float, float, float]] = None
        self._odom_world: Optional[Tuple[float, float, float]] = None
        self._logged_first_publish: bool = False
        self._last_goal_world: Optional[Tuple[float, float, float]] = None

        self._odom_frame: Optional[str] = None
        self._warned_odom_frame_mismatch: bool = False
        self._warned_tf_missing: bool = False

        self._logged_bounds_warn: bool = False

        self._pgm_meta = None
        self._pgm_occ = None
        self._pgm_map_yaml: Optional[str] = None
        self._pgm_target_resolution: Optional[float] = None
        self._pgm_inflation_radius: Optional[float] = None
        self._pgm_treat_unknown: Optional[bool] = None

        # TF listener for map->base transform (robot_localization typically populates TF)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        if len(self._configured_waypoints) >= 2:
            self._goal_world = self._configured_waypoints[-1][:3]
            self._last_goal_world = self._goal_world
            self.get_logger().info(
                f"Using configured waypoints as startup goal path ({len(self._configured_waypoints)} waypoints)"
            )

        self.get_logger().info(
            f"Listening for goal on {self.goal_topic}; publishing path on {self.plan_topic} @ {self.publish_frequency}Hz; "
            f"publishing local goal/ref on {self.goal_local_topic} and {self.ref_traj_local_topic}"
        )

    def _wrap_pi(self, angle: float) -> float:
        return float((angle + math.pi) % (2.0 * math.pi) - math.pi)

    def _build_viz_robot_cfg(self):
        planner_cfg = self.planner_cfg if isinstance(self.planner_cfg, dict) else {}
        mppi_cfg = planner_cfg.get("MPPI", {}) if isinstance(planner_cfg, dict) else {}
        vertices_list = mppi_cfg.get("vertices", None)
        if not vertices_list:
            mosaic_unit_vertices = mppi_cfg.get("mosaic_unit_vertices", [])
            if mosaic_unit_vertices:
                vertices_list = [mosaic_unit_vertices]
        if not vertices_list:
            return None

        class _VizRobotConfig:
            def __init__(self, vertices, wheelbase):
                self.vertices_list = vertices
                self.wheelbase = wheelbase

        wheelbase = float(mppi_cfg.get("wheelbase", 1.6))
        return _VizRobotConfig(vertices_list, wheelbase)

    def _parse_waypoints(self, cfg) -> List[Tuple[float, float, float, Optional[int]]]:
        out: List[Tuple[float, float, float, Optional[int]]] = []
        if cfg is None:
            return out

        if isinstance(cfg, list):
            if all(isinstance(w, dict) for w in cfg):
                for item in cfg:
                    try:
                        x = float(item.get("x"))
                        y = float(item.get("y"))
                    except Exception:
                        continue
                    yaw = item.get("yaw", 0.0)
                    try:
                        yaw_f = float(yaw)
                    except Exception:
                        yaw_f = 0.0
                    wid = item.get("id", None)
                    try:
                        wid_i = int(wid) if wid is not None else None
                    except Exception:
                        wid_i = None
                    out.append((x, y, yaw_f, wid_i))
                return out

            if all(isinstance(v, (int, float)) for v in cfg):
                vals = [float(v) for v in cfg]
                for idx in range(0, len(vals), 3):
                    if idx + 1 >= len(vals):
                        break
                    x = vals[idx]
                    y = vals[idx + 1]
                    yaw = vals[idx + 2] if (idx + 2) < len(vals) else 0.0
                    out.append((x, y, yaw, None))
                return out

        return out

    def _build_line_segment(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
    ) -> List[Tuple[float, float, float]]:
        sx, sy, syaw = start
        gx, gy, gyaw = goal
        dx = gx - sx
        dy = gy - sy
        dist = float(math.hypot(dx, dy))
        if dist <= 1e-9:
            return [(sx, sy, syaw), (gx, gy, gyaw)]

        step = self.path_resolution if self.path_resolution > 0.0 else max(self.grid_resolution, 0.2)
        count = max(2, int(math.floor(dist / step)) + 1)
        yaw_line = _yaw_from_xy(dx, dy, fallback=syaw)
        pts: List[Tuple[float, float, float]] = []
        for i in range(count):
            alpha = float(i) / float(count - 1)
            x = sx + alpha * dx
            y = sy + alpha * dy
            yaw = yaw_line if i < count - 1 else gyaw
            pts.append((float(x), float(y), float(yaw)))
        return pts

    def _build_path_from_configured_waypoints(self) -> List[Tuple[float, float, float]]:
        wps = self._configured_waypoints
        if len(wps) < 2:
            return []

        if all(w[3] is not None for w in wps):
            wps = sorted(wps, key=lambda item: int(item[3] or 0))

        path_type = str(self.path_type)
        combined: List[Tuple[float, float, float]] = []

        for index in range(len(wps) - 1):
            start = wps[index][:3]
            goal = wps[index + 1][:3]

            segment: Optional[List[Tuple[float, float, float]]] = None
            if path_type == "astar":
                segment = self._build_astar_points(start=start, goal=goal)

            if not segment or len(segment) < 2:
                segment = self._build_line_segment(start, goal)

            if index > 0 and segment:
                segment = segment[1:]
            combined.extend(segment)

        return combined

    def _quat_to_yaw(self, q) -> float:
        return float(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _transform_goal_to_map_frame(self, msg: PoseStamped) -> Optional[Tuple[float, float, float]]:
        """Return goal (x,y,yaw) expressed in self.map_frame.

        RViz can publish /goal_pose in whatever its fixed frame is (often 'map' or 'odom').
        If it doesn't match map_frame, we try to transform it via TF. If TF isn't available,
        return None so we don't drive toward a goal in the wrong frame.
        """
        src_frame = str(msg.header.frame_id) if msg.header.frame_id else self.map_frame
        q = msg.pose.orientation
        yaw_src = self._quat_to_yaw(q)

        x_src = float(msg.pose.position.x)
        y_src = float(msg.pose.position.y)

        if src_frame == self.map_frame:
            return (x_src, y_src, yaw_src)

        # Try TF: map_frame <- src_frame
        try:
            at_time = Time.from_msg(msg.header.stamp) if (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0) else Time()
            tf = self._tf_buffer.lookup_transform(self.map_frame, src_frame, at_time)
        except (LookupException, ConnectivityException, ExtrapolationException):
            try:
                tf = self._tf_buffer.lookup_transform(self.map_frame, src_frame, Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                self.get_logger().warn(
                    f"Received /goal_pose in frame '{src_frame}', but cannot transform to map_frame='{self.map_frame}'. "
                    "Set RViz fixed frame to match map_frame or provide TF between frames.",
                    throttle_duration_sec=2.0,
                )
                return None

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        yaw_tf = self._quat_to_yaw(tf.transform.rotation)

        c = float(math.cos(yaw_tf))
        s = float(math.sin(yaw_tf))
        x_map = tx + c * x_src - s * y_src
        y_map = ty + s * x_src + c * y_src
        yaw_map = self._wrap_pi(yaw_tf + yaw_src)
        return (x_map, y_map, yaw_map)

    def _lookup_robot_pose_map(self, at_time: Optional[Time] = None) -> Optional[Tuple[float, float, float]]:
        """Return (x,y,yaw) of base_frame in map_frame.

        Prefers TF (robot_localization output). Falls back to odom pose if it
        already is in map_frame.
        """
        if self.use_tf_pose:
            tf = None
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
                    tf = None

            if tf is not None:
                tx = float(tf.transform.translation.x)
                ty = float(tf.transform.translation.y)
                yaw = self._quat_to_yaw(tf.transform.rotation)
                if self.use_geometry_center_pose:
                    if self.geometry_center_offset_x != 0.0 or self.geometry_center_offset_y != 0.0:
                        c = float(math.cos(yaw))
                        s = float(math.sin(yaw))
                        dx = c * float(self.geometry_center_offset_x) - s * float(self.geometry_center_offset_y)
                        dy = s * float(self.geometry_center_offset_x) + c * float(self.geometry_center_offset_y)
                        tx += dx
                        ty += dy
                return (tx, ty, yaw)

            if self._odom_world is not None and self._odom_frame is not None:
                if self._odom_frame == self.map_frame:
                    return self._odom_world

                try:
                    tf_mo = self._tf_buffer.lookup_transform(
                        self.map_frame,
                        self._odom_frame,
                        at_time if at_time is not None else Time(),
                    )
                except (LookupException, ConnectivityException, ExtrapolationException):
                    try:
                        tf_mo = self._tf_buffer.lookup_transform(self.map_frame, self._odom_frame, Time())
                    except (LookupException, ConnectivityException, ExtrapolationException):
                        if not self._warned_tf_missing:
                            self.get_logger().warn(
                                f"TF '{self.map_frame}' -> '{self.base_frame}' is missing. Also cannot transform "
                                f"/odom frame '{self._odom_frame}' into map_frame '{self.map_frame}'. "
                                "Provide TF map->odom (e.g., localization), or set planning to odom.",
                                throttle_duration_sec=2.0,
                            )
                            self._warned_tf_missing = True
                        return None

                ox, oy, oyaw = self._odom_world
                tx = float(tf_mo.transform.translation.x)
                ty = float(tf_mo.transform.translation.y)
                yaw_tf = self._quat_to_yaw(tf_mo.transform.rotation)
                c = float(math.cos(yaw_tf))
                s = float(math.sin(yaw_tf))
                mx = tx + c * float(ox) - s * float(oy)
                my = ty + s * float(ox) + c * float(oy)
                myaw = self._wrap_pi(yaw_tf + float(oyaw))
                return (mx, my, myaw)

        if self._odom_world is not None:
            # If odom is actually in map frame, this is usable.
            # Otherwise it will be in odom frame and map->odom TF is required.
            return self._odom_world

        return None

    def _world_to_robot_xy(self, *, dx: float, dy: float, robot_yaw: float) -> Tuple[float, float]:
        c = float(math.cos(robot_yaw))
        s = float(math.sin(robot_yaw))
        # world -> robot
        x_local = c * dx + s * dy
        y_local = -s * dx + c * dy
        return (x_local, y_local)

    def _build_local_ref(self, global_pts: np.ndarray, robot_pose_map: Tuple[float, float, float]) -> np.ndarray:
        """Generate a local reference trajectory in base_frame from a global path."""
        rx, ry, ryaw = robot_pose_map
        if global_pts.ndim != 2 or global_pts.shape[1] < 3 or global_pts.shape[0] < 2:
            return np.zeros((0, 3), dtype=np.float32)

        horizon = int(self.local_horizon)
        stride = int(self.local_stride)

        # Find closest path index using full search (no index memory).
        d2_full = (global_pts[:, 0] - rx) ** 2 + (global_pts[:, 1] - ry) ** 2
        if d2_full.size == 0:
            closest = 0
        else:
            closest = int(np.argmin(d2_full))

        sel = global_pts[closest::stride]
        if sel.shape[0] >= horizon:
            sel = sel[:horizon]
        elif sel.shape[0] >= 1:
            pad_n = horizon - sel.shape[0]
            last = sel[-1:, :]
            sel = np.concatenate([sel, np.repeat(last, pad_n, axis=0)], axis=0)
        else:
            return np.zeros((0, 3), dtype=np.float32)

        dx = sel[:, 0] - rx
        dy = sel[:, 1] - ry

        c = float(math.cos(ryaw))
        s = float(math.sin(ryaw))
        x_local = c * dx + s * dy
        y_local = -s * dx + c * dy
        yaw_local = np.array([self._wrap_pi(float(y) - ryaw) for y in sel[:, 2]], dtype=np.float32)

        traj = np.stack([x_local.astype(np.float32), y_local.astype(np.float32), yaw_local], axis=1)
        # Guarantee fixed horizon length for downstream MPPI.
        if traj.shape[0] != horizon:
            if traj.shape[0] > horizon:
                traj = traj[:horizon]
            elif traj.shape[0] > 0:
                pad_n = horizon - traj.shape[0]
                traj = np.concatenate([traj, np.repeat(traj[-1:, :], pad_n, axis=0)], axis=0)
        return traj

    def _path_msg_from_local(self, pts_local: np.ndarray, stamp_msg) -> Path:
        msg = Path()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = stamp_msg

        if pts_local.ndim != 2 or pts_local.shape[1] < 2:
            return msg

        for i in range(pts_local.shape[0]):
            ps = PoseStamped()
            ps.header.frame_id = self.base_frame
            ps.header.stamp = msg.header.stamp
            ps.pose.position.x = float(pts_local[i, 0])
            ps.pose.position.y = float(pts_local[i, 1])
            yaw = float(pts_local[i, 2]) if pts_local.shape[1] > 2 else 0.0
            ps.pose.orientation = yaw_to_quat(yaw)
            msg.poses.append(ps)
        return msg

    def _pose_msg_goal_local(self, goal_local: Tuple[float, float, float], stamp_msg) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = stamp_msg
        msg.pose.position.x = float(goal_local[0])
        msg.pose.position.y = float(goal_local[1])
        msg.pose.orientation = yaw_to_quat(float(goal_local[2]))
        return msg

    def _goal_cb(self, msg: PoseStamped) -> None:
        # Goal comes from RViz SetGoal tool (2D Goal Pose). It may be in 'map' or 'odom'
        # depending on RViz fixed frame.
        goal = self._transform_goal_to_map_frame(msg)
        if goal is None:
            return

        # RViz / tools can occasionally deliver duplicates; ignore tiny repeats
        # to avoid double re-planning and repeated /plan publishes.
        if self._last_goal_world is not None:
            dx = goal[0] - self._last_goal_world[0]
            dy = goal[1] - self._last_goal_world[1]
            dyaw = float((goal[2] - self._last_goal_world[2] + math.pi) % (2.0 * math.pi) - math.pi)
            if (dx * dx + dy * dy) < (1e-6) and abs(dyaw) < 1e-3:
                return

        self._last_goal_world = goal
        self._goal_world = goal
        self._cached_path_msg = None
        self._cached_global_pts = None
        self._logged_first_publish = False
        self.get_logger().info(
            f"Received /goal_pose: ({self._goal_world[0]:.2f},{self._goal_world[1]:.2f}) - rebuilding plan"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_frame = str(msg.header.frame_id)
        if (not self.use_tf_pose) and (self._odom_frame and self.map_frame != self._odom_frame) and (not self._warned_odom_frame_mismatch):
            self._warned_odom_frame_mismatch = True
            self.get_logger().warn(
                f"use_tf_pose:=false but map_frame='{self.map_frame}' != odom frame_id='{self._odom_frame}'. "
                "This will make the global path frame inconsistent with the pose used to build local references. "
                "Set map_frame to the odom frame (often 'odom') or enable TF pose.",
                throttle_duration_sec=2.0,
            )
        q = msg.pose.pose.orientation
        yaw = float(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self._odom_world = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw)

    def _build_path_points(self) -> List[Tuple[float, float, float]]:
        path_type = str(self.path_type)

        if len(self._configured_waypoints) >= 2:
            pts = self._build_path_from_configured_waypoints()
            if len(pts) >= 2:
                return pts

        if self._goal_world is None:
            return []

        # Start from TF/odom if available; otherwise use configured start.
        robot_pose = self._lookup_robot_pose_map()
        if robot_pose is not None:
            sx, sy, syaw = robot_pose
        elif self._odom_world is not None:
            sx, sy, syaw = self._odom_world
        else:
            sx, sy, syaw = self.start

        gx, gy, gyaw = self._goal_world

        if path_type == "astar":
            pts = self._build_astar_points(
                start=(sx, sy, syaw),
                goal=(gx, gy, gyaw),
            )
            if pts is not None and len(pts) >= 2:
                return pts

        if path_type == "waypoints":
            pts = [(x, y, yaw) for x, y, yaw, _ in self._configured_waypoints]
            if len(pts) >= 2:
                return pts

        n = int(self.num_points)
        dx = gx - sx
        dy = gy - sy
        yaw_line = _yaw_from_xy(dx, dy, fallback=syaw)

        pts = []
        for k in range(n):
            alpha = float(k) / float(n - 1)
            x = sx + alpha * dx
            y = sy + alpha * dy
            yaw = yaw_line if k < (n - 1) else float(gyaw)
            pts.append((x, y, yaw))
        return pts

    def _resample_path(
        self, pts: List[Tuple[float, float, float]], resolution: float
    ) -> List[Tuple[float, float, float]]:
        if len(pts) < 2:
            return pts

        step = float(resolution)
        if step <= 0.0:
            return pts

        arr = np.asarray(pts, dtype=np.float32)
        xy = arr[:, :2]
        seg = np.diff(xy, axis=0)
        seg_len = np.linalg.norm(seg, axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg_len)])
        total = float(cum[-1])
        if total <= 1e-6:
            return pts

        num = int(math.floor(total / step))
        distances = np.linspace(0.0, total, num + 1, dtype=np.float32)
        idx = np.searchsorted(cum, distances, side="right") - 1
        idx = np.clip(idx, 0, len(seg_len) - 1)

        seg_start = cum[idx]
        seg_d = seg_len[idx]
        t = (distances - seg_start) / np.where(seg_d > 1e-6, seg_d, 1.0)
        t = t.reshape((-1, 1))

        start_xy = xy[idx]
        end_xy = xy[idx + 1]
        new_xy = start_xy + (end_xy - start_xy) * t

        if arr.shape[1] >= 3:
            yaws = arr[:, 2]
            yaw_start = yaws[idx]
            yaw_end = yaws[idx + 1]
            dyaw = (yaw_end - yaw_start + np.pi) % (2.0 * np.pi) - np.pi
            new_yaw = yaw_start + dyaw * t[:, 0]
            new_yaw[-1] = yaws[-1]
        else:
            seg_dir = end_xy - start_xy
            new_yaw = np.arctan2(seg_dir[:, 1], seg_dir[:, 0])

        out = np.stack(
            [new_xy[:, 0], new_xy[:, 1], new_yaw.astype(np.float32)], axis=1
        )
        return [(float(x), float(y), float(yaw)) for x, y, yaw in out]

    def _is_map_yaml(self, path: str) -> bool:
        try:
            with open(path, "r") as f:
                cfg = yaml.safe_load(f) or {}
        except Exception:
            return False
        return isinstance(cfg, dict) and "image" in cfg

    def _resolve_map_yaml(self) -> Optional[str]:
        if self.map_yaml:
            return self.map_yaml

        candidate = self.map_config_file
        if not candidate:
            return None

        if not os.path.isabs(candidate):
            candidate = os.path.join(self._pkg_dir, candidate)
        if os.path.exists(candidate) and self._is_map_yaml(candidate):
            return candidate
        return None

    def _ensure_pgm_map_loaded(self, map_yaml_path: str) -> bool:
        target_resolution = float(self.grid_resolution) if self.grid_resolution > 0.0 else 0.0
        inflation_radius = float(self.pgm_inflation_radius)
        treat_unknown = bool(self.treat_unknown_as_occupied)

        if (
            self._pgm_meta is not None
            and self._pgm_occ is not None
            and self._pgm_map_yaml == map_yaml_path
            and self._pgm_target_resolution == target_resolution
            and self._pgm_inflation_radius == inflation_radius
            and self._pgm_treat_unknown == treat_unknown
        ):
            return True

        if not os.path.exists(map_yaml_path):
            self.get_logger().warn(f"Map YAML not found: {map_yaml_path}")
            return False

        try:
            meta, occ = pgm_conversion.build_occupancy_from_map_yaml(
                map_yaml_path,
                target_resolution=target_resolution,
                inflation_radius=inflation_radius,
                treat_unknown_as_occupied=treat_unknown,
            )
            if abs(meta.origin_yaw) > 1e-3:
                self.get_logger().warn(
                    f"Map origin yaw={meta.origin_yaw:.3f} rad is not supported; treating as 0."
                )
            self._pgm_meta = meta
            self._pgm_occ = occ
            self._pgm_map_yaml = map_yaml_path
            self._pgm_target_resolution = target_resolution
            self._pgm_inflation_radius = inflation_radius
            self._pgm_treat_unknown = treat_unknown
            inflation_cells = int(max(0.0, inflation_radius) / float(meta.resolution))
            self.get_logger().info(
                f"Loaded PGM map {map_yaml_path} (res={meta.resolution}, size={meta.width}x{meta.height}), "
                f"inflation={inflation_cells} cells"
            )
            return True
        except Exception as exc:
            self.get_logger().warn(f"Failed to load PGM map {map_yaml_path}: {exc}")
            return False

    def _build_astar_points_from_map_yaml(
        self,
        *,
        map_yaml_path: str,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
    ) -> Optional[List[Tuple[float, float, float]]]:
        if not self._ensure_pgm_map_loaded(map_yaml_path):
            return None

        assert self._pgm_meta is not None
        assert self._pgm_occ is not None

        start_g = pgm_conversion.world_to_grid(self._pgm_meta, start[0], start[1])
        goal_g = pgm_conversion.world_to_grid(self._pgm_meta, goal[0], goal[1])
        if start_g is None or goal_g is None:
            if not self._logged_bounds_warn:
                meta = self._pgm_meta
                x_min = float(meta.origin_x)
                y_min = float(meta.origin_y)
                x_max = float(meta.origin_x + meta.width * meta.resolution)
                y_max = float(meta.origin_y + meta.height * meta.resolution)
                self.get_logger().warn(
                    "Start/goal out of bounds (one-time debug): "
                    f"start=({start[0]:.3f},{start[1]:.3f},{start[2]:.3f}) "
                    f"goal=({goal[0]:.3f},{goal[1]:.3f},{goal[2]:.3f}) "
                    f"map_bounds=([x:{x_min:.3f},{x_max:.3f}] [y:{y_min:.3f},{y_max:.3f}]) "
                    f"map_frame='{self.map_frame}' base_frame='{self.base_frame}' goal_frame='{self.goal_topic}'"
                )
                self._logged_bounds_warn = True
            self.get_logger().warn(
                "Start/goal outside map bounds: "
                f"start_map=({start[0]:.3f},{start[1]:.3f},{start[2]:.3f}) "
                f"goal_map=({goal[0]:.3f},{goal[1]:.3f},{goal[2]:.3f}) "
                f"start_in_map={start_g is not None} goal_in_map={goal_g is not None}"
            )
            return None

        path_cells = pgm_conversion.astar(self._pgm_occ, start=start_g, goal=goal_g)
        if not path_cells or len(path_cells) < 2:
            self.get_logger().warn("A* failed to find a path on the PGM map")
            return None

        pts: List[Tuple[float, float, float]] = []
        for i, (cx, cy) in enumerate(path_cells):
            x, y = pgm_conversion.grid_to_world(self._pgm_meta, cx, cy)
            if i < len(path_cells) - 1:
                nx, ny = pgm_conversion.grid_to_world(self._pgm_meta, path_cells[i + 1][0], path_cells[i + 1][1])
                yaw = _yaw_from_xy(nx - x, ny - y, fallback=start[2])
            else:
                yaw = float(goal[2])
            pts.append((float(x), float(y), float(yaw)))

        if len(pts) >= 2:
            sx, sy, syaw = start
            gx, gy, gyaw = goal
            yaw_start = _yaw_from_xy(pts[1][0] - sx, pts[1][1] - sy, fallback=syaw)
            pts[0] = (float(sx), float(sy), float(yaw_start))
            pts[-1] = (float(gx), float(gy), float(gyaw))

        return pts

    def _build_astar_points(
        self, *, start: Tuple[float, float, float], goal: Tuple[float, float, float]
    ) -> Optional[List[Tuple[float, float, float]]]:
        map_yaml = self._resolve_map_yaml()
        if map_yaml:
            return self._build_astar_points_from_map_yaml(
                map_yaml_path=map_yaml,
                start=start,
                goal=goal,
            )

        if PathSearch is None or env_config_to_grid is None or ddr_map_to_irsim_map is None:
            self.get_logger().warn(
                "A* requested but exact_mppi dependencies are unavailable; falling back to line.",
                throttle_duration_sec=2.0,
            )
            return None

        try:
            ddr_pkg_dir = get_package_share_directory("ddr_minimal_sim")

            planner_cfg = self.planner_cfg if isinstance(self.planner_cfg, dict) else {}

            map_name = self.map_config_file
            map_path = map_name if os.path.isabs(map_name) else os.path.join(ddr_pkg_dir, "config", "maps", map_name)
            if not os.path.exists(map_path):
                self.get_logger().warn(f"DDR map not found: {map_path}; falling back to line")
                return None

            with open(map_path, "r") as f:
                ddr_map_cfg = yaml.safe_load(f) or {}
            irsim_cfg = ddr_map_to_irsim_map(ddr_map_cfg)
            world_cfg = irsim_cfg.get("world", {})
            grid_origin = world_cfg.get("offset", [0.0, 0.0])

            grid_resolution = float(self.grid_resolution)
            inflation = float(self.grid_inflation)
            grid_map = env_config_to_grid(irsim_cfg, resolution=grid_resolution, inflation_radius=inflation)

            vehicle_polygons = planner_cfg.get("MPPI", {}).get("vertices")
            path_searcher = PathSearch(
                grid_map,
                resolution=grid_resolution,
                origin=grid_origin,
                curve_style="astar",
                vehicle_polygons=vehicle_polygons,
            )
            grid_path, _ = path_searcher.find_initial_path(start, goal)
            if not grid_path:
                self.get_logger().warn("A* failed to produce a path; falling back to line")
                return None

            world_path = path_searcher.path_to_world_coords(grid_path, interval=grid_resolution)
            if not world_path:
                self.get_logger().warn("A* failed to produce a path; falling back to line")
                return None

            pts: List[Tuple[float, float, float]] = []
            for p in world_path:
                a = np.asarray(p, dtype=np.float32).reshape((-1,))
                if a.size >= 3:
                    pts.append((float(a[0]), float(a[1]), float(a[2])))
            if pts:
                gx, gy, gyaw = goal
                pts[-1] = (float(gx), float(gy), float(gyaw))
            return pts
        except Exception as e:
            self.get_logger().warn(f"A* path build failed: {e}; falling back to line")
            return None

    def _build_path_msg(self, pts: List[Tuple[float, float, float]]) -> Path:
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y, yaw) in pts:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = msg.header.stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = yaw_to_quat(float(yaw))
            msg.poses.append(ps)

        return msg

    def _refresh_cached_messages(self) -> None:
        pts = self._build_path_points()
        if len(pts) < 2:
            self._cached_path_msg = None
            self._cached_global_pts = None
            return

        if self.path_resolution > 0.0:
            pts = self._resample_path(pts, self.path_resolution)

        # New path => restart closest-point tracking.
        self._cached_path_msg = self._build_path_msg(pts)

        # Cache numeric representation of the global path for fast local transforms.
        # This is independent of message timestamps (which are refreshed in the timer).
        try:
            self._cached_global_pts = np.asarray(pts, dtype=np.float32)
        except Exception:
            self._cached_global_pts = None

        if self._odom_world is not None:
            sx, sy, _ = self._odom_world
        else:
            sx, sy, _ = self.start
        assert self._goal_world is not None
        self.get_logger().info(
            f"Global path ready: {len(pts)} pts start=({sx:.2f},{sy:.2f}) goal=({self._goal_world[0]:.2f},{self._goal_world[1]:.2f})"
        )

    def _ensure_cached_global_pts(self) -> Optional[np.ndarray]:
        """Return cached global path as (x,y,yaw) np.ndarray.

        Preferred source is the path points generated during planning.
        Falls back to extracting yaw from the cached Path message if needed.
        """
        if self._cached_global_pts is not None:
            return self._cached_global_pts
        if self._cached_path_msg is None:
            return None

        try:
            poses = self._cached_path_msg.poses
            xs = np.array([p.pose.position.x for p in poses], dtype=np.float32)
            ys = np.array([p.pose.position.y for p in poses], dtype=np.float32)
            thetas = np.array(
                [
                    float(
                        math.atan2(
                            2.0
                            * (
                                p.pose.orientation.w * p.pose.orientation.z
                                + p.pose.orientation.x * p.pose.orientation.y
                            ),
                            1.0
                            - 2.0
                            * (
                                p.pose.orientation.y * p.pose.orientation.y
                                + p.pose.orientation.z * p.pose.orientation.z
                            ),
                        )
                    )
                    for p in poses
                ],
                dtype=np.float32,
            )
            global_pts = np.stack([xs, ys, thetas], axis=1)
            self._cached_global_pts = global_pts
            return global_pts
        except Exception:
            return None

    def _on_timer(self) -> None:
        # Wait for an externally provided goal pose (RViz SetGoal).
        if self._goal_world is None:
            return

        if self._cached_path_msg is None:
            self._refresh_cached_messages()
            if self._cached_path_msg is None:
                return

        assert self._cached_path_msg is not None

        # Use a single timestamp for TF lookup + published messages.
        now_clock = self.get_clock().now()
        now = now_clock.to_msg()

        self._cached_path_msg.header.stamp = now
        for ps in self._cached_path_msg.poses:
            ps.header.stamp = now

        self.plan_pub.publish(self._cached_path_msg)

        # Publish robot-frame goal + local reference trajectory for MPPI consumption.
        robot_pose = self._lookup_robot_pose_map(Time.from_msg(now))
        if robot_pose is not None:
            rx, ry, ryaw = robot_pose
            gx, gy, gyaw = self._goal_world
            dxg = gx - rx
            dyg = gy - ry
            gxl, gyl = self._world_to_robot_xy(dx=dxg, dy=dyg, robot_yaw=ryaw)
            goal_local = (gxl, gyl, self._wrap_pi(float(gyaw) - ryaw))
            try:
                self.goal_local_pub.publish(self._pose_msg_goal_local(goal_local, now))
            except Exception:
                pass

            # Build global pts from cached msg to avoid recomputing planner
            try:
                global_pts = self._ensure_cached_global_pts()
                if global_pts is not None:
                    ref_local = self._build_local_ref(global_pts, robot_pose)
                    if ref_local.shape[0] >= 2:
                        self.ref_traj_local_pub.publish(self._path_msg_from_local(ref_local, now))
            except Exception:
                pass

            # Robot footprint visualization in map frame.
            if self._viz_manager is not None and self._viz_robot_cfg is not None:
                try:
                    robot_state = np.array(robot_pose[:3], dtype=np.float32).reshape((3, 1))
                    self._viz_manager.publish_visualization(
                        planner=self,
                        robot_state=robot_state,
                        robot_config=self._viz_robot_cfg,
                    )
                except Exception:
                    pass

        if not self._logged_first_publish:
            self._logged_first_publish = True
            self.get_logger().info(
                f"Published /plan ({len(self._cached_path_msg.poses)} poses)"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GlobalRefPathNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
