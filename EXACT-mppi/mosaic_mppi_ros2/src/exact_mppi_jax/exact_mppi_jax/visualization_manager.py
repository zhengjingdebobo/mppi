#!/usr/bin/env python

"""Visualization manager for EXACT MPPI ROS2 nodes.

Handles all visualization-related functionality independently from main planning logic.
Provides clean separation of concerns and optional visualization with minimal overhead.

"""

import numpy as np
from math import sin, cos

from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import MarkerArray, Marker
import rclpy

class VisualizationManager:
    """Manages visualization markers for the ROS2 bridge nodes.

    Responsibilities:
    - Create and publish optional debug point markers
    - Create and publish robot multi-polygon markers
    - Create and publish robot footprint markers
    - Handle enable/disable flags for each visualization type
    - Minimize data copying and lock contention

    """

    def __init__(self, node, config):
        """Initialize visualization manager.

        Args:
            node: ROS2 Node instance (for publishers, logger, clock)
            config: Configuration dictionary containing:
                - enable_visualization: bool - Master enable switch
                - enable_dune_markers: bool - Legacy debug point markers
                - enable_robot_marker: bool - Enable robot marker
                - enable_mosaic_marker: bool - Enable robot polygon markers
                - enable_rollout_markers: bool - Enable rollout trajectory markers
                - map_frame: str - Map frame ID
                - marker_size: float - Marker size
                - marker_z: float - Robot marker height
                - dune_markers_topic: str - Legacy debug marker topic name
                - robot_marker_topic: str - Robot marker topic name
                - mosaic_markers_topic: str - Robot polygon marker topic name
                - rollout_markers_topic: str - Rollout markers topic name
                - rollout_downsample: int - Downsample factor for rollout visualization
                - state_lock: threading.Lock - Shared state lock

        """
        self.node = node

        # Enable flags
        self.enable_visualization = config['enable_visualization']
        self.enable_dune_markers = config['enable_dune_markers']
        self.enable_robot_marker = config['enable_robot_marker']
        self.enable_mosaic_marker = config.get('enable_mosaic_marker', True)
        self.enable_rollout_markers = config.get('enable_rollout_markers', False)

        # Visualization parameters
        self.map_frame = config['map_frame']
        self.marker_size = config['marker_size']
        self.marker_z = config['marker_z']
        self.rollout_downsample = config.get('rollout_downsample', 10)  # Show every Nth rollout

        # Thread safety
        self._state_lock = config['state_lock']

        # Create publishers only if visualization is enabled
        if self.enable_visualization:
            self._create_publishers(config)
            self.node.get_logger().info("Visualization enabled")
            if self.enable_dune_markers:
                self.node.get_logger().info("  - Debug point markers: enabled")
            if self.enable_robot_marker:
                self.node.get_logger().info("  - Robot marker: enabled")
            if self.enable_mosaic_marker:
                self.node.get_logger().info("  - Robot polygon markers: enabled")
            if self.enable_rollout_markers:
                self.node.get_logger().info("  - Rollout markers: enabled")
        else:
            self.node.get_logger().info("Visualization disabled")

    def _create_publishers(self, config):
        """Create ROS2 publishers for visualization markers."""
        if self.enable_dune_markers:
            self.dune_markers_pub = self.node.create_publisher(
                MarkerArray,
                config['dune_markers_topic'],
                10
            )

        if self.enable_robot_marker:
            self.robot_marker_pub = self.node.create_publisher(
                Marker,
                config['robot_marker_topic'],
                10
            )

        if self.enable_mosaic_marker:
            self.mosaic_markers_pub = self.node.create_publisher(
                MarkerArray,
                config.get('mosaic_markers_topic', '/mosaic_markers'),
                10
            )

        if self.enable_rollout_markers:
            self.rollout_markers_pub = self.node.create_publisher(
                MarkerArray,
                config.get('rollout_markers_topic', '/rollout_markers'),
                10
            )

    def publish_visualization(self, planner, robot_state, robot_config=None, rollout_states=None):
        """Publish all enabled visualization markers.

        Args:
            planner: ROS2 bridge node or MPPI handler object
                     (may contain legacy dune_points debug markers)
            robot_state: Robot state numpy array (3, 1) [x, y, theta]
            robot_config: Optional robot shape config with vertices_list.
                          If None, tries to get from planner.robot
            rollout_states: Optional numpy array of rollout trajectories (K, T, 3)
                            where K is number of samples, T is time steps

        """
        # Early exit if visualization disabled
        if not self.enable_visualization:
            return

        # Lock only to copy needed data (minimize lock time)
        with self._state_lock:
            # Only copy data for enabled markers
            if self.enable_dune_markers:
                dune_points = (
                    planner.dune_points.copy()
                    if hasattr(planner, 'dune_points') and planner.dune_points is not None else None
                )
            else:
                dune_points = None

            if self.enable_robot_marker or self.enable_mosaic_marker:
                robot_state_copy = (
                    robot_state.copy()
                    if robot_state is not None else None
                )
                # Use provided robot_config, or try to get from planner
                if robot_config is None:
                    robot_config = getattr(planner, 'robot', None)
            else:
                robot_state_copy = None

        # Generate and publish markers (all outside lock)
        if self.enable_dune_markers and dune_points is not None:
            dune_markers = self._generate_dune_markers(dune_points)
            if dune_markers is not None:
                self.dune_markers_pub.publish(dune_markers)

        if self.enable_robot_marker and robot_state_copy is not None:
            robot_marker = self._generate_robot_marker(
                robot_state_copy, robot_config
            )
            if robot_marker is not None:
                self.robot_marker_pub.publish(robot_marker)

        if self.enable_mosaic_marker and robot_state_copy is not None:
            mosaic_markers = self._generate_mosaic_markers(
                robot_state_copy, robot_config
            )
            if mosaic_markers is not None:
                self.mosaic_markers_pub.publish(mosaic_markers)

        if self.enable_rollout_markers and rollout_states is not None:
            rollout_markers = self._generate_rollout_markers(rollout_states)
            if rollout_markers is not None:
                self.rollout_markers_pub.publish(rollout_markers)

    def _generate_dune_markers(self, dune_points):
        """Generate legacy debug point visualization markers.

        Args:
            dune_points: numpy array of debug points (2, n)

        Returns:
            MarkerArray with debug point markers

        """
        if dune_points is None:
            return None

        marker_array = MarkerArray()
        timestamp = self.node.get_clock().now().to_msg()

        for index, point in enumerate(dune_points.T):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = timestamp

            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size
            marker.color.a = 1.0

            # Purple color for debug points
            marker.color.r = 160 / 255
            marker.color.g = 32 / 255
            marker.color.b = 240 / 255

            marker.id = index
            marker.type = Marker.CUBE
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = 0.3
            marker.pose.orientation = Quaternion()

            marker_array.markers.append(marker)

        return marker_array

    def _generate_robot_marker(self, robot_state, robot_config):
        """Generate robot footprint visualization marker.

        Args:
            robot_state: numpy array [x, y, theta] (3, 1)
            robot_config: robot configuration object
                          Preferred path: object with vertices_list and wheelbase
                          Legacy fallback: object with shape, length, width, wheelbase

        Returns:
            Marker representing robot footprint

        """
        if robot_state is None or robot_config is None:
            return None

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.node.get_clock().now().to_msg()

        marker.color.a = 0.7
        # Green color for robot
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.id = 0

        x = float(robot_state[0, 0])
        y = float(robot_state[1, 0])
        theta = float(robot_state[2, 0])

        # Preferred shape config path: object with vertices_list.
        # Legacy fallback: object with shape/length/width.
        if hasattr(robot_config, 'vertices_list') and robot_config.vertices_list:
            # Compute bounding box from polygon vertices.
            vertices_list = robot_config.vertices_list
            all_vertices = []
            for polygon in vertices_list:
                all_vertices.extend(polygon)
            
            if all_vertices:
                xs = [v[0] for v in all_vertices]
                ys = [v[1] for v in all_vertices]
                length = max(xs) - min(xs)
                width = max(ys) - min(ys)
                # Center offset
                center_x = (max(xs) + min(xs)) / 2
                center_y = (max(ys) + min(ys)) / 2
            else:
                length = 1.0
                width = 1.0
                center_x = 0.0
                center_y = 0.0

            wheelbase = getattr(robot_config, 'wheelbase', length * 0.8)

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z
            marker.type = Marker.CUBE

            # Transform center to world frame
            marker_x = x + cos(theta) * center_x - sin(theta) * center_y
            marker_y = y + sin(theta) * center_x + cos(theta) * center_y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0.0
            marker.pose.orientation = self._yaw_to_quat(theta)

        elif hasattr(robot_config, 'shape') and robot_config.shape == "rectangle":
            # Legacy rectangular robot config
            length = robot_config.length
            width = robot_config.width
            wheelbase = robot_config.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z

            marker.type = Marker.CUBE

            # Adjust position for Ackermann kinematics
            if hasattr(robot_config, 'kinematics') and robot_config.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0.0
            marker.pose.orientation = self._yaw_to_quat(theta)
        else:
            # Fallback: simple cube at robot position
            marker.scale.x = 1.0
            marker.scale.y = 0.5
            marker.scale.z = self.marker_z
            marker.type = Marker.CUBE
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation = self._yaw_to_quat(theta)

        return marker

    def _generate_mosaic_markers(self, robot_state, robot_config):
        """Generate robot multi-polygon visualization markers.

        The robot is represented as multiple polygons (blocks),
        each transformed according to the robot's current pose.

        Args:
            robot_state: numpy array [x, y, theta] (3, 1)
            robot_config: robot configuration object with vertices_list

        Returns:
            MarkerArray containing LINE_STRIP markers for each polygon

        """
        if robot_state is None or robot_config is None:
            return None

        # Get vertices list from robot config
        vertices_list = getattr(robot_config, 'vertices_list', None)
        if vertices_list is None or len(vertices_list) == 0:
            return None

        marker_array = MarkerArray()
        timestamp = self.node.get_clock().now().to_msg()

        # Robot pose
        x = float(robot_state[0, 0])
        y = float(robot_state[1, 0])
        theta = float(robot_state[2, 0])

        # Rotation matrix for transforming local vertices to world frame
        cos_t = cos(theta)
        sin_t = sin(theta)

        # Color palette for different polygons (cycling through colors)
        colors = [
            (0.0, 0.8, 0.2, 0.9),    # Green
            (0.2, 0.6, 1.0, 0.9),    # Blue
            (1.0, 0.5, 0.0, 0.9),    # Orange
            (0.8, 0.2, 0.8, 0.9),    # Purple
            (1.0, 1.0, 0.0, 0.9),    # Yellow
            (0.0, 1.0, 1.0, 0.9),    # Cyan
        ]

        for idx, polygon_vertices in enumerate(vertices_list):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = timestamp
            marker.ns = "mosaic_model"
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Line width
            marker.scale.x = 0.05  # Line thickness

            # Color (cycle through palette)
            color = colors[idx % len(colors)]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]

            # Transform each vertex from local frame to world frame
            # polygon_vertices is a list of [x, y] pairs
            for vertex in polygon_vertices:
                local_x = vertex[0]
                local_y = vertex[1]

                # Apply rotation and translation
                world_x = x + cos_t * local_x - sin_t * local_y
                world_y = y + sin_t * local_x + cos_t * local_y

                point = Point()
                point.x = world_x
                point.y = world_y
                point.z = 0.1  # Slightly above ground for visibility
                marker.points.append(point)

            # Close the polygon by adding the first point again
            if len(polygon_vertices) > 0:
                first_vertex = polygon_vertices[0]
                local_x = first_vertex[0]
                local_y = first_vertex[1]
                world_x = x + cos_t * local_x - sin_t * local_y
                world_y = y + sin_t * local_x + cos_t * local_y

                point = Point()
                point.x = world_x
                point.y = world_y
                point.z = 0.1
                marker.points.append(point)

            marker.pose.orientation.w = 1.0  # Identity quaternion
            marker_array.markers.append(marker)

        return marker_array

    def _generate_rollout_markers(self, rollout_states):
        """Generate rollout trajectory visualization markers.
        
        Visualizes the MPPI sampled rollout trajectories as semi-transparent lines.
        Uses downsampling to reduce visual clutter and improve performance.

        Args:
            rollout_states: numpy array of shape (K, T, 3) where
                           K = number of samples
                           T = time steps (horizon)
                           3 = [x, y, theta]

        Returns:
            MarkerArray containing LINE_STRIP markers for each rollout

        """
        if rollout_states is None or len(rollout_states) == 0:
            return None

        marker_array = MarkerArray()
        timestamp = self.node.get_clock().now().to_msg()

        K = rollout_states.shape[0]  # Number of samples
        T = rollout_states.shape[1]  # Time steps

        # Downsample to reduce visual clutter
        downsample = max(1, self.rollout_downsample)
        selected_indices = list(range(0, K, downsample))

        # First, add a DELETE_ALL marker to clear previous rollouts
        # Use a separate namespace for the delete marker to avoid id conflict
        delete_marker = Marker()
        delete_marker.header.frame_id = self.map_frame
        delete_marker.header.stamp = timestamp
        delete_marker.ns = "rollout_trajectories"
        delete_marker.id = -1  # Use -1 to avoid conflict with trajectory markers
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for marker_id, k in enumerate(selected_indices):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = timestamp
            marker.ns = "rollout_trajectories"
            marker.id = marker_id + 1  # Start from 1 to avoid conflict with delete marker
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Line width
            marker.scale.x = 0.03  # Thin lines for rollouts

            # Semi-transparent color based on position in the sample set
            # Use gradient from blue to red to show distribution
            ratio = k / max(K - 1, 1)
            marker.color.r = float(ratio)
            marker.color.g = 0.2
            marker.color.b = float(1.0 - ratio)
            marker.color.a = 0.4  # Semi-transparent

            # Add trajectory points
            for t in range(T):
                point = Point()
                point.x = float(rollout_states[k, t, 0])
                point.y = float(rollout_states[k, t, 1])
                point.z = 0.05  # Slightly above ground
                marker.points.append(point)

            marker.pose.orientation.w = 1.0
            marker_array.markers.append(marker)

        return marker_array


    @staticmethod
    def _yaw_to_quat(yaw):
        """Convert yaw angle to quaternion.

        Args:
            yaw: Yaw angle in radians

        Returns:
            Quaternion message

        """
        quater = Quaternion()
        quater.x = 0.0
        quater.y = 0.0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)
        return quater
