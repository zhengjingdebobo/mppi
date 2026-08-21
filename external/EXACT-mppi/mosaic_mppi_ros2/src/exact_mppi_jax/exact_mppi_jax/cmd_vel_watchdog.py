#!/usr/bin/env python

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile


class CmdVelWatchdogNode(Node):
    """Forwards MPPI velocity commands and publishes zero on timeout.

    Inputs and outputs are intentionally unchanged from the existing launch files:
      - subscribes to `input_cmd_vel_topic` (default: `/mppi_cmd_vel`)
      - publishes to `output_cmd_vel_topic` (default: `/cmd_vel`)
    """

    def __init__(self) -> None:
        super().__init__(
            "cmd_vel_watchdog",
            automatically_declare_parameters_from_overrides=True,
        )

        input_topic = str(self._get_param_value("input_cmd_vel_topic", "/mppi_cmd_vel"))
        output_topic = str(self._get_param_value("output_cmd_vel_topic", "/cmd_vel"))
        timeout_s = float(self._get_param_value("timeout_s", 0.3))
        publish_frequency = float(self._get_param_value("publish_frequency", 30.0))

        self.timeout_s = max(timeout_s, 0.0)
        self.publish_period = 1.0 / max(publish_frequency, 1.0)
        self._lock = threading.Lock()
        self._last_msg: Optional[Twist] = None
        self._last_rx_time: Optional[float] = None
        self._timed_out = False

        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, output_topic, qos)
        self.cmd_sub = self.create_subscription(
            Twist,
            input_topic,
            self._cmd_cb,
            qos,
        )
        self.timer = self.create_timer(self.publish_period, self._on_timer)

        self.get_logger().info(
            f"cmd_vel_watchdog forwarding {input_topic} -> {output_topic} with timeout={self.timeout_s:.3f}s"
        )

    def _get_param_value(self, name: str, default):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return default

    def _cmd_cb(self, msg: Twist) -> None:
        now = self.get_clock().now().nanoseconds * 1.0e-9
        with self._lock:
            copied = Twist()
            copied.linear.x = msg.linear.x
            copied.linear.y = msg.linear.y
            copied.linear.z = msg.linear.z
            copied.angular.x = msg.angular.x
            copied.angular.y = msg.angular.y
            copied.angular.z = msg.angular.z
            self._last_msg = copied
            self._last_rx_time = now
            self._timed_out = False

        self.cmd_pub.publish(copied)

    def _on_timer(self) -> None:
        if self.timeout_s <= 0.0:
            return

        now = self.get_clock().now().nanoseconds * 1.0e-9
        publish_zero = False
        with self._lock:
            if self._last_rx_time is None:
                publish_zero = True
            elif (now - self._last_rx_time) > self.timeout_s:
                publish_zero = True

            if publish_zero and self._timed_out:
                return
            if publish_zero:
                self._timed_out = True

        if publish_zero:
            self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelWatchdogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
