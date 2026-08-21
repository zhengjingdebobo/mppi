#!/usr/bin/env python

from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class CostBreakdownVisualizer(Node):
    """Visualize MPPI cost breakdown with a live matplotlib bar chart."""

    def __init__(self) -> None:
        super().__init__("mppi_cost_breakdown_viz", automatically_declare_parameters_from_overrides=True)

        if not self.has_parameter("cost_breakdown_topic"):
            self.declare_parameter("cost_breakdown_topic", "/mppi_cost_breakdown")
        if not self.has_parameter("plot_rate"):
            self.declare_parameter("plot_rate", 10.0)
        if not self.has_parameter("value_scale"):
            self.declare_parameter("value_scale", 1.0)
        if not self.has_parameter("max_height"):
            self.declare_parameter("max_height", 5.0)
        if not self.has_parameter("min_height"):
            self.declare_parameter("min_height", 0.01)
        if not self.has_parameter("bar_width"):
            self.declare_parameter("bar_width", 0.6)
        if not self.has_parameter("bar_spacing"):
            self.declare_parameter("bar_spacing", 0.4)
        if not self.has_parameter("show_values"):
            self.declare_parameter("show_values", True)
        if not self.has_parameter("title"):
            self.declare_parameter("title", "MPPI Cost Breakdown")

        self.cost_topic = str(self.get_parameter("cost_breakdown_topic").value)
        self.plot_rate = float(self.get_parameter("plot_rate").value)
        self.value_scale = float(self.get_parameter("value_scale").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.bar_width = float(self.get_parameter("bar_width").value)
        self.bar_spacing = float(self.get_parameter("bar_spacing").value)
        self.show_values = bool(self.get_parameter("show_values").value)
        self.title = str(self.get_parameter("title").value)

        if self.plot_rate <= 0.0:
            self.plot_rate = 5.0

        self._latest_values: Optional[np.ndarray] = None
        self._latest_keys: Optional[List[str]] = None
        self._bars = None
        self._value_text = []
        self._x = None

        try:
            import matplotlib.pyplot as plt
            import matplotlib.cm as cm
        except Exception as exc:
            raise RuntimeError(
                "matplotlib is required for cost_breakdown_viz. "
                "Install it in your ROS environment."
            ) from exc

        self._plt = plt
        self._cm = cm
        self._plt.ion()
        self._fig, self._ax = self._plt.subplots()
        self._ax.set_title(self.title)
        self._ax.set_ylabel("cost (scaled)")
        self._ax.grid(True, axis="y", alpha=0.3)
        self._ax.set_ylim(0.0, max(self.max_height, self.min_height))

        self.create_subscription(Float32MultiArray, self.cost_topic, self._cost_cb, 10)
        self.create_timer(1.0 / self.plot_rate, self._update_plot)

        self.get_logger().info(f"Cost breakdown visualizer listening on {self.cost_topic}")

    @staticmethod
    def _parse_keys(msg: Float32MultiArray, count: int) -> List[str]:
        if msg.layout.dim:
            label = msg.layout.dim[0].label or ""
            if label.startswith("keys:"):
                keys = [k for k in label[len("keys:") :].split(",") if k]
                if len(keys) == count:
                    return keys
        return [f"cost_{i}" for i in range(count)]

    def _cost_cb(self, msg: Float32MultiArray) -> None:
        values = np.asarray(msg.data, dtype=np.float32).reshape((-1,))
        if values.size == 0:
            return
        keys = self._parse_keys(msg, int(values.size))
        self._latest_values = values
        self._latest_keys = keys

    def _setup_bars(self, keys: List[str]) -> None:
        n = len(keys)
        step = self.bar_width + self.bar_spacing
        self._x = np.arange(n, dtype=np.float32) * step
        self._ax.clear()
        self._ax.set_title(self.title)
        self._ax.set_ylabel("cost (scaled)")
        self._ax.grid(True, axis="y", alpha=0.3)
        self._ax.set_ylim(0.0, max(self.max_height, self.min_height))
        self._ax.set_xticks(self._x)
        self._ax.set_xticklabels(keys, rotation=35, ha="right")
        self._bars = self._ax.bar(self._x, np.full(n, self.min_height), width=self.bar_width)
        self._value_text = []
        if self.show_values:
            for i in range(n):
                txt = self._ax.text(
                    self._x[i],
                    self.min_height * 0.5,
                    "",
                    ha="center",
                    va="center",
                    fontsize=8,
                    rotation=90,
                )
                self._value_text.append(txt)

    def _update_plot(self) -> None:
        if self._latest_values is None or self._latest_keys is None:
            self._plt.pause(0.001)
            return

        values = self._latest_values
        keys = self._latest_keys
        if self._bars is None or (len(keys) != len(self._bars)):
            self._setup_bars(keys)

        heights = np.clip(
            np.abs(values) * self.value_scale,
            self.min_height,
            self.max_height,
        )
        h_min = float(np.min(heights))
        h_max = float(np.max(heights))
        denom = max(h_max - h_min, 1e-6)

        for i, bar in enumerate(self._bars):
            bar.set_height(float(heights[i]))
            t = (float(heights[i]) - h_min) / denom
            color = self._cm.get_cmap("coolwarm")(t)
            bar.set_color(color)

        if self.show_values and len(self._value_text) == len(heights):
            for i, txt in enumerate(self._value_text):
                txt.set_text(f"{values[i]:.3f}")
                txt.set_position((self._x[i], float(heights[i]) * 0.5))

        self._fig.canvas.draw_idle()
        self._plt.pause(0.001)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CostBreakdownVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
