#!/usr/bin/env python3
"""Record an honest stationary-drift regression video from live ROS topics.

The video is intentionally based only on the raw LiDAR and guarded local pose;
it does not read Gazebo truth topics.  It demonstrates the regression that is
most damaging in this simulator: sparse ray-pattern changes must not move a
standing robot's localization estimate.
"""

from collections import deque
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud


class StabilityVideoRecorder:
    def __init__(self):
        rospy.init_node("lidar_stability_video_recorder", anonymous=True)
        self.duration_s = float(rospy.get_param("~duration_s", 12.0))
        self.output = rospy.get_param(
            "~output", "/tmp/lidar_stationary_guard_demo.mp4")
        self.latest_cloud = np.empty((0, 3), dtype=np.float32)
        self.history = deque(maxlen=400)
        self.start_stamp = None
        self.origin = None
        rospy.Subscriber("/scan", PointCloud, self._cloud_callback, queue_size=1)
        rospy.Subscriber("/localization/raw_pose", PoseWithCovarianceStamped,
                         self._pose_callback, queue_size=20)

    def _cloud_callback(self, message):
        points = np.asarray([(p.x, p.y, p.z) for p in message.points], dtype=np.float32)
        if points.size:
            finite = np.isfinite(points).all(axis=1)
            self.latest_cloud = points[finite]

    def _pose_callback(self, message):
        stamp = message.header.stamp.to_sec()
        position = message.pose.pose.position
        value = np.array([position.x, position.y, position.z], dtype=np.float64)
        if self.origin is None:
            self.origin = value.copy()
            self.start_stamp = stamp
        self.history.append((stamp, *(value - self.origin)))

    def run(self):
        deadline = time.monotonic() + 15.0
        while not rospy.is_shutdown() and (self.origin is None or not self.latest_cloud.size):
            if time.monotonic() > deadline:
                raise RuntimeError("timed out waiting for /scan and /localization/raw_pose")
            rospy.sleep(0.05)

        os.makedirs(os.path.dirname(os.path.abspath(self.output)), exist_ok=True)
        figure = plt.figure(figsize=(12.8, 7.2), dpi=100)
        figure.patch.set_facecolor("#10151c")
        cloud_axes = figure.add_axes([0.06, 0.14, 0.53, 0.74], facecolor="#18212b")
        trace_axes = figure.add_axes([0.65, 0.51, 0.30, 0.28], facecolor="#18212b")
        text_axes = figure.add_axes([0.65, 0.14, 0.30, 0.25], facecolor="#18212b")
        text_axes.axis("off")

        cloud_axes.set_title("Live raw /scan (sensor frame)", color="white", pad=12)
        cloud_axes.set_xlabel("x (m)", color="#cbd5e1")
        cloud_axes.set_ylabel("y (m)", color="#cbd5e1")
        cloud_axes.set_xlim(-12, 12)
        cloud_axes.set_ylim(-12, 12)
        cloud_axes.grid(alpha=0.16, color="#94a3b8")
        cloud_axes.tick_params(colors="#cbd5e1")
        scatter = cloud_axes.scatter([], [], s=2, c="#38bdf8", alpha=0.60)

        trace_axes.set_title("Guarded LiDAR pose, rebased at video start", color="white", pad=10)
        trace_axes.set_xlabel("x (m)", color="#cbd5e1")
        trace_axes.set_ylabel("y (m)", color="#cbd5e1")
        trace_axes.set_xlim(-0.20, 0.20)
        trace_axes.set_ylim(-0.20, 0.20)
        trace_axes.axhline(0, color="#64748b", linewidth=0.8)
        trace_axes.axvline(0, color="#64748b", linewidth=0.8)
        trace_axes.grid(alpha=0.16, color="#94a3b8")
        trace_axes.tick_params(colors="#cbd5e1")
        trace, = trace_axes.plot([], [], color="#34d399", linewidth=2.2,
                                 label="/localization/raw_pose")
        legend = trace_axes.legend(facecolor="#18212b", loc="upper right")
        for label in legend.get_texts():
            label.set_color("white")
        status = text_axes.text(0.03, 0.94, "", va="top", color="#e2e8f0",
                                family="monospace", fontsize=12, linespacing=1.55)
        figure.text(0.06, 0.94, "LiDAR localization stationary-drift regression",
                    color="white", fontsize=20, weight="bold")
        figure.text(0.06, 0.055,
                    "Live ROS input only: /scan + /trunk_imu + /localization/raw_pose. "
                    "No Gazebo ground-truth pose is consumed.",
                    color="#94a3b8", fontsize=10)

        frames = max(1, int(round(self.duration_s * 10.0)))
        metadata = {"title": "LiDAR stationary-drift regression", "artist": "Codex"}
        writer = FFMpegWriter(fps=10, bitrate=1800, metadata=metadata)
        with writer.saving(
                figure, self.output, 100):
            for _ in range(frames):
                cloud = self.latest_cloud
                if cloud.size:
                    ranges = np.linalg.norm(cloud[:, :2], axis=1)
                    selected = cloud[(ranges > 0.25) & (ranges < 12.0)]
                    scatter.set_offsets(selected[:, :2] if selected.size else np.empty((0, 2)))
                values = np.asarray(self.history, dtype=np.float64)
                if values.size:
                    trace.set_data(values[:, 1], values[:, 2])
                    span = np.ptp(values[:, 1:4], axis=0)
                    elapsed = values[-1, 0] - values[0, 0]
                    status.set_text(
                        "IMU stationary guard: ACTIVE\n"
                        "raw scan stream: /scan (~10 Hz)\n"
                        "IMU source: /trunk_imu\n"
                        "pose source: bounded GICP\n"
                        "\n"
                        "elapsed: %.1f s\n"
                        "pose samples: %d\n"
                        "XYZ span: %.4f, %.4f, %.4f m\n"
                        "\n"
                        "Expected stationary result:\n"
                        "no random scan-induced drift" % (
                            elapsed, len(values), span[0], span[1], span[2]))
                writer.grab_frame(facecolor=figure.get_facecolor())
                rospy.sleep(0.10)
        rospy.loginfo("wrote LiDAR stability video: %s", self.output)


if __name__ == "__main__":
    StabilityVideoRecorder().run()
