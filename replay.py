#!/usr/bin/env python3
"""Derived from DUARO-hanium/replay_final; separate CSVs, common start and final-pose hold."""

import csv
import math
import time

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint_1"]
DXL_RESOURCES = ["dxl11", "dxl12", "dxl13", "dxl14", "dxl15", "dxl16"]
VOLTAGE_VOLTS_PER_RAW = 0.1


class DualFollowerReplayer(Node):
    def __init__(self, **kwargs):
        super().__init__("omx_dual_follower_replayer", **kwargs)
        self.declare_parameter("input_file", "trajectory.csv")
        self.declare_parameter("metrics_file", "dual_replay_metrics.csv")
        self.declare_parameter("left_input_file", "")
        self.declare_parameter("right_input_file", "")
        self.declare_parameter("joint_prefixes", ["", ""])
        self.declare_parameter("start_delay_s", 5.0)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("confirm", True)
        self.input_file = self.get_parameter("input_file").value
        self.metrics_path = self.get_parameter("metrics_file").value
        self.files = [self.get_parameter(key).value or self.input_file
                      for key in ("left_input_file", "right_input_file")]
        self.prefixes = list(self.get_parameter("joint_prefixes").value)
        self.align = float(self.get_parameter("start_delay_s").value)
        self.speed = float(self.get_parameter("speed").value)
        if (len(self.prefixes) != 2 or not math.isfinite(self.align) or self.align <= 0.1
                or not math.isfinite(self.speed) or self.speed <= 0):
            raise ValueError("Need two joint_prefixes, start_delay_s > 0.1 and speed > 0")
        self.actual = {name: None for name in ("follower1", "follower2")}
        self.actual_times = {name: 0.0 for name in self.actual}
        self.sent = False
        self.finished = False
        self.stop_requested = lambda: False

        self.trajectory_publishers = [
            self.create_publisher(
                JointTrajectory, f"/{name}/replay_controller/joint_trajectory", 10
            )
            for name in ("follower1", "follower2")
        ]
        self.latest_states = {"follower1": None, "follower2": None}
        self.metrics_subscriptions = [
            self.create_subscription(
                DynamicJointState,
                f"/{name}/dynamic_joint_states",
                lambda msg, robot=name: self.state_callback(robot, msg),
                10,
            )
            for name in ("follower1", "follower2")
        ]
        self.actual_subscriptions = [self.create_subscription(
            JointState, f"/{name}/joint_states",
            lambda msg, name=name, prefix=prefix: self.actual_callback(name, prefix, msg),
            qos_profile_sensor_data)
            for name, prefix in zip(self.actual, self.prefixes)]
        self.metrics_file = None
        self.metrics_writer = None
        self.metrics_start = None
        self.metrics_count = 0
        self.timer = self.create_timer(1.0 / 30.0, self.metrics_callback)

    def actual_callback(self, robot, prefix, msg):
        values = dict(zip(msg.name, msg.position))
        names = [prefix + joint for joint in JOINTS]
        if all(name in values and math.isfinite(values[name]) for name in names):
            self.actual[robot] = [values[name] for name in names]
            self.actual_times[robot] = time.monotonic()

    def ready(self):
        return (all(pub.get_subscription_count() > 0 for pub in self.trajectory_publishers)
                and all(self.actual[name] is not None and time.monotonic() - stamp < 0.5
                        for name, stamp in self.actual_times.items())
                and all(states is not None and all(
                    states.get(resource, {}).get('Hardware Error Status') == 0
                    for resource in DXL_RESOURCES) for states in self.latest_states.values()))

    def state_callback(self, robot, msg):
        self.latest_states[robot] = {
            resource: dict(zip(interfaces.interface_names, interfaces.values))
            for resource, interfaces in zip(msg.joint_names, msg.interface_values)
        }
        for resource, interfaces in self.latest_states[robot].items():
            error = interfaces.get('Hardware Error Status', 0)
            if math.isfinite(error) and error:
                raise RuntimeError(f'{robot} {resource}: hardware error {error}')

    def start_metrics(self):
        self.metrics_file = open(self.metrics_path, "w", newline="")
        self.metrics_writer = csv.writer(self.metrics_file)
        header = ["time"]
        for robot in ("follower1", "follower2"):
            for joint in JOINTS:
                header.extend([
                    f"{robot}_{joint}_current_raw",
                    f"{robot}_{joint}_current_a",
                    f"{robot}_{joint}_voltage_v",
                    f"{robot}_{joint}_power_w",
                ])
            header.append(f"{robot}_total_power_w")
        self.metrics_writer.writerow(header)
        self.metrics_start = time.monotonic()

    def metrics_callback(self):
        if self.metrics_writer is None:
            return
        row = [time.monotonic() - self.metrics_start]
        for robot in ("follower1", "follower2"):
            states = self.latest_states[robot] or {}
            total_power = 0.0
            has_power = False
            for resource in DXL_RESOURCES:
                values = states.get(resource, {})
                current_raw = values.get("Present Current", math.nan)
                voltage_raw = values.get("Present Input Voltage", math.nan)
                current = current_raw * 0.001 if resource in ('dxl14', 'dxl15', 'dxl16') and math.isfinite(current_raw) else math.nan
                voltage = voltage_raw * VOLTAGE_VOLTS_PER_RAW if math.isfinite(voltage_raw) else math.nan
                power = voltage * current if math.isfinite(voltage) and math.isfinite(current) else math.nan
                row.extend([current_raw, current, voltage, power])
                if math.isfinite(power):
                    total_power += power
                    has_power = True
            row.append(total_power if has_power else math.nan)
        self.metrics_writer.writerow(row)
        self.metrics_count += 1
        if self.metrics_count % 30 == 0:
            self.metrics_file.flush()

    def load_trajectory(self, path, prefix):
        rows = []
        with open(path, "r", newline="") as file:
            for row in csv.DictReader(file):
                stamp, positions = float(row["time"]), [float(row[j]) for j in JOINTS]
                if not all(math.isfinite(value) for value in (stamp, *positions)) or stamp < 0:
                    raise ValueError(f"{path}: nonfinite/negative data")
                if rows and not 0 < stamp - rows[-1][0] <= 0.5:
                    raise ValueError(f"{path}: timestamps must increase with gaps <= 0.5s")
                rows.append((stamp, positions))
        if len(rows) < 2:
            raise RuntimeError(f"{path}: need at least two samples")
        first_time = rows[0][0]
        msg = JointTrajectory()
        msg.joint_names = [prefix + joint for joint in JOINTS]
        for stamp, positions in rows:
            relative = (stamp - first_time) / self.speed + self.align
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(relative)
            point.time_from_start.nanosec = int((relative - int(relative)) * 1_000_000_000)
            msg.points.append(point)
        return msg, [stamp for stamp, _ in rows]

    def hold(self):
        if not self.sent:
            return
        for robot, prefix, publisher in zip(self.actual, self.prefixes, self.trajectory_publishers):
            msg = JointTrajectory()
            msg.joint_names = [prefix + joint for joint in JOINTS]
            point = JointTrajectoryPoint()
            point.positions = self.actual[robot]
            point.time_from_start.nanosec = 200_000_000
            msg.points = [point]
            publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn("Replay interrupted; sent hold commands to both controllers.")

    def run(self):
        loaded = [self.load_trajectory(path, prefix) for path, prefix in zip(self.files, self.prefixes)]
        if loaded[0][1] != loaded[1][1]:
            raise ValueError("Left/right CSV timelines differ; use files from the same recording")
        duration = (loaded[0][1][-1] - loaded[0][1][0]) / self.speed
        self.get_logger().warn(
            f"Both followers will approach the first pose over {self.align}s, then replay."
        )
        if self.get_parameter("confirm").value and input("Type play to move both followers: ").strip() != "play":
            raise RuntimeError("Replay cancelled before sending commands")
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ready():
                break
        if self.stop_requested():
            return
        if not rclpy.ok() or not self.ready():
            raise RuntimeError("Need connected controllers and fresh joint states for both arms")
        if any(self.count_publishers(pub.topic_name) != 1 for pub in self.trajectory_publishers):
            raise RuntimeError("Another command publisher is active; stop teleoperation/ACT first")

        self.start_metrics()
        start = self.get_clock().now().nanoseconds + 1_000_000_000
        self.sent = True
        for robot, publisher, (trajectory, _) in zip(self.actual, self.trajectory_publishers, loaded):
            initial = JointTrajectoryPoint()
            initial.positions = list(self.actual[robot])
            initial.time_from_start.nanosec = 100_000_000
            trajectory.points.insert(0, initial)
            trajectory.header.stamp.sec = start // 1_000_000_000
            trajectory.header.stamp.nanosec = start % 1_000_000_000
            publisher.publish(trajectory)
        self.get_logger().info(f"Sent {len(trajectory.points)} points to both followers")

        end = time.monotonic() + 1.0 + self.align + duration + 0.5
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.stop_requested():
                self.hold()
                return
            if not self.ready():
                raise RuntimeError("Follower feedback/controller connection lost during replay")
        self.finished = True
        self.get_logger().info(
            f"Replay finished; saved {self.metrics_count} rows to {self.metrics_path}"
        )

    def destroy_node(self):
        if self.metrics_file is not None:
            self.metrics_file.flush()
            self.metrics_file.close()
        super().destroy_node()
