# Copyright (c) 2026 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Opt-in DDS test: real recorder, remapped subscription, and message contents."""

import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

from driving_log_replayer_v2.evaluation_topic import CONFIG_KEY
from driving_log_replayer_v2.evaluation_topic import DEFAULT_POINTCLOUD_TOPIC
from driving_log_replayer_v2.evaluation_topic import EvaluationTopic

pytestmark = pytest.mark.skipif(
    os.environ.get("DLR_RUN_ROS_INTEGRATION") != "1",
    reason="Set DLR_RUN_ROS_INTEGRATION=1 on a host with DDS networking",
)

EXPECTED_TARGET_SUBSCRIBERS = 2
SELECTED_POINT_X = 2.0

RECORDER = """
import json, sys
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction
from driving_log_replayer_v2.launch.rosbag import launch_bag_recorder
service = LaunchService()
service.context.launch_configurations.update(json.loads(sys.argv[1]))
service.include_launch_description(LaunchDescription([OpaqueFunction(function=launch_bag_recorder)]))
sys.exit(service.run())
"""


def cloud(value: float) -> PointCloud2:
    msg = PointCloud2(height=1, width=1, point_step=12, row_step=12)
    msg.header.frame_id = "base_link"
    msg.header.stamp.sec = 100
    msg.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    msg.data = struct.pack("fff", value, 0.0, 0.0)
    return msg


@pytest.mark.parametrize("monitored", [False, True])
def test_recording_selected_topic(  # noqa: PLR0915 - Keep ROS resource cleanup with setup.
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, *, monitored: bool
) -> None:
    monkeypatch.setenv("ROS_DOMAIN_ID", "187")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "logs"))
    topic = "/perception/radar_lidar_fusion/debug/appended_radar_points"
    target = EvaluationTopic(
        topic=topic,
        topic_rate={
            "module": "perception",
            "node_name_suffix": "test_existing",
        }
        if monitored
        else None,
    )
    bag = tmp_path / "bag"
    conf = {
        "use_case": "obstacle_segmentation",
        "storage": "mcap",
        "result_bag_path": str(bag),
        "override_topics_regex": f"^{DEFAULT_POINTCLOUD_TOPIC}$",
        CONFIG_KEY: target.model_dump_json(),
    }
    rclpy.init()
    node = rclpy.create_node(
        "evaluation_topic_test",
        cli_args=[
            "--ros-args",
            "-r",
            f"{DEFAULT_POINTCLOUD_TOPIC}:={topic}",
        ],
        use_global_arguments=False,
    )
    received = []
    node.create_subscription(
        PointCloud2, DEFAULT_POINTCLOUD_TOPIC, received.append, qos_profile_sensor_data
    )
    # Explicitly create publishers with no remap so the original and target stay distinct.
    publisher_node = rclpy.create_node("evaluation_topic_publishers", use_global_arguments=False)
    original = publisher_node.create_publisher(
        PointCloud2, DEFAULT_POINTCLOUD_TOPIC, qos_profile_sensor_data
    )
    selected = publisher_node.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
    unrelated = publisher_node.create_publisher(
        PointCloud2, "/not_selected", qos_profile_sensor_data
    )
    clock = publisher_node.create_publisher(Clock, "/clock", 10)
    diagnostics = publisher_node.create_publisher(DiagnosticArray, "/diagnostics", 10)
    with (tmp_path / "recorder.log").open("w") as log:
        process = subprocess.Popen(
            [sys.executable, "-c", RECORDER, json.dumps(conf)],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                clock.publish(Clock(clock=cloud(0.0).header.stamp))
                rclpy.spin_once(node, timeout_sec=0.1)
                assert process.poll() is None, (tmp_path / "recorder.log").read_text()
                if (
                    selected.get_subscription_count() >= EXPECTED_TARGET_SUBSCRIBERS
                    and original.get_subscription_count() >= 1
                ):
                    break
            assert selected.get_subscription_count() >= EXPECTED_TARGET_SUBSCRIBERS
            assert original.get_subscription_count() >= 1
            for _ in range(20):
                clock.publish(Clock(clock=cloud(0.0).header.stamp))
                original.publish(cloud(1.0))
                selected.publish(cloud(SELECTED_POINT_X))
                unrelated.publish(cloud(3.0))
                if monitored:
                    msg = DiagnosticArray()
                    msg.header.stamp.sec = 100
                    msg.status = [DiagnosticStatus(name=target.topic_rate.diagnostic_name)]
                    diagnostics.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.1)
            assert received
            assert all(
                struct.unpack_from("f", bytes(msg.data))[0] == SELECTED_POINT_X for msg in received
            )
            process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
            assert process.returncode == 0, (tmp_path / "recorder.log").read_text()
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            node.destroy_node()
            publisher_node.destroy_node()
            rclpy.shutdown()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    values = {DEFAULT_POINTCLOUD_TOPIC: [], topic: []}
    diagnostic_count = 0
    while reader.has_next():
        name, raw, _ = reader.read_next()
        assert name in {DEFAULT_POINTCLOUD_TOPIC, topic, "/diagnostics"}
        if name == "/diagnostics":
            diagnostic_count += 1
        else:
            msg = deserialize_message(raw, PointCloud2)
            values[name].append(struct.unpack_from("f", bytes(msg.data))[0])
    assert values[DEFAULT_POINTCLOUD_TOPIC]
    assert set(values[DEFAULT_POINTCLOUD_TOPIC]) == {1.0}
    assert values[topic]
    assert set(values[topic]) == {SELECTED_POINT_X}
    assert bool(diagnostic_count) is monitored
