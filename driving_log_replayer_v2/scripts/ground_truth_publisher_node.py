#!/usr/bin/env python3

# Copyright (c) 2025 TIER IV.inc
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

import json
from pathlib import Path

import driving_log_replayer_v2.perception_eval_conversions as eval_conversions
import numpy as np
import rclpy
import ros2_numpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class GroundTruthPublisherNode(Node):
    """Publish ground truth point cloud from t4_dataset annotations."""

    CLOUD_DIM = 5
    TS_DIFF_THRESH = 75000

    def __init__(self) -> None:
        super().__init__("ground_truth_publisher_node")

        try:
            # Declare parameters
            self.declare_parameter("t4_dataset_path", "")
            self.declare_parameter("ground_label", [1])
            self.declare_parameter("obstacle_label", [4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 22, 23])
            self.declare_parameter("publish_ground_only", False)

            t4_dataset_path_str = self.get_parameter("t4_dataset_path").get_parameter_value().string_value
            if not t4_dataset_path_str:
                self.get_logger().error("t4_dataset_path parameter is empty")
                return

            t4_dataset_path = Path(t4_dataset_path_str)
            self._ground_label = self.get_parameter("ground_label").get_parameter_value().integer_array_value
            self._obstacle_label = self.get_parameter("obstacle_label").get_parameter_value().integer_array_value
            self._publish_ground_only = (
                self.get_parameter("publish_ground_only").get_parameter_value().bool_value
            )

            self.get_logger().info(f"Loading ground truth from: {t4_dataset_path}")
            self.get_logger().info(
                f"Publish mode: {'ground only' if self._publish_ground_only else 'obstacle only (default)'}"
            )

            # Load ground truth data
            self._ground_truth: dict[int, dict[str, np.ndarray]] = {}
            self._load_ground_truth(t4_dataset_path)

            # Create subscriber with BEST_EFFORT QoS to match publisher
            self._sub_pointcloud = self.create_subscription(
                PointCloud2,
                "/sensing/lidar/concatenated/pointcloud",
                self.pointcloud_cb,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=10,
                ),
            )

            # Create publisher
            self._pub_ground_truth = self.create_publisher(
                PointCloud2,
                "/driving_log_replayer_v2/ground_truth/pointcloud",
                10,
            )

            self.get_logger().info(
                f"Ground truth publisher initialized. Loaded {len(self._ground_truth)} frames."
            )
        except Exception as e:
            import traceback
            self.get_logger().error(
                f"Failed to initialize ground truth publisher node: {e}\n{traceback.format_exc()}"
            )
            # Initialize empty ground truth to prevent crashes
            self._ground_truth = {}
            self._ground_label = []
            self._obstacle_label = []
            self._publish_ground_only = False
            # Create publisher anyway so node doesn't crash
            self._pub_ground_truth = self.create_publisher(
                PointCloud2,
                "/driving_log_replayer_v2/ground_truth/pointcloud",
                10,
            )

    def _load_ground_truth(self, t4_dataset_path: Path) -> None:
        """Load ground truth point cloud data from t4_dataset."""
        # Load point cloud data
        sample_data_path = t4_dataset_path / "annotation" / "sample_data.json"
        if not sample_data_path.exists():
            self.get_logger().error(f"sample_data.json not found at {sample_data_path}")
            return

        self.get_logger().info(f"Loading sample_data.json from {sample_data_path}")
        with sample_data_path.open() as f:
            sample_data = json.load(f)
        self.get_logger().info(f"Loaded {len(sample_data)} samples from sample_data.json")
        sample_data = [d for d in sample_data if d["filename"].split(".")[-2] == "pcd"]
        self.get_logger().info(f"Filtered to {len(sample_data)} PCD samples")

        # Load gt annotation data
        lidar_seg_json_path = t4_dataset_path / "annotation" / "lidarseg.json"
        if not lidar_seg_json_path.exists():
            self.get_logger().error(f"lidarseg.json not found at {lidar_seg_json_path}")
            return

        with lidar_seg_json_path.open() as f:
            lidar_seg_data = json.load(f)

        token_to_seg_data = {}
        for annotation_data in lidar_seg_data:
            token_to_seg_data[annotation_data["sample_data_token"]] = annotation_data
        self.get_logger().info(f"Loaded {len(token_to_seg_data)} annotation entries from lidarseg.json")

        loaded_count = 0
        for sample in sample_data:
            raw_points_file_path = t4_dataset_path / sample["filename"]
            if not raw_points_file_path.exists():
                continue

            raw_points = np.fromfile(raw_points_file_path.as_posix(), dtype=np.float32)
            token = sample["token"]

            if token not in token_to_seg_data:
                continue

            annotation_file_path = t4_dataset_path / token_to_seg_data[token]["filename"]
            if not annotation_file_path.exists():
                continue

            labels = np.fromfile(annotation_file_path.as_posix(), dtype=np.uint8)
            points: np.ndarray = raw_points.reshape((-1, self.CLOUD_DIM))

            if points.shape[0] != labels.shape[0]:
                self.get_logger().warning(
                    f"Point cloud and label size mismatch for timestamp {sample['timestamp']}"
                )
                continue

            self._ground_truth[int(sample["timestamp"])] = {
                "points": points,
                "labels": labels,
            }
            loaded_count += 1

        self.get_logger().info(f"Successfully loaded {loaded_count} ground truth frames")

    def _get_gt_frame_ts(self, unix_time: int) -> int:
        """Get the closest ground truth frame timestamp."""
        if not self._ground_truth:
            return -1

        ts_itr = iter(self._ground_truth.keys())
        ret_ts: int = int(next(ts_itr))
        min_diff: int = abs(unix_time - ret_ts)

        for _ in range(1, len(self._ground_truth)):
            sample_ts = next(ts_itr)
            diff_time = abs(unix_time - sample_ts)
            if diff_time < min_diff:
                min_diff = diff_time
                ret_ts = sample_ts

        if min_diff > self.TS_DIFF_THRESH:
            return -1

        return ret_ts

    def _create_colored_pointcloud(
        self, points: np.ndarray, labels: np.ndarray, header: Header
    ) -> PointCloud2:
        """Create colored point cloud from points and labels."""
        # Filter points based on labels
        if self._publish_ground_only:
            # Only publish ground points
            mask = np.isin(labels, self._ground_label)
        else:
            # Publish obstacle points only (non-ground points)
            mask = np.isin(labels, self._obstacle_label)

        filtered_points = points[mask]
        filtered_labels = labels[mask]

        if len(filtered_points) == 0:
            # Return empty point cloud
            empty_pcd = np.zeros(
                0,
                dtype=[
                    ("x", np.float32),
                    ("y", np.float32),
                    ("z", np.float32),
                    ("r", np.uint8),
                    ("g", np.uint8),
                    ("b", np.uint8),
                ],
            )
            ros_pcd = ros2_numpy.msgify(PointCloud2, empty_pcd)
            ros_pcd.header = header
            return ros_pcd

        # Create colored point cloud
        num_points = len(filtered_points)
        pcd_data = np.zeros(
            num_points,
            dtype=[
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("r", np.uint8),
                ("g", np.uint8),
                ("b", np.uint8),
            ],
        )

        # Set positions
        pcd_data["x"] = filtered_points[:, 0]
        pcd_data["y"] = filtered_points[:, 1]
        pcd_data["z"] = filtered_points[:, 2]

        # Set colors based on labels
        # Ground: green, Obstacle: red
        for i, label in enumerate(filtered_labels):
            if label in self._ground_label:
                pcd_data["g"][i] = 255  # Green for ground
            elif label in self._obstacle_label:
                pcd_data["r"][i] = 255  # Red for obstacles

        ros_pcd = ros2_numpy.msgify(PointCloud2, pcd_data)
        ros_pcd.header = header
        return ros_pcd

    def pointcloud_cb(self, msg: PointCloud2) -> None:
        """Callback for point cloud subscription."""
        # Log first callback to confirm it's being called
        if not hasattr(self, "_first_callback_logged"):
            self.get_logger().info(
                f"First pointcloud callback received. Ground truth frames: {len(self._ground_truth) if hasattr(self, '_ground_truth') else 0}"
            )
            self._first_callback_logged = True

        try:
            # Check if publisher is initialized
            if not hasattr(self, "_pub_ground_truth") or self._pub_ground_truth is None:
                self.get_logger().warn("Publisher not initialized", throttle_duration_sec=5.0)
                return

            # Check if ground truth is loaded
            if not hasattr(self, "_ground_truth") or not self._ground_truth:
                self.get_logger().warn(
                    f"Ground truth not loaded. Loaded frames: {len(self._ground_truth) if hasattr(self, '_ground_truth') else 0}. Publishing empty point cloud.",
                    throttle_duration_sec=5.0,
                )
                # Publish empty point cloud even if ground truth is not loaded
                empty_pcd = np.zeros(
                    0,
                    dtype=[
                        ("x", np.float32),
                        ("y", np.float32),
                        ("z", np.float32),
                        ("r", np.uint8),
                        ("g", np.uint8),
                        ("b", np.uint8),
                    ],
                )
                ros_pcd = ros2_numpy.msgify(PointCloud2, empty_pcd)
                ros_pcd.header = msg.header
                self._pub_ground_truth.publish(ros_pcd)
                return

            header_timestamp_microsec: int = eval_conversions.unix_time_microsec_from_ros_msg(
                msg.header
            )

            gt_frame_ts = self._get_gt_frame_ts(header_timestamp_microsec)

            if gt_frame_ts < 0:
                # No ground truth found, publish empty point cloud
                self.get_logger().debug(
                    f"No ground truth found for timestamp {header_timestamp_microsec}. "
                    f"Available timestamps: {sorted(list(self._ground_truth.keys()))[:5]}...",
                    throttle_duration_sec=2.0,
                )
                empty_pcd = np.zeros(
                    0,
                    dtype=[
                        ("x", np.float32),
                        ("y", np.float32),
                        ("z", np.float32),
                        ("r", np.uint8),
                        ("g", np.uint8),
                        ("b", np.uint8),
                    ],
                )
                ros_pcd = ros2_numpy.msgify(PointCloud2, empty_pcd)
                ros_pcd.header = msg.header
                self._pub_ground_truth.publish(ros_pcd)
                return

            # Get ground truth point cloud
            gt_frame_cloud: np.ndarray = self._ground_truth[gt_frame_ts]["points"]
            gt_frame_label: np.ndarray = self._ground_truth[gt_frame_ts]["labels"]

            # Create colored point cloud
            colored_pcd = self._create_colored_pointcloud(gt_frame_cloud, gt_frame_label, msg.header)
            self._pub_ground_truth.publish(colored_pcd)
            self.get_logger().debug(
                f"Published ground truth point cloud for timestamp {header_timestamp_microsec}",
                throttle_duration_sec=1.0,
            )
        except Exception as e:
            import traceback
            self.get_logger().error(
                f"Error in pointcloud_cb: {e}\n{traceback.format_exc()}",
                throttle_duration_sec=5.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

