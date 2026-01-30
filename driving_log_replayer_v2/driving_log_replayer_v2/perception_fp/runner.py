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


from __future__ import annotations

import argparse
from typing import Any
from typing import TYPE_CHECKING

from autoware_perception_msgs.msg import DetectedObjects
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import TrackedObjects
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
import numpy as np
from perception_eval.common.object import DynamicObject
from perception_eval.config import PerceptionEvaluationConfig
from rclpy.time import Duration
from rosbag2_py import TopicMetadata
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from driving_log_replayer_v2.ground_segmentation.runner import convert_to_ground_seg
from driving_log_replayer_v2.perception.evaluator import PerceptionInvalidReason
from driving_log_replayer_v2.perception.runner import convert_to_perception_eval
from driving_log_replayer_v2.perception_fp.evaluation_manager import PerceptionFPEvaluationManager
from driving_log_replayer_v2.perception_fp.models import PerceptionFPData
from driving_log_replayer_v2.perception_fp.models import PerceptionFPResult
from driving_log_replayer_v2.perception_fp.models import PerceptionFPScenario
from driving_log_replayer_v2.perception_fp.models import Polygon3D
from driving_log_replayer_v2.post_process.ros2_utils import convert_to_homogeneous_matrix
from driving_log_replayer_v2.post_process.ros2_utils import lookup_transform
from driving_log_replayer_v2.post_process.runner import ConvertedData
from driving_log_replayer_v2.post_process.runner import Runner
from driving_log_replayer_v2.post_process.runner import UseCaseInfo

if TYPE_CHECKING:
    from driving_log_replayer_v2.post_process.evaluator import FrameResult
    from driving_log_replayer_v2.result import ResultWriter


def convert_polygon3d_to_ros_msg_in_base_link(
    header: Header,
    namespace: str,
    polygon: Polygon3D,
    marker_id: int,
    color: ColorRGBA,
    max_z: float,
    min_z: float,
) -> Marker:
    """Convert Polygon3D to ROS message."""
    mesh_marker = Marker(
        header=header,
        type=Marker.TRIANGLE_LIST,
        action=Marker.ADD,
        ns=namespace,
        id=marker_id,
        color=color,
        scale=Vector3(x=1.0, y=1.0, z=1.0),
        lifetime=Duration(seconds=0.2).to_msg(),
    )

    boundary_coords = list(polygon.exterior.coords)

    xyz: list[tuple[float, float, float]] = [
        (float(x), float(y), float(z)) for x, y, z in boundary_coords
    ]

    z_off_min = float(polygon.z_min) if polygon.z_min is not None else float(min_z)
    z_off_max = float(polygon.z_max) if polygon.z_max is not None else float(max_z)

    bottom = [Point(x=x, y=y, z=z + z_off_min) for x, y, z in xyz]
    top = [Point(x=x, y=y, z=z + z_off_max) for x, y, z in xyz]

    # bottom face
    for i in range(1, len(bottom) - 1):
        mesh_marker.points.extend([bottom[0], bottom[i], bottom[i + 1]])

    # top face
    for i in range(1, len(top) - 1):
        mesh_marker.points.extend([top[0], top[i], top[i + 1]])

    # side faces
    n = len(xyz)
    for i in range(n):
        j = (i + 1) % n
        mesh_marker.points.extend([bottom[i], top[i], top[j]])
        mesh_marker.points.extend([bottom[i], top[j], bottom[j]])

    return mesh_marker


def convert_non_detection_area_to_ros_msg(
    header: Header,
    data: dict[str, Polygon3D],
) -> MarkerArray:
    """Convert non-detection area data to ROS message."""
    # The value if z_min and z_max are not set in the Polygon3D
    max_z = 10.0
    min_z = -10.0
    namespace = "non_detection_area"
    header.frame_id = "base_link"
    color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)

    non_detection_area_marker = MarkerArray()
    for idx, (criterion_name, polygon_3d) in enumerate(data.items()):
        # add polygon marker
        if polygon_3d.frame_id != "base_link":
            polygon_3d.convert_map_to_base_link()
        non_detection_area_marker.markers.append(
            convert_polygon3d_to_ros_msg_in_base_link(
                header, namespace, polygon_3d, idx * 2, color, max_z, min_z
            )
        )

        # text z aligned to polygon's z definition
        coords = np.asarray(polygon_3d.exterior.coords, dtype=float)
        center_z = float(np.mean(coords[:, 2]))
        z_off_min = float(polygon_3d.z_min) if polygon_3d.z_min is not None else float(min_z)
        z_off_max = float(polygon_3d.z_max) if polygon_3d.z_max is not None else float(max_z)
        text_z = center_z + 0.5 * (z_off_min + z_off_max)

        # add text marker
        non_detection_area_marker.markers.append(
            Marker(
                header=header,
                ns=namespace + "_text",
                id=idx * 2 + 1,
                type=Marker.TEXT_VIEW_FACING,
                action=Marker.ADD,
                lifetime=Duration(seconds=0.2).to_msg(),
                pose=Pose(
                    position=Point(
                        x=polygon_3d.centroid.x, y=polygon_3d.centroid.y, z=text_z
                    ),  # above ground
                    orientation=Quaternion(w=1.0),  # no rotation because text faces viewer
                ),
                scale=Vector3(z=0.8),
                color=color,
                text=criterion_name,
            )
        )
    return non_detection_area_marker


def convert_fp_objects_to_ros_msg(header: Header, data: dict[str, PerceptionFPData]) -> MarkerArray:
    """Convert false positive objects data to ROS message."""
    namespace = "fp_objects"
    color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.3)

    fp_objects_marker = MarkerArray()
    for idx, (_, objects) in enumerate(data.items()):
        for obj_idx, obj in enumerate(objects):
            if isinstance(obj, DynamicObject):
                position = Point(
                    x=obj.state.position[0],
                    y=obj.state.position[1],
                    z=obj.state.position[2],
                )
                orientation = Quaternion(
                    x=obj.state.orientation[0],
                    y=obj.state.orientation[1],
                    z=obj.state.orientation[2],
                    w=obj.state.orientation[3],
                )
            elif isinstance(obj, np.ndarray):
                position = Point(x=float(obj[0]), y=float(obj[1]), z=float(obj[2]))
                orientation = Quaternion(w=1.0)  # no rotation

            # add sphere marker for each false positive object
            fp_objects_marker.markers.append(
                Marker(
                    header=header,
                    ns=namespace,
                    id=idx * 1000 + obj_idx,
                    type=Marker.SPHERE,
                    action=Marker.ADD,
                    lifetime=Duration(seconds=0.2).to_msg(),
                    pose=Pose(
                        position=position,
                        orientation=orientation,
                    ),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=color,
                )
            )
    return fp_objects_marker


class PerceptionFPRunner(Runner):
    def __init__(
        self,
        scenario_path: str,
        rosbag_dir_path: str,
        t4_dataset_path: str,
        result_jsonl_path: str,
        result_archive_path: str,
        storage: str,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        enable_analysis: str,
    ) -> None:
        super().__init__(
            PerceptionFPScenario,
            scenario_path,
            rosbag_dir_path,
            t4_dataset_path,
            result_jsonl_path,
            result_archive_path,
            storage,
            evaluation_topics_with_task,
            degradation_topic,
            enable_analysis,
        )

    def _get_use_case_info_list(
        self,
        scenario: PerceptionFPScenario,
        evaluation_topics_with_task: dict[str, list[str]],
        degradation_topic: str,
        result_jsonl_path: str,
        result_archive_path: str,
    ) -> list[UseCaseInfo]:
        _ = result_archive_path  # unused
        return [
            UseCaseInfo(
                evaluation_manager_class=PerceptionFPEvaluationManager,
                result_class=PerceptionFPResult,
                conditions=scenario.Evaluation.Conditions,
                name="perception_fp",
                evaluation_topics_with_task=evaluation_topics_with_task,
                degradation_topic=degradation_topic,
                result_jsonl_path=result_jsonl_path,
            )
        ]

    def _get_external_record_topics(self) -> list[TopicMetadata]:
        return [
            TopicMetadata(
                name="/driving_log_replayer_v2/perception_fp/results",
                type="std_msgs/msg/String",
                serialization_format="cdr",
                offered_qos_profiles="",
            ),
            TopicMetadata(
                name="/driving_log_replayer_v2/perception_fp/non_detection_area",
                type="visualization_msgs/msg/MarkerArray",
                serialization_format="cdr",
                offered_qos_profiles="",
            ),
            TopicMetadata(
                name="/driving_log_replayer_v2/perception_fp/fp_objects",
                type="visualization_msgs/msg/MarkerArray",
                serialization_format="cdr",
                offered_qos_profiles="",
            ),
        ]

    @property
    def perception_fp_eval_manager(self) -> PerceptionFPEvaluationManager:
        return self._use_cases["perception_fp"].evaluation_manager

    @property
    def perception_fp_result(self) -> PerceptionFPResult:
        return self._use_cases["perception_fp"].result

    @property
    def perception_fp_result_writer(self) -> ResultWriter[PerceptionFPResult]:
        return self._use_cases["perception_fp"].result_writer

    def _convert_ros_msg_to_data(
        self,
        topic_name: str,
        msg: Any,
        subscribed_timestamp_nanosec: int,
    ) -> ConvertedData:
        _ = topic_name  # unused
        if isinstance(msg, (DetectedObjects, PredictedObjects, TrackedObjects)):
            perception_eval_config = PerceptionEvaluationConfig(
                frame_id=msg.header.frame_id,
                evaluation_config_dict={
                    "target_labels": ["car", "truck", "bus", "bicycle", "motorcycle", "pedestrian"],
                    # dummy values below
                    "evaluation_task": "tracking",
                    "label_prefix": "autoware",
                    "merge_similar_labels": False,
                    "matching_label_policy": "default",
                    "max_x_position": 0.0,
                    "max_y_position": 0.0,
                },
                # dummy values below
                dataset_paths=[""],
                result_root_directory="dummy",
            )
            return convert_to_perception_eval(
                msg,
                subscribed_timestamp_nanosec,
                perception_eval_config,
            )
        if isinstance(msg, PointCloud2):
            return convert_to_ground_seg(msg, subscribed_timestamp_nanosec)

        err_msg = f"Unsupported message type: {type(msg)} for topic: {topic_name}"
        raise ValueError(err_msg)

    def _write_result(
        self,
        frame_result: FrameResult,
        header: Header,
        subscribed_timestamp_nanosec: int,
    ) -> None:
        if frame_result.is_valid:
            map_to_base_link: TransformStamped = lookup_transform(
                self._rosbag_manager.get_tf_buffer(),
                header.stamp,
                "base_link",
                "map",
            )
            homogeneous_matrix = convert_to_homogeneous_matrix(map_to_base_link)
            timestamp_sec = header.stamp.sec + header.stamp.nanosec * 1e-9
            self.perception_fp_result.set_frame(
                timestamp_sec,
                header.frame_id,
                frame_result.data,
                frame_result.skip_counter,
                homogeneous_matrix,
            )
        elif frame_result.invalid_reason == PerceptionInvalidReason.INVALID_ESTIMATED_OBJECTS:
            self.perception_fp_result.set_warn_frame(frame_result.data, frame_result.skip_counter)

        res_str = self.perception_fp_result_writer.write_result_with_time(
            self.perception_fp_result, subscribed_timestamp_nanosec
        )
        self._rosbag_manager.write_results(
            "/driving_log_replayer_v2/perception_fp/results",
            String(data=res_str),
            header.stamp,
        )

        # copy header because modifying frame_id to visualize the non detection area
        marker_header = Header()
        marker_header.stamp = header.stamp
        marker_header.frame_id = header.frame_id

        marker_non_detection_area = convert_non_detection_area_to_ros_msg(
            marker_header,
            self.perception_fp_result.get_non_detection_area(),
        )
        self._rosbag_manager.write_results(
            "/driving_log_replayer_v2/perception_fp/non_detection_area",
            marker_non_detection_area,
            marker_header.stamp,
        )

        marker_fp_objects = convert_fp_objects_to_ros_msg(
            header,
            self.perception_fp_result.get_fp_objects(),
        )
        self._rosbag_manager.write_results(
            "/driving_log_replayer_v2/perception_fp/fp_objects",
            marker_fp_objects,
            header.stamp,
        )

    def _evaluate_on_post_process(self) -> None:
        pass

    def _analysis(self) -> None:
        pass


def evaluate(
    scenario_path: str,
    rosbag_dir_path: str,
    t4_dataset_path: str,
    result_jsonl_path: str,
    result_archive_path: str,
    storage: str,
    evaluation_pointcloud_topic: str,
    evaluation_object_topic: str,
    enable_analysis: str,
) -> None:
    evaluation_topics_with_task = {
        "dummy_task": [
            i
            for i in [evaluation_pointcloud_topic, evaluation_object_topic]
            if i not in ("", "None")
        ],
    }

    runner = PerceptionFPRunner(
        scenario_path,
        rosbag_dir_path,
        t4_dataset_path,
        result_jsonl_path,
        result_archive_path,
        storage,
        evaluation_topics_with_task,
        "",
        enable_analysis,
    )

    runner.evaluate()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate perception false positive rosbag")
    parser.add_argument("--scenario-path", required=True, help="File path to scenario files")
    parser.add_argument(
        "--rosbag-dir-path",
        required=True,
        help="Directory path to rosbag which is outputted by Autoware",
    )
    parser.add_argument("--t4-dataset-path", required=True, help="Directory path to t4dataset")
    parser.add_argument(
        "--result-jsonl-path", required=True, help="Output file path for the result in JSONL format"
    )
    parser.add_argument(
        "--result-archive-path", required=True, help="Output directory path for the result"
    )
    parser.add_argument(
        "--storage",
        required=True,
        help="Output rosbag storage type mcap or sqlite3",
    )

    parser.add_argument(
        "--evaluation-pointcloud-topic",
        default="",
        help="ROS pointcloud topic name to evaluate. If you do not want to evaluate pointcloud topic, set '' or 'None'.",
    )
    parser.add_argument(
        "--evaluation-object-topic",
        default="",
        help="ROS object topic name to evaluate. If you do not want to evaluate object topic, set '' or 'None'.",
    )
    parser.add_argument(
        "--enable-analysis",
        default="false",
        help="If set, enable detailed analysis and output the analysis result",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    evaluate(
        args.scenario_path,
        args.rosbag_dir_path,
        args.t4_dataset_path,
        args.result_jsonl_path,
        args.result_archive_path,
        args.storage,
        args.evaluation_pointcloud_topic,
        args.evaluation_object_topic,
        args.enable_analysis,
    )


if __name__ == "__main__":
    main()
