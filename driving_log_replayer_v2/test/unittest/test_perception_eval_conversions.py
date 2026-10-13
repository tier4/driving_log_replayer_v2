# Copyright (c) 2023 TIER IV.inc
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

import inspect

from autoware_perception_msgs.msg import DetectedObject
from autoware_perception_msgs.msg import ObjectClassification
from autoware_perception_msgs.msg import PredictedObject
from autoware_perception_msgs.msg import Shape as MsgShape
from autoware_perception_msgs.msg import TrackedObject
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon as RosPolygon
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion as RosQuaternion
from geometry_msgs.msg import Vector3
from perception_eval.common.object import DynamicObject
from perception_eval.common.shape import ShapeType
from perception_eval.config import PerceptionEvaluationConfig
from pyquaternion.quaternion import Quaternion
import pytest
from shapely.geometry import Polygon
from std_msgs.msg import Header

from driving_log_replayer_v2.perception_eval_conversions import (
    DYNAMIC_OBJECT_ACCEPTS_EXISTENCE_PROBABILITY,
)
from driving_log_replayer_v2.perception_eval_conversions import footprint_from_ros_msg
from driving_log_replayer_v2.perception_eval_conversions import list_dynamic_object_from_ros_msg
from driving_log_replayer_v2.perception_eval_conversions import orientation_from_ros_msg
from driving_log_replayer_v2.perception_eval_conversions import position_from_ros_msg
from driving_log_replayer_v2.perception_eval_conversions import unix_time_microsec_from_ros_msg


def test_unix_time_from_ros_msg() -> None:
    unix_time_microsec = unix_time_microsec_from_ros_msg(
        Header(stamp=Time(sec=1234567890, nanosec=123456789))
    )
    assert unix_time_microsec == 1234567890123456  # noqa


def test_position_from_ros_msg() -> None:
    tuple_position = position_from_ros_msg(Point(x=1.0, y=2.0, z=3.0))
    assert tuple_position == (1.0, 2.0, 3.0)


def test_orientation_from_ros_msg() -> None:
    eval_quaternion = orientation_from_ros_msg(RosQuaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    assert eval_quaternion == Quaternion(1.0, 0.0, 0.0, 0.0)


def test_footprint_from_ros_msg_normal() -> None:
    ros_points = [
        Point32(x=0.0, y=0.0, z=0.0),
        Point32(x=1.0, y=1.0, z=1.0),
        Point32(x=2.0, y=2.0, z=2.0),
    ]
    eval_polygon = footprint_from_ros_msg(RosPolygon(points=ros_points), ShapeType.POLYGON)
    coords = ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0))
    assert eval_polygon == Polygon(coords)


def test_footprint_from_ros_msg_invalid_footprint() -> None:
    ros_points = [
        Point32(x=0.0, y=0.0, z=0.0),
        Point32(x=1.0, y=1.0, z=1.0),
    ]
    eval_polygon = footprint_from_ros_msg(RosPolygon(points=ros_points), ShapeType.BOUNDING_BOX)
    assert eval_polygon is None


def test_footprint_from_ros_msg_empty_footprint() -> None:
    ros_points = []
    eval_polygon = footprint_from_ros_msg(RosPolygon(points=ros_points), ShapeType.BOUNDING_BOX)
    assert eval_polygon is None


def create_evaluation_config() -> PerceptionEvaluationConfig:
    return PerceptionEvaluationConfig(
        dataset_paths=["/tmp/dlr"],  # noqa
        frame_id="base_link",
        result_root_directory="/tmp/dlr/result/{TIME}",  # noqa
        evaluation_config_dict={
            "evaluation_task": "detection",
            "target_labels": ["car", "bicycle", "pedestrian", "motorbike", "unknown"],
            "max_x_position": 200.0,
            "max_y_position": 200.0,
            "center_distance_thresholds": [[1.0, 1.0, 1.0, 1.0, 1.0]],
            "plane_distance_thresholds": [2.0],
            "iou_2d_thresholds": [0.5],
            "iou_3d_thresholds": [0.5],
            "min_point_numbers": [0, 0, 0, 0, 0],
            "label_prefix": "autoware",
        },
        load_raw_data=False,
    )


def create_box_shape() -> MsgShape:
    return MsgShape(type=MsgShape.BOUNDING_BOX, dimensions=Vector3(x=4.0, y=2.0, z=1.5))


def create_classification() -> list[ObjectClassification]:
    return [ObjectClassification(label=ObjectClassification.CAR, probability=0.9)]


def create_pose() -> Pose:
    return Pose(
        position=Point(x=10.0, y=1.0, z=0.5),
        orientation=RosQuaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def create_ros_objects(
    existence_probability: float,
) -> tuple[DetectedObject, TrackedObject, PredictedObject]:
    detected = DetectedObject(
        existence_probability=existence_probability,
        classification=create_classification(),
        shape=create_box_shape(),
    )
    detected.kinematics.pose_with_covariance.pose = create_pose()

    tracked = TrackedObject(
        existence_probability=existence_probability,
        classification=create_classification(),
        shape=create_box_shape(),
    )
    tracked.object_id.uuid = [1] * 16
    tracked.kinematics.pose_with_covariance.pose = create_pose()

    predicted = PredictedObject(
        existence_probability=existence_probability,
        classification=create_classification(),
        shape=create_box_shape(),
    )
    predicted.object_id.uuid = [2] * 16
    predicted.kinematics.initial_pose_with_covariance.pose = create_pose()
    return detected, tracked, predicted


@pytest.mark.parametrize("existence_probability", [0.0, 0.75, 1.0])
def test_list_dynamic_object_from_ros_msg_passes_existence_probability(
    existence_probability: float,
) -> None:
    """The ROS existence_probability reaches DynamicObject when perception_eval accepts it."""
    evaluation_config = create_evaluation_config()
    for ros_object in create_ros_objects(existence_probability):
        estimated_objects = list_dynamic_object_from_ros_msg(
            1234567890123456, [ros_object], evaluation_config
        )
        assert isinstance(estimated_objects, list)
        assert len(estimated_objects) == 1
        estimated_object = estimated_objects[0]
        assert isinstance(estimated_object, DynamicObject)
        # the semantic score is still the classification probability, whatever the existence is
        assert estimated_object.semantic_score == pytest.approx(0.9)
        if DYNAMIC_OBJECT_ACCEPTS_EXISTENCE_PROBABILITY:
            assert estimated_object.existence_probability == pytest.approx(existence_probability)
            assert isinstance(estimated_object.existence_probability, float)
        else:
            # old perception_eval: the keyword is not passed, so the conversion must still succeed
            assert getattr(estimated_object, "existence_probability", None) is None


def test_dynamic_object_accepts_existence_probability_matches_perception_eval() -> None:
    expected = "existence_probability" in inspect.signature(DynamicObject.__init__).parameters
    assert DYNAMIC_OBJECT_ACCEPTS_EXISTENCE_PROBABILITY is expected
