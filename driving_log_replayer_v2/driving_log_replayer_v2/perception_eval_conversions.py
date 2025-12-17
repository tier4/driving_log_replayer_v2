# Copyright (c) 2021 TIER IV.inc
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

from dataclasses import dataclass
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from autoware_perception_msgs.msg import DetectedObject
from autoware_perception_msgs.msg import ObjectClassification
from autoware_perception_msgs.msg import PredictedObject
from autoware_perception_msgs.msg import PredictedPath
from autoware_perception_msgs.msg import Shape as MsgShape
from autoware_perception_msgs.msg import TrackedObject
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time
import fastjsonschema
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon as RosPolygon
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion as RosQuaternion
from geometry_msgs.msg import Vector3
import numpy as np
from perception_eval.common import ObjectType
from perception_eval.common.dataset import FrameGroundTruth
from perception_eval.common.object import DynamicObject
from perception_eval.common.object import ObjectState
from perception_eval.common.schema import FrameID
from perception_eval.common.shape import Shape
from perception_eval.common.shape import ShapeType
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation.result.object_result import DynamicObjectWithPerceptionResult
from perception_eval.evaluation.result.perception_pass_fail_result import PassFailResult
from pyquaternion.quaternion import Quaternion
from rclpy.time import Duration
from shapely.geometry import Polygon
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

"""
In autoware_perception_evaluation, unix_time is defined as microseconds.
"""


def unix_time_microsec_from_ros_msg(ros_header: Header) -> int:
    return ros_header.stamp.sec * pow(10, 6) + ros_header.stamp.nanosec // 1000


def unix_time_microsec_from_ros_timestamp(ros_timestamp: Time | DurationMsg) -> int:
    return ros_timestamp.sec * pow(10, 6) + ros_timestamp.nanosec // 1000


def unix_time_microsec_from_ros_time_nanosec(ros_time_nanosec: int) -> int:
    return ros_time_nanosec // 1000


def position_from_ros_msg(ros_position: Point) -> tuple[int, int, int]:
    return (ros_position.x, ros_position.y, ros_position.z)


def orientation_from_ros_msg(ros_orientation: RosQuaternion) -> Quaternion:
    return Quaternion(ros_orientation.w, ros_orientation.x, ros_orientation.y, ros_orientation.z)


def path_timestamps_from_ros_msg(ros_path: PredictedPath) -> list[int]:
    # NOTE: predicted path starts from the current object pose
    duration = unix_time_microsec_from_ros_timestamp(ros_path.time_step)
    return [duration * i for i in range(1, len(ros_path.path))]


def path_positions_from_ros_msg(ros_path: PredictedPath) -> list[tuple[float, float, float]]:
    # NOTE: predicted path starts from the current object pose
    return [position_from_ros_msg(pose.position) for pose in ros_path.path[1:]]


def path_orientations_from_ros_msg(ros_path: PredictedPath) -> list[Quaternion]:
    # NOTE: predicted path starts from the current object pose
    return [orientation_from_ros_msg(pose.orientation) for pose in ros_path.path[1:]]


def dimensions_from_ros_msg(
    ros_dimensions: Vector3,
    shape_type_num: int,
) -> tuple[float, float, float]:
    if shape_type_num == 1:
        # cylinder
        return (ros_dimensions.x, ros_dimensions.x, ros_dimensions.z)
    return (ros_dimensions.y, ros_dimensions.x, ros_dimensions.z)


def velocity_from_ros_msg(ros_velocity: Vector3) -> tuple[float, float, float]:
    return (ros_velocity.x, ros_velocity.y, ros_velocity.z)


def footprint_from_ros_msg(ros_footprint: RosPolygon) -> Polygon | None:
    coords = []
    for ros_point in ros_footprint.points:
        coords.append((ros_point.x, ros_point.y, ros_point.z))
    if len(coords) >= 3:  # noqa
        # polygon must be more than 3 points
        return Polygon(coords)
    # footprint.points of bounding_box and cylinder are empty, so return None
    return None


def uuid_from_ros_msg(ros_uuid: np.ndarray) -> str:
    """
    Convert uuid from unique_identifier_msgs.msg.UUID to string.

    Args:
    ----
        ros_uuid (np.ndarray): (16,) in uint8
    Returns:
        uuid (str)

    """
    return "".join([str(uuid_) for uuid_ in ros_uuid])


def object_state_to_ros_box_and_uuid(
    gt_state: ObjectState,
    header: Header,
    namespace: str,
    marker_id: int,
    color: ColorRGBA,
    text: str,
) -> tuple[Marker, Marker]:
    pose = Pose(
        position=Point(x=gt_state.position[0], y=gt_state.position[1], z=gt_state.position[2]),
        orientation=RosQuaternion(
            w=gt_state.orientation[0],
            x=gt_state.orientation[1],
            y=gt_state.orientation[2],
            z=gt_state.orientation[3],
        ),
    )
    # nuScenes box order is width, length, height
    scale = Vector3(x=gt_state.size[1], y=gt_state.size[0], z=gt_state.size[2])
    bbox = Marker(
        header=header,
        ns=namespace,
        id=marker_id,
        type=Marker.CUBE,
        action=Marker.ADD,
        lifetime=Duration(seconds=0.2).to_msg(),
        pose=pose,
        scale=scale,
        color=color,
    )
    uuid = Marker(
        header=header,
        ns=namespace + "_uuid",
        id=marker_id,
        type=Marker.TEXT_VIEW_FACING,
        action=Marker.ADD,
        lifetime=Duration(seconds=0.2).to_msg(),
        pose=pose,
        scale=Vector3(z=0.8),
        color=color,
        text=text,
    )
    return bbox, uuid


def frame_ground_truth_to_ros_box_and_uuid(
    frame_ground_truth: FrameGroundTruth, header: Header
) -> MarkerArray:
    marker_ground_truth = MarkerArray()
    color_success = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)

    for cnt, obj in enumerate(frame_ground_truth.objects, start=1):
        bbox, uuid = object_state_to_ros_box_and_uuid(
            obj.state,
            header,
            "ground_truth",
            cnt,
            color_success,
            str(obj.semantic_label.label) + ": " + obj.uuid,
        )
        marker_ground_truth.markers.append(bbox)
        marker_ground_truth.markers.append(uuid)

    return marker_ground_truth


@dataclass
class ScoresData:
    center_distance: float | None
    center_distance_bev: float | None
    plane_distance: float | None
    iou_3d: float | None
    iou_2d: float | None

    def __str__(self) -> str:
        text = f"CD: {self.center_distance:.2f}, " if self.center_distance is not None else ""
        text += (
            f"CD_BEV: {self.center_distance_bev:.2f}, "
            if self.center_distance_bev is not None
            else ""
        )
        text += f"PD: {self.plane_distance:.2f}, " if self.plane_distance is not None else ""
        text += f"IoU3D: {self.iou_3d:.2f}, " if self.iou_3d is not None else ""
        text += f"IoU2D: {self.iou_2d:.2f}" if self.iou_2d is not None else ""
        return text


def dynamic_objects_to_ros_points(
    obj: DynamicObjectWithPerceptionResult | DynamicObject,
    header: Header,
    score_scale: Vector3,
    color: ColorRGBA,
    namespace: str,
    cnt: int,
    *,
    tp_gt: bool,
) -> tuple[Marker, Marker]:
    if isinstance(obj, DynamicObjectWithPerceptionResult):
        if tp_gt:
            if obj.ground_truth_object is not None:
                # tpのgtを出したい場合、tpならば必ずground_truthのペアがいる
                pose = Pose(
                    position=Point(
                        x=obj.ground_truth_object.state.position[0],
                        y=obj.ground_truth_object.state.position[1],
                        z=obj.ground_truth_object.state.position[2],
                    ),
                    orientation=RosQuaternion(
                        w=obj.ground_truth_object.state.orientation[0],
                        x=obj.ground_truth_object.state.orientation[1],
                        y=obj.ground_truth_object.state.orientation[2],
                        z=obj.ground_truth_object.state.orientation[3],
                    ),
                )
                obj_scale = Vector3(
                    x=obj.ground_truth_object.state.size[1],
                    y=obj.ground_truth_object.state.size[0],
                    z=obj.ground_truth_object.state.size[2],
                )
            # skip showing scores for gt objects
            scores = ScoresData(
                center_distance=None,
                center_distance_bev=None,
                plane_distance=None,
                iou_3d=None,
                iou_2d=None,
            )
        else:
            pose = Pose(
                position=Point(
                    x=obj.estimated_object.state.position[0],
                    y=obj.estimated_object.state.position[1],
                    z=obj.estimated_object.state.position[2],
                ),
                orientation=RosQuaternion(
                    w=obj.estimated_object.state.orientation[0],
                    x=obj.estimated_object.state.orientation[1],
                    y=obj.estimated_object.state.orientation[2],
                    z=obj.estimated_object.state.orientation[3],
                ),
            )
            obj_scale = Vector3(
                x=obj.estimated_object.state.size[1],
                y=obj.estimated_object.state.size[0],
                z=obj.estimated_object.state.size[2],
            )

            # show the matching scores only tp objects by estimated object and fp objects
            scores = ScoresData(
                center_distance=obj.center_distance.value
                if obj.center_distance is not None
                else -1.0,
                center_distance_bev=obj.center_distance_bev.value
                if obj.center_distance_bev is not None
                else -1.0,
                plane_distance=obj.plane_distance.value if obj.plane_distance is not None else -1.0,
                iou_3d=obj.iou_3d.value if obj.iou_3d is not None else -1.0,
                iou_2d=obj.iou_2d.value if obj.iou_2d is not None else -1.0,
            )

    if isinstance(obj, DynamicObject):
        pose = Pose(
            position=Point(
                x=obj.state.position[0],
                y=obj.state.position[1],
                z=obj.state.position[2],
            ),
            orientation=RosQuaternion(
                w=obj.state.orientation[0],
                x=obj.state.orientation[1],
                y=obj.state.orientation[2],
                z=obj.state.orientation[3],
            ),
        )
        obj_scale = Vector3(
            x=obj.state.size[1],
            y=obj.state.size[0],
            z=obj.state.size[2],
        )
        # skip showing scores for DynamicObject because they do not have scores
        scores = ScoresData(
            center_distance=None,
            center_distance_bev=None,
            plane_distance=None,
            iou_3d=None,
            iou_2d=None,
        )

    result = Marker(
        header=header,
        ns=namespace,
        id=cnt,
        type=Marker.CUBE,
        action=Marker.ADD,
        lifetime=Duration(seconds=0.2).to_msg(),
        pose=pose,
        scale=obj_scale,
        color=color,
    )

    score_text = Marker(
        header=header,
        ns=namespace + "_score",
        id=cnt,
        type=Marker.TEXT_VIEW_FACING,
        action=Marker.ADD,
        lifetime=Duration(seconds=0.2).to_msg(),
        pose=pose,
        scale=score_scale,
        color=color,
        text=str(scores),
    )

    return result, score_text


def pass_fail_result_to_ros_points_array(pass_fail: PassFailResult, header: Header) -> MarkerArray:
    marker_results = MarkerArray()
    score_scale = Vector3(x=0.4, y=0.4, z=0.8)

    if objs := pass_fail.tp_object_results:
        # estimated obj
        for cnt, obj in enumerate(objs):
            result, score_text = dynamic_objects_to_ros_points(
                obj,
                header,
                score_scale,
                ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7),
                "tp_est",
                cnt,
                tp_gt=False,
            )
            marker_results.markers.append(result)
            marker_results.markers.append(score_text)
    if objs := pass_fail.tp_object_results:
        # ground truth obj
        for cnt, obj in enumerate(objs):
            result, score_text = dynamic_objects_to_ros_points(
                obj,
                header,
                score_scale,
                ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7),
                "tp_gt",
                cnt,
                tp_gt=True,
            )
            marker_results.markers.append(result)
            marker_results.markers.append(score_text)
    if objs := pass_fail.fp_object_results:
        # estimated obj
        for cnt, obj in enumerate(objs):
            result, score_text = dynamic_objects_to_ros_points(
                obj,
                header,
                score_scale,
                ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.7),
                "fp",
                cnt,
                tp_gt=False,
            )
            marker_results.markers.append(result)
            marker_results.markers.append(score_text)
    if objs := pass_fail.fn_objects:
        # ground truth obj
        for cnt, obj in enumerate(objs):
            result, score_text = dynamic_objects_to_ros_points(
                obj,
                header,
                score_scale,
                ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7),
                "fn",
                cnt,
                tp_gt=False,
            )
            marker_results.markers.append(result)
            marker_results.markers.append(score_text)
    return marker_results


def summarize_pass_fail_result(pass_fail: PassFailResult) -> dict:
    return {
        "TP": f"{len(pass_fail.tp_object_results)} {result_label_list(pass_fail.tp_object_results)}",
        "FP": f"{len(pass_fail.fp_object_results)} {result_label_list(pass_fail.fp_object_results)}",
        "FN": f"{len(pass_fail.fn_objects)} {object_label_list(pass_fail.fn_objects)}",
        "TN": f"{len(pass_fail.tn_objects)} {object_label_list(pass_fail.tn_objects)}"
        if pass_fail.frame_pass_fail_config.evaluation_task.is_fp_validation()
        else "null",
    }


def result_label_list(results: list[DynamicObjectWithPerceptionResult]) -> str:
    rtn_str = "["
    for i, result in enumerate(results):
        obj_type = result.estimated_object
        if i > 0:
            rtn_str += ", "
        rtn_str += obj_type.semantic_label.name
    rtn_str += "]"
    return rtn_str


def object_label_list(objects: list[ObjectType]) -> str:
    rtn_str = "["
    for i, obj in enumerate(objects):
        if i > 0:
            rtn_str += ", "
        rtn_str += obj.semantic_label.name
    rtn_str += "]"
    return rtn_str


def calc_position_error(
    tuple1: tuple[float, float, float],
    tuple2: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(map(lambda x, y: x - y, tuple1, tuple2))


def fill_xyz(tuple_: tuple[float, float, float] | None) -> dict:
    if tuple_ is None:
        return {"x": np.nan, "y": np.nan, "z": np.nan}
    return {
        "x": tuple_[0],
        "y": tuple_[1],
        "z": tuple_[2],
    }


def fill_xyzw(tuple_: tuple[float, float, float, float] | None) -> dict:
    if tuple_ is None:
        return {"x": np.nan, "y": np.nan, "z": np.nan, "w": np.nan}
    return {
        "x": tuple_[0],
        "y": tuple_[1],
        "z": tuple_[2],
        "w": tuple_[3],
    }


def fill_xyzw_quat(q: Quaternion | None) -> dict:
    if q is None:
        return {"x": np.nan, "y": np.nan, "z": np.nan, "w": np.nan}
    return {
        "x": q.x,
        "y": q.y,
        "z": q.z,
        "w": q.w,
    }


def get_perception_label_str(classification: ObjectClassification) -> str:
    label_str_dict = {
        ObjectClassification.UNKNOWN: "unknown",
        ObjectClassification.CAR: "car",
        ObjectClassification.TRUCK: "truck",
        ObjectClassification.BUS: "bus",
        ObjectClassification.TRAILER: "trailer",
        ObjectClassification.MOTORCYCLE: "motorbike",
        ObjectClassification.BICYCLE: "bicycle",
        ObjectClassification.PEDESTRIAN: "pedestrian",
    }
    return label_str_dict.get(classification.label, "other")


def list_dynamic_object_from_ros_msg(
    unix_time_microsec: int,
    objects: list[DetectedObject] | list[TrackedObject] | list[PredictedObject],
    evaluator_config: PerceptionEvaluationConfig,
) -> list[DynamicObject] | str:
    def get_most_probable_classification(
        array_classification: list[ObjectClassification],
    ) -> ObjectClassification:
        index: int = array_classification.index(
            max(array_classification, key=lambda x: x.probability),
        )
        return array_classification[index]

    # return str(error_msg) when footprint points are invalid
    estimated_objects: list[DynamicObject] = []
    for perception_object in objects:
        # check footprint length
        if 1 <= len(perception_object.shape.footprint.points) < 3:  # noqa
            return f"Unexpected footprint length: {len(perception_object.shape.footprint.points)=}"

        most_probable_classification = get_most_probable_classification(
            perception_object.classification,
        )
        label = evaluator_config.label_converter.convert_label(
            name=get_perception_label_str(most_probable_classification),
        )

        uuid = None
        if isinstance(perception_object, TrackedObject | PredictedObject):
            uuid = uuid_from_ros_msg(perception_object.object_id.uuid)

        shape_type = ShapeType.BOUNDING_BOX
        shape_type_num = perception_object.shape.type
        if shape_type_num == MsgShape.POLYGON:
            shape_type = ShapeType.POLYGON

        assert len(evaluator_config.frame_ids) == 1, "Only one frame id is supported"
        estimated_object = DynamicObject(
            unix_time=unix_time_microsec,
            frame_id=evaluator_config.frame_ids[0],
            position=position_from_ros_msg(
                perception_object.kinematics.pose_with_covariance.pose.position
                if isinstance(perception_object, DetectedObject | TrackedObject)
                else perception_object.kinematics.initial_pose_with_covariance.pose.position
            ),
            orientation=orientation_from_ros_msg(
                perception_object.kinematics.pose_with_covariance.pose.orientation
                if isinstance(perception_object, DetectedObject | TrackedObject)
                else perception_object.kinematics.initial_pose_with_covariance.pose.orientation
            ),
            shape=Shape(
                shape_type=shape_type,
                size=dimensions_from_ros_msg(
                    perception_object.shape.dimensions,
                    shape_type_num,
                ),
                footprint=footprint_from_ros_msg(
                    perception_object.shape.footprint,
                ),
            ),
            velocity=velocity_from_ros_msg(
                perception_object.kinematics.twist_with_covariance.twist.linear
                if isinstance(perception_object, DetectedObject | TrackedObject)
                else perception_object.kinematics.initial_twist_with_covariance.twist.linear
            ),
            pose_covariance=perception_object.kinematics.pose_with_covariance.covariance
            if isinstance(perception_object, DetectedObject | TrackedObject)
            else perception_object.kinematics.initial_pose_with_covariance.covariance,
            twist_covariance=perception_object.kinematics.twist_with_covariance.covariance
            if isinstance(perception_object, DetectedObject | TrackedObject)
            else perception_object.kinematics.initial_twist_with_covariance.covariance,
            semantic_score=most_probable_classification.probability,
            semantic_label=label,
            uuid=uuid,
            relative_timestamps=[
                path_timestamps_from_ros_msg(path)
                for path in perception_object.kinematics.predicted_paths
            ]
            if isinstance(perception_object, PredictedObject)
            else None,
            predicted_positions=[
                path_positions_from_ros_msg(path)
                for path in perception_object.kinematics.predicted_paths
            ]
            if isinstance(perception_object, PredictedObject)
            else None,
            predicted_orientations=[
                path_orientations_from_ros_msg(path)
                for path in perception_object.kinematics.predicted_paths
            ]
            if isinstance(perception_object, PredictedObject)
            else None,
            predicted_scores=[
                path.confidence for path in perception_object.kinematics.predicted_paths
            ]
            if isinstance(perception_object, PredictedObject)
            else None,
        )
        estimated_objects.append(estimated_object)
    return estimated_objects


# utils for writing each perception frame result to a file
class FrameDescriptionWriter:
    schema: dict = None
    validate_func = None

    @classmethod
    def load_schema(cls) -> None:
        if cls.schema is None or cls.validate_func is None:
            package_share_directory = get_package_share_directory("driving_log_replayer_v2")
            schema_file_path = Path(
                package_share_directory,
                "config",
                "perception",
                "object_output_schema.json",
            )
            with schema_file_path.open() as file:
                cls.schema = json.load(file)
            cls.validate_func = fastjsonschema.compile(cls.schema)

    @classmethod
    def is_object_structure_valid(cls, objdata: dict | None) -> bool:
        cls.load_schema()
        try:
            cls.validate_func(objdata)
        except fastjsonschema.exceptions.JsonSchemaException:
            return False
        else:
            return True

    @staticmethod
    def object_to_description(obj: ObjectType | None) -> dict:
        if obj is None:
            return {}
        return {
            "label": obj.semantic_label.name,
            "uuid": obj.uuid,
            "position": fill_xyz(obj.state.position),
            "velocity": fill_xyz(obj.state.velocity),
            "orientation": fill_xyzw_quat(obj.state.orientation),  # pyQuaternion to dict
            "shape": fill_xyz(obj.state.size),
        }

    @staticmethod
    def dynamic_object_result_to_error_description(
        obj: DynamicObjectWithPerceptionResult | None,
    ) -> dict:
        null_return = dict.fromkeys(["pose_error", "heading_error", "velocity_error", "bev_error"])
        if obj is None or obj.ground_truth_object is None:
            return null_return

        pose_error = calc_position_error(
            obj.ground_truth_object.state.position,
            obj.estimated_object.state.position,
        )
        bev_error = obj.distance_error_bev
        heading_error = obj.heading_error
        velocity_error = obj.velocity_error
        return {
            "pose_error": fill_xyz(pose_error),
            "heading_error": fill_xyz(heading_error),
            "velocity_error": fill_xyz(velocity_error),
            "bev_error": bev_error,
        }

    @staticmethod
    def object_to_covariance_description(obj: ObjectType | None) -> dict:
        if obj is None:
            return {
                "pose_covariance": [],
                "twist_covariance": [],
            }

        pose_covariance = (
            obj.state.pose_covariance.tolist() if obj.state.has_pose_covariance else []
        )
        twist_covariance = (
            obj.state.twist_covariance.tolist() if obj.state.has_twist_covariance else []
        )
        return {
            "pose_covariance": pose_covariance,
            "twist_covariance": twist_covariance,
        }

    @staticmethod
    def extract_pass_fail_objects_description(pass_fail: PassFailResult) -> list[dict]:
        """
        Extract detailed objects results from PassFailResult.

        Args:
        ----
            pass_fail (PassFailResult): PassFailResult object

        Returns: see json schema in config/perception/object_output_schema.json
        -------
            list[dict]: List of objects descriptions.
                Each element is a dictionary with the following keys:
                - "status": "TP" | "FP" | "FN"
                - "object_type": "GT" | "EST"
                - "distance_from_ego": float|None # distance from ego vehicle
                - "label": str
                - "uuid": optional[str]
                - "position": { "x": float, "y": float, "z": float }
                - "velocity": { "x": float, "y": float, "z": float }
                - "orientation": { "x": float, "y": float, "z": float, "w": float }
                - "shape": optional[{ "x": float, "y": float, "z": float }]
                - "pose_error": optional[{ "x": float, "y": float, "z": float }]
                - "heading_error": optional[{ "x": float, "y": float, "z": float }]
                - "velocity_error": optional[{ "x": float, "y": float, "z": float }]
                - "bev_error": optional[float] # distance between GT and EST
                - "pose_covariance": optional[list[float]]
                - "twist_covariance": optional[list[float]]

                These description can separated to following categories and each category are generated by corresponding functions:
                - Test status and object type
                - Object information
                - Error information
                - Covariance information

        """
        # get this filename for assertion error message
        filename: str = __file__

        # TODO: remove try-except block after perception eval is properly updated
        try:
            ego2map_matrix = pass_fail.ego2map
            has_map_to_base_link = ego2map_matrix is not None and len(ego2map_matrix) > 0
        except AttributeError:
            ego2map_matrix = pass_fail.transforms

            has_map_to_base_link = ego2map_matrix.get((FrameID.BASE_LINK, FrameID.MAP)) is not None

        gt_descriptions = []
        est_descriptions = []

        # for TP objects
        for tp_object in pass_fail.tp_object_results:
            tp_gt = tp_object.ground_truth_object
            tp_est = tp_object.estimated_object
            gt_distance_bev = (
                tp_gt.get_distance_bev(ego2map_matrix) if has_map_to_base_link and tp_gt else None
            )
            est_distance_bev = (
                tp_est.get_distance_bev(ego2map_matrix) if has_map_to_base_link else None
            )
            error_description = FrameDescriptionWriter.dynamic_object_result_to_error_description(
                tp_object,
            )
            cov_description = FrameDescriptionWriter.object_to_covariance_description(tp_est)
            # GT object dict
            gt_tp_description = {
                "status": "TP",
                "object_type": "GT",
                "distance_from_ego": gt_distance_bev,
                **FrameDescriptionWriter.object_to_description(tp_gt),
                **error_description,
                **FrameDescriptionWriter.object_to_covariance_description(
                    tp_gt,
                ),  # Assume gt has no covariance
            }
            # Estimated object dict
            est_tp_description = {
                "status": "TP",
                "object_type": "EST",
                "distance_from_ego": est_distance_bev,
                **FrameDescriptionWriter.object_to_description(tp_est),
                **error_description,
                **cov_description,
            }
            # tp_gt is optional (in the test)
            if tp_gt:
                assert FrameDescriptionWriter.is_object_structure_valid(gt_tp_description), (
                    "GT TP object description is invalid in file: " + filename
                )
                gt_descriptions.append(gt_tp_description)
            assert FrameDescriptionWriter.is_object_structure_valid(est_tp_description), (
                "EST TP object description is invalid in file: " + filename
            )
            est_descriptions.append(est_tp_description)

        # for FP objects
        for fp_object in pass_fail.fp_object_results:
            fp_est = fp_object.estimated_object
            error_description = FrameDescriptionWriter.dynamic_object_result_to_error_description(
                fp_object,
            )
            est_distance_bev = (
                fp_est.get_distance_bev(ego2map_matrix) if has_map_to_base_link else None
            )
            fp_object_description = FrameDescriptionWriter.object_to_description(fp_est)
            cov_description = FrameDescriptionWriter.object_to_covariance_description(fp_est)
            # Estimated object dict
            est_fp_description = {
                "status": "FP",
                "object_type": "EST",
                "distance_from_ego": est_distance_bev,
                **fp_object_description,
                **error_description,
                **cov_description,
            }
            assert FrameDescriptionWriter.is_object_structure_valid(est_fp_description), (
                "EST FP object description is invalid in file: " + filename
            )
            est_descriptions.append(est_fp_description)

        # for FN objects
        for fn_object in pass_fail.fn_objects:
            fn_gt = fn_object
            gt_distance_bev = (
                fn_gt.get_distance_bev(ego2map_matrix) if has_map_to_base_link else None
            )
            fn_obj_description = FrameDescriptionWriter.object_to_description(fn_gt)
            # GT object dict
            gt_fn_description = {
                "status": "FN",
                "object_type": "GT",
                "distance_from_ego": gt_distance_bev,
                **fn_obj_description,
                **FrameDescriptionWriter.object_to_covariance_description(
                    fn_gt,
                ),  # Assume gt has no covariance
                **FrameDescriptionWriter.dynamic_object_result_to_error_description(
                    None,
                ),  # Assume gt has no error
            }
            assert FrameDescriptionWriter.is_object_structure_valid(gt_fn_description), (
                "GT FN object description is invalid in file: " + filename
            )
            gt_descriptions.append(gt_fn_description)

        return gt_descriptions + est_descriptions
