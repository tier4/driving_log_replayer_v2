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

"""result.jsonl descriptions and RViz markers of a `PerceptionFrameRecord`."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import TYPE_CHECKING

from ament_index_python.packages import get_package_share_directory
import fastjsonschema
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion as RosQuaternion
from geometry_msgs.msg import Vector3
import numpy as np
from rclpy.time import Duration
from std_msgs.msg import ColorRGBA
from t4perceval import geometry
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

if TYPE_CHECKING:
    from std_msgs.msg import Header

    from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import ObjectTable
    from driving_log_replayer_v2.perception.t4perceval_adapter.frame_result import (
        PerceptionFrameRecord,
    )

__all__ = (
    "FrameDescriptionWriter",
    "ground_truth_markers",
    "result_markers",
    "summarize_pass_fail",
)

MARKER_LIFETIME_SEC = 0.2


def _label_list(labels: tuple[str, ...]) -> str:
    return "[" + ", ".join(labels) + "]"


def summarize_pass_fail(record: PerceptionFrameRecord) -> dict:
    """Return the `PassFail.Info` entry of result.jsonl."""
    return {
        "TP": f"{record.num_tp} {_label_list(record.tp_est_labels())}",
        "FP": f"{record.num_fp} {_label_list(record.fp_labels())}",
        "FN": f"{record.num_fn} {_label_list(record.fn_labels())}",
        "TN": "null",
    }


# -- object descriptions ---------------------------------------------------------------------


def fill_xyz(values: np.ndarray | None) -> dict:
    if values is None:
        return {"x": np.nan, "y": np.nan, "z": np.nan}
    return {"x": float(values[0]), "y": float(values[1]), "z": float(values[2])}


def fill_xyzw(quaternion_xyzw: np.ndarray | None) -> dict:
    if quaternion_xyzw is None:
        return {"x": np.nan, "y": np.nan, "z": np.nan, "w": np.nan}
    return {
        "x": float(quaternion_xyzw[0]),
        "y": float(quaternion_xyzw[1]),
        "z": float(quaternion_xyzw[2]),
        "w": float(quaternion_xyzw[3]),
    }


def _null_errors() -> dict:
    return dict.fromkeys(["pose_error", "heading_error", "velocity_error", "bev_error"])


class FrameDescriptionWriter:
    """Writes the `Objects` entries of result.jsonl, validated against the json schema."""

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
    def object_to_description(table: ObjectTable, row: int, record: PerceptionFrameRecord) -> dict:
        return {
            "label": record.labels.name(int(table.class_id[row])),
            "uuid": table.uuids[row],
            "position": fill_xyz(table.position[row]),
            "velocity": fill_xyz(table.velocity[row]),
            "orientation": fill_xyzw(table.quaternion[row]),
            "shape": fill_xyz(table.size[row]),
        }

    @staticmethod
    def object_to_covariance_description(table: ObjectTable, row: int) -> dict:
        return {
            "pose_covariance": []
            if table.pose_covariance is None
            else table.pose_covariance[row].tolist(),
            "twist_covariance": []
            if table.twist_covariance is None
            else table.twist_covariance[row].tolist(),
        }

    @staticmethod
    def extract_pass_fail_objects_description(record: PerceptionFrameRecord) -> list[dict]:
        """
        Extract the objects of the frame: GT TP, GT FN, then EST TP, EST FP.

        See config/perception/object_output_schema.json for the fields.
        """
        filename: str = __file__
        est = record.estimation
        gt = record.ground_truth
        errors = record.pair_errors()
        est_distance = est.bev_distance()
        gt_distance = gt.bev_distance()

        gt_descriptions: list[dict] = []
        est_descriptions: list[dict] = []

        for pair, (est_row, gt_row) in enumerate(
            zip(record.matches.tp_est, record.matches.tp_gt, strict=True)
        ):
            error_description = {
                "pose_error": fill_xyz(errors.position[pair]),
                "heading_error": fill_xyz(errors.heading[pair]),
                "velocity_error": fill_xyz(errors.velocity[pair]),
                "bev_error": float(errors.bev[pair]),
            }
            gt_tp_description = {
                "status": "TP",
                "object_type": "GT",
                "distance_from_ego": float(gt_distance[gt_row]),
                **FrameDescriptionWriter.object_to_description(gt, int(gt_row), record),
                **error_description,
                **FrameDescriptionWriter.object_to_covariance_description(gt, int(gt_row)),
            }
            est_tp_description = {
                "status": "TP",
                "object_type": "EST",
                "distance_from_ego": float(est_distance[est_row]),
                **FrameDescriptionWriter.object_to_description(est, int(est_row), record),
                **error_description,
                **FrameDescriptionWriter.object_to_covariance_description(est, int(est_row)),
            }
            assert FrameDescriptionWriter.is_object_structure_valid(gt_tp_description), (
                "GT TP object description is invalid in file: " + filename
            )
            assert FrameDescriptionWriter.is_object_structure_valid(est_tp_description), (
                "EST TP object description is invalid in file: " + filename
            )
            gt_descriptions.append(gt_tp_description)
            est_descriptions.append(est_tp_description)

        for est_row in record.matches.fp_est:
            est_fp_description = {
                "status": "FP",
                "object_type": "EST",
                "distance_from_ego": float(est_distance[est_row]),
                **FrameDescriptionWriter.object_to_description(est, int(est_row), record),
                **_null_errors(),
                **FrameDescriptionWriter.object_to_covariance_description(est, int(est_row)),
            }
            assert FrameDescriptionWriter.is_object_structure_valid(est_fp_description), (
                "EST FP object description is invalid in file: " + filename
            )
            est_descriptions.append(est_fp_description)

        for gt_row in record.matches.fn_gt:
            gt_fn_description = {
                "status": "FN",
                "object_type": "GT",
                "distance_from_ego": float(gt_distance[gt_row]),
                **FrameDescriptionWriter.object_to_description(gt, int(gt_row), record),
                **FrameDescriptionWriter.object_to_covariance_description(gt, int(gt_row)),
                **_null_errors(),
            }
            assert FrameDescriptionWriter.is_object_structure_valid(gt_fn_description), (
                "GT FN object description is invalid in file: " + filename
            )
            gt_descriptions.append(gt_fn_description)

        return gt_descriptions + est_descriptions


# -- markers ---------------------------------------------------------------------------------


@dataclass
class ScoresData:
    label: str
    center_distance: float | None
    center_distance_bev: float | None
    plane_distance: float | None
    iou_3d: float | None
    iou_2d: float | None

    def __str__(self) -> str:
        text = f"{self.label}\n"
        text += f"CD: {self.center_distance:.2f}, " if self.center_distance is not None else ""
        text += (
            f"CD_BEV: {self.center_distance_bev:.2f}, "
            if self.center_distance_bev is not None
            else ""
        )
        text += f"PD: {self.plane_distance:.2f}, " if self.plane_distance is not None else ""
        text += f"IoU3D: {self.iou_3d:.2f}, " if self.iou_3d is not None else ""
        text += f"IoU2D: {self.iou_2d:.2f}" if self.iou_2d is not None else ""
        return text


def _pose(position: np.ndarray, quaternion_xyzw: np.ndarray) -> Pose:
    return Pose(
        position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])),
        orientation=RosQuaternion(
            x=float(quaternion_xyzw[0]),
            y=float(quaternion_xyzw[1]),
            z=float(quaternion_xyzw[2]),
            w=float(quaternion_xyzw[3]),
        ),
    )


def _scale(size_wlh: np.ndarray) -> Vector3:
    # t4perceval size is (width, length, height), a marker scale is (length, width, height)
    return Vector3(x=float(size_wlh[1]), y=float(size_wlh[0]), z=float(size_wlh[2]))


def _box_and_text(
    table: ObjectTable,
    row: int,
    header: Header,
    namespace: str,
    marker_id: int,
    color: ColorRGBA,
    text: str,
    text_scale: Vector3,
    text_suffix: str,
) -> tuple[Marker, Marker]:
    pose = _pose(table.position[row], table.quaternion[row])
    box = Marker(
        header=header,
        ns=namespace,
        id=marker_id,
        type=Marker.CUBE,
        action=Marker.ADD,
        lifetime=Duration(seconds=MARKER_LIFETIME_SEC).to_msg(),
        pose=pose,
        scale=_scale(table.size[row]),
        color=color,
    )
    label = Marker(
        header=header,
        ns=namespace + text_suffix,
        id=marker_id,
        type=Marker.TEXT_VIEW_FACING,
        action=Marker.ADD,
        lifetime=Duration(seconds=MARKER_LIFETIME_SEC).to_msg(),
        pose=pose,
        scale=text_scale,
        color=color,
        text=text,
    )
    return box, label


def ground_truth_markers(record: PerceptionFrameRecord, header: Header) -> MarkerArray:
    """Green boxes of the kept ground truths, with label and uuid."""
    markers = MarkerArray()
    color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)
    gt = record.ground_truth
    for row in range(len(gt)):
        box, uuid = _box_and_text(
            gt,
            row,
            header,
            "ground_truth",
            row + 1,
            color,
            f"{record.labels.name(int(gt.class_id[row]))}\n{gt.uuids[row]}",
            Vector3(z=0.8),
            "_uuid",
        )
        markers.markers.append(box)
        markers.markers.append(uuid)
    return markers


def _tp_scores(record: PerceptionFrameRecord) -> list[ScoresData]:
    est = record.estimation
    gt = record.ground_truth
    est_rows = record.matches.tp_est
    gt_rows = record.matches.tp_gt
    scores: list[ScoresData] = []
    if len(est_rows) == 0:
        return scores
    e_pos, e_quat, e_size = est.position[est_rows], est.quaternion[est_rows], est.size[est_rows]
    g_pos, g_quat, g_size = gt.position[gt_rows], gt.quaternion[gt_rows], gt.size[gt_rows]
    center = np.linalg.norm(e_pos - g_pos, axis=1)
    center_bev = np.linalg.norm(e_pos[:, :2] - g_pos[:, :2], axis=1)
    iou_3d = np.diagonal(geometry.pairwise_volume_iou(e_pos, e_quat, e_size, g_pos, g_quat, g_size))
    iou_2d = np.diagonal(geometry.pairwise_bev_iou(e_pos, e_quat, e_size, g_pos, g_quat, g_size))
    labels = record.tp_est_labels()
    for pair in range(len(est_rows)):
        scores.append(
            ScoresData(
                label=labels[pair],
                center_distance=float(center[pair]),
                center_distance_bev=float(center_bev[pair]),
                plane_distance=float(record.matches.tp_score[pair]),
                iou_3d=float(iou_3d[pair]),
                iou_2d=float(iou_2d[pair]),
            )
        )
    return scores


def result_markers(record: PerceptionFrameRecord, header: Header) -> MarkerArray:
    """Boxes of the TP estimations, TP ground truths, FP estimations and FN ground truths."""
    markers = MarkerArray()
    text_scale = Vector3(x=0.4, y=0.4, z=0.8)
    est = record.estimation
    gt = record.ground_truth

    def add(
        table: ObjectTable, row: int, namespace: str, index: int, color: ColorRGBA, text: str
    ) -> None:
        box, label = _box_and_text(
            table, row, header, namespace, index, color, text, text_scale, "_score"
        )
        markers.markers.append(box)
        markers.markers.append(label)

    tp_scores = _tp_scores(record)
    for index, (est_row, score) in enumerate(zip(record.matches.tp_est, tp_scores, strict=True)):
        add(est, int(est_row), "tp_est", index, ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7), str(score))
    for index, gt_row in enumerate(record.matches.tp_gt):
        label = record.labels.name(int(gt.class_id[gt_row]))
        add(gt, int(gt_row), "tp_gt", index, ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7), f"{label}\n")
    for index, est_row in enumerate(record.matches.fp_est):
        label = record.labels.name(int(est.class_id[est_row]))
        score = ScoresData(label, -1.0, -1.0, -1.0, -1.0, -1.0)
        add(est, int(est_row), "fp", index, ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.7), str(score))
    for index, gt_row in enumerate(record.matches.fn_gt):
        label = record.labels.name(int(gt.class_id[gt_row]))
        add(gt, int(gt_row), "fn", index, ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7), f"{label}\n")
    return markers
