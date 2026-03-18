# Copyright (c) 2024 TIER IV.inc
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
import math
from typing import Any

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import numpy as np
from std_msgs.msg import Header


def convert_to_float(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {k: convert_to_float(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [convert_to_float(v) for v in obj]
    if isinstance(obj, int | float):
        return float(obj)
    return obj


def pose_str_to_dict(pose_str: str) -> dict:
    pose_dict_number = json.loads(pose_str)
    return convert_to_float(pose_dict_number)


def offset_pose_dict_forward(pose_dict: dict, offset_m: float) -> dict:
    orientation = pose_dict["orientation"]
    qx = orientation["x"]
    qy = orientation["y"]
    qz = orientation["z"]
    qw = orientation["w"]

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        msg = "orientation quaternion norm is zero"
        raise ValueError(msg)

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    forward_x = 1.0 - 2.0 * (qy * qy + qz * qz)
    forward_y = 2.0 * (qx * qy + qz * qw)
    forward_z = 2.0 * (qx * qz - qy * qw)

    position = pose_dict["position"]
    return {
        "position": {
            "x": position["x"] + offset_m * forward_x,
            "y": position["y"] + offset_m * forward_y,
            "z": position["z"] + offset_m * forward_z,
        },
        "orientation": {
            "x": orientation["x"],
            "y": orientation["y"],
            "z": orientation["z"],
            "w": orientation["w"],
        },
    }


def pose_dict_to_initial_pose(pose_dict: dict) -> PoseWithCovarianceStamped:
    covariance = np.array(
        [
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.06853892326654787,
        ],
    )
    pose = PoseWithCovariance(
        pose=Pose(
            position=Point(**pose_dict["position"]),
            orientation=Quaternion(**pose_dict["orientation"]),
        ),
        covariance=covariance,
    )
    return PoseWithCovarianceStamped(header=Header(frame_id="map"), pose=pose)


def arg_to_initial_pose(pose_str: str) -> PoseWithCovarianceStamped:
    pose_dict = pose_str_to_dict(pose_str)
    return pose_dict_to_initial_pose(pose_dict)


def arg_to_goal_pose(pose_str: str) -> Pose:
    pose_dict = pose_str_to_dict(pose_str)
    return Pose(
        position=Point(**pose_dict["position"]),
        orientation=Quaternion(**pose_dict["orientation"]),
    )
