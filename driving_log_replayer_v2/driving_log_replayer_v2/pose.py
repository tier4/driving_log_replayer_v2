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

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import numpy as np
from std_msgs.msg import Header


def arg_to_initial_pose(pose_str: str) -> PoseWithCovarianceStamped:
    pose_dict = json.loads(pose_str)
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


def arg_to_goal_pose(pose_str: str) -> Pose:
    pose_dict = json.loads(pose_str)
    return Pose(
        position=Point(**pose_dict["position"]),
        orientation=Quaternion(**pose_dict["orientation"]),
    )
