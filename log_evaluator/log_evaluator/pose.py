import json

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import numpy as np
from std_msgs.msg import Header


def arg_to_msg(pose_str: str) -> PoseWithCovarianceStamped:
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
