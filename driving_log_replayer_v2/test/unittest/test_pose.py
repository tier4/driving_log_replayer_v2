from driving_log_replayer_v2.pose import pose_str_to_dict


def test_pose_str_to_dict() -> None:
    pose_str = (
        '{"position": {"x": 1, "y": 2, "z": 3}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}'
    )

    pose_dict = pose_str_to_dict(pose_str)
    assert pose_dict == {
        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }
