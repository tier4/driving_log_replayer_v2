from importlib import import_module


def test_load_perception_config() -> None:
    module_name = "driving_log_replayer_v2.launch.perception"
    launch_config = import_module(module_name)
    assert (
        launch_config.RECORD_TOPIC
        == "^/tf$|^/sensing/lidar/concatenated/pointcloud$|^/perception/object_recognition/detection/objects$|^/perception/object_recognition/tracking/objects$|^/perception/object_recognition/objects$|^/perception/object_recognition/tracking/multi_object_tracker/debug/.*|^/perception/object_recognition/detection/.*/debug/pipeline_latency_ms$|^/driving_log_replayer_v2/.*|^/sensing/camera/.*"
    )
    assert launch_config.AUTOWARE_DISABLE == {
        "localization": "false",
        "planning": "false",
        "control": "false",
    }
