from importlib import import_module
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import LogInfo

from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchContext
from launch.actions import LogInfo



def create_2d_detector_container(context, camera_id=0, image_type="image_raw"):
    conf = context.launch_configurations
    use_bytetrack = conf["use_bytetrack"]
    ns = "/perception/object_recognition/detection"
    yolox_ns = ns
    if use_bytetrack == "true":
        yolox_ns = "/perception/object_recognition/detection/yolox"
    input_image_topic = f"/sensing/camera/camera{camera_id}/{image_type}"
    yolox_output_topics = yolox_ns + "/rois"
    yolox_package_dir = Path(get_package_share_directory("autoware_tensorrt_yolox"))
    yolox_param_path = Path(yolox_package_dir, "config", "yolox_s_plus_opt.param.yaml").as_posix()
    yolox_params = {
        "model_path": conf["yolox_model_path"],
        "label_path": conf["yolox_label_path"],
        "color_map_path": conf["yolox_color_map_path"],
    }
    yolox_node = ComposableNode(
        package="autoware_tensorrt_yolox",
        plugin="autoware::tensorrt_yolox::TrtYoloXNode",  # Adjust plugin name as needed
        name="tensorrt_yolox_node" + str(camera_id),
        namespace=ns,
        parameters=[
            yolox_param_path, yolox_params
        ],
        remappings=[
            ("~/in/image", input_image_topic),
            ("~/out/objects", yolox_output_topics + camera_id),
        ],
    )

    decompress_param_path=Path(
        get_package_share_directory("autoware_image_transport_decompressor"),
        "config",
        "image_transport_decompressor.param.yaml",
    ).as_posix()
    decompresor_node = ComposableNode(
        package="autoware_image_transport_decompressor",
        plugin="autoware::image_preprocessor::ImageTransportDecompressor",
        name="decompress_node" + str(camera_id),
        namespace=ns,
        parameters=[decompress_param_path],
        remappings=[
            ("~/input/compressed_image", input_image_topic + "/compressed"),
            ("~/output/raw_image", input_image_topic),
        ],
    )

    composable_nodes = [yolox_node, decompresor_node]

    bytetrack_package_dir = Path(get_package_share_directory("autoware_bytetrack"))
    bytetrack_param_path = Path(bytetrack_package_dir, "config", "bytetrack.param.yaml").as_posix()
    if use_bytetrack == "true":
        bytetrack_node = ComposableNode(
            package="autoware_bytetrack",
            plugin="autoware::bytetrack::ByteTrackNode",
            name="bytetrack_node" + str(camera_id),
            namespace=ns,
            parameters=[
                bytetrack_param_path,
            ],
            remappings=[
                ("~/in/rect", yolox_ns + "/rois" + str(camera_id)),
                ("~/out/objects", ns + "/rois" + str(camera_id)),
                ("~/out/objects/debug/uuid", ns + "/rois" + str(camera_id) + "/rois/debug/uuid"),
            ],
        )
        composable_nodes.append(bytetrack_node)

    container =  ComposableNodeContainer(
        name="camera_2d_detector_container" + str(camera_id),
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )
    return GroupAction([PushRosNamespace(ns), container])


def launch_camera_2d_detector(context: LaunchContext) -> list:  # for launching tensorrt_yolox
    conf = context.launch_configurations
    if conf["with_2d_detector"] != "true":
        return [LogInfo(msg="2D detector is not launched.")]
    camera_ids = conf["camera_ids"].split(",")
    camera_2d_detector_containers = [create_2d_detector_container(context, camera_id=camera_id, image_type="image_raw") for camera_id in camera_ids]
    return camera_2d_detector_containers