# Copyright (c) 2025 TIER IV.inc
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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import GroupAction
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode


def create_2d_detector_container(
    yolox_param_file: str,
    yolox_model_path: str,
    yolox_label_path: str,
    yolox_color_map_path: str,
    use_bytetrack: str,
    camera_id: str,
    image_type: str,
) -> GroupAction:
    """
    Create a container with composable nodes for 2D object detection from camera images.

    Args:
        yolox_param_file: Path to the YOLOX parameter file.
        yolox_model_path: Path to the YOLOX model file.
        yolox_label_path: Path to the YOLOX label file.
        yolox_color_map_path: Path to the YOLOX color map file.
        use_bytetrack: Whether to include the ByteTrack node ('true' or 'false').
        camera_id: Camera identifier (e.g., '0', '1').
        image_type: Type of image topic (e.g., 'image_raw', 'image_rect_color').

    Returns:
        GroupAction containing the detector container.

    """
    base_ns = "/perception/object_recognition/detection"
    yolox_ns = base_ns if use_bytetrack != "true" else f"{base_ns}/yolox"
    input_image_topic = f"/sensing/camera/camera{camera_id}/{image_type}"
    yolox_output_topic = f"{yolox_ns}/rois"

    # YOLOX node
    yolox_package_dir = Path(get_package_share_directory("autoware_tensorrt_yolox"))
    yolox_param_path = Path(yolox_package_dir, "config", yolox_param_file).as_posix()
    yolox_params = {
        "model_path": yolox_model_path,
        "label_path": yolox_label_path,
        "color_map_path": yolox_color_map_path,
    }
    yolox_node = ComposableNode(
        package="autoware_tensorrt_yolox",
        plugin="autoware::tensorrt_yolox::TrtYoloXNode",
        name=f"tensorrt_yolox_node{camera_id}",
        namespace=base_ns,
        parameters=[yolox_param_path, yolox_params],
        remappings=[
            ("~/in/image", input_image_topic),
            ("~/out/objects", f"{yolox_output_topic}{camera_id}"),
        ],
    )

    # Decompressor node
    decompress_param_path = Path(
        get_package_share_directory("autoware_image_transport_decompressor"),
        "config",
        "image_transport_decompressor.param.yaml",
    ).as_posix()
    decompressor_node = ComposableNode(
        package="autoware_image_transport_decompressor",
        plugin="autoware::image_preprocessor::ImageTransportDecompressor",
        name=f"decompress_node{camera_id}",
        namespace=base_ns,
        parameters=[decompress_param_path],
        remappings=[
            ("~/input/compressed_image", f"{input_image_topic}/compressed"),
            ("~/output/raw_image", input_image_topic),
        ],
    )

    composable_nodes = [yolox_node, decompressor_node]

    # Optional ByteTrack node
    if use_bytetrack == "true":
        bytetrack_package_dir = Path(get_package_share_directory("autoware_bytetrack"))
        bytetrack_param_path = Path(
            bytetrack_package_dir, "config", "bytetrack.param.yaml"
        ).as_posix()
        bytetrack_node = ComposableNode(
            package="autoware_bytetrack",
            plugin="autoware::bytetrack::ByteTrackNode",
            name=f"bytetrack_node{camera_id}",
            namespace=base_ns,
            parameters=[bytetrack_param_path],
            remappings=[
                ("~/in/rect", f"{yolox_ns}/rois{camera_id}"),
                ("~/out/objects", f"{base_ns}/rois{camera_id}"),
                ("~/out/objects/debug/uuid", f"{base_ns}/rois{camera_id}/rois/debug/uuid"),
            ],
        )
        composable_nodes.append(bytetrack_node)

    container = ComposableNodeContainer(
        name=f"camera_2d_detector_container{camera_id}",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )
    return GroupAction([PushRosNamespace(base_ns), container])


def launch_camera_2d_detector(context: LaunchContext) -> list:
    """
    Launch 2D detector containers for all specified cameras if enabled in configuration.

    Args:
        context: LaunchContext containing launch configurations.

    Returns:
        List of GroupAction objects for each camera detector container.

    """
    conf = context.launch_configurations
    yolox_param_file = conf.get("yolox_param_file", "")
    yolox_model_path = conf.get("yolox_model_path", "")
    yolox_label_path = conf.get("yolox_label_path", "")
    yolox_color_map_path = conf.get("yolox_color_map_path", "")
    use_bytetrack = conf.get("use_bytetrack", "false")
    camera_ids_str = conf.get("camera_ids", "")
    if not camera_ids_str.strip():
        return [
            LogInfo(
                msg="2D detector cannot be launched because no camera IDs specified for 2D detector."
            )
        ]
    camera_ids = camera_ids_str.split(",")
    image_type = conf.get("image_type", "image_raw")
    return [
        create_2d_detector_container(
            yolox_param_file,
            yolox_model_path,
            yolox_label_path,
            yolox_color_map_path,
            use_bytetrack,
            camera_id,
            image_type,
        )
        for camera_id in camera_ids
    ]
