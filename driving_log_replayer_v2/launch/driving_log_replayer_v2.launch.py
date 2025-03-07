# Copyright (c) 2022 TIER IV.inc
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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from driving_log_replayer_v2.launch.argument import get_launch_arguments

PACKAGE_NAME = "driving_log_replayer_v2"


def launch_setup(context: LaunchContext) -> None:
    pre_process_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(PACKAGE_NAME), "/launch/pre-process.launch.py"]
        ),
    )
    pre_process_launch.execute(context)

    # simulation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [get_package_share_directory(PACKAGE_NAME), "/launch/simulation.launch.py"]
    #     ),
    # )
    # simulation_launch.execute(context)

    # # post-process.launch.pyを実行
    # post_process_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [get_package_share_directory("your_package_name"), "/launch/post-process.launch.py"]
    #     ),
    # )
    # post_process_launch.execute(context)
    return


def generate_launch_description() -> LaunchDescription:
    launch_arguments = get_launch_arguments()
    return LaunchDescription(
        [*launch_arguments, OpaqueFunction(function=launch_setup)],
    )
