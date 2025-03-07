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


from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import OpaqueFunction

from driving_log_replayer_v2.launch.argument import ensure_arg_compatibility
from driving_log_replayer_v2.launch.argument import get_launch_arguments
from driving_log_replayer_v2.launch.ndt_convergence import launch_ndt_convergence
from driving_log_replayer_v2.launch.use_case import launch_use_case


def select_launch(context: LaunchContext) -> list:
    conf = context.launch_configurations
    if conf["use_case"] == "ndt_convergence":
        return launch_ndt_convergence(context)
    return launch_use_case()


def generate_launch_description() -> LaunchDescription:
    launch_arguments = get_launch_arguments()
    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=ensure_arg_compatibility),
            OpaqueFunction(function=select_launch),
        ],
    )
