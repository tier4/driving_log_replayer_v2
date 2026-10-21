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

from dataclasses import dataclass
from pathlib import Path
import time

from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnShutdown
from launch.events.process import ProcessExited

from driving_log_replayer_v2.additional_launches import FAILURE_NAME
from driving_log_replayer_v2.additional_launches import initialize_run
from driving_log_replayer_v2.additional_launches import load_additional_launches
from driving_log_replayer_v2.additional_launches import ResolvedLaunch
from driving_log_replayer_v2.additional_launches import write_json_atomically
from driving_log_replayer_v2.shutdown_once import ShutdownOnce


@dataclass
class AdditionalLaunchRun:
    archive: Path
    run_id: str
    stopping: bool = False
    failed: bool = False

    def on_shutdown(self, event: object, context: LaunchContext) -> list:  # noqa: ARG002
        self.stopping = True
        return []

    def on_exit(
        self, specification: ResolvedLaunch, event: ProcessExited, context: LaunchContext
    ) -> list:
        if self.stopping or self.failed or context.is_shutdown:
            return []
        self.failed = True
        write_json_atomically(
            self.archive / FAILURE_NAME,
            {
                "run_id": self.run_id,
                "launch": specification.specification.model_dump(),
                "process_name": event.process_name,
                "returncode": event.returncode,
                "system_time": time.time(),
            },
        )
        return [ShutdownOnce(reason="AdditionalLaunchFailed")]


def launch_additional_launches(context: LaunchContext) -> list:
    conf = context.launch_configurations
    # Other use cases retain the existing actions and do not resolve extra packages.
    if conf["use_case"] != "obstacle_segmentation":
        return []
    launches = load_additional_launches(
        conf["scenario_path"], record_only=conf["record_only"] != "false"
    )
    if not launches:
        return []
    archive = Path(conf["result_archive_path"])
    state = AdditionalLaunchRun(archive, initialize_run(archive, launches))
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=state.on_shutdown))]
    for index, specification in enumerate(launches):
        process = ExecuteProcess(
            cmd=specification.command(),
            shell=False,
            output="screen",
            name=f"additional_launch_{index}",
        )

        def on_exit(
            event: ProcessExited,
            launch_context: LaunchContext,
            launch: ResolvedLaunch = specification,
        ) -> list:
            return state.on_exit(launch, event, launch_context)

        # Register before starting: even an immediate parser error must be recorded.
        actions.extend(
            [
                RegisterEventHandler(OnProcessExit(target_action=process, on_exit=on_exit)),
                process,
            ]
        )
    return actions
