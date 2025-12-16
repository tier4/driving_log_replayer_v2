#!/usr/bin/env python3

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

from collections.abc import Callable

from std_msgs.msg import String

from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.evaluator import evaluator_main
from driving_log_replayer_v2.open_loop_trajectory import OpenLoopTrajectoryScenario
from driving_log_replayer_v2.result import DummyResult


class OpenLoopTrajectoryEvaluator(DLREvaluatorV2):
    def __init__(
        self,
        name: str,
        scenario_class: Callable = OpenLoopTrajectoryScenario,
        result_class: Callable = DummyResult,
    ) -> None:
        super().__init__(
            name,
            scenario_class,
            result_class,
            "/driving_log_replayer/open_loop_trajectory/results",
        )
        self._scenario: OpenLoopTrajectoryScenario
        self._result: DummyResult

        # Set success dummy result for initial state
        dummy_result = {
            "Result": {
                "Success": True,
                "Summary": "Open Loop Trajectory Evaluation is initialized as successful. This use case only records rosbag data.",
            },
            "Stamp": {"System": 0.0},
            "Frame": {},
        }
        dummy_result_str = self._result_writer.write_line(dummy_result)
        self._pub_result.publish(String(data=dummy_result_str))


@evaluator_main
def main() -> DLREvaluatorV2:
    return OpenLoopTrajectoryEvaluator("open_loop_trajectory_evaluator")


if __name__ == "__main__":
    main()
