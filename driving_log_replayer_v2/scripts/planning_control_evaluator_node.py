#!/usr/bin/env python3

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

from collections.abc import Callable

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from tier4_metric_msgs.msg import MetricArray

from driving_log_replayer_v2.diagnostics import DiagnosticsResult
from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.evaluator import evaluator_main
from driving_log_replayer_v2.planning_control import PlanningControlResult
from driving_log_replayer_v2.planning_control import PlanningControlScenario


class PlanningControlEvaluator(DLREvaluatorV2):
    def __init__(
        self,
        name: str,
        scenario_class: Callable = PlanningControlScenario,
        result_class: Callable = PlanningControlResult,
    ) -> None:
        super().__init__(name, scenario_class, result_class)
        self._scenario: PlanningControlScenario
        self._result: PlanningControlResult
        self._diag_result: DiagnosticsResult = DiagnosticsResult(
            self._scenario.IncludeUseCase.Conditions
        )

        self._latest_control_metrics = MetricArray()

        self.__sub_control_metrics = self.create_subscription(
            MetricArray,
            "/control/control_evaluator/metrics",
            self.control_cb,
            1,
        )

        self.__sub_autonomous_emergency_braking = self.create_subscription(
            MetricArray,
            "/control/autonomous_emergency_braking/metrics",
            self.aeb_cb,
            1,
        )

        self.__sub_diag = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diag_cb,
            100,
        )

    def aeb_cb(self, msg: MetricArray) -> None:
        self._result.set_frame(msg, self._latest_control_metrics)
        if self._result.frame != {}:
            self._result_writer.write_result(self._result)

    def control_cb(self, msg: MetricArray) -> None:
        self._latest_control_metrics = msg

    def diag_cb(self, msg: DiagnosticArray) -> None:
        if len(msg.status) == 0:
            return
        diag_status: DiagnosticStatus = msg.status[0]
        if (
            diag_status.hardware_id
            not in self._scenario.IncludeUseCase.Conditions.target_hardware_ids
        ):
            return
        self._diag_result.set_frame(msg)
        if self._diag_result.frame != {}:
            self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluatorV2:
    return PlanningControlEvaluator("planning_control_evaluator")


if __name__ == "__main__":
    main()
