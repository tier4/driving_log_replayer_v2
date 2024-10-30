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

from diagnostic_msgs.msg import DiagnosticArray
from tier4_metric_msgs.msg import MetricArray

from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.evaluator import evaluator_main
from driving_log_replayer_v2.planning_control import PlanningControlResult
from driving_log_replayer_v2.planning_control import PlanningControlScenario

TARGET_DIAG_NAME = "control_validator: control_validation_rolling_back"


class PlanningControlEvaluator(DLREvaluatorV2):
    def __init__(self, name: str) -> None:
        super().__init__(name, PlanningControlScenario, PlanningControlResult)
        self._scenario: PlanningControlScenario
        self._result: PlanningControlResult

        self.__sub_planning_metrics = self.create_subscription(
            MetricArray,
            "/planning/planning_evaluator/metrics",
            lambda msg, module_type="planning": self.metrics_cb(msg, module_type),
            1,
        )  # 今これを出すやつがいない

        self.__sub_control_metrics = self.create_subscription(
            MetricArray,
            "/control/control_evaluator/metrics",
            lambda msg, module_type="control": self.metrics_cb(msg, module_type),
            1,
        )

        self.__sub_validator_diagnostics = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diagnostics_cb,
            100,
        )

    def metrics_cb(self, msg: MetricArray, module: str) -> None:
        self._result.set_metric_frame(msg, module)
        if self._result.frame != {}:
            self._result_writer.write_result(self._result)

    def diagnostics_cb(self, msg: DiagnosticArray) -> None:
        if len(msg.status) == 0:
            return
        diag_status = msg.status[0]
        if diag_status.name != TARGET_DIAG_NAME:
            return
        self._result.set_diag_frame(diag_status)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluatorV2:
    return PlanningControlEvaluator("planning_control_evaluator")


if __name__ == "__main__":
    main()
