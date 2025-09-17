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

from autoware_internal_planning_msgs.msg import PlanningFactorArray
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import String
from tier4_metric_msgs.msg import MetricArray

from driving_log_replayer_v2.diagnostics import DiagnosticsResult
from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.evaluator import evaluator_main
from driving_log_replayer_v2.planning_control import MetricResult
from driving_log_replayer_v2.planning_control import PlanningControlScenario
from driving_log_replayer_v2.planning_control import PlanningFactorResult
from driving_log_replayer_v2.result import ResultWriter


class PlanningControlEvaluator(DLREvaluatorV2):
    def __init__(
        self,
        name: str,
        scenario_class: Callable = PlanningControlScenario,
        result_class: Callable = MetricResult,
    ) -> None:
        super().__init__(
            name, scenario_class, result_class, "/driving_log_replayer/metrics_results"
        )
        self._scenario: PlanningControlScenario
        self._result: MetricResult

        self._latest_control_metrics = MetricArray()

        metric_conditions = self._scenario.Evaluation.Conditions.MetricConditions
        if metric_conditions == []:
            skip_test = {
                "Result": {"Success": True, "Summary": "Metric Test is Skipped"},
                "Stamp": {"System": 0.0},
                "Frame": {},
            }
            result_str = self._result_writer.write_line(skip_test)
            self._pub_result.publish(String(data=result_str))

        else:
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

        pf_conditions = self._scenario.Evaluation.Conditions.PlanningFactorConditions
        if pf_conditions != []:
            self._pub_pf_result = self.create_publisher(
                String, "/driving_log_replayer/planning_factor_results", 1
            )

            self._planning_factor_result = PlanningFactorResult(pf_conditions)
            self._planning_factor_result_writer: ResultWriter = ResultWriter(
                self._result_archive_path.joinpath("planning_factor_result.jsonl"),
                self.get_clock(),
                pf_conditions,
            )

            self.__sub_factors = []
            for pfc in pf_conditions:
                self.__sub_factors.append(
                    self.create_subscription(
                        PlanningFactorArray,
                        pfc.topic,
                        lambda msg, topic=pfc.topic: self.factor_cb(msg, topic),
                        1,
                    )
                )

        if self._scenario.include_use_case is not None:
            self._pub_diag_result = self.create_publisher(
                String, "/driving_log_replayer/diagnostics_results", 1
            )

            diag_conditions = self._scenario.include_use_case.Conditions
            self._diag_result: DiagnosticsResult = DiagnosticsResult(diag_conditions)

            self._diag_result_writer: ResultWriter = ResultWriter(
                self._result_archive_path.joinpath("diag_result.jsonl"),
                self.get_clock(),
                diag_conditions,
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
            res_str = self._result_writer.write_result(self._result)
            if self._pub_result is not None:
                self._pub_result.publish(String(data=res_str))

    def control_cb(self, msg: MetricArray) -> None:
        self._latest_control_metrics = msg

    def diag_cb(self, msg: DiagnosticArray) -> None:
        if len(msg.status) == 0:
            return
        diag_status: DiagnosticStatus = msg.status[0]
        if (
            diag_status.hardware_id
            not in self._scenario.include_use_case.Conditions.target_hardware_ids
        ):
            return
        self._diag_result.set_frame(msg)
        if self._diag_result.frame != {}:
            res_str = self._diag_result_writer.write_result(self._diag_result)
            self._pub_diag_result.publish(String(data=res_str))

    def factor_cb(self, msg: PlanningFactorArray, topic: str) -> None:
        self._planning_factor_result.set_frame(msg, topic)
        if self._planning_factor_result.frame != {}:
            res_str = self._planning_factor_result_writer.write_result(self._planning_factor_result)
            self._pub_pf_result.publish(String(data=res_str))


@evaluator_main
def main() -> DLREvaluatorV2:
    return PlanningControlEvaluator("planning_control_evaluator")


if __name__ == "__main__":
    main()
