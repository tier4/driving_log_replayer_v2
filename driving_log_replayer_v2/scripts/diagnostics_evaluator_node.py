#!/usr/bin/env python3

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


from collections.abc import Callable

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus

from driving_log_replayer_v2.diagnostics import DiagnosticsResult
from driving_log_replayer_v2.diagnostics import DiagnosticsScenario
from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.evaluator import evaluator_main


class DiagnosticsEvaluator(DLREvaluatorV2):
    def __init__(
        self,
        name: str,
        scenario_class: Callable = DiagnosticsScenario,
        result_class: Callable = DiagnosticsResult,
    ) -> None:
        super().__init__(name, scenario_class, result_class)
        self._scenario: DiagnosticsScenario
        self._result: DiagnosticsResult

        self.__sub_diag = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diag_cb,
            100,
        )

    def diag_cb(self, msg: DiagnosticArray) -> None:
        if len(msg.status) == 0:
            return
        diag_status: DiagnosticStatus = msg.status[0]
        if diag_status.hardware_id not in self._scenario.Evaluation.Conditions.target_ids:
            return
        self._result.set_frame(msg)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluatorV2:
    return DiagnosticsEvaluator("diagnostics_evaluator")


if __name__ == "__main__":
    main()
