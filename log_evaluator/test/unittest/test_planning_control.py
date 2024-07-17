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

from log_evaluator.planning_control import PlanningControlScenario
from log_evaluator.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: PlanningControlScenario = load_sample_scenario(
        "planning_control",
        PlanningControlScenario,
    )
    assert scenario.ScenarioName == "sample_planning_control"
    assert scenario.Evaluation.Conditions.ControlConditions[0].condition_type == "all_of"
