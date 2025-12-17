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

from driving_log_replayer_v2.scenario import load_sample_scenario
from driving_log_replayer_v2.time_step_based_trajectory import TimeStepBasedTrajectoryScenario


def test_scenario() -> None:
    scenario: TimeStepBasedTrajectoryScenario = load_sample_scenario(
        "time_step_based_trajectory",
        TimeStepBasedTrajectoryScenario,
    )
    assert scenario.ScenarioName == "sample_time_step_based_trajectory"
    assert scenario.Evaluation.UseCaseName == "time_step_based_trajectory"
    assert scenario.Evaluation.UseCaseFormatVersion == "0.1.0"
    assert scenario.Evaluation.Conditions.control_conditions is None


def test_scenario_with_empty_conditions() -> None:
    """Test that control_conditions can be None or empty list."""
    scenario: TimeStepBasedTrajectoryScenario = load_sample_scenario(
        "time_step_based_trajectory",
        TimeStepBasedTrajectoryScenario,
    )
    # Verify that None is acceptable for control_conditions
    assert scenario.Evaluation.Conditions.control_conditions is None
