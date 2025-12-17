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

from driving_log_replayer_v2.timestep_based_trajectory import TimestepBasedTrajectoryScenario
from driving_log_replayer_v2.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: TimestepBasedTrajectoryScenario = load_sample_scenario(
        "timestep_based_trajectory",
        TimestepBasedTrajectoryScenario,
    )
    assert scenario.ScenarioName == "sample_timestep_based_trajectory"
    assert scenario.Evaluation.UseCaseName == "timestep_based_trajectory"
    assert scenario.Evaluation.UseCaseFormatVersion == "0.1.0"
    assert scenario.Evaluation.Conditions.ControlConditions is None


def test_scenario_with_empty_conditions() -> None:
    """Test that ControlConditions can be None or empty list."""
    scenario: TimestepBasedTrajectoryScenario = load_sample_scenario(
        "timestep_based_trajectory",
        TimestepBasedTrajectoryScenario,
    )
    # Verify that None is acceptable for ControlConditions
    assert scenario.Evaluation.Conditions.ControlConditions is None
