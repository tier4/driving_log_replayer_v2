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

from typing import Literal

from pydantic import BaseModel

from driving_log_replayer_v2.scenario import Scenario


class Conditions(BaseModel):
    control_conditions: list | None = None


class Evaluation(BaseModel):
    UseCaseName: Literal["time_step_based_trajectory"]
    UseCaseFormatVersion: Literal["0.1.0"]
    Conditions: Conditions
    Datasets: list[dict]


class TimeStepBasedTrajectoryScenario(Scenario):
    Evaluation: Evaluation
