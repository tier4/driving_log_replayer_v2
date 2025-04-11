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

import numpy as np

from driving_log_replayer_v2.perception.models import Metrics
from perception_eval.tool import PerceptionAnalyzer3D


def analyze(analyzer: PerceptionAnalyzer3D) -> dict[str, dict[str, dict[str, float]]]:
    result: dict[str, dict[str, dict[str, float]]] = {}
    result = {
        distance: {
            label: {
                metrics: np.nan for metrics in Metrics.Score + Metrics.Error
            } for label in metrics.Label
        } for distance in metrics.DISTANCE
    }
    for i in range(len(Metrics.DISTANCE) - 1):
        analysis = analyzer.analyze(distance=(Metrics.DISTANCE[i], Metrics.DISTANCE[i + 1]))
        for label in Metrics.Label:
            score_df = data.score
            for score in Metrics.Score:
                value = score_df[score][label]
                value.store_csv()
            # error
            error_df = data.error
            for error in Metrics.Error:
                value = error_df[error][label]
                value.store_csv()
    return result