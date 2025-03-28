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

import re

from perception_eval.common.evaluation_task import EvaluationTask

from driving_log_replayer_v2.launch.perception import EVALUATION_DEGRADATION_TOPIC
from driving_log_replayer_v2.launch.perception import EVALUATION_DETECTION_TOPIC
from driving_log_replayer_v2.launch.perception import EVALUATION_PREDICTION_TOPIC
from driving_log_replayer_v2.launch.perception import EVALUATION_TRACKING_TOPIC


def convert_topic_list_from_regex_str(record_topic_str: str) -> list[str]:
    """Extract pure topic names list from the regex topic name string."""
    topics = []
    for raw in record_topic_str.strip().split("|"):
        line = raw.strip().rstrip("\\")
        if not line:
            continue
        line = re.sub(r"^\^/?", "/", line)
        line = re.sub(r"\$$", "", line)
        topics.append(line)
    return topics


def load_evaluation_topics(
    evaluation_detection_topic_regex: str,
    evaluation_tracking_topic_regex: str,
    evaluation_prediction_topic_regex: str,
) -> dict[EvaluationTask, list[str]]:
    evaluation_topics: dict[EvaluationTask, list[str]] = {}
    if evaluation_detection_topic_regex == "":
        evaluation_detection_topic = convert_topic_list_from_regex_str(
            EVALUATION_DETECTION_TOPIC,
        )
    else:
        evaluation_detection_topic = convert_topic_list_from_regex_str(
            evaluation_detection_topic_regex,
        )
    evaluation_topics[EvaluationTask.DETECTION] = evaluation_detection_topic
    if evaluation_tracking_topic_regex == "":
        evaluation_tracking_topic = convert_topic_list_from_regex_str(
            EVALUATION_TRACKING_TOPIC,
        )
    else:
        evaluation_tracking_topic = convert_topic_list_from_regex_str(
            evaluation_tracking_topic_regex,
        )
    evaluation_topics[EvaluationTask.TRACKING] = evaluation_tracking_topic
    if evaluation_prediction_topic_regex == "":
        evaluation_prediction_topic = convert_topic_list_from_regex_str(
            EVALUATION_PREDICTION_TOPIC,
        )
    else:
        evaluation_prediction_topic = convert_topic_list_from_regex_str(
            evaluation_prediction_topic_regex,
        )
    evaluation_topics[EvaluationTask.PREDICTION] = evaluation_prediction_topic
    return evaluation_topics


def load_degradation_topics(
    evaluation_degradation_topic_regex: str,
) -> str:
    if evaluation_degradation_topic_regex == "":
        evaluation_degradation_topic = convert_topic_list_from_regex_str(
            EVALUATION_DEGRADATION_TOPIC,
        )
    else:
        evaluation_degradation_topic = convert_topic_list_from_regex_str(
            evaluation_degradation_topic_regex,
        )

    assert len(evaluation_degradation_topic) == 1, "Degradation topic must be one"
    return evaluation_degradation_topic[0]
