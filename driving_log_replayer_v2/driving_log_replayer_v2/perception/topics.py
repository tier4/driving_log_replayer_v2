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
) -> dict[str, list[str]]:
    evaluation_topics: dict[str, list[str]] = {}
    evaluation_topics["detection"] = convert_topic_list_from_regex_str(
        evaluation_detection_topic_regex,
    )
    evaluation_topics["tracking"] = convert_topic_list_from_regex_str(
        evaluation_tracking_topic_regex,
    )
    evaluation_topics["prediction"] = convert_topic_list_from_regex_str(
        evaluation_prediction_topic_regex,
    )
    return evaluation_topics
