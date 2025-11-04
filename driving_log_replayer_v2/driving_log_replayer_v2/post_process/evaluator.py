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

from __future__ import annotations

from abc import ABC
from abc import abstractmethod
from enum import Enum
import logging
from pathlib import Path
from typing import TypeVar

from pydantic import BaseModel
from pydantic import model_validator

from driving_log_replayer_v2.post_process.logger import configure_logger


class InvalidReason(str, Enum):
    """Invalid reason for common format."""


InvalidReasonType = TypeVar("InvalidReasonType", bound=InvalidReason)


class FrameResultData:
    """Data for common format."""


FrameResultDataType = TypeVar("FrameResultDataType", bound=FrameResultData)


class FrameResult(BaseModel):
    """Frame result for common format."""

    is_valid: bool
    data: FrameResultDataType | None = None
    invalid_reason: InvalidReasonType | None = None
    skip_counter: int

    @model_validator(mode="after")
    def check_validity(self) -> FrameResult:
        if self.is_valid and self.data is None:
            err_msg = "data must be set when is_valid is True"
            raise ValueError(err_msg)
        if not self.is_valid and self.invalid_reason is None:
            err_msg = "invalid_reason must be set when is_valid is False"
            raise ValueError(err_msg)
        return self


class Evaluator(ABC):
    """
    Base class for evaluator.

    Responsible for following items:
        evaluating each frame
    """

    def __init__(self, result_archive_path: str, evaluation_topic: str) -> None:
        # instance variables
        self._logger: logging.Logger

        result_archive_w_topic_path = Path(result_archive_path)
        dir_name = evaluation_topic.lstrip("/").replace("/", ".")
        result_archive_w_topic_path = result_archive_w_topic_path.joinpath(dir_name)
        result_archive_w_topic_path.mkdir(parents=True, exist_ok=True)
        self._logger = configure_logger(
            log_file_directory=result_archive_w_topic_path,
            console_log_level=logging.INFO,
            file_log_level=logging.INFO,
            logger_name=dir_name,
        )

    @abstractmethod
    def evaluate_frame(
        self,
        header_timestamp: int,  # do not care time unit
        subscribed_timestamp: int,  # do not care time unit
        data: FrameResultDataType,
    ) -> FrameResult:
        """
        Evaluate a single frame.

        Args:
            header_timestamp (int): Timestamp from the message header. Time unit is not specified.
            subscribed_timestamp (int): Timestamp when the message was subscribed. Time unit is not specified.
            data (FrameResultDataType): Data to be evaluated.

        Returns:
            FrameResult: The result of the evaluation.

        """
        raise NotImplementedError


EvaluatorType = TypeVar("EvaluatorType", bound=Evaluator)
