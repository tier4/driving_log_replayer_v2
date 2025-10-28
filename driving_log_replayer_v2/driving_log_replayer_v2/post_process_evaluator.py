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
from typing import Any

from pydantic import BaseModel
from pydantic import model_validator


class FrameResult(BaseModel):
    is_valid: bool
    data: Any | None = None
    invalid_reason: str | None = None
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
    @abstractmethod
    def frame_evaluate(
        self,
        header_timestamp_microsec: int,
        subscribed_timestamp_microsec: int,
        data: object,
        *args: Any,
        **kwargs: Any,
    ) -> FrameResult:
        raise NotImplementedError
