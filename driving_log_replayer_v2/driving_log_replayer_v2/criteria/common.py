# Copyright (c) 2023 TIER IV.inc
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

from enum import Enum
from numbers import Number

__all__ = (
    "CriteriaLevel",
    "CriteriaMethod",
    "SuccessFail",
    "load_levels",
    "load_methods",
)


class SuccessFail(Enum):
    """Enum object represents evaluated result is success or fail."""

    SUCCESS = "Success"
    FAIL = "Fail"

    def __str__(self) -> str:
        return self.value

    def is_success(self) -> bool:
        """
        Return whether success or fail.

        Returns
        -------
            bool: Success or fail.

        """
        return self == SuccessFail.SUCCESS

    def __and__(self, other: SuccessFail) -> SuccessFail:
        return SuccessFail.SUCCESS if self.is_success() and other.is_success() else SuccessFail.FAIL


class CriteriaLevel(Enum):
    """
    Enum object represents criteria level.

    PERFECT == 100.0
    HARD    >= 75.0
    NORMAL  >= 50.0
    EASY    >= 25.0

    CUSTOM  >= SCORE YOU SPECIFIED [0.0, 100.0]
    """

    PERFECT = 100.0
    HARD = 75.0
    NORMAL = 50.0
    EASY = 25.0

    CUSTOM = None

    def is_valid(self, score: Number, *, is_error: bool) -> bool:
        """
        Return whether the score satisfied the level.

        Args:
        ----
            score (Number): Calculated score.
            is_error (bool): Indicates whether input score is error or not.
                if True, higher score is better. Otherwise lower error is better.

        Returns:
        -------
            bool: Whether the score satisfied the level.

        """
        return score <= self.value if is_error else score >= self.value

    @classmethod
    def from_str(cls, value: str) -> CriteriaLevel:
        """
        Construct instance from.

        Args:
        ----
            value (str): _description_

        Returns:
        -------
            CriteriaLevel: _description_

        """
        name: str = value.upper()
        assert name != "CUSTOM", "If you want to use custom level, use from_number."
        assert name in cls.__members__, "value must be PERFECT, HARD, NORMAL, or EASY"
        return cls.__members__[name]

    @classmethod
    def from_number(cls, value: Number) -> CriteriaLevel:
        """
        Construct `CriteriaLevel.CUSTOM` with custom value.

        Args:
        ----
            value (Number): Level value which is must be [0.0, 100.0].

        Returns:
        -------
            CriteriaLevel: `CriteriaLevel.CUSTOM` with custom value.

        """
        if cls.CUSTOM._value_ is not None and float(value) != cls.CUSTOM._value_:
            err_msg = "Cannot use different value for CUSTOM of CriteriaLevel."
            raise ValueError(err_msg)
        min_range = 0.0
        max_range = 100.0
        assert min_range <= value <= max_range, (
            f"Custom level must be [0.0, 100.0], but got {value}."
        )
        cls.CUSTOM._value_ = float(value)
        return cls.CUSTOM


class CriteriaMethod(Enum):
    """
    Enum object represents methods of criteria .

    Attributes
    ----------
        - NUM_TP: TP (or TN) rate for all estimated and GT objects `NumTP / (NumTP + NumFP)`.
        - NUM_GT_TP: TP (or TN) rate for all GT objects `NumTP / NumGT`.
        - LABEL: Whether label is correct or not.
        - VELOCITY_X_ERROR: Error of x direction velocity [m/s].
        - VELOCITY_Y_ERROR: Error of y direction velocity [m/s].
        - SPEED_ERROR: Error of speed [m/s].
        - YAW_ERROR: Error of yaw [rad].
        - METRICS_SCORE: Accuracy score for classification, otherwise mAP score is used.
        - METRICS_SCORE_MAPH: mAPH score.

    """

    NUM_TP = "num_tp"
    NUM_GT_TP = "num_gt_tp"
    LABEL = "label"
    VELOCITY_X_ERROR = "velocity_x_error"
    VELOCITY_Y_ERROR = "velocity_y_error"
    SPEED_ERROR = "speed_error"
    YAW_ERROR = "yaw_error"
    METRICS_SCORE = "metrics_score"
    METRICS_SCORE_MAPH = "metrics_score_maph"

    @classmethod
    def from_str(cls, value: str) -> CriteriaMethod:
        """
        Construct instance from name in string.

        Args:
        ----
            value (str): Name of enum.

        Returns:
        -------
            CriteriaMode: `CriteriaMode` instance.

        """
        name: str = value.upper()
        assert name in cls.__members__, (
            "value must be NUM_TP, LABEL, METRICS_SCORE, or METRICS_SCORE_MAPH"
        )
        return cls.__members__[name]


def load_methods(
    methods_input: str | list[str] | CriteriaMethod | list[CriteriaMethod],
) -> list[CriteriaMethod]:
    """
    Load `CriteriaMethod` enum.

    Args:
    ----
        methods_input (str | list[str] | CriteriaMethod | list[CriteriaMethod]): Criteria method instance or name.

    Returns:
    -------
        list[CriteriaMethod]: Instance.

    """
    if isinstance(methods_input, str):
        loaded_methods = [CriteriaMethod.from_str(methods_input)]
    elif isinstance(methods_input, CriteriaMethod):
        loaded_methods = [methods_input]
    elif isinstance(methods_input, list):
        if isinstance(methods_input[0], str):
            loaded_methods = [CriteriaMethod.from_str(method) for method in methods_input]
        elif isinstance(methods_input[0], CriteriaMethod):
            loaded_methods = methods_input

    for method in loaded_methods:
        assert isinstance(method, CriteriaMethod), f"Invalid type of method: {type(method)}"
    return loaded_methods


def load_levels(
    levels_input: str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel],
) -> list[CriteriaLevel]:
    """
    Load `CriteriaLevel`.

    Args:
    ----
        levels_input (str | list[str] | Number | list[Number] | CriteriaLevel | list[CriteriaLevel]): Criteria level instance, name or value.

    Returns:
    -------
        list[CriteriaLevel]: Instance.

    """
    if isinstance(levels_input, str):
        levels_output = [CriteriaLevel.from_str(levels_input)]
    elif isinstance(levels_input, Number):
        levels_output = [CriteriaLevel.from_number(levels_input)]
    elif isinstance(levels_input, CriteriaLevel):
        levels_output = [levels_input]
    elif isinstance(levels_input, list):
        if isinstance(levels_input[0], str):
            levels_output = [CriteriaLevel.from_str(level) for level in levels_input]
        elif isinstance(levels_input[0], Number):
            levels_output = [CriteriaLevel.from_number(level) for level in levels_input]
        elif isinstance(levels_input[0], CriteriaLevel):
            levels_output = levels_input
    for level in levels_output:
        assert isinstance(level, CriteriaLevel), f"Invalid type of level: {type(level)}"
    return levels_output
