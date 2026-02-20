# Copyright (c) 2026 TIER IV.inc
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
from perception_eval.common.label import AutowareLabel
from perception_eval.common.label import Label
from perception_eval.common.object import DynamicObject
from perception_eval.common.schema import FrameID
from perception_eval.common.shape import Shape
from perception_eval.common.shape import ShapeType
from pydantic import ValidationError
from pyquaternion import Quaternion
import pytest

from driving_log_replayer_v2.perception_fp.models import Criteria
from driving_log_replayer_v2.perception_fp.models import NonDetectionArea
from driving_log_replayer_v2.perception_fp.models import PerceptionFP
from driving_log_replayer_v2.perception_fp.models import PerceptionFPScenario
from driving_log_replayer_v2.perception_fp.models import Polygon3D
from driving_log_replayer_v2.perception_fp.models import TimeStamp
from driving_log_replayer_v2.scenario import load_sample_scenario


def test_scenario() -> None:
    scenario: PerceptionFPScenario = load_sample_scenario("perception_fp", PerceptionFPScenario)
    assert scenario.Evaluation.Conditions.Criterion[0].criteria_name == "drivable_area"
    assert scenario.Evaluation.Conditions.Criterion[1].timestamp[0].start == 1649138852.0  # noqa


def test_polygon_clockwise_ok() -> None:
    NonDetectionArea(
        polygon_2d=[[10.0, 1.5, 0.0], [10.0, -1.5, 0.0], [0.0, -1.5, 0.0], [0.0, 1.5, 0.0]],
        z_min=0.0,
        z_max=1.5,
    )


def test_polygon_clockwise_ng() -> None:
    with pytest.raises(ValidationError):
        NonDetectionArea(
            polygon_2d=[[10.0, 1.5, 0.0], [0.0, 1.5, 0.0], [0.0, -1.5, 0.0], [10.0, -1.5, 0.0]],
            z_min=0.0,
            z_max=1.5,
        )


def test_timestamp_ok() -> None:
    TimeStamp(start=100.0, end=120.0)


def test_timestamp_ng() -> None:
    with pytest.raises(ValidationError):
        TimeStamp(start=120.0, end=100.0)


def test_timestamp_is_valid() -> None:
    timestamp = TimeStamp(start=100.0, end=120.0)
    assert timestamp.is_valid(110.0)
    assert not timestamp.is_valid(90.0)
    assert not timestamp.is_valid(130.0)


@pytest.fixture
def create_polygon3d() -> Polygon3D:
    return Polygon3D.from_args(
        [[10.0, 1.5, 0.0], [10.0, -1.5, 0.0], [0.0, -1.5, 0.0], [0.0, 1.5, 0.0]],
        frame_id="map",
        z_min=-10.0,
        z_max=10.0,
    )


def test_convert_map_to_base_link(create_polygon3d: Polygon3D) -> None:
    map_to_base_link = np.eye(4)
    map_to_base_link[:3, 3] = [5.0, 0.0, 0.0]
    map_to_base_link[:3, :3] = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    create_polygon3d.map_to_base_link = map_to_base_link
    create_polygon3d.convert_map_to_base_link()

    assert np.allclose(
        np.array(create_polygon3d.geom.exterior.coords)[:-1],
        np.array([[15.0, 1.5, 0.0], [15.0, -1.5, 0.0], [5.0, -1.5, 0.0], [5.0, 1.5, 0.0]]),
    )
    assert create_polygon3d.z_min == -10.0  # noqa
    assert create_polygon3d.z_max == 10.0  # noqa


@pytest.fixture
def create_dynamic_object() -> DynamicObject:
    return DynamicObject(
        unix_time=123,
        frame_id=FrameID.MAP,
        position=(0.5, 1.0, 3.0),
        orientation=Quaternion(),
        shape=Shape(ShapeType.BOUNDING_BOX, (1.0, 2.0, 6.0)),
        velocity=(1.0, 2.0, 3.0),
        semantic_score=0.5,
        semantic_label=Label(AutowareLabel.CAR, "car"),
    )


@pytest.fixture
def create_criteria() -> Criteria:
    return Criteria(
        criteria_name="test",
        PassRate=95.0,
        non_detection_area=NonDetectionArea(
            polygon_2d=[[10.0, 1.5, 0.0], [10.0, -1.5, 0.0], [0.0, -1.5, 0.0], [0.0, 1.5, 0.0]],
        ),
        timestamp=[TimeStamp(start=0.0, end=200.0)],
        topic_type=["bbox", "pointcloud"],
    )


def test_in_non_detection_area(
    create_dynamic_object: DynamicObject, create_criteria: Criteria
) -> None:
    perception_fp = PerceptionFP(name=create_criteria.criteria_name, condition=create_criteria)
    timestamp = create_dynamic_object.unix_time * 1e3  # convert from microsec to nanosec

    result = perception_fp.set_frame(
        timestamp=timestamp,
        frame_id="map",
        data=[create_dynamic_object],
        map_to_base_link=np.eye(4),
    )

    assert result == {
        "PassFail": {
            "Result": {
                "Total": "Fail",
                "Frame": "Fail",
            },
            "Info": {},
        },
    }
