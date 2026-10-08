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
import pytest
from t4perceval import FRAME
from t4perceval import Recording
from t4perceval import Store
from t4perceval import TimePoint
from t4perceval import TIMESTAMP
from t4perceval import Trackings3D
from t4perceval import Transform3D

from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import DEFAULT_TOLERANCE_NS
from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import EGO_TRANSFORM_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import GROUND_TRUTH_PATH
from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import GroundTruthScene
from driving_log_replayer_v2.perception.t4perceval_adapter.ground_truth import restamp
from driving_log_replayer_v2.perception.t4perceval_adapter.labels import build_label_registry

FRAME_INTERVAL_NS = 100_000_000  # 10 Hz
BASE_TIME_NS = 1_624_157_578_750_212_000


def make_scene(num_frames: int, *, with_transforms: bool = True) -> GroundTruthScene:
    registry = build_label_registry()
    store = Store()
    for frame in range(num_frames):
        at = TimePoint.at(frame=frame, timestamp_ns=BASE_TIME_NS + frame * FRAME_INTERVAL_NS)
        store.log(
            GROUND_TRUTH_PATH,
            Trackings3D(
                position=[[float(frame), 0.0, 0.0]],
                quaternion=[[0.0, 0.0, 0.0, 1.0]],
                size=[[1.0, 2.0, 1.5]],
                class_id=[registry.class_id("car")],
                confidence=[1.0],
                instance_id=[0],
            ),
            at=at,
            frame_id="map",
        )
        if with_transforms:
            store.log(
                EGO_TRANSFORM_PATH,
                Transform3D(
                    translation=[10.0, 0.0, 0.0],
                    rotation=[0.0, 0.0, 0.0, 1.0],
                    child_frame_id="base_link",
                ),
                at=at,
                frame_id="map",
            )
    return GroundTruthScene(Recording.of(store, labels=registry))


def test_nearest_frame_within_tolerance() -> None:
    scene = make_scene(5)
    assert scene.num_frames == 5  # noqa: PLR2004
    exact = scene.nearest(BASE_TIME_NS + 2 * FRAME_INTERVAL_NS)
    assert exact is not None
    assert (exact.frame, exact.frame_name) == (2, "2")
    # 40 ms after frame 2 is still frame 2, 60 ms after is frame 3
    assert scene.nearest(BASE_TIME_NS + 2 * FRAME_INTERVAL_NS + 40_000_000).frame == 2  # noqa: PLR2004
    assert scene.nearest(BASE_TIME_NS + 2 * FRAME_INTERVAL_NS + 60_000_000).frame == 3  # noqa: PLR2004


def test_nearest_frame_tolerance_boundary() -> None:
    scene = make_scene(2)
    last = BASE_TIME_NS + FRAME_INTERVAL_NS
    assert scene.nearest(last + DEFAULT_TOLERANCE_NS).frame == 1
    assert scene.nearest(last + DEFAULT_TOLERANCE_NS + 1) is None
    assert scene.nearest(BASE_TIME_NS - DEFAULT_TOLERANCE_NS - 1) is None
    assert scene.nearest(BASE_TIME_NS, tolerance_ns=0).frame == 0


def test_the_same_frame_may_serve_several_estimations() -> None:
    scene = make_scene(3)
    first = scene.nearest(BASE_TIME_NS + 10_000_000)
    second = scene.nearest(BASE_TIME_NS + 20_000_000)
    assert first.frame == second.frame == 0


def test_empty_scene_has_no_frame() -> None:
    assert make_scene(0).nearest(BASE_TIME_NS) is None


def test_frame_chunks_are_restamped_to_the_evaluation_frame() -> None:
    scene = make_scene(3)
    chunks = scene.frame_chunks(2, at=7)
    assert [str(chunk.entity_path) for chunk in chunks] == [GROUND_TRUTH_PATH, EGO_TRANSFORM_PATH]
    for chunk in chunks:
        assert chunk.index(FRAME).times.tolist() == [7]
        # the timestamp keeps the ground truth time
        assert chunk.index(TIMESTAMP).times.tolist() == [BASE_TIME_NS + 2 * FRAME_INTERVAL_NS]
    assert chunks[0].frame_id == "map"
    assert np.allclose(chunks[0].columns[next(iter(chunks[0].columns))].values[0][0], 2.0)


def test_frame_chunks_without_transforms() -> None:
    scene = make_scene(2, with_transforms=False)
    assert scene.has_transforms is False
    assert len(scene.frame_chunks(1, at=1)) == 1


def test_missing_frame_raises() -> None:
    scene = make_scene(2)
    with pytest.raises(KeyError):
        scene.objects_chunk(5)


def test_restamp_keeps_other_timelines() -> None:
    scene = make_scene(1)
    chunk = restamp(scene.objects_chunk(0), 42)
    assert chunk.index(FRAME).times.tolist() == [42]
    assert chunk.index(TIMESTAMP).times.tolist() == [BASE_TIME_NS]
