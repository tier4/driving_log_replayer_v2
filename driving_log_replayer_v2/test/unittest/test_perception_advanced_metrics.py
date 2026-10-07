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

from __future__ import annotations

from collections import Counter
import hashlib
import json
import logging
import math
from pathlib import Path
from types import SimpleNamespace
from typing import ClassVar

import numpy as np
import pytest

try:
    from perception_eval.common.label import AutowareLabel
    from perception_eval.common.label import Label
    from perception_eval.common.object import DynamicObject
    from perception_eval.common.schema import FrameID
    from perception_eval.common.shape import Shape
    from perception_eval.common.shape import ShapeType
    from perception_eval.evaluation.metrics.detection.config import AdvancedDetectionMetricsConfig
    from perception_eval.evaluation.metrics.detection.frame import DetectionFrame
    from perception_eval.evaluation.metrics.detection.prepared import PreparedSamples
    from pyquaternion import Quaternion

    PREPARED_API_AVAILABLE = True
except ImportError:
    PREPARED_API_AVAILABLE = False

from driving_log_replayer_v2.perception import advanced_metrics
from driving_log_replayer_v2.perception.evaluator import PerceptionEvaluator
from driving_log_replayer_v2.post_process.evaluation_manager import IgnoreFrames
from driving_log_replayer_v2.post_process.evaluation_manager import parse_ignore_frames

NUM_REAL_KEPT_FRAMES = 3  # frames 0-2 of the real-API test, frame 3 is ignored

# ---------------------------------------------------------------------------------------------
# fakes standing in for perception_eval's prepared-samples API (same attribute names)
# ---------------------------------------------------------------------------------------------


class FakePreparedSamples:
    """Stand-in for perception_eval's PreparedSamples: frames, counts, warnings, to_npz()."""

    def __init__(self, frame_names: list[str], warnings: tuple[str, ...] = ()) -> None:
        self.schema_version = 1
        self.config_digest = "sha256:" + "0" * 64
        self.frames = tuple(SimpleNamespace(frame_name=name) for name in frame_names)
        self.counts = {
            "input_frames": len(frame_names),
            "skipped_frames": 0,
            "polygon_as_box": 0,
            "non_finite_score": 0,
            "pred_total": 3 * len(frame_names),
            "pred_kept": 3 * len(frame_names),
            "zero_score": 0,
        }
        self.warnings = tuple(warnings)
        self.producer = None

    def to_npz(self, path: str | Path, producer: dict | None = None) -> str:
        self.producer = producer
        path = Path(path)
        np.savez_compressed(path, frame_name=np.array([f.frame_name for f in self.frames]))
        return hashlib.sha256(path.read_bytes()).hexdigest()


class FakeMetricReport:
    def __init__(self, values: dict[str, float], coverage: dict | None = None) -> None:
        self.values = values
        self.coverage = coverage if coverage is not None else {}
        self.warnings: list[str] = []


class FakeSuite:
    """Records the frames it was given and returns the fake prepared samples / report."""

    instances: ClassVar[list[FakeSuite]] = []

    def __init__(self, config: object) -> None:
        self.config = config
        self.prepared_frames: list | None = None
        self.evaluate_prepared_calls = 0
        FakeSuite.instances.append(self)

    def prepare(self, frames: list, target_labels: list | None = None) -> FakePreparedSamples:  # noqa: ARG002
        self.prepared_frames = list(frames)
        return FakePreparedSamples([frame.frame_name for frame in frames])

    def evaluate_prepared(self, prepared: FakePreparedSamples) -> FakeMetricReport:
        self.evaluate_prepared_calls += 1
        return FakeMetricReport(
            {"detection/corner_mean_car": 0.5, "detection/corner_mean_bicycle": math.nan},
            coverage={"road": (len(prepared.frames), len(prepared.frames))},
        )


class FakeAdvancedConfig:
    class_names = ("car", "bicycle")

    def serialization(self) -> dict:
        return {
            "filters": [{"name": "road", "type": "region", "regions": ["road"]}],
            "components": [{"type": "corner_error", "tp_threshold": 2.0, "percentiles": (95.0,)}],
            "map": {
                "resolver": "t4_scene_directory",
                "data_root": "/secret",
                "mapping": {"a": "b"},
            },
            "score_source": "existence_probability",
            "polygon_shapes": "as_box",
        }


def create_metrics_config(*, advanced: bool = True) -> SimpleNamespace:
    return SimpleNamespace(
        detection_config=SimpleNamespace(
            advanced_detection_metrics=FakeAdvancedConfig() if advanced else None
        )
    )


def create_frame_result(frame_name: str, *, with_detection_frame: bool = True) -> SimpleNamespace:
    return SimpleNamespace(
        frame_name=frame_name,
        detection_frame=SimpleNamespace(frame_name=frame_name) if with_detection_frame else None,
    )


def create_meta(
    tmp_path: Path, num_frame_results: int = 5, num_ignored: int = 1
) -> advanced_metrics.ReportMeta:
    return advanced_metrics.ReportMeta(
        topic="/perception/object_recognition/objects",
        evaluation_task="detection",
        frame_id="base_link",
        t4_dataset_path=str(tmp_path / "dataset" / "scene_a"),
        evaluation_config_dict={"target_labels": ["car", "bicycle"], "max_x_position": 100.0},
        advanced_config=FakeAdvancedConfig(),
        num_frame_results=num_frame_results,
        num_ignored_frames=num_ignored,
    )


def reject_constant(name: str) -> None:
    err_msg = f"non-strict JSON constant: {name}"
    raise ValueError(err_msg)


def load_strict_report(out_dir: Path) -> dict:
    """json.loads refusing NaN/Infinity, like a strict JSON parser does."""
    text = (out_dir / advanced_metrics.REPORT_NAME).read_text()
    return json.loads(text, parse_constant=reject_constant)


@pytest.fixture
def fake_suite(monkeypatch: pytest.MonkeyPatch) -> type[FakeSuite]:
    FakeSuite.instances = []
    monkeypatch.setattr(advanced_metrics, "_load_suite_class", lambda: FakeSuite)
    return FakeSuite


# ---------------------------------------------------------------------------------------------
# frame_filter_digest
# ---------------------------------------------------------------------------------------------


def test_frame_filter_digest_is_stable_and_key_order_independent() -> None:
    config_a = {
        "target_labels": ["car", "pedestrian"],
        "max_x_position": 100.0,
        "max_y_position": 100.0,
        "min_point_numbers": [0, 0],
        "label_prefix": "autoware",
        "evaluation_task": "detection",  # not a filter key, must not matter
        "center_distance_thresholds": [[1.0, 1.0]],  # not a filter key either
    }
    config_b = {
        "label_prefix": "autoware",
        "min_point_numbers": [0, 0],
        "max_y_position": 100.0,
        "max_x_position": 100.0,
        "target_labels": ["car", "pedestrian"],
        "evaluation_task": "tracking",
        "max_distance": None,  # missing and null are the same
    }
    digest_a = advanced_metrics.frame_filter_digest(config_a)
    assert digest_a.startswith("sha256:")
    assert len(digest_a) == len("sha256:") + 64
    assert digest_a == advanced_metrics.frame_filter_digest(config_a)
    assert digest_a == advanced_metrics.frame_filter_digest(config_b)

    config_c = dict(config_a, max_x_position=50.0)
    assert advanced_metrics.frame_filter_digest(config_c) != digest_a


# ---------------------------------------------------------------------------------------------
# write_outputs
# ---------------------------------------------------------------------------------------------


def test_write_outputs_writes_strict_json_with_nan_as_null(tmp_path: Path) -> None:
    prepared = FakePreparedSamples(
        ["0", "1", "2"], warnings=("1 POLYGON-shaped object evaluated as box",)
    )
    report = FakeMetricReport(
        {
            "detection/corner_mean_car": 0.5,
            "detection/corner_mean_bicycle": math.nan,
            "inf": math.inf,
        },
        coverage={"road": (3, 3), "collision": (0, 3)},
    )
    report.warnings.append("filter 'collision' covered 0/3 frames")

    advanced_metrics.write_outputs(
        tmp_path, prepared=prepared, report=report, meta=create_meta(tmp_path)
    )

    data = load_strict_report(tmp_path)
    assert data["schema"] == "perception_eval.advanced_detection_report"
    assert data["schema_version"] == 1
    assert data["status"] == "ok"
    assert data["error"] is None
    assert data["values"] == {
        "detection/corner_mean_car": 0.5,
        "detection/corner_mean_bicycle": None,
        "inf": None,
    }
    assert data["coverage"] == {"road": [3, 3], "collision": [0, 3]}
    assert data["warnings"] == [
        "filter 'collision' covered 0/3 frames",
        "1 POLYGON-shaped object evaluated as box",
    ]
    assert data["frames"] == {
        "frame_results": 5,
        "ignored": 1,
        "evaluated": 4,
        "skipped": 0,
        "prepared": 3,
    }
    assert data["counts"] == {
        "pred_total": 9,
        "pred_kept": 9,
        "non_finite_score": 0,
        "polygon_as_box": 0,
        "zero_score": 0,
        "skipped_frames": 0,
    }
    assert data["topic"] == "/perception/object_recognition/objects"
    assert data["evaluation_task"] == "detection"
    assert data["frame_id"] == "base_link"
    assert data["scene"] == {"dataset_name": "scene_a", "map_available": False}
    assert data["producer"]["name"] == "driving_log_replayer_v2"
    assert data["config_digest"] == prepared.config_digest
    assert data["frame_filter_digest"] == advanced_metrics.frame_filter_digest(
        {"target_labels": ["car", "bicycle"], "max_x_position": 100.0}
    )
    # map paths are not echoed, class_names are added, tuples become lists
    assert data["config"]["map"] == {"resolver": "t4_scene_directory"}
    assert data["config"]["class_names"] == ["car", "bicycle"]
    assert data["config"]["components"][0]["percentiles"] == [95.0]

    samples_path = tmp_path / advanced_metrics.SAMPLES_NAME
    assert samples_path.exists()
    assert data["samples_file"] == {
        "name": advanced_metrics.SAMPLES_NAME,
        "sha256": hashlib.sha256(samples_path.read_bytes()).hexdigest(),
        "num_frames": 3,
    }
    assert prepared.producer["name"] == "driving_log_replayer_v2"


def test_write_outputs_is_atomic_and_leaves_no_tmp_file(tmp_path: Path) -> None:
    prepared = FakePreparedSamples(["0", "1"])
    advanced_metrics.write_outputs(
        tmp_path, prepared=prepared, report=FakeMetricReport({"a": 1.0}), meta=create_meta(tmp_path)
    )
    names = sorted(path.name for path in tmp_path.iterdir())
    assert names == [advanced_metrics.REPORT_NAME, advanced_metrics.SAMPLES_NAME]
    assert not any(".tmp" in name for name in names)


def test_write_outputs_error_status_writes_no_samples(tmp_path: Path) -> None:
    # a stale samples file from a previous attempt must not survive an error report
    (tmp_path / advanced_metrics.SAMPLES_NAME).write_bytes(b"stale")

    advanced_metrics.write_outputs(
        tmp_path, prepared=None, report=None, meta=create_meta(tmp_path), error="RuntimeError: boom"
    )

    data = load_strict_report(tmp_path)
    assert data["status"] == "error"
    assert data["error"] == "RuntimeError: boom"
    assert data["values"] == {}
    assert data["coverage"] == {}
    assert data["samples_file"] is None
    assert data["frames"]["prepared"] == 0
    assert not (tmp_path / advanced_metrics.SAMPLES_NAME).exists()
    assert sorted(path.name for path in tmp_path.iterdir()) == [advanced_metrics.REPORT_NAME]


def test_write_outputs_turns_a_failing_samples_writer_into_an_error_report(tmp_path: Path) -> None:
    class BrokenPrepared(FakePreparedSamples):
        def to_npz(self, path: str | Path, producer: dict | None = None) -> str:  # noqa: ARG002
            Path(path).write_bytes(b"partial")
            err_msg = "disk full"
            raise OSError(err_msg)

    advanced_metrics.write_outputs(
        tmp_path,
        prepared=BrokenPrepared(["0"]),
        report=FakeMetricReport({"a": 1.0}),
        meta=create_meta(tmp_path),
    )

    data = load_strict_report(tmp_path)
    assert data["status"] == "error"
    assert "disk full" in data["error"]
    assert sorted(path.name for path in tmp_path.iterdir()) == [advanced_metrics.REPORT_NAME]


def test_write_outputs_without_prepared_frames_is_empty(tmp_path: Path) -> None:
    advanced_metrics.write_outputs(tmp_path, prepared=None, report=None, meta=create_meta(tmp_path))
    data = load_strict_report(tmp_path)
    assert data["status"] == "empty"
    assert data["error"] is None
    assert data["samples_file"] is None


# ---------------------------------------------------------------------------------------------
# compute
# ---------------------------------------------------------------------------------------------


def test_compute_excludes_ignored_frames_and_runs_the_suite_once(
    fake_suite: type[FakeSuite],
) -> None:
    frame_results = [create_frame_result(str(i)) for i in range(5)]
    frame_results.append(create_frame_result("5", with_detection_frame=False))
    before = list(frame_results)

    result = advanced_metrics.compute(
        frame_results, create_metrics_config(), ignored_frame_names=["1", "3"]
    )

    assert result is not None
    prepared, report = result
    assert [frame.frame_name for frame in prepared.frames] == ["0", "2", "4"]
    assert "detection/corner_mean_car" in report.values
    assert len(fake_suite.instances) == 1
    assert [f.frame_name for f in fake_suite.instances[0].prepared_frames] == ["0", "2", "4"]
    assert fake_suite.instances[0].evaluate_prepared_calls == 1
    # the input list is untouched
    assert frame_results == before


def test_compute_returns_none_without_config_or_detection_frames(
    fake_suite: type[FakeSuite],
) -> None:
    frame_results = [create_frame_result("0")]
    assert advanced_metrics.compute(frame_results, create_metrics_config(advanced=False)) is None
    assert (
        advanced_metrics.compute(
            SimpleNamespace(detection_config=None), create_metrics_config(advanced=False)
        )
        is None
    )

    no_detection_frames = [create_frame_result("0", with_detection_frame=False)]
    assert advanced_metrics.compute(no_detection_frames, create_metrics_config()) is None
    assert fake_suite.instances == []


# ---------------------------------------------------------------------------------------------
# PerceptionEvaluator.write_advanced_detection_outputs (cloud path, enable_metrics_details false)
# ---------------------------------------------------------------------------------------------


class FakeEvaluationManager:
    def __init__(self, frame_names: list[str], *, advanced: bool = True) -> None:
        self.frame_results = [create_frame_result(name) for name in frame_names]
        self.evaluator_config = SimpleNamespace(
            metrics_config=create_metrics_config(advanced=advanced)
        )


def create_evaluator(
    tmp_path: Path,
    ignore_frames: IgnoreFrames,
    frame_names: list[str],
    *,
    evaluation_task: str = "detection",
    advanced: bool = True,
) -> tuple[PerceptionEvaluator, FakeEvaluationManager]:
    """Build a PerceptionEvaluator without a t4_dataset, setting only what the method needs."""
    evaluator = PerceptionEvaluator.__new__(PerceptionEvaluator)
    inner_evaluator = FakeEvaluationManager(frame_names, advanced=advanced)
    prefix = "_PerceptionEvaluator__"
    for name, value in {
        "skip_counter": 0,
        "evaluated_frame_position": 0,
        "skip_reasons": Counter(),
        "evaluator": inner_evaluator,
        "ignore_frames": ignore_frames,
        "ignored_frames_removed": False,
        "advanced_outputs_written": False,
        "num_frame_results": 0,
        "logger": logging.getLogger("test_perception_advanced_metrics"),
        "critical_object_filter_config": None,
        "frame_pass_fail_config": None,
        "evaluation_topic": "/perception/object_recognition/objects",
        "evaluation_task": evaluation_task,
        "frame_id_str": "base_link",
        "t4_dataset_path": str(tmp_path / "scene_b"),
        "evaluation_config_dict": {"target_labels": ["car", "bicycle"]},
        "result_archive_w_topic_path": tmp_path
        / "result_archive"
        / "perception.object_recognition.objects",
    }.items():
        setattr(evaluator, prefix + name, value)
    return evaluator, inner_evaluator


def test_evaluator_writes_outputs_without_mutating_frame_results(
    tmp_path: Path, fake_suite: type[FakeSuite]
) -> None:
    """The false path keeps every frame in frame_results (the pkl) but leaves the ignored ones out."""
    evaluator, inner_evaluator = create_evaluator(
        tmp_path, parse_ignore_frames("1,last:1"), [str(i) for i in range(6)]
    )
    before = list(inner_evaluator.frame_results)

    evaluator.write_advanced_detection_outputs()

    # frame_results is the same list with the same frames, in the same order
    assert inner_evaluator.frame_results is not None
    assert inner_evaluator.frame_results == before
    assert [frame.frame_name for frame in inner_evaluator.frame_results] == [
        "0",
        "1",
        "2",
        "3",
        "4",
        "5",
    ]
    # "1" is ignored by frame_name, "5" by last:1
    assert [f.frame_name for f in fake_suite.instances[0].prepared_frames] == ["0", "2", "3", "4"]

    out_dir = tmp_path / "result_archive" / "perception.object_recognition.objects"
    data = load_strict_report(out_dir)
    assert data["status"] == "ok"
    assert data["frames"] == {
        "frame_results": 6,
        "ignored": 2,
        "evaluated": 4,
        "skipped": 0,
        "prepared": 4,
    }
    assert data["scene"]["dataset_name"] == "scene_b"
    assert data["samples_file"]["num_frames"] == data["frames"]["prepared"]
    assert (out_dir / advanced_metrics.SAMPLES_NAME).exists()


def test_evaluator_writes_nothing_for_fp_validation_or_without_the_section(
    tmp_path: Path, fake_suite: type[FakeSuite]
) -> None:
    evaluator, _ = create_evaluator(
        tmp_path, IgnoreFrames(), ["0", "1"], evaluation_task="fp_validation"
    )
    evaluator.write_advanced_detection_outputs()
    evaluator, _ = create_evaluator(tmp_path, IgnoreFrames(), ["0", "1"], advanced=False)
    evaluator.write_advanced_detection_outputs()

    assert not (tmp_path / "result_archive").exists()
    assert fake_suite.instances == []


def test_evaluator_writes_an_error_report_when_the_suite_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class ExplodingSuite(FakeSuite):
        def prepare(self, frames: list, target_labels: list | None = None) -> FakePreparedSamples:  # noqa: ARG002
            err_msg = "no lanelet map"
            raise RuntimeError(err_msg)

    monkeypatch.setattr(advanced_metrics, "_load_suite_class", lambda: ExplodingSuite)
    evaluator, _ = create_evaluator(tmp_path, IgnoreFrames(), ["0", "1"])

    evaluator.write_advanced_detection_outputs()  # must not raise

    out_dir = tmp_path / "result_archive" / "perception.object_recognition.objects"
    data = load_strict_report(out_dir)
    assert data["status"] == "error"
    assert data["error"] == "RuntimeError: no lanelet map"
    assert not (out_dir / advanced_metrics.SAMPLES_NAME).exists()


def test_evaluator_reuses_the_scene_result_outputs(
    tmp_path: Path, fake_suite: type[FakeSuite]
) -> None:
    """The true path hands over what get_scene_result() computed: the suite is not run again."""
    evaluator, inner_evaluator = create_evaluator(tmp_path, IgnoreFrames(), ["0", "1", "2"])
    prepared = FakePreparedSamples(["0", "1", "2"])
    scene_result = SimpleNamespace(
        detection_prepared=prepared, detection_metric_report=FakeMetricReport({"a": 0.25})
    )

    evaluator._PerceptionEvaluator__write_advanced_detection_outputs(  # noqa: SLF001
        inner_evaluator.frame_results, num_frame_results=3, scene_result=scene_result
    )

    assert fake_suite.instances == []
    data = load_strict_report(tmp_path / "result_archive" / "perception.object_recognition.objects")
    assert data["status"] == "ok"
    assert data["values"] == {"a": 0.25}
    assert data["frames"]["ignored"] == 0


# ---------------------------------------------------------------------------------------------
# real perception_eval (skipped until the prepared-samples API is installed)
# ---------------------------------------------------------------------------------------------


@pytest.mark.skipif(
    not PREPARED_API_AVAILABLE, reason="perception_eval has no prepared-samples API"
)
def test_compute_with_the_real_perception_eval(tmp_path: Path) -> None:
    def make_object(x: float, score: float) -> DynamicObject:
        return DynamicObject(
            unix_time=0,
            frame_id=FrameID.BASE_LINK,
            position=(x, 0.0, 0.5),
            orientation=Quaternion(),
            shape=Shape(ShapeType.BOUNDING_BOX, (4.0, 2.0, 1.5)),
            velocity=(0.0, 0.0, 0.0),
            semantic_score=score,
            semantic_label=Label(AutowareLabel.CAR, "car"),
        )

    config = AdvancedDetectionMetricsConfig.from_dict(
        {"components": [{"type": "heading_flip"}]}, [AutowareLabel.CAR]
    )
    metrics_config = SimpleNamespace(
        detection_config=SimpleNamespace(advanced_detection_metrics=config)
    )
    frame_results = [
        SimpleNamespace(
            frame_name=str(i),
            detection_frame=DetectionFrame(
                frame_name=str(i),
                unix_time=i,
                scene_id=None,
                estimated_objects=(make_object(10.0 + i, 0.9),),
                ground_truth_objects=(make_object(10.2 + i, 1.0),),
            ),
        )
        for i in range(4)
    ]

    result = advanced_metrics.compute(frame_results, metrics_config, ignored_frame_names=["3"])
    assert result is not None
    prepared, report = result
    assert len(prepared.frames) == NUM_REAL_KEPT_FRAMES
    assert report.values

    meta = advanced_metrics.ReportMeta(
        topic="/perception/object_recognition/objects",
        evaluation_task="detection",
        frame_id="base_link",
        t4_dataset_path=str(tmp_path),
        evaluation_config_dict={"target_labels": ["car"]},
        advanced_config=config,
        num_frame_results=4,
        num_ignored_frames=1,
    )
    advanced_metrics.write_outputs(tmp_path, prepared=prepared, report=report, meta=meta)
    data = load_strict_report(tmp_path)
    assert data["status"] == "ok"
    assert data["config_digest"] == prepared.config_digest
    assert data["perception_eval"]["prepared_schema_version"] == prepared.schema_version
    reloaded = PreparedSamples.from_npz(tmp_path / advanced_metrics.SAMPLES_NAME)
    assert len(reloaded.frames) == NUM_REAL_KEPT_FRAMES
    assert reloaded.config_digest == prepared.config_digest


def test_evaluator_writes_the_outputs_only_once(
    tmp_path: Path, fake_suite: type[FakeSuite]
) -> None:
    evaluator, _ = create_evaluator(tmp_path, IgnoreFrames(), ["0", "1"])
    evaluator.write_advanced_detection_outputs()
    evaluator.write_advanced_detection_outputs()
    assert len(fake_suite.instances) == 1
