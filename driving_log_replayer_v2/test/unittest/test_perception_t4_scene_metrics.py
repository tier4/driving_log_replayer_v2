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

from pathlib import Path

import numpy as np
import pandas as pd
import pytest
from t4perceval import FRAME
from t4perceval import InstanceRegistry
from t4perceval_test_utils import make_labels
from t4perceval_test_utils import make_record

from driving_log_replayer_v2.perception.analyze import analyze
from driving_log_replayer_v2.perception.analyze import load_analysis
from driving_log_replayer_v2.perception.analyze import PerceptionAnalysis
from driving_log_replayer_v2.perception.t4perceval_adapter.config import from_scenario
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import apply_statistics
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import build_scene_store
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import (
    confusion_matrix_dict,
)
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import ERROR_METRICS
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import ERROR_STATISTICS
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import error_statistics
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import final_score
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import pass_fail_rates
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import read_archive
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import run_scene_pipeline
from driving_log_replayer_v2.perception.t4perceval_adapter.scene_metrics import write_archive
from driving_log_replayer_v2.perception.t4perceval_adapter.systems import ESTIMATION_KEPT_PATH

LABELS = ["car", "pedestrian"]


def make_config(task: str = "detection"):  # noqa: ANN201
    return from_scenario(
        {
            "evaluation_config_dict": {
                "target_labels": LABELS,
                "center_distance_thresholds": [1.0, 2.0],
                "plane_distance_thresholds": [2.0],
                "iou_2d_thresholds": [0.5],
            }
        },
        {"target_labels": LABELS},
        {"target_labels": LABELS, "matching_threshold_list": [2.0, 2.0]},
        evaluation_task=task,
        frame_id="base_link" if task == "detection" else "map",
    )


@pytest.fixture
def records() -> list:
    registry = make_labels()
    return [
        make_record(tp=2, fp=1, registry=registry, frame_index=1, frame_name="0"),
        make_record(tp=1, fn=1, registry=registry, frame_index=2, frame_name="1"),
        make_record(
            tp=0, fn=2, label="pedestrian", registry=registry, frame_index=3, frame_name="2"
        ),
    ]


def test_scene_store_concatenates_frames(records: list) -> None:
    store = build_scene_store(records)
    assert store.times(ESTIMATION_KEPT_PATH, FRAME).tolist() == [1, 2, 3]
    assert len(store.chunks(ESTIMATION_KEPT_PATH)) == 1


def test_pass_fail_rates(records: list) -> None:
    rates = pass_fail_rates(records, LABELS)
    assert rates["TP"] == {
        "ALL": pytest.approx(3 / 6),
        "car": pytest.approx(3 / 4),
        "pedestrian": 0.0,
    }
    assert rates["FP"] == {
        "ALL": pytest.approx(1 / 4),
        "car": pytest.approx(1 / 4),
        "pedestrian": 0.0,
    }
    assert rates["FN"] == {
        "ALL": pytest.approx(3 / 6),
        "car": pytest.approx(1 / 4),
        "pedestrian": 1.0,
    }
    assert rates["TN"] == {"ALL": 0.0, "car": 0.0, "pedestrian": 0.0}


def test_final_score_keys(records: list) -> None:
    config = make_config()
    store = build_scene_store(records)
    run_scene_pipeline(store, config, make_labels(), InstanceRegistry())
    score = final_score(store, records, config, make_labels())
    assert set(score) == {
        "TP",
        "FP",
        "FN",
        "TN",
        "AP(Center Distance)",
        "APH(Center Distance)",
        "AP(Plane Distance)",
        "APH(Plane Distance)",
        "AP(IoU 2D)",
        "APH(IoU 2D)",
    }
    for values in score.values():
        assert list(values) == ["ALL", *LABELS]
    # 3 car TPs and 1 car FP, all at confidence 0.5: a finite AP below 1 for car, nothing for
    # pedestrian (no estimation) and the mean over the target labels in ALL
    car_ap = score["AP(Center Distance)"]["car"]
    assert 0.0 < car_ap < 1.0
    pedestrian_ap = score["AP(Center Distance)"]["pedestrian"]
    assert np.isnan(pedestrian_ap) or pedestrian_ap == 0.0
    assert score["AP(Center Distance)"]["ALL"] == pytest.approx(
        car_ap if np.isnan(pedestrian_ap) else (car_ap + pedestrian_ap) / 2
    )
    confusion = confusion_matrix_dict(store, make_labels(), LABELS)
    assert confusion == {
        "car": {"car": 3, "pedestrian": 0},
        "pedestrian": {"car": 0, "pedestrian": 0},
    }


def test_error_statistics_shape(records: list) -> None:
    error = error_statistics(records, LABELS)
    assert list(error) == ["ALL", *LABELS]
    assert list(error["ALL"]) == list(ERROR_STATISTICS)
    assert list(error["ALL"]["average"]) == list(ERROR_METRICS)
    assert error["ALL"]["average"]["x"] == 0.0
    assert error["car"]["max"]["nn_plane"] == pytest.approx(0.5)
    assert np.isnan(error["pedestrian"]["average"]["x"])
    assert np.isnan(error["ALL"]["average"]["vx"])  # no velocity in the records


def test_apply_statistics() -> None:
    values = [1.0, -3.0, float("nan")]
    assert apply_statistics(values, "average") == -1.0
    assert apply_statistics(values, "max") == 3.0  # noqa: PLR2004
    assert apply_statistics(values, "min") == 1.0
    assert np.isnan(apply_statistics([], "rms"))
    with pytest.raises(ValueError, match="Invalid statistics"):
        apply_statistics(values, "median")


def test_archive_round_trip_and_analysis(records: list, tmp_path: Path) -> None:
    config = make_config()
    labels = make_labels()
    store = build_scene_store(records)
    pipeline = run_scene_pipeline(store, config, labels, InstanceRegistry())
    archive_dir = write_archive(
        tmp_path / "topic",
        store=store,
        records=records,
        config=config,
        labels=labels,
        instances=InstanceRegistry(),
        topic="/perception/objects",
        pipeline=pipeline,
    )
    assert (archive_dir / "scene_result.t4eval" / "manifest.json").is_file()
    assert (archive_dir / "evaluation_config.json").is_file()
    archive = read_archive(archive_dir)
    assert archive.config == config
    assert [r.frame_name for r in archive.records] == ["0", "1", "2"]
    assert [(r.num_tp, r.num_fp, r.num_fn) for r in archive.records] == [
        (2, 1, 0),
        (1, 0, 1),
        (0, 0, 2),
    ]
    assert archive.records[0].ground_truth.uuids == (None, None)  # uuids are not stored

    analysis = load_analysis(tmp_path)
    result = analysis.analyze()
    assert result.score["TP"]["car"] == pytest.approx(3 / 4)
    near = analysis.analyze(distance=(0.0, 1.5))
    # only the objects at x = 1 remain: TP in frames 1 and 2, FN in frame 3
    assert near.score["TP"]["ALL"] == pytest.approx(2 / 3)
    assert near.score["FP"]["ALL"] == 0.0


def test_analysis_csv_columns(records: list, tmp_path: Path) -> None:
    config = make_config()
    analysis = PerceptionAnalysis(records, config, make_labels(), InstanceRegistry())
    analyze(analysis, tmp_path, "4", "2", "/perception/objects")
    table = pd.read_csv(tmp_path / "analysis_result.csv")
    assert list(table.columns[:4]) == ["evaluation_task", "topic_name", "label", "distance"]
    assert "AP(Center Distance)" in table.columns
    assert "x_average" in table.columns
    assert "consecutive_fn_spans_max" in table.columns
    assert table["distance"].tolist() == ["0-2", "0-2", "0-2", "2-4", "2-4", "2-4"]
    assert table["label"].tolist() == ["ALL", "car", "pedestrian"] * 2


def test_consecutive_fn_spans() -> None:
    registry = make_labels()
    # the same ground truth uuid (gt-0 of pedestrian) is missed in frames 1, 2 and 3 (0.1 s apart)
    missed = [
        make_record(
            fn=1,
            label="pedestrian",
            registry=registry,
            frame_index=i,
            frame_name=str(i),
            timestamp_ns=i * 100_000_000,
        )
        for i in (1, 2, 3, 5)
    ]
    analysis = PerceptionAnalysis(missed, make_config(), registry, InstanceRegistry())
    assert analysis.consecutive_fn_spans(None, "pedestrian") == [pytest.approx(0.2)]
    assert analysis.consecutive_fn_spans(None, "car") == []
