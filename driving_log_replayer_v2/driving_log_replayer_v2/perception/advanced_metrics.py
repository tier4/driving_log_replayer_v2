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

"""
Driving-aware (advanced) detection metric outputs of one perception evaluation.

perception_eval evaluates the opt-in ``advanced_detection_metrics`` section of
``evaluation_config_dict`` per scene. This module writes its outcome next to ``scene_result.pkl``
so that a database evaluation can pool the scenes later without re-running the suite:

* ``advanced_detection_metrics.json``: per-scenario report (strict JSON, NaN -> null).
* ``advanced_detection_samples.npz``: the prepared per-frame samples the pooled metrics are
  computed from (``PreparedSamples.to_npz``).

Everything that touches perception_eval's prepared-samples API is imported lazily and guarded, so
the evaluation still runs on a perception_eval which does not have the API yet; in that case only
an ``error`` report is written.
"""

from __future__ import annotations

import contextlib
from dataclasses import dataclass
import hashlib
import importlib
import importlib.metadata
import json
import logging
import math
from pathlib import Path
from typing import Any
from typing import TYPE_CHECKING
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

if TYPE_CHECKING:
    from collections.abc import Iterable
    from collections.abc import Mapping
    from collections.abc import Sequence

REPORT_NAME = "advanced_detection_metrics.json"
SAMPLES_NAME = "advanced_detection_samples.npz"
REPORT_SCHEMA = "perception_eval.advanced_detection_report"
REPORT_SCHEMA_VERSION = 1
PRODUCER_NAME = "driving_log_replayer_v2"
PREPARED_MODULE = "perception_eval.evaluation.metrics.detection.prepared"
SUITE_MODULE = "perception_eval.evaluation.metrics.detection.suite"

# evaluation_config_dict keys which decide which objects reach the metrics (missing -> null)
FRAME_FILTER_KEYS: tuple[str, ...] = (
    "target_labels",
    "max_x_position",
    "max_y_position",
    "max_distance",
    "min_distance",
    "min_point_numbers",
    "confidence_threshold",
    "label_prefix",
    "merge_similar_labels",
    "allow_matching_unknown",
)
# map paths are machine specific, they are removed from the config echoed in the report
MAP_PATH_KEYS: tuple[str, ...] = ("data_root", "mapping")

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ReportMeta:
    """
    Scenario-level facts of the report which do not come from perception_eval.

    Attributes:
        topic (str): Evaluated topic.
        evaluation_task (str): detection/tracking/prediction.
        frame_id (str): Frame id of the evaluated objects.
        t4_dataset_path (str): Path of the evaluated t4_dataset (scene).
        evaluation_config_dict (Mapping[str, Any]): Raw ``evaluation_config_dict`` of the scenario.
        advanced_config (Any | None): Parsed ``AdvancedDetectionMetricsConfig`` or None.
        num_frame_results (int): Number of frame results before the ignored frames are removed.
        num_ignored_frames (int): Number of frame results removed by ``ignore_frames``.

    """

    topic: str
    evaluation_task: str
    frame_id: str
    t4_dataset_path: str
    evaluation_config_dict: Mapping[str, Any]
    advanced_config: Any | None
    num_frame_results: int
    num_ignored_frames: int


def is_available() -> bool:
    """Whether the installed perception_eval has the prepared-samples API."""
    return _import_optional(PREPARED_MODULE) is not None


def _import_optional(module_name: str) -> Any | None:
    try:
        return importlib.import_module(module_name)
    except ImportError:
        return None


def _load_suite_class() -> type:
    """Import ``AdvancedDetectionSuite`` lazily (tests replace this function)."""
    module = _import_optional(SUITE_MODULE)
    if module is None:
        err_msg = f"{SUITE_MODULE} is not available in the installed perception_eval"
        raise ImportError(err_msg)
    return module.AdvancedDetectionSuite


def canonical_json(data: Any) -> str:
    return json.dumps(json_safe(data), sort_keys=True, separators=(",", ":"), allow_nan=False)


def sha256_digest(text: str) -> str:
    return "sha256:" + hashlib.sha256(text.encode("utf-8")).hexdigest()


def frame_filter_digest(evaluation_config_dict: Mapping[str, Any] | None) -> str:
    """
    Digest of the object-filter part of ``evaluation_config_dict``.

    Scenes pooled together must have been filtered the same way; the digest is stable across key
    order and treats a missing key as null.
    """
    config = evaluation_config_dict or {}
    return sha256_digest(canonical_json({key: config.get(key) for key in FRAME_FILTER_KEYS}))


def get_advanced_config(metrics_config: Any) -> Any | None:
    """``AdvancedDetectionMetricsConfig`` of a ``MetricsScoreConfig`` or None when not configured."""
    detection_config = getattr(metrics_config, "detection_config", None)
    return getattr(detection_config, "advanced_detection_metrics", None)


def select_frame_results(frame_results: Sequence[Any], ignored_frame_names: Iterable[str]) -> list:
    """Frame results whose ``frame_name`` is not ignored (a new list, the input is not touched)."""
    ignored = {str(name) for name in ignored_frame_names}
    return [
        frame_result
        for frame_result in frame_results
        if str(frame_result.frame_name) not in ignored
    ]


def compute(
    frame_results: Sequence[Any],
    metrics_config: Any,
    *,
    ignored_frame_names: Iterable[str] = (),
) -> tuple[Any, Any] | None:
    """
    Run the advanced detection suite once over the retained detection frames.

    Args:
        frame_results (Sequence[PerceptionFrameResult]): Frame results of the scene, in order.
        metrics_config (MetricsScoreConfig): Metrics config the frames were evaluated with.
        ignored_frame_names (Iterable[str]): ``frame_name`` of the frames to leave out.

    Returns:
        tuple[PreparedSamples, MetricReport] | None: None when the advanced section is not
            configured or when no frame carries a ``detection_frame``.

    """
    advanced_config = get_advanced_config(metrics_config)
    if advanced_config is None:
        return None
    detection_frames = [
        frame_result.detection_frame
        for frame_result in select_frame_results(frame_results, ignored_frame_names)
        if getattr(frame_result, "detection_frame", None) is not None
    ]
    if not detection_frames:
        return None
    suite = _load_suite_class()(advanced_config)
    prepared = suite.prepare(detection_frames)
    report = suite.evaluate_prepared(prepared)
    return prepared, report


def write_outputs(
    out_dir: str | Path,
    *,
    prepared: Any | None,
    report: Any | None,
    meta: ReportMeta,
    error: str | None = None,
) -> None:
    """
    Write the report (and the samples when there are any) atomically into ``out_dir``.

    Never raises: a failure is logged and, when possible, turned into an ``error`` report.
    """
    out_dir = Path(out_dir)
    samples_sha256: str | None = None
    try:
        out_dir.mkdir(parents=True, exist_ok=True)
        if error is None and prepared is not None:
            try:
                samples_sha256 = _write_samples(out_dir / SAMPLES_NAME, prepared)
            except Exception as err:
                logger.exception("Failed to write %s", SAMPLES_NAME)
                error = f"{type(err).__name__}: {err}"
        if error is not None:
            # keep the two files consistent: an error report has no samples
            prepared = None
            report = None
            (out_dir / SAMPLES_NAME).unlink(missing_ok=True)
        report_dict = build_report(
            prepared=prepared,
            report=report,
            meta=meta,
            error=error,
            samples_sha256=samples_sha256,
        )
        _write_json(out_dir / REPORT_NAME, report_dict)
    except Exception as err:
        logger.exception("Failed to write %s", REPORT_NAME)
        try:
            (out_dir / SAMPLES_NAME).unlink(missing_ok=True)
            _write_json(
                out_dir / REPORT_NAME,
                build_report(
                    prepared=None,
                    report=None,
                    meta=meta,
                    error=f"{type(err).__name__}: {err}",
                    samples_sha256=None,
                ),
            )
        except Exception:
            logger.exception("Failed to write the error report %s", REPORT_NAME)


def build_report(
    *,
    prepared: Any | None,
    report: Any | None,
    meta: ReportMeta,
    error: str | None,
    samples_sha256: str | None,
) -> dict:
    """Build the report dictionary (JSON safe, NaN already converted to None)."""
    if error is not None:
        status = "error"
    elif prepared is not None and len(prepared.frames) > 0:
        status = "ok"
    else:
        status = "empty"

    counts = dict(getattr(prepared, "counts", None) or {})
    num_prepared = len(prepared.frames) if prepared is not None else 0
    num_evaluated = meta.num_frame_results - meta.num_ignored_frames
    frames = {
        "frame_results": meta.num_frame_results,
        "ignored": meta.num_ignored_frames,
        "evaluated": num_evaluated,
        "skipped": int(counts.get("skipped_frames", 0)),
        "prepared": num_prepared,
    }

    warnings: list[str] = list(getattr(report, "warnings", None) or [])
    for warning in getattr(prepared, "warnings", None) or ():
        if warning not in warnings:
            warnings.append(warning)

    dataset_path = Path(meta.t4_dataset_path)
    report_dict = {
        "schema": REPORT_SCHEMA,
        "schema_version": REPORT_SCHEMA_VERSION,
        "status": status,
        "error": error,
        "producer": {"name": PRODUCER_NAME, "version": producer_version()},
        "perception_eval": {
            "version": perception_eval_version(),
            "prepared_schema_version": prepared_schema_version(),
        },
        "topic": meta.topic,
        "evaluation_task": meta.evaluation_task,
        "frame_id": meta.frame_id,
        "scene": {
            "dataset_name": dataset_path.name,
            "map_available": Path(dataset_path, "map", "lanelet2_map.osm").exists(),
        },
        "config_digest": _config_digest(prepared, meta.advanced_config),
        "frame_filter_digest": frame_filter_digest(meta.evaluation_config_dict),
        "config": _config_dict(meta.advanced_config),
        "frames": frames,
        "counts": {
            key: int(counts.get(key, 0))
            for key in (
                "pred_total",
                "pred_kept",
                "non_finite_score",
                "polygon_as_box",
                "zero_score",
                "skipped_frames",
            )
        },
        "coverage": {
            str(name): [int(covered), int(seen)]
            for name, (covered, seen) in (getattr(report, "coverage", None) or {}).items()
        },
        "values": dict(getattr(report, "values", None) or {}),
        "warnings": warnings,
        "samples_file": (
            {"name": SAMPLES_NAME, "sha256": samples_sha256, "num_frames": num_prepared}
            if samples_sha256 is not None
            else None
        ),
    }
    return json_safe(report_dict)


def json_safe(data: Any) -> Any:
    """Convert to plain JSON types; non-finite floats become None (strict JSON has no NaN)."""
    if isinstance(data, dict):
        return {str(key): json_safe(value) for key, value in data.items()}
    if isinstance(data, list | tuple | set | frozenset):
        return [json_safe(value) for value in data]
    if hasattr(data, "item") and not isinstance(data, str):  # numpy scalars
        data = data.item()
    if isinstance(data, float):
        return data if math.isfinite(data) else None
    if isinstance(data, bool | int | str) or data is None:
        return data
    return str(data)


def producer_version() -> str | None:
    """Version of driving_log_replayer_v2 from package.xml, falling back to the python metadata."""
    candidates = [Path(__file__).resolve().parents[2] / "package.xml"]
    with contextlib.suppress(PackageNotFoundError):
        candidates.insert(0, Path(get_package_share_directory(PRODUCER_NAME), "package.xml"))
    for package_xml in candidates:
        try:
            version = ET.parse(package_xml).getroot().findtext("version")  # noqa: S314
        except (OSError, ET.ParseError):
            continue
        if version:
            return version.strip()
    return _distribution_version(PRODUCER_NAME)


def perception_eval_version() -> str | None:
    version = _distribution_version("perception_eval")
    if version is not None:
        return version
    module = _import_optional("perception_eval")
    return getattr(module, "__version__", None) if module is not None else None


def prepared_schema_version() -> int | None:
    module = _import_optional(PREPARED_MODULE)
    return getattr(module, "PREPARED_SCHEMA_VERSION", None) if module is not None else None


def _distribution_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def _config_digest(prepared: Any | None, advanced_config: Any | None) -> str | None:
    digest = getattr(prepared, "config_digest", None)
    if digest is not None:
        return str(digest)
    if advanced_config is None:
        return None
    module = _import_optional(PREPARED_MODULE)
    if module is None:
        return None
    try:
        return str(module.config_digest(advanced_config))
    except Exception:
        logger.exception("Failed to compute the config digest")
        return None


def _config_dict(advanced_config: Any | None) -> dict | None:
    """``AdvancedDetectionMetricsConfig.serialization()`` without map paths, with class_names."""
    if advanced_config is None:
        return None
    try:
        data = dict(advanced_config.serialization())
    except Exception:
        logger.exception("Failed to serialize advanced_detection_metrics")
        return None
    map_section = data.get("map")
    if isinstance(map_section, dict):
        data["map"] = {key: value for key, value in map_section.items() if key not in MAP_PATH_KEYS}
    data["class_names"] = list(getattr(advanced_config, "class_names", ()))
    return data


def _write_samples(path: Path, prepared: Any) -> str:
    """Write the samples atomically, return their sha256 hex digest."""
    # NOTE: numpy appends ".npz" to a file name which does not end with it, so keep the suffix.
    tmp_path = path.with_name(f"{path.stem}.tmp{path.suffix}")
    try:
        sha256 = prepared.to_npz(
            tmp_path, producer={"name": PRODUCER_NAME, "version": str(producer_version())}
        )
        tmp_path.replace(path)
    finally:
        tmp_path.unlink(missing_ok=True)
    return str(sha256)


def _write_json(path: Path, data: dict) -> None:
    """Write strict JSON atomically (``<name>.tmp`` then ``os.replace``)."""
    tmp_path = path.with_name(path.name + ".tmp")
    try:
        with tmp_path.open("w", encoding="utf-8") as json_file:
            json.dump(data, json_file, indent=2, allow_nan=False)
            json_file.write("\n")
        tmp_path.replace(path)
    finally:
        tmp_path.unlink(missing_ok=True)
