from __future__ import annotations

import csv
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from driving_log_replayer_v2.real_log_sim_comparison.reidentify import extract
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.csv_schema import CACHE_NAME
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.csv_schema import CSV_COLUMNS
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.csv_schema import SIGNAL_TOPICS


def _message(tag: str) -> SimpleNamespace:
    if tag == "mode":
        return SimpleNamespace(mode=3)
    if tag == "velocity":
        return SimpleNamespace(longitudinal_velocity=4.5)
    if tag == "steering":
        return SimpleNamespace(steering_tire_angle=0.1)
    if tag == "gear":
        return SimpleNamespace(report=2)
    if tag == "kinematic":
        position = SimpleNamespace(x=1.0, y=2.0)
        orientation = SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)
        pose = SimpleNamespace(pose=SimpleNamespace(position=position, orientation=orientation))
        linear = SimpleNamespace(x=4.0, y=0.2)
        angular = SimpleNamespace(z=0.05)
        twist = SimpleNamespace(twist=SimpleNamespace(linear=linear, angular=angular))
        return SimpleNamespace(pose=pose, twist=twist)
    if tag == "accel":
        linear = SimpleNamespace(x=0.3)
        return SimpleNamespace(accel=SimpleNamespace(accel=SimpleNamespace(linear=linear)))
    if tag == "cmd":
        longitudinal = SimpleNamespace(velocity=5.0, acceleration=0.4)
        lateral = SimpleNamespace(steering_tire_angle=0.12)
        return SimpleNamespace(longitudinal=longitudinal, lateral=lateral)
    raise AssertionError(tag)


def _messages(duration_ns: int = 3_000_000_000) -> list[tuple[str, object, int]]:
    messages = []
    for tag, topic in SIGNAL_TOPICS.items():
        timestamps = (1_000_000_000,) if tag in {"mode", "gear"} else (0, duration_ns)
        messages.extend((topic, _message(tag), timestamp) for timestamp in timestamps)
    return sorted(messages, key=lambda item: item[2])


class _FakeReader:
    def __init__(
        self,
        messages: list[tuple[str, object, int]],
        *,
        available_topics: set[str] | None = None,
    ) -> None:
        self.messages = messages
        self.available_topics = available_topics or set(SIGNAL_TOPICS.values())
        self.index = 0
        self.read_count = 0
        self.filter = None

    def get_all_topics_and_types(self) -> list[SimpleNamespace]:
        return [
            SimpleNamespace(name=topic, type=f"test_msgs/msg/{index}")
            for index, topic in enumerate(sorted(self.available_topics))
        ]

    def set_filter(self, storage_filter: object) -> None:
        self.filter = storage_filter

    def has_next(self) -> bool:
        return self.index < len(self.messages)

    def read_next(self) -> tuple[str, object, int]:
        item = self.messages[self.index]
        self.index += 1
        self.read_count += 1
        return item


def _dataset(tmp_path: Path, name: str = "dataset-a", suffix: str = ".mcap") -> Path:
    dataset = tmp_path / "datasets" / name
    input_bag = dataset / "input_bag"
    input_bag.mkdir(parents=True)
    (input_bag / f"bag{suffix}").touch()
    return dataset


def _install_fake_ros(monkeypatch: pytest.MonkeyPatch, reader: _FakeReader) -> None:
    monkeypatch.setattr(extract, "_open_reader", lambda _bag: reader)
    monkeypatch.setattr(extract, "get_message", lambda type_name: type_name)
    monkeypatch.setattr(extract, "deserialize_message", lambda payload, _type: payload)


def _write_valid_cache(path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for tag, value_columns in extract.SIGNAL_COLUMNS.items():
            timestamps = (0,) if tag in {"mode", "gear"} else (0, 3_000_000_000)
            for timestamp in timestamps:
                row = {"t_ns": timestamp, "topic": tag}
                row.update({column: 1 for column in value_columns})
                writer.writerow(row)


@pytest.mark.parametrize(
    ("suffix", "storage_id"),
    [(".db3", "sqlite3"), (".mcap", "mcap")],
)
def test_resolve_single_db3_or_mcap(
    tmp_path: Path, suffix: str, storage_id: str
) -> None:
    dataset = _dataset(tmp_path, suffix=suffix)

    bag = extract._resolve_bag_input(dataset)

    assert bag.uri == dataset / "input_bag" / f"bag{suffix}"
    assert bag.storage_id == storage_id


def test_rosbag2_metadata_uses_directory_for_split_bag(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path)
    (dataset / "input_bag" / "bag_1.mcap").touch()
    (dataset / "input_bag" / "metadata.yaml").touch()

    bag = extract._resolve_bag_input(dataset)

    assert bag == extract.BagInput(dataset / "input_bag", "")


def test_extract_one_reads_all_topics_in_one_pass_and_writes_csv(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    dataset = _dataset(tmp_path)
    reader = _FakeReader(_messages())
    _install_fake_ros(monkeypatch, reader)

    extraction = extract._extract_dataset(dataset, force=False)
    result = extraction.path

    assert extraction.status == "extracted"
    assert result == dataset / CACHE_NAME
    assert result is not None
    assert reader.read_count == len(reader.messages)
    assert set(reader.filter.topics) == set(SIGNAL_TOPICS.values())
    with result.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 12
    assert {row["topic"] for row in rows} == set(SIGNAL_TOPICS)
    kinematic = next(row for row in rows if row["topic"] == "kinematic")
    assert float(kinematic["yaw"]) == pytest.approx(0.0)
    assert float(kinematic["pitch"]) == pytest.approx(0.0)


def test_collection_reuses_cache_and_reports_skipped_dataset(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    cached = tmp_path / "datasets" / "cached"
    cached.mkdir(parents=True)
    _write_valid_cache(cached / CACHE_NAME)
    (tmp_path / "datasets" / "missing").mkdir()
    monkeypatch.setattr(
        extract,
        "_open_reader",
        lambda _bag: pytest.fail("cached dataset must not open its bag"),
    )

    summary = extract.extract_collection(tmp_path)

    assert summary == {
        "n_datasets": 2,
        "n_cached": 1,
        "n_extracted": 0,
        "n_skipped": 1,
        "n_failed": 0,
        "valid_datasets": ["cached"],
        "skipped": [
            {"dataset_id": "missing", "reason": "input_bag ディレクトリがありません"}
        ],
        "failed": [],
    }
    captured = capsys.readouterr()
    assert "missing: skipped" in captured.err
    assert extract._cache_is_valid(cached / CACHE_NAME)


def test_missing_or_empty_required_topic_is_skipped(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    missing_dataset = _dataset(tmp_path, "missing-topic")
    cmd_topic = SIGNAL_TOPICS["cmd"]
    missing_reader = _FakeReader(
        _messages(), available_topics=set(SIGNAL_TOPICS.values()) - {cmd_topic}
    )
    _install_fake_ros(monkeypatch, missing_reader)

    missing = extract._extract_dataset(missing_dataset, force=False)

    assert missing.status == "skipped"
    assert cmd_topic in missing.reason
    assert not (missing_dataset / CACHE_NAME).exists()

    empty_dataset = _dataset(tmp_path, "empty-topic")
    empty_reader = _FakeReader(
        [message for message in _messages() if message[0] != cmd_topic]
    )
    _install_fake_ros(monkeypatch, empty_reader)

    empty = extract._extract_dataset(empty_dataset, force=False)

    assert empty.status == "skipped"
    assert f"必須 topic が空: {cmd_topic}" == empty.reason
    assert not (empty_dataset / CACHE_NAME).exists()


def test_collection_reports_rosbag_reader_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cached = tmp_path / "datasets" / "cached"
    cached.mkdir(parents=True)
    _write_valid_cache(cached / CACHE_NAME)
    _dataset(tmp_path, "broken")
    monkeypatch.setattr(extract, "_open_reader", Mock(side_effect=OSError("cannot open bag")))

    summary = extract.extract_collection(tmp_path)

    assert summary["n_failed"] == 1
    assert summary["failed"] == [
        {"dataset_id": "broken", "reason": "OSError: cannot open bag"}
    ]


def test_common_interval_shorter_than_two_seconds_is_skipped(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    dataset = _dataset(tmp_path)
    reader = _FakeReader(_messages(duration_ns=1_999_999_999))
    _install_fake_ros(monkeypatch, reader)

    result = extract._extract_dataset(dataset, force=False)

    assert result.status == "skipped"
    assert "2秒未満" in result.reason
    assert not (dataset / CACHE_NAME).exists()


def test_force_does_not_leave_stale_cache_when_regeneration_fails(tmp_path: Path) -> None:
    dataset = tmp_path / "datasets" / "dataset-a"
    dataset.mkdir(parents=True)
    cache = dataset / CACHE_NAME
    cache.write_text("stale\n", encoding="utf-8")

    result = extract._extract_dataset(dataset, force=True)

    assert result.status == "skipped"
    assert not cache.exists()


def test_invalid_cache_is_rebuilt_from_input_bag(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    dataset = _dataset(tmp_path)
    cache = dataset / CACHE_NAME
    cache.write_text("truncated\n", encoding="utf-8")
    reader = _FakeReader(_messages())
    _install_fake_ros(monkeypatch, reader)

    result = extract._extract_dataset(dataset, force=False)

    assert result.status == "extracted"
    assert extract._cache_is_valid(cache)


def test_cache_rejects_non_finite_signal_value(tmp_path: Path) -> None:
    cache = tmp_path / CACHE_NAME
    _write_valid_cache(cache)
    with cache.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    next(row for row in rows if row["topic"] == "cmd")["cmd_accel"] = "nan"
    with cache.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)

    assert not extract._cache_is_valid(cache)


def test_collection_fails_when_no_valid_dataset(tmp_path: Path) -> None:
    (tmp_path / "datasets" / "invalid").mkdir(parents=True)

    with pytest.raises(RuntimeError, match="有効な dataset が0件"):
        extract.extract_collection(tmp_path)
