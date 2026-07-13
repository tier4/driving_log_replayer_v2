from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from driving_log_replayer_v2.real_log_sim_comparison.reidentify.split import load_split_manifest
from driving_log_replayer_v2.real_log_sim_comparison.reidentify.split import stratified_split


def test_split_is_seeded_stratified_and_exact_for_372() -> None:
    dataset_ids = [f"dataset-{index:03d}" for index in range(372)]
    strata = {
        dataset_id: f"stratum-{index % 9}"
        for index, dataset_id in enumerate(dataset_ids)
    }

    first = stratified_split(dataset_ids, strata, seed=42)
    second = stratified_split(reversed(dataset_ids), strata, seed=42)

    assert first == second
    assert {name: len(values) for name, values in first.items()} == {
        "train": 260,
        "validation": 56,
        "test": 56,
    }
    assert set().union(*map(set, first.values())) == set(dataset_ids)


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"schema_version": 99}, "schema mismatch"),
        ({"seed": 7}, "seed mismatch"),
        ({"ratios": {"train": 0.8, "validation": 0.1, "test": 0.1}}, "ratios mismatch"),
        (
            {"splits": {"train": ["dataset-a"], "validation": ["dataset-a"], "test": []}},
            "duplicate dataset IDs",
        ),
    ],
)
def test_load_manifest_rejects_non_reproducible_configuration(
    tmp_path: Path, override: dict, message: str
) -> None:
    manifest = {
        "schema_version": 1,
        "seed": 42,
        "ratios": {"train": 0.70, "validation": 0.15, "test": 0.15},
        "splits": {"train": ["dataset-a"], "validation": [], "test": []},
    }
    manifest.update(override)
    path = tmp_path / "split_manifest.yaml"
    path.write_text(yaml.safe_dump(manifest), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        load_split_manifest(path)
