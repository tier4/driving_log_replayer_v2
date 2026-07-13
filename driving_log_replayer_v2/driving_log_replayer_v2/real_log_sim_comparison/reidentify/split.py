"""Deterministic driving-characteristic dataset split for reidentification.

The manifest is the boundary that prevents test-set leakage: direct fitting and
Optuna consume ``train``, candidate selection consumes ``validation``, and the
selected candidate is evaluated once on ``test``.
"""
from __future__ import annotations

import argparse
from collections.abc import Mapping
from collections import defaultdict
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml

from .load_data import build_resampled, discover_cached_datasets, read_dataset_csv
from .settings import RESAMPLE_DT

SPLIT_SEED = 42
SPLIT_RATIOS = {"train": 0.70, "validation": 0.15, "test": 0.15}
SPLIT_SCHEMA_VERSION = 1


def driving_features(ds: dict | None) -> dict[str, float | str]:
    """Return compact, stable features used solely for split stratification."""
    if ds is None or len(ds.get("vx", ())) == 0:
        return {
            "speed_p90": 0.0,
            "curve_fraction": 0.0,
            "accel_fraction": 0.0,
            "decel_fraction": 0.0,
            "stratum": "unknown",
        }

    drive = np.asarray(ds["gear_drive"], dtype=bool)
    if not np.any(drive):
        drive = np.ones(len(ds["vx"]), dtype=bool)
    vx = np.abs(np.asarray(ds["vx"], dtype=float))[drive]
    wz = np.abs(np.asarray(ds["wz"], dtype=float))[drive]
    a_cmd = np.asarray(ds["a_cmd"], dtype=float)[drive]
    speed_p90 = float(np.percentile(vx, 90)) if len(vx) else 0.0
    curve_fraction = float(np.mean(wz > 0.04)) if len(wz) else 0.0
    accel_fraction = float(np.mean(a_cmd > 0.15)) if len(a_cmd) else 0.0
    decel_fraction = float(np.mean(a_cmd < -0.15)) if len(a_cmd) else 0.0

    speed_class = "low" if speed_p90 < 5.0 else ("mid" if speed_p90 < 12.0 else "high")
    curve_class = "curve" if curve_fraction >= 0.05 else "straight"
    if accel_fraction >= 0.05 and decel_fraction >= 0.05:
        long_class = "mixed"
    elif decel_fraction >= 0.05:
        long_class = "decel"
    elif accel_fraction >= 0.05:
        long_class = "accel"
    else:
        long_class = "steady"
    return {
        "speed_p90": speed_p90,
        "curve_fraction": curve_fraction,
        "accel_fraction": accel_fraction,
        "decel_fraction": decel_fraction,
        "stratum": f"{speed_class}_{curve_class}_{long_class}",
    }


def _target_counts(n: int, ratios: dict[str, float]) -> dict[str, int]:
    names = tuple(ratios)
    exact = {name: n * float(ratios[name]) for name in names}
    counts = {name: int(np.floor(exact[name])) for name in names}
    remaining = n - sum(counts.values())
    order = sorted(names, key=lambda name: (-(exact[name] - counts[name]), names.index(name)))
    for name in order[:remaining]:
        counts[name] += 1
    return counts


def stratified_split(
    dataset_ids: Iterable[str],
    strata: dict[str, str] | None = None,
    *,
    seed: int = SPLIT_SEED,
    ratios: dict[str, float] | None = None,
) -> dict[str, list[str]]:
    """Split IDs with exact global counts and approximate per-stratum ratios."""
    ratios = dict(ratios or SPLIT_RATIOS)
    if not np.isclose(sum(ratios.values()), 1.0):
        raise ValueError(f"split ratios must sum to one: {ratios}")
    ids = sorted(str(x) for x in dataset_ids)
    if len(ids) != len(set(ids)):
        raise ValueError("dataset IDs must be unique")
    target = _target_counts(len(ids), ratios)
    rng = np.random.default_rng(seed)
    grouped: dict[str, list[str]] = defaultdict(list)
    for ds_id in ids:
        grouped[(strata or {}).get(ds_id, "unknown")].append(ds_id)
    for group in grouped.values():
        rng.shuffle(group)

    result = {name: [] for name in ratios}
    global_count = {name: 0 for name in ratios}
    # Large strata first makes their ratios dominate while the global target
    # constraint guarantees 70/15/15 even with many singleton strata.
    for stratum in sorted(grouped, key=lambda key: (-len(grouped[key]), key)):
        local_count = {name: 0 for name in ratios}
        for placed, ds_id in enumerate(grouped[stratum]):
            eligible = [name for name in ratios if global_count[name] < target[name]]
            if not eligible:  # pragma: no cover - defensive, targets sum to n
                raise RuntimeError("split target accounting exhausted early")

            def score(name: str) -> tuple[float, float, int]:
                local_deficit = (placed + 1) * ratios[name] - local_count[name]
                global_deficit = target[name] - global_count[name]
                return (local_deficit, global_deficit / max(target[name], 1), -tuple(ratios).index(name))

            chosen = max(eligible, key=score)
            result[chosen].append(ds_id)
            local_count[chosen] += 1
            global_count[chosen] += 1

    for values in result.values():
        values.sort()
    return result


def build_split_manifest(collection_dir: Path, *, seed: int = SPLIT_SEED) -> dict:
    tasks = sorted(discover_cached_datasets(collection_dir), key=lambda item: item[0])
    features: dict[str, dict] = {}
    for ds_id, csv_path in tasks:
        try:
            ds = build_resampled(
                read_dataset_csv(csv_path), RESAMPLE_DT, context=f"split:{ds_id}",
            )
        except Exception:  # an unusable bag still belongs to a deterministic split
            ds = None
        features[ds_id] = driving_features(ds)
    strata = {ds_id: str(value["stratum"]) for ds_id, value in features.items()}
    splits = stratified_split(features, strata, seed=seed)
    assignment = {
        ds_id: {
            "split": split_name,
            "stratum": strata[ds_id],
            "features": {k: v for k, v in features[ds_id].items() if k != "stratum"},
        }
        for split_name, split_ids in splits.items()
        for ds_id in split_ids
    }
    return {
        "schema_version": SPLIT_SCHEMA_VERSION,
        "seed": seed,
        "ratios": dict(SPLIT_RATIOS),
        "stratification": "driving_characteristics_v1",
        "splits": splits,
        "datasets": dict(sorted(assignment.items())),
    }


def write_split_manifest(collection_dir: Path, out: Path, *, seed: int = SPLIT_SEED) -> dict:
    manifest = build_split_manifest(collection_dir, seed=seed)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(manifest, stream, allow_unicode=True, sort_keys=False)
    return manifest


def load_split_manifest(
    path: Path,
    *,
    expected_seed: int | None = SPLIT_SEED,
    expected_ratios: Mapping[str, float] | None = SPLIT_RATIOS,
) -> dict:
    with path.open(encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    if not isinstance(manifest, dict) or not isinstance(manifest.get("splits"), dict):
        raise ValueError(f"invalid split manifest: {path}")
    if manifest.get("schema_version") != SPLIT_SCHEMA_VERSION:
        raise ValueError(
            f"split manifest schema mismatch: expected={SPLIT_SCHEMA_VERSION}, "
            f"actual={manifest.get('schema_version')!r}"
        )
    if expected_seed is not None and manifest.get("seed") != expected_seed:
        raise ValueError(
            f"split manifest seed mismatch: expected={expected_seed}, "
            f"actual={manifest.get('seed')!r}"
        )
    if expected_ratios is not None:
        actual_ratios = manifest.get("ratios")
        if not isinstance(actual_ratios, Mapping) or set(actual_ratios) != set(expected_ratios):
            raise ValueError(
                f"split manifest ratio keys mismatch: expected={dict(expected_ratios)}, "
                f"actual={actual_ratios!r}"
            )
        if any(
            not np.isclose(float(actual_ratios[name]), float(expected_ratios[name]))
            for name in expected_ratios
        ):
            raise ValueError(
                f"split manifest ratios mismatch: expected={dict(expected_ratios)}, "
                f"actual={actual_ratios!r}"
            )

    values = [str(value) for split_values in manifest["splits"].values() for value in split_values]
    if len(values) != len(set(values)):
        raise ValueError("split manifest contains duplicate dataset IDs")
    return manifest


def ids_for_split(manifest: dict | None, split_name: str) -> set[str] | None:
    if manifest is None:
        return None
    values = manifest.get("splits", {}).get(split_name, [])
    return {str(value) for value in values}


def validate_manifest_dataset_ids(manifest: dict, dataset_ids: Iterable[str]) -> None:
    expected = {str(value) for values in manifest.get("splits", {}).values() for value in values}
    actual = {str(value) for value in dataset_ids}
    if expected != actual:
        missing = sorted(expected - actual)
        added = sorted(actual - expected)
        raise ValueError(
            "split manifest does not match cached datasets; regenerate it explicitly: "
            f"missing={missing}, added={added}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate or validate the deterministic reidentification split manifest"
    )
    parser.add_argument("--collection-dir", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=SPLIT_SEED)
    args = parser.parse_args()

    dataset_ids = [dataset_id for dataset_id, _ in discover_cached_datasets(args.collection_dir)]
    if args.out.exists():
        manifest = load_split_manifest(args.out, expected_seed=args.seed)
        validate_manifest_dataset_ids(manifest, dataset_ids)
        print(f"[split] validated existing manifest: {args.out}")
        return
    manifest = write_split_manifest(args.collection_dir, args.out, seed=args.seed)
    counts = {name: len(values) for name, values in manifest["splits"].items()}
    print(f"[split] generated: {args.out} ({counts})")


if __name__ == "__main__":
    main()
