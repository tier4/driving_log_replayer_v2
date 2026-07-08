"""ROS-free path helpers for result bundles and lite bags."""

from __future__ import annotations

from pathlib import Path


def resolve_bundle_dir(raw: Path) -> Path:
    """Resolve a bundle directory that contains lite/."""
    raw = Path(raw)
    for c in (
        raw,
        raw / "result_archive" / "real_log_sim_comparison",
        raw / "real_log_sim_comparison",
    ):
        if (c / "lite").is_dir():
            return c.resolve()
    raise FileNotFoundError(f"lite/ を含むバンドルが見つかりません: {raw}")


def resolve_lite_bag(lite_dir: Path, name_stem: str) -> Path | None:
    """Resolve `<name>.lite.mcap` or `<name>.lite` without importing rosbag2."""
    lite_dir = Path(lite_dir)
    for cand in (lite_dir / f"{name_stem}.lite.mcap", lite_dir / f"{name_stem}.lite"):
        if cand.exists():
            return cand
    return None


def sim_tag_from_bag(bag: Path) -> str:
    """Return run tag from a sim lite bag path."""
    name = Path(bag).name
    if name.endswith(".mcap"):
        name = name[: -len(".mcap")]
    if name.endswith(".lite"):
        name = name[: -len(".lite")]
    return name


def resolve_primary_sim_bag(lite_dir: Path) -> Path | None:
    """Auto-detect the primary sim lite bag, preferring sim_normal."""
    lite_dir = Path(lite_dir)
    stems: list[str] = []
    for pat in ("sim_*.lite", "sim_*.lite.mcap"):
        for p in lite_dir.glob(pat):
            stem = sim_tag_from_bag(p)
            if stem and stem not in stems:
                stems.append(stem)
    if not stems:
        return None
    stems.sort(key=lambda s: (s != "sim_normal", s))
    return resolve_lite_bag(lite_dir, stems[0])
