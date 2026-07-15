"""Cross-dataset normalized scores used by reidentify optimization."""

from __future__ import annotations

import statistics as stats

# 直進・低ダイナミクス走行の極小 baseline で相対誤差が暴発しないようにする分母フロア。
# horizon N に比例させる (旧: N=10..300 の固定辞書と同値)。任意の horizon で使える。
YAW_FLOOR_PER_STEP = 0.006  # deg/step
LONG_FLOOR_PER_STEP = 0.1   # cm/step
LAT_FLOOR_PER_STEP = 0.03   # cm/step

POS_W = 0.5    # 縦・横 各成分の重み。pos を縦横に分けても yaw:位置 = 1:1 を維持する

def normalize_components(m: dict, baseline: dict, h: int) -> dict:
    """1 dataset・1 horizon の誤差 RMSE をフロアクリップ付き baseline 比で正規化する。

    m / baseline: {"yaw" [deg], "long" [cm], "lat" [cm]} (rmse_by_horizon の値)。
    返り値は生値 + 正規化値 {yaw, long, lat, nyaw, nlong, nlat}。
    """
    return {
        "yaw": m["yaw"],
        "long": m["long"],
        "lat": m["lat"],
        "nyaw": m["yaw"] / max(baseline["yaw"], YAW_FLOOR_PER_STEP * h),
        "nlong": m["long"] / max(baseline["long"], LONG_FLOOR_PER_STEP * h),
        "nlat": m["lat"] / max(baseline["lat"], LAT_FLOOR_PER_STEP * h),
    }


def aggregate_normalized(
    per_ds_metrics: list[tuple[str, dict[int, dict]]],
    baselines: dict[str, dict[int, dict]],
    horizons: tuple[int, ...],
) -> dict:
    """Dataset 横断で per-dataset 正規化した yaw/縦/横の mean と worst(max) を horizon 別に返す。

    per_ds_metrics: [(dataset_id, {h: {"yaw","long","lat"}})] — 評価対象の per-DS 誤差。
    baselines: {dataset_id: {h: {"yaw","long","lat"}}} — baseline model の正規化基準。
    返り値:
      per_ds: [{dataset_id, by_h: {h: {yaw,long,lat, nyaw,nlong,nlat}}}]
      by_h:   {h: {nyaw_mean,nyaw_worst, nlong_mean,nlong_worst, nlat_mean,nlat_worst}}
    """
    per_ds = [
        {
            "dataset_id": ds_id,
            "by_h": {h: normalize_components(m[h], baselines[ds_id][h], h) for h in horizons},
        }
        for ds_id, m in per_ds_metrics
    ]

    by_h_agg = {}
    for h in horizons:
        nyaws = [d["by_h"][h]["nyaw"] for d in per_ds]
        nlongs = [d["by_h"][h]["nlong"] for d in per_ds]
        nlats = [d["by_h"][h]["nlat"] for d in per_ds]
        by_h_agg[h] = {
            "nyaw_mean": stats.mean(nyaws),
            "nyaw_worst": max(nyaws),
            "nlong_mean": stats.mean(nlongs),
            "nlong_worst": max(nlongs),
            "nlat_mean": stats.mean(nlats),
            "nlat_worst": max(nlats),
        }
    return {"per_ds": per_ds, "by_h": by_h_agg}


def robust_score(
    agg: dict,
    horizons: tuple[int, ...],
) -> float:
    """ロバスト目的関数: 全 horizon の正規化 mean + worst (yaw + 0.5·縦 + 0.5·横)。小さいほど良い。

    horizon を等重みで集約する。縦・横は各 POS_W 倍で合算し yaw:位置 = 1:1 に保つ
    (pos を縦横へ分割しても yaw の相対重みが半減しないようにする)。mean だけだと縦/横の mean を
    稼ぐ proxy が特定エリアの worst を悪化させても採用されてしまうため、worst を重み付きで加えて
    mean と worst を両立させる。worst-case 項の重みは 0.5 とする。
    """
    s = 0.0
    for h in horizons:
        b = agg["by_h"][h]
        s += b["nyaw_mean"] + POS_W * (b["nlong_mean"] + b["nlat_mean"])
        s += 0.5 * (b["nyaw_worst"] + POS_W * (b["nlong_worst"] + b["nlat_worst"]))
    return s


def format_agg(tag: str, agg: dict, horizons: tuple[int, ...]) -> str:
    """集約結果の 1 行サマリ (探索ログ用)。

    ``aggregate_normalized`` 形式を表示する。
    """
    seg = []
    for h in horizons:
        b = agg["by_h"][h]
        seg.append(
            f"N{h}[ny_m={b['nyaw_mean']:.3f}/w={b['nyaw_worst']:.3f} "
            f"nlo_m={b['nlong_mean']:.3f}/w={b['nlong_worst']:.3f} "
            f"nla_m={b['nlat_mean']:.3f}/w={b['nlat_worst']:.3f}]"
        )
    return f"{tag:14s} " + " ".join(seg)
