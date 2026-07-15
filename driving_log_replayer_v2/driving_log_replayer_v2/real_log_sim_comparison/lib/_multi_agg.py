"""Cross-dataset normalized scores used by reidentify optimization."""

from __future__ import annotations

import statistics as stats

# 直進・低ダイナミクス走行の極小 baseline で相対誤差が暴発しないようにする分母フロア。
# horizon N に比例させる (旧: N=10..300 の固定辞書と同値)。任意の horizon で使える。
YAW_FLOOR_PER_STEP = 0.006  # deg/step
LONG_FLOOR_PER_STEP = 0.1   # cm/step
LAT_FLOOR_PER_STEP = 0.03   # cm/step

# steer/ax は安定な 1 次遅れ系の状態量で、open-loop 誤差が N≈20 (steer) / N≈9 (ax) で
# 定常値に飽和する (プラトー特性)。そのためフロアは N に比例させず定数とする。
# 値は baseline モデルの per-dataset RMSE 分布 (N=10/30, 318 datasets) の p10。
STEER_FLOOR_DEG = 0.12   # deg (baseline steer@N=10/30 p10 = 0.128/0.131)
AX_FLOOR_MPS2 = 0.10     # m/s^2 (baseline ax@N=10/30 p10 = 0.105/0.112)

POS_W = 0.5    # 縦・横 各成分の重み。pos を縦横に分けても yaw:位置 = 1:1 を維持する
ACT_W = 0.5    # アクチュエータ (steer/ax) 成分の重み。robust_score の act_horizons 項で使用

def normalize_components(m: dict, baseline: dict, h: int) -> dict:
    """1 dataset・1 horizon の誤差 RMSE をフロアクリップ付き baseline 比で正規化する。

    m / baseline: {"yaw" [deg], "long" [cm], "lat" [cm]} (rmse_by_horizon の値)。
    返り値は生値 + 正規化値 {yaw, long, lat, nyaw, nlong, nlat}。
    m / baseline の両方に steer [deg] / ax [m/s^2] があれば nsteer / nax も返す。
    steer/ax のフロアはプラトー特性 (誤差が N 非依存に飽和) により h を掛けない定数。
    """
    out = {
        "yaw": m["yaw"],
        "long": m["long"],
        "lat": m["lat"],
        "nyaw": m["yaw"] / max(baseline["yaw"], YAW_FLOOR_PER_STEP * h),
        "nlong": m["long"] / max(baseline["long"], LONG_FLOOR_PER_STEP * h),
        "nlat": m["lat"] / max(baseline["lat"], LAT_FLOOR_PER_STEP * h),
    }
    if "steer" in m and "steer" in baseline:
        out["steer"] = m["steer"]
        out["nsteer"] = m["steer"] / max(baseline["steer"], STEER_FLOOR_DEG)
    if "ax" in m and "ax" in baseline:
        out["ax"] = m["ax"]
        out["nax"] = m["ax"] / max(baseline["ax"], AX_FLOOR_MPS2)
    return out


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
        # steer/ax は normalize_components が両キーを持つ場合のみ集約する (後方互換)。
        for key in ("nsteer", "nax"):
            if all(key in d["by_h"][h] for d in per_ds):
                values = [d["by_h"][h][key] for d in per_ds]
                by_h_agg[h][f"{key}_mean"] = stats.mean(values)
                by_h_agg[h][f"{key}_worst"] = max(values)
    return {"per_ds": per_ds, "by_h": by_h_agg}


def robust_score(
    agg: dict,
    horizons: tuple[int, ...],
    *,
    act_horizons: tuple[int, ...] = (),
    act_w: float = ACT_W,
) -> float:
    """ロバスト目的関数: 全 horizon の正規化 mean + worst (yaw + 0.5·縦 + 0.5·横)。小さいほど良い。

    horizon を等重みで集約する。縦・横は各 POS_W 倍で合算し yaw:位置 = 1:1 に保つ
    (pos を縦横へ分割しても yaw の相対重みが半減しないようにする)。mean だけだと縦/横の mean を
    稼ぐ proxy が特定エリアの worst を悪化させても採用されてしまうため、worst を重み付きで加えて
    mean と worst を両立させる。worst-case 項の重みは 0.5 とする。

    act_horizons を与えると、その各 horizon でアクチュエータ項
    act_w·(nsteer_mean + nax_mean) + 0.5·act_w·(nsteer_worst + nax_worst) を加算する。
    steer/ax の open-loop 誤差は N≈20 までに定常値へ飽和する (プラトー特性) ため、
    全 horizon でなく過渡 (N=10) とプラトー (N=30) の代表 2 点に限定するのが前提。
    デフォルト act_horizons=() は旧目的関数 (yaw/long/lat のみ) と完全一致する。
    """
    s = 0.0
    for h in horizons:
        b = agg["by_h"][h]
        s += b["nyaw_mean"] + POS_W * (b["nlong_mean"] + b["nlat_mean"])
        s += 0.5 * (b["nyaw_worst"] + POS_W * (b["nlong_worst"] + b["nlat_worst"]))
    for h in act_horizons:
        b = agg["by_h"][h]
        s += act_w * (b["nsteer_mean"] + b["nax_mean"])
        s += 0.5 * act_w * (b["nsteer_worst"] + b["nax_worst"])
    return s


def format_agg(tag: str, agg: dict, horizons: tuple[int, ...]) -> str:
    """集約結果の 1 行サマリ (探索ログ用)。

    ``aggregate_normalized`` 形式を表示する。
    """
    seg = []
    for h in horizons:
        b = agg["by_h"][h]
        text = (
            f"N{h}[ny_m={b['nyaw_mean']:.3f}/w={b['nyaw_worst']:.3f} "
            f"nlo_m={b['nlong_mean']:.3f}/w={b['nlong_worst']:.3f} "
            f"nla_m={b['nlat_mean']:.3f}/w={b['nlat_worst']:.3f}"
        )
        if "nsteer_mean" in b and "nax_mean" in b:
            text += (
                f" ns_m={b['nsteer_mean']:.3f}/w={b['nsteer_worst']:.3f}"
                f" na_m={b['nax_mean']:.3f}/w={b['nax_worst']:.3f}"
            )
        seg.append(text + "]")
    return f"{tag:14s} " + " ".join(seg)
