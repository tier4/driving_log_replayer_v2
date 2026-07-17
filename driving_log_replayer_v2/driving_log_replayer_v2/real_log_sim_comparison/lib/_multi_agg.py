"""Cross-dataset normalized scores used by reidentify optimization."""

from __future__ import annotations

import math
import statistics as stats

# objective v3 のフロア: baseline モデルの per-dataset RMSE 分布の p10 を horizon 別に採用
# (steer/ax フロアと同じ方法論。openloop_j6_16_onwards, 318 datasets, metrics.csv, 2026-07-16)。
# score horizon は settings.HORIZONS に集約されているため、テーブル外の h は fail fast にする。
FLOOR_TABLE: dict[int, dict[str, float]] = {
    10:  {"yaw": 0.069, "long": 1.5,   "lat": 0.24},
    30:  {"yaw": 0.176, "long": 5.3,   "lat": 1.0},
    70:  {"yaw": 0.364, "long": 18.4,  "lat": 3.9},
    150: {"yaw": 0.635, "long": 66.3,  "lat": 14.6},
    300: {"yaw": 0.893, "long": 175.3, "lat": 32.9},
}

# steer/ax は安定な 1 次遅れ系の状態量で、open-loop 誤差が N≈20 (steer) / N≈9 (ax) で
# 定常値に飽和する (プラトー特性)。そのためフロアは N に比例させず定数とする。
# 値は baseline モデルの per-dataset RMSE 分布 (N=10/30, 318 datasets) の p10。
STEER_FLOOR_DEG = 0.12   # deg (baseline steer@N=10/30 p10 = 0.128/0.131)
AX_FLOOR_MPS2 = 0.10     # m/s^2 (baseline ax@N=10/30 p10 = 0.105/0.112)

POS_W = 0.5    # 縦・横 各成分の重み。pos を縦横に分けても yaw:位置 = 1:1 を維持する
ACT_W = 0.5    # アクチュエータ (steer/ax) 成分の重み。robust_score の act_horizons 項で使用

# worst 側テールの分位点。CVaR@90% = 正規化比の上位 10% の平均。
# max だと dataset 数が多いとき単一の外れ dataset が score を支配する
# (318 datasets の実測で worst 項が score の 61%、外れ 1 件除外で score が −9.3% 変動)。
CVAR_Q = 0.90


def cvar_worst(values: list[float]) -> float:
    """上位 ceil(n·(1-CVAR_Q)) 件の平均 (CVaR@90%)。n≤10 では max に一致する。"""
    ordered = sorted(values, reverse=True)
    k = max(1, math.ceil(len(ordered) * (1.0 - CVAR_Q)))
    return stats.mean(ordered[:k])


def _floor(key: str, h: int) -> float:
    if h not in FLOOR_TABLE:
        raise ValueError(f"FLOOR_TABLE に horizon N={h} のフロアがありません (対応: {sorted(FLOOR_TABLE)})")
    return FLOOR_TABLE[h][key]


def normalize_components(m: dict, baseline: dict, h: int) -> dict:
    """1 dataset・1 horizon の誤差 RMSE をフロアクリップ付き baseline 比で正規化する。

    m / baseline: {"yaw" [deg], "long" [cm], "lat" [cm]} (rmse_by_horizon の値)。
    返り値は生値 + 正規化値 {yaw, long, lat, nyaw, nlong, nlat}。
    m / baseline の両方に steer [deg] / ax [m/s^2] があれば nsteer / nax も返す。
    steer/ax のフロアはプラトー特性 (誤差が N 非依存に飽和) により h を掛けない定数。
    FLOOR_TABLE に定義された horizon のみを受け付ける。
    """
    out = {
        "yaw": m["yaw"],
        "long": m["long"],
        "lat": m["lat"],
        "nyaw": m["yaw"] / max(baseline["yaw"], _floor("yaw", h)),
        "nlong": m["long"] / max(baseline["long"], _floor("long", h)),
        "nlat": m["lat"] / max(baseline["lat"], _floor("lat", h)),
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
    """Dataset 横断で per-dataset 正規化した yaw/縦/横の mean・worst(max)・cvar を horizon 別に返す。

    per_ds_metrics: [(dataset_id, {h: {"yaw","long","lat"}})] — 評価対象の per-DS 誤差。
    baselines: {dataset_id: {h: {"yaw","long","lat"}}} — baseline model の正規化基準。
    返り値:
      per_ds: [{dataset_id, by_h: {h: {yaw,long,lat, nyaw,nlong,nlat}}}]
      by_h:   {h: {nyaw_mean,nyaw_worst,nyaw_cvar, nlong_..., nlat_...}}
    worst は max、cvar は CVaR@90%。
    """
    per_ds = [
        {
            "dataset_id": ds_id,
            "by_h": {
                h: normalize_components(m[h], baselines[ds_id][h], h)
                for h in horizons
            },
        }
        for ds_id, m in per_ds_metrics
    ]

    by_h_agg = {}
    for h in horizons:
        by_h_agg[h] = {}
        for key in ("nyaw", "nlong", "nlat"):
            values = [d["by_h"][h][key] for d in per_ds]
            by_h_agg[h][f"{key}_mean"] = stats.mean(values)
            by_h_agg[h][f"{key}_worst"] = max(values)
            by_h_agg[h][f"{key}_cvar"] = cvar_worst(values)
        # steer/ax は normalize_components が両キーを持つ場合のみ集約する (後方互換)。
        for key in ("nsteer", "nax"):
            if all(key in d["by_h"][h] for d in per_ds):
                values = [d["by_h"][h][key] for d in per_ds]
                by_h_agg[h][f"{key}_mean"] = stats.mean(values)
                by_h_agg[h][f"{key}_worst"] = max(values)
                by_h_agg[h][f"{key}_cvar"] = cvar_worst(values)
    return {"per_ds": per_ds, "by_h": by_h_agg}


def robust_score(
    agg: dict,
    horizons: tuple[int, ...],
    *,
    act_horizons: tuple[int, ...] = (),
    act_w: float = ACT_W,
    worst_stat: str = "max",
) -> float:
    """ロバスト目的関数: 全 horizon の正規化 mean + worst (yaw + 0.5·縦 + 0.5·横)。小さいほど良い。

    horizon を等重みで集約する。縦・横は各 POS_W 倍で合算し yaw:位置 = 1:1 に保つ
    (pos を縦横へ分割しても yaw の相対重みが半減しないようにする)。mean だけだと縦/横の mean を
    稼ぐ proxy が特定エリアの worst を悪化させても採用されてしまうため、worst を重み付きで加えて
    mean と worst を両立させる。worst-case 項の重みは 0.5 とする。

    worst_stat は worst 項の統計量: "max" (単一最悪値) か "cvar" (CVaR@90% =
    上位 10% 平均)。max は dataset 数が多いとき単一の外れ dataset に score が支配されるため、
    最適化には cvar を使う。

    act_horizons を与えると、その各 horizon でアクチュエータ項
    act_w·(nsteer_mean + nax_mean) + 0.5·act_w·(nsteer_worst + nax_worst) を加算する。
    steer/ax の open-loop 誤差は N≈20 までに定常値へ飽和する (プラトー特性) ため、
    全 horizon でなく過渡 (N=10) とプラトー (N=30) の代表 2 点に限定するのが前提。
    デフォルト act_horizons=() では yaw/long/lat のみを集約する。
    """
    if worst_stat == "max":
        suffix = "worst"
    elif worst_stat == "cvar":
        suffix = "cvar"
    else:
        raise ValueError(f"worst_stat は 'max' か 'cvar' を指定してください: {worst_stat!r}")
    s = 0.0
    for h in horizons:
        b = agg["by_h"][h]
        s += b["nyaw_mean"] + POS_W * (b["nlong_mean"] + b["nlat_mean"])
        s += 0.5 * (b[f"nyaw_{suffix}"] + POS_W * (b[f"nlong_{suffix}"] + b[f"nlat_{suffix}"]))
    for h in act_horizons:
        b = agg["by_h"][h]
        s += act_w * (b["nsteer_mean"] + b["nax_mean"])
        s += 0.5 * act_w * (b[f"nsteer_{suffix}"] + b[f"nax_{suffix}"])
    return s


def format_agg(tag: str, agg: dict, horizons: tuple[int, ...]) -> str:
    """集約結果の 1 行サマリ (探索ログ用)。値は mean/cvar(@90%)/worst(max) の順。

    ``aggregate_normalized`` 形式を表示する。
    """
    seg = []
    for h in horizons:
        b = agg["by_h"][h]

        def fmt(key: str, b: dict = b) -> str:
            text = f"{b[f'{key}_mean']:.3f}"
            if f"{key}_cvar" in b:
                text += f"/{b[f'{key}_cvar']:.3f}"
            return text + f"/{b[f'{key}_worst']:.3f}"

        text = f"N{h}[ny={fmt('nyaw')} nlo={fmt('nlong')} nla={fmt('nlat')}"
        if "nsteer_mean" in b and "nax_mean" in b:
            text += f" ns={fmt('nsteer')} na={fmt('nax')}"
        seg.append(text + "]")
    return f"{tag:14s} " + " ".join(seg)
