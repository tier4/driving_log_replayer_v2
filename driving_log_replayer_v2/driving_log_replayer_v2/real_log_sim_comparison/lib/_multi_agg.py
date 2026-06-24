"""データセット横断の正規化集約・ロバストスコア (multi_dataset_tune / step13 の共有純関数).

per-dataset の N-step 終端誤差 RMSE (yaw/縦/横) を baseline 比で正規化してから
dataset 横断の mean / worst(max) を取る。生の deg/cm は baseline 誤差の大きい dataset が
支配し「全 dataset で良い」を達成できないため (= ロバスト化の核心)。

正規化 baseline は cases.yaml の overlay.reference_tag ケース (無補正 delay モデル相当) の
per-dataset メトリクスとする。rollout を伴わない集計 (step13) と rollout を伴う同定
(multi_dataset_tune) の両方が同じ定義を共有する。
"""

from __future__ import annotations

import statistics as stats

# 評価する horizon 群。step7 sweep と同じ最大 horizon (N=20) に加え、より長い N=40 で
# dynamics 累積差を観測する。スコア (robust_score) は両 horizon を等重みで集約する。
HORIZONS = (20, 40)

# per-dataset 正規化の分母フロア (horizon 別・成分別)。ほぼ直進・低ダイナミクス走行は baseline
# 誤差が極小で、相対誤差 (err/baseline) が暴発し worst-case を支配する (絶対値は微小なのに)。
# pos を縦/横に分けると縦・横でスケールが大きく異なる。20 dataset の baseline 分布
# (multi_dataset_tune.load_datasets の print) では【縦の方が横より大きい】(縦 1.3〜7.4cm vs
# 横 0.1〜3.4cm @N20; baseline モデルは縦方向の遅延/時定数が支配的)。フロアが大きすぎると
# baseline ですら正規化値が 1 未満になりその成分の寄与が一律縮小される (= 信号を殺す) ため、
# 各成分は分布下位の低ダイナ走行のみをクリップする水準に校正する (横は小さいのでフロアも低め)。
YAW_FLOOR = {20: 0.12, 40: 0.24}   # deg (分布下位 ~20% の低ダイナ yaw をクリップ)
LONG_FLOOR = {20: 2.0, 40: 4.5}    # cm  (縦は誤差が大きい → フロアも大きめ)
LAT_FLOOR = {20: 0.6, 40: 1.2}     # cm  (横は誤差が小さい → フロアも小さめ)

WORST_W = 0.5  # worst-case 項の重み (mean+worst 両方を balance する方針)
POS_W = 0.5    # 縦・横 各成分の重み。pos を縦横に分けても yaw:位置 = 1:1 を維持する
#              (位置 = 0.5·縦 + 0.5·横)。旧 nyaw+npos のバランスと整合させるための補正。


def normalize_components(m: dict, baseline: dict, h: int) -> dict:
    """1 dataset・1 horizon の誤差 RMSE をフロアクリップ付き baseline 比で正規化する。

    m / baseline: {"yaw" [deg], "long" [cm], "lat" [cm]} (rmse_by_horizon の値)。
    返り値は生値 + 正規化値 {yaw, long, lat, nyaw, nlong, nlat}。
    """
    return {
        "yaw": m["yaw"],
        "long": m["long"],
        "lat": m["lat"],
        "nyaw": m["yaw"] / max(baseline["yaw"], YAW_FLOOR[h]),
        "nlong": m["long"] / max(baseline["long"], LONG_FLOOR[h]),
        "nlat": m["lat"] / max(baseline["lat"], LAT_FLOOR[h]),
    }


def aggregate_normalized(
    per_ds_metrics: list[tuple[str, dict[int, dict]]],
    baselines: dict[str, dict[int, dict]],
    horizons: tuple[int, ...] = HORIZONS,
) -> dict:
    """Dataset 横断で per-dataset 正規化した yaw/縦/横の mean と worst(max) を horizon 別に返す。

    per_ds_metrics: [(dataset_id, {h: {"yaw","long","lat"}})] — 評価対象の per-DS 誤差。
    baselines: {dataset_id: {h: {"yaw","long","lat"}}} — 正規化基準 (reference_tag ケース)。
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
    horizons: tuple[int, ...] = HORIZONS,
    worst_w: float = WORST_W,
) -> float:
    """ロバスト目的関数: 全 horizon の正規化 mean + worst (yaw + 0.5·縦 + 0.5·横)。小さいほど良い。

    horizon を等重みで集約する。縦・横は各 POS_W 倍で合算し yaw:位置 = 1:1 に保つ
    (pos を縦横へ分割しても yaw の相対重みが半減しないようにする)。mean だけだと縦/横の mean を
    稼ぐ proxy が特定エリアの worst を悪化させても採用されてしまうため、worst を重み付きで加えて
    mean と worst を両立させる。worst_w でチューニング時に重みを調整できる (default=WORST_W=0.5)。
    """
    s = 0.0
    for h in horizons:
        b = agg["by_h"][h]
        s += b["nyaw_mean"] + POS_W * (b["nlong_mean"] + b["nlat_mean"])
        s += worst_w * (b["nyaw_worst"] + POS_W * (b["nlong_worst"] + b["nlat_worst"]))
    return s


def steer_score(
    agg: dict,
    horizons: tuple[int, ...] = HORIZONS,
    worst_w: float = WORST_W,
) -> float:
    """ステアパラメータ専用スコア: yaw + lat のみ (long は無視)。小さいほど良い。

    2フェーズ独立チューニングの Phase 2 用。long⊥steer が成立するため long を除外する。
    """
    s = 0.0
    for h in horizons:
        b = agg["by_h"][h]
        s += b["nyaw_mean"] + POS_W * b["nlat_mean"]
        s += worst_w * (b["nyaw_worst"] + POS_W * b["nlat_worst"])
    return s


def acc_score(
    agg: dict,
    horizons: tuple[int, ...] = HORIZONS,
    worst_w: float = WORST_W,
) -> float:
    """加速度パラメータ専用スコア: long のみ (yaw/lat は無視)。小さいほど良い。

    2フェーズ独立チューニングの Phase 1 用。long⊥steer が成立するため yaw/lat を除外する。
    """
    s = 0.0
    for h in horizons:
        b = agg["by_h"][h]
        s += b["nlong_mean"]
        s += worst_w * b["nlong_worst"]
    return s


def score_formula_md(horizons: tuple[int, ...] = HORIZONS, worst_w: float = WORST_W) -> str:
    """robust_score の定義を Markdown 1 行で返す (レポート埋め込み用)。"""
    return (
        f"`score = Σ_h (nyaw_mean + {POS_W}·(nlong_mean + nlat_mean)) "
        f"+ {worst_w}·(nyaw_worst + {POS_W}·(nlong_worst + nlat_worst))`  "
        f"(h ∈ {list(horizons)}、縦横各 {POS_W} で yaw:位置=1:1、小さいほど良い)"
    )


def format_agg(tag: str, agg: dict, horizons: tuple[int, ...] = HORIZONS) -> str:
    """集約結果の 1 行サマリ (探索ログ用)。"""
    seg = []
    for h in horizons:
        b = agg["by_h"][h]
        seg.append(
            f"N{h}[ny_m={b['nyaw_mean']:.3f}/w={b['nyaw_worst']:.3f} "
            f"nlo_m={b['nlong_mean']:.3f}/w={b['nlong_worst']:.3f} "
            f"nla_m={b['nlat_mean']:.3f}/w={b['nlat_worst']:.3f}]"
        )
    return f"{tag:14s} " + " ".join(seg)
