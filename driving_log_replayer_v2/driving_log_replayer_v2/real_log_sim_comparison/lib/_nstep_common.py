"""N-step オープンループ解析 (step5/6/7) の共有定数・集計ヘルパー.

nstep_delta.csv の統一スキーマ (horizon, tr, err_ds_long/lat, err_steer,
pos_err, yaw_err_deg, ...) を消費するモジュール間で、メトリクス定義・
注意書き・horizon 集計を一元管理する。
"""

from __future__ import annotations

from collections.abc import Iterable
import math
from pathlib import Path
import sys

import numpy as np
import pandas as pd

# 誤差メトリクス定義: (列名, スケール, ラベル, 単位, データソース注)。
# step5 の時系列/散布図と step6 の overlay が同じ並びで使う。
ERR_METRICS: list[tuple[str, float, str, str, str]] = [
    ("err_ds_long", 100.0, "縦方向誤差", "cm",
     "実機: kinematic_state/pose.position  モデル: rollout 終端 state_[0,1]"),
    ("err_ds_lat", 100.0, "横方向誤差", "cm",
     "実機: kinematic_state/pose.position  モデル: rollout 終端 state_[0,1]"),
    ("err_steer", 180.0 / math.pi, "ステア予測誤差", "deg",
     "実機: steering_status/tire_angle  モデル: state_[4]+steer_bias"),
    ("yaw_err_deg", 1.0, "yaw 誤差", "deg",
     "実機: kinematic_state/pose.orientation  モデル: rollout 終端 state_[2]"),
    ("err_vx", 1.0, "速度予測誤差", "m/s",
     "実機: velocity_status vx[k_end]  モデル: rollout 終端 state_[3]"),
    ("err_ax", 1.0, "加速度予測誤差", "m/s²",
     "実機: localization/acceleration ax[k_end]  モデル: rollout 終端 state_[5]"),
]

# yaw/wz 誤差パネルに必ず添える seed バイアス注意 (step5 run_rollout docstring 参照)
YAW_SEED_NOTE = (
    "注: 小 N の yaw 誤差は seed (k_us=0 bicycle 逆算) 由来のバイアスを含む。"
    "dynamics 差の判定は大 N を参照"
)


def to_seconds(df: pd.DataFrame, t0_ns: int) -> pd.DataFrame:
    """`t_ns` を `t` [s] に変換したコピーを返す。"""
    out = df.copy()
    out["t"] = (out["t_ns"] - t0_ns) / 1e9
    return out.drop(columns=["t_ns"])


def interp_or_zeros(
    t_new: np.ndarray,
    t_ref: np.ndarray,
    values: np.ndarray,
    *,
    empty_dtype: np.dtype | type = np.float64,
) -> np.ndarray:
    """`np.interp` の空系列用フォールバック付きラッパー。"""
    if len(t_ref) == 0:
        return np.zeros_like(t_new, dtype=empty_dtype)
    return np.interp(t_new, t_ref, values)


def wrap_pi(angle):
    """角度差を [-pi, pi) に正規化する。"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def local_ds(
    dx: np.ndarray,
    dy: np.ndarray,
    cos_y: np.ndarray,
    sin_y: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """yaw 基準の車両ローカル座標系へ変位を射影する。"""
    return dx * cos_y + dy * sin_y, -dx * sin_y + dy * cos_y


def rms(values: np.ndarray) -> float:
    """NaN を無視して RMS を返す。"""
    arr = np.asarray(values, dtype=float)
    return float(np.sqrt(np.nanmean(arr * arr)))


def metrics_description_md() -> str:
    """各レポート冒頭に埋め込むメトリクス説明 (Markdown)。

    ERR_METRICS / YAW_SEED_NOTE を一次情報として、N-step オープンループ評価で
    使う縦/横/yaw/steer 誤差の定義・座標系・データソースと N-step の意味を一元的に記す。
    step5(summary.txt) は `# ` プレフィックスを付けて行コメントとして埋め込む。
    """
    return (
        "## メトリクス説明\n"
        "\n"
        "各開始点 k0 で実機状態 (過去コマンド履歴を含む) にリセットし、実コマンド系列を\n"
        "N ステップ連続適用 (途中リセット無し = free-running) した終端 k_end=k0+N の予測状態と\n"
        "実機状態を比較した終端誤差の RMSE を horizon (N) 別に集計する。\n"
        "N を増やすほど dynamics 差 (k_us / wheelbase / 各時定数) の累積が顕在化する。\n"
        "\n"
        "- **縦方向誤差 (long)** [cm]: k0 時点の実機ヨーを基準とした車両ローカル座標系での\n"
        "  前後方向の変位誤差。実機 kinematic_state/pose.position と rollout 終端 state_[0,1] の差。\n"
        "- **横方向誤差 (lat)** [cm]: 同ローカル座標系での左右方向の変位誤差 (データソースは縦と同じ)。\n"
        "- **yaw 誤差 (yaw)** [deg]: ヨー角の差 (−π〜π に正規化)。\n"
        "  実機 kinematic_state/pose.orientation と rollout 終端 state_[2] の差。\n"
        "- **ステア予測誤差 (steer)** [deg]: 実機 steering_status/tire_angle と"
        " モデル state_[4]+steer_bias の差。\n"
        "- **速度予測誤差 (vx)** [m/s]: 実機 velocity_status と rollout 終端 state_[3] の差。\n"
        "  縦方向ダイナミクス (acc 時定数/むだ時間/スケーリング) の累積効果を保持し、終端評価でも弁別力が高い。\n"
        "- **加速度予測誤差 (ax)** [m/s²]: 実機 localization/acceleration と rollout 終端 state_[5] の差。\n"
        "  瞬時応答を直接反映するが信号が雑なため、縦方向同定では vx を主・ax を副に用いる。\n"
        "\n"
        f"> {YAW_SEED_NOTE}。\n"
    )


def n1(df: pd.DataFrame) -> pd.DataFrame:
    """最小 horizon (通常 N=1, 毎ステップリセット相当) のサブセットを返す。"""
    return df[df["horizon"] == df["horizon"].min()]


def load_case_csvs(nstep_root: Path, tags: list[str], *, verbose: bool = False) -> dict[str, pd.DataFrame]:
    """nstep/<tag>/nstep_delta.csv を全 tag 分読み込む。欠損 tag はスキップする。"""
    out: dict[str, pd.DataFrame] = {}
    for tag in tags:
        csv = Path(nstep_root) / tag / "nstep_delta.csv"
        if not csv.exists():
            if verbose:
                print(f"[WARN] {csv} が無いため case={tag} をスキップ", file=sys.stderr)
            continue
        df = pd.read_csv(csv)
        if df.empty:
            continue
        out[tag] = df
    return out


def common_horizons(horizon_sets: Iterable[Iterable[int]]) -> list[int]:
    """全集合に共通する horizon の昇順リストを返す。"""
    return sorted(int(h) for h in set.intersection(*map(set, horizon_sets)))


def parabolic_min(xs: list[float], ys: list[float]) -> float | None:
    """argmin 近傍 3 点に二次フィットして頂点 (サブグリッド最小) を返す。端なら None。

    パラメータ sweep 同定 (step7) の共有ヘルパー。
    """
    i = int(np.argmin(ys))
    if i == 0 or i == len(xs) - 1:
        return None
    x0, x1, x2 = xs[i - 1], xs[i], xs[i + 1]
    y0, y1, y2 = ys[i - 1], ys[i], ys[i + 1]
    denom = (x0 - x1) * (x0 - x2) * (x1 - x2)
    if abs(denom) < 1e-18:
        return None
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom
    b = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / denom
    if a <= 0:  # 下に凸でなければ頂点は最小でない
        return None
    return -b / (2 * a)


def rmse_by_horizon(df: pd.DataFrame) -> dict[int, dict[str, float]]:
    """horizon 別の終端誤差 RMSE を返す。

    {N: {"pos","long","lat" [cm], "yaw" [deg], "steer" [deg], "vx" [m/s], "ax" [m/s²]}}

    vx/ax 列が無い古い nstep_delta.csv との後方互換のため、列欠落時はそのキーを省く。
    """

    out: dict[int, dict[str, float]] = {}
    for horizon in sorted(df["horizon"].unique()):
        sub = df[df["horizon"] == horizon]
        rec = {
            "pos": rms(sub["pos_err"].values) * 100.0,
            "long": rms(sub["err_ds_long"].values) * 100.0,
            "lat": rms(sub["err_ds_lat"].values) * 100.0,
            "yaw": rms(sub["yaw_err_deg"].values),
            "steer": rms(sub["err_steer"].values) * 180.0 / math.pi,
        }
        if "err_vx" in sub.columns:
            rec["vx"] = rms(sub["err_vx"].values)  # [m/s]
        if "err_ax" in sub.columns:
            rec["ax"] = rms(sub["err_ax"].values)  # [m/s²]
        out[int(horizon)] = rec
    return out


def n1_case_metrics(df: pd.DataFrame) -> dict[str, float]:
    """N=1 のケース別 RMSE を集計する。"""
    df1 = n1(df)
    return {
        "steer": rms(df1["err_steer"].values) * (180.0 / math.pi),
        "long": rms(df1["err_ds_long"].values) * 100.0,
        "lat": rms(df1["err_ds_lat"].values) * 100.0,
        "n": float(len(df1)),
    }


def n1_summary_lines(case_dfs: dict[str, pd.DataFrame], cases_cfg) -> list[str]:
    """N=1 のケース横断 RMSE 表を Markdown 行にする。"""
    ps: dict[str, dict[str, float]] = {}
    for tag, df in case_dfs.items():
        if df.empty:
            continue
        ps[tag] = n1_case_metrics(df)
    ref_tag = cases_cfg.overlay.reference_tag
    ref_steer = ps.get(ref_tag, {}).get("steer") if ref_tag else None

    lines: list[str] = ["## N=1 (毎ステップリセット) RMSE\n"]
    lines.append(
        "| tag | vehicle_model | n_steps | RMSE err_steer [deg] | Δsteer vs ref [deg] | "
        "RMSE err_ds_long [cm] | RMSE err_ds_lat [cm] | note |"
    )
    lines.append("|---|---|---:|---:|---:|---:|---:|---|")
    for case in cases_cfg.cases:
        if case.tag not in ps:
            lines.append(f"| {case.tag} | {case.vehicle_model_type} | — | — | — | — | — | 出力欠損 |")
            continue
        d = ps[case.tag]
        if case.tag == ref_tag:
            delta = "基準"
        elif ref_steer is not None:
            delta = f"{d['steer'] - ref_steer:+.4f}"
        else:
            delta = "—"
        lines.append(
            f"| {case.tag} | {case.vehicle_model_type} | {int(d['n'])} | "
            f"{d['steer']:.4f} | {delta} | {d['long']:.3f} | {d['lat']:.3f} |  |"
        )
    lines.append("")
    lines.append(
        "> N=1 は毎ステップ実機状態にリセットするため、k_us/wheelbase の累積差は "
        "位置 RMSE にほぼ現れない。dynamics 差は下記 horizon 別 RMSE 表を参照。"
    )
    lines.append("")
    return lines


def horizon_summary_lines(
    roll: dict[str, dict[int, dict[str, float]]],
    cases_cfg,
) -> list[str]:
    """horizon 別の終端誤差 RMSE 表を Markdown 行にする。"""
    roll = {t: r for t, r in roll.items() if r}
    if not roll:
        return []

    horizons = common_horizons(r.keys() for r in roll.values())
    ref_tag = cases_cfg.overlay.reference_tag
    ref_roll = roll.get(ref_tag) if horizons else None

    lines: list[str] = ["## horizon 別 終端誤差 RMSE (free-running, ケース横断)\n"]
    head = (
        "| tag |"
        + "".join(f" 縦@N{h}[cm] |" for h in horizons)
        + "".join(f" 横@N{h}[cm] |" for h in horizons)
        + "".join(f" yaw@N{h}[deg] |" for h in horizons)
    )
    if ref_roll:
        head += f" Δyaw@N{horizons[-1]} vs ref [deg] |"
    lines.append(head)
    lines.append("|---|" + "---:|" * (len(horizons) * 3 + (1 if ref_roll else 0)))
    for case in cases_cfg.cases:
        r = roll.get(case.tag)
        if not r:
            continue
        row = f"| {case.tag} |"
        row += "".join(f" {r[h]['long']:.2f} |" for h in horizons)
        row += "".join(f" {r[h]['lat']:.2f} |" for h in horizons)
        row += "".join(f" {r[h]['yaw']:.3f} |" for h in horizons)
        if ref_roll:
            hl = horizons[-1]
            if case.tag == ref_tag:
                row += " 基準 |"
            else:
                row += f" {r[hl]['yaw'] - ref_roll[hl]['yaw']:+.3f} |"
        lines.append(row)
    lines.append("")
    lines.append(
        "> N ステップ連続予測 (途中リセット無し) の終端誤差。N を増やすほど dynamics 差が "
        "累積し、k_us/wheelbase の効果が分離する (パラメータ同定の指標)。"
        "小 N の yaw RMSE は seed (k_us=0 bicycle 逆算) 由来のバイアスを含む点に注意。"
    )
    lines.append("")
    return lines


def _finite_or_none(x) -> float | None:
    xf = float(x)
    return xf if math.isfinite(xf) else None


def build_cases_metrics_payload(
    case_dfs: dict[str, pd.DataFrame],
    roll: dict[str, dict[int, dict[str, float]]],
    cases_cfg,
    horizons: list[int],
    physical_validity: dict | None,
    *,
    physical_validity_jsonable,
) -> dict:
    """cases_metrics.json の payload を構築する。"""
    cases: dict[str, dict] = {}
    for case in cases_cfg.cases:
        df = case_dfs.get(case.tag)
        r = roll.get(case.tag)
        if df is None or not r:
            continue
        cases[case.tag] = {
            "vehicle_model": case.vehicle_model_type,
            "n_steps_n1": int(len(n1(df))),
            "by_h": {str(h): {k: _finite_or_none(v) for k, v in m.items()} for h, m in r.items()},
        }
    return {
        "schema_version": 1,
        "reference_tag": cases_cfg.overlay.reference_tag,
        "horizons": [int(h) for h in horizons],
        "cases": cases,
        "physical_validity": physical_validity_jsonable(physical_validity),
    }
