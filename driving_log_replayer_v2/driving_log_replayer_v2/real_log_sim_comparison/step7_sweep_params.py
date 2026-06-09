#!/usr/bin/env python3
"""Stage 7: 車両モデルパラメータを N-step rollout sweep で同定する.

Stage 1〜6 の後段に位置する独立ステージ。Stage 1 が生成した実機 lite
(`lite/real.lite/`) を SSOT とし、各パラメータをグリッドでスイープして
`step5.run_rollout` (free-running N-step) を回し、最大 horizon の終端誤差 RMSE を
最小化する値を実効パラメータとして同定する (グリッド最小 + 放物線サブグリッド)。

なぜ rollout で同定するか:
  - N=1 (毎ステップリセット) は dynamics パラメータに非感度。
  - 小 N の yaw/wz 誤差は steer を k_us=0 の運動学逆算で seed するため
    seeding バイアスを含む (step5.run_rollout docstring 参照)。
  - free-running rollout は seed が k0 のみで、N ステップ後は真の dynamics が
    支配するため、最大 horizon の RMSE 最小化が実効パラメータの推定になる。

スイープ対象 (SWEEP_SPECS):
  - 横方向系 (同定メトリクス yaw or 横 RMSE):
      k_us / steer_time_constant / steer_time_delay / steer_bias / steer_dead_band /
      debug_steer_scaling_factor
      (wheelbase は実測値が正しいため sweep せず固定)
  - 縦方向系 (同定メトリクス 縦 RMSE):
      acc_time_constant / acc_time_delay / debug_acc_scaling_factor
  - 2D ペア (PAIR_SPECS): k_us × steer_time_constant / k_us × debug_steer_scaling_factor
      の yaw RMSE ヒートマップ
      (1 パラメータずつの sweep では見えないトレードオフ・谷の形状を可視化)

evaluator_node が Stage 6 の後に env (BEST_MODEL_BASE_DIR) のみで実行する (追加設定不要)。
手動実行・対象/グリッド変更も可能:
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.step7_sweep_params \
        --base-dir <out_dir> \
        [--params k_us,pair_k_us_steer_time_constant] \
        [--grid k_us=0,0.01,0.02 --grid steer_time_constant=0.1,0.2,0.4] \
        [--metric acc_time_constant=ax]  # 同定メトリクス上書き (既定: 縦=vx, 横=yaw/lat) \
        [--horizons 2,5,10,20,40] [--stride 5]

出力: <base-dir>/comparison/param_sweep/
    <name>_sweep.{csv,svg} (各パラメータ), pair_<a>_<b>.{csv,svg} (2D),
    param_sweep_summary.md (同定値一覧)
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path
import sys

import numpy as np
import pandas as pd

from . import step5_analyze_nstep as s5
import plotly.graph_objects as go

from .lib._fig_io import write_fig_json
from .lib._figures import build_fig_pair_sweep, build_fig_sweep, build_fig_sweep_overview
from .lib._figures._common import viridis_colors
from .lib._io import resolve_lite_bag
from .lib._nstep_common import metrics_description_md, parabolic_min, rmse_by_horizon
from .lib._runtime_config import add_common_cli_arguments, build_runtime_config

_MODEL_TYPE = "delay_steer_acc_geared_wo_fall_guard"  # 全 dynamics パラメータを持つ対応モデル

# 同定メトリクス (rmse_by_horizon のキー) → (表示名, 単位, res 列名)
_METRIC_INFO = {
    "yaw": ("yaw RMSE", "deg", "yaw_rmse_deg"),
    "lat": ("横方向 RMSE", "cm", "lat_rmse_cm"),
    "long": ("縦方向 RMSE", "cm", "long_rmse_cm"),
    "pos": ("位置 RMSE", "cm", "pos_rmse_cm"),
    "steer": ("ステア RMSE", "deg", "steer_rmse_deg"),
    "vx": ("速度 RMSE", "m/s", "vx_rmse_ms"),
    "ax": ("加速度 RMSE", "m/s²", "ax_rmse_ms2"),
}


@dataclass(frozen=True)
class SweepSpec:
    """1 パラメータの sweep 定義。

    grid_abs: 絶対値グリッド。grid_rel: 仕様値 (load_sim_params) に対する倍率グリッド
    (どちらか一方を指定)。affects_gt: _prepare_gt の出力 (kinematic seed や delay queue 窓)
    がパラメータに依存する場合 True — グリッド点ごとに GT を再計算する。
    """

    name: str
    label: str
    unit: str
    metric: str  # _METRIC_INFO のキー
    grid_abs: tuple[float, ...] | None = None
    grid_rel: tuple[float, ...] | None = None
    affects_gt: bool = False
    # params 欠落時にモデルが使う実効既定値 (k_us=0, scaling=1 等)。仕様値表示にも使う。
    default: float | None = None
    # 自動端拡張の物理的上下限 (これを超える拡張はしない)
    bounds: tuple[float, float] = (-float("inf"), float("inf"))
    # sweep 図に同定メトリクスと並べて表示する副メトリクス
    secondary_metrics: tuple[str, ...] = ("pos",)

    def base_value(self, base_params: dict) -> float | None:
        v = base_params.get(self.name, self.default)
        return float(v) if v is not None else None

    def grid(self, base_params: dict) -> list[float]:
        if self.grid_abs is not None:
            return sorted(self.grid_abs)
        base = self.base_value(base_params)
        return sorted(round(m * base, 6) for m in self.grid_rel)


# スイープ対象の既定定義。グリッドは CLI --grid name=v1,v2,... で上書き可能。
SWEEP_SPECS: list[SweepSpec] = [
    # --- 横方向系 ---
    SweepSpec(
        "k_us", "アンダーステア係数", "rad/(m/s²)", "yaw",
        grid_abs=(0.0, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05),
        default=0.0,
        bounds=(0.0, 0.1),
    ),
    SweepSpec(
        "steer_time_constant", "ステア時定数", "s", "yaw",
        grid_rel=(0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0),
        bounds=(0.01, 2.0),
        secondary_metrics=("steer", "pos"),
    ),
    SweepSpec(
        "steer_time_delay", "ステアむだ時間", "s", "yaw",
        grid_rel=(0.0, 0.5, 1.0, 1.5, 2.0, 3.0),
        affects_gt=True,  # delay queue 長 → 過去 cmd 窓の拡張幅が変わる
        bounds=(0.0, 1.0),
        secondary_metrics=("steer", "pos"),
    ),
    SweepSpec(
        "steer_bias", "ステアバイアス", "rad", "lat",
        grid_abs=(-0.01, -0.005, -0.002, -0.001, 0.0, 0.0005, 0.001, 0.002, 0.005, 0.01),
        bounds=(-0.05, 0.05),
        secondary_metrics=("steer", "pos"),
    ),
    SweepSpec(
        "steer_dead_band", "ステア不感帯", "rad", "lat",
        grid_abs=(0.0, 0.001, 0.002, 0.005, 0.01, 0.02),
        bounds=(0.0, 0.05),
        secondary_metrics=("steer", "pos"),
    ),
    SweepSpec(
        "debug_steer_scaling_factor", "ステアスケーリング", "-", "yaw",
        grid_abs=(0.80, 0.85, 0.90, 0.95, 1.0, 1.05, 1.10, 1.15, 1.20),
        default=1.0,
        bounds=(0.5, 1.5),
        secondary_metrics=("steer", "pos"),
    ),
    # --- 縦方向系 (rollout は加減速の全区間を含むため発進フィットより拘束が広い) ---
    # 同定メトリクスは速度 (vx): 終端評価では瞬時 ax は時定数を「忘れる」が積分量の vx は
    # 加減速ダイナミクスの累積効果を保持するため弁別力が高い (ax/long は副メトリクスで併記)。
    # 目的関数の切替は CLI --metric name=ax|long で可能。
    SweepSpec(
        "acc_time_constant", "加速度時定数", "s", "vx",
        grid_rel=(0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0),
        bounds=(0.01, 2.0),
        secondary_metrics=("ax", "long", "pos"),
    ),
    SweepSpec(
        "acc_time_delay", "加速度むだ時間", "s", "vx",
        grid_rel=(0.0, 0.5, 1.0, 1.5, 2.0, 3.0),
        affects_gt=True,
        bounds=(0.0, 1.0),
        secondary_metrics=("ax", "long", "pos"),
    ),
    SweepSpec(
        "debug_acc_scaling_factor", "加速度スケーリング", "-", "vx",
        grid_abs=(0.80, 0.85, 0.90, 0.95, 1.0, 1.05, 1.10, 1.15, 1.20),
        default=1.0,
        bounds=(0.5, 1.5),
        secondary_metrics=("ax", "long", "pos"),
    ),
]

# 2D ペア sweep: (param_a, param_b, metric)。1 パラメータずつでは見えない相関を可視化。
# wheelbase は実測値が正しいため固定 (sweep 対象外)。
PAIR_SPECS: list[tuple[str, str, str]] = [
    ("k_us", "steer_time_constant", "yaw"),
    ("k_us", "debug_steer_scaling_factor", "yaw"),
]
# 2D 用の縮小グリッド (フルグリッドの直積はコストが大きいため)
_PAIR_GRID_ABS: dict[str, tuple[float, ...]] = {
    "k_us": (0.0, 0.01, 0.02, 0.03, 0.04, 0.05),
}
_PAIR_GRID_ABS["debug_steer_scaling_factor"] = (0.80, 0.85, 0.90, 0.95, 1.0, 1.10)
_PAIR_GRID_REL: dict[str, tuple[float, ...]] = {
    "steer_time_constant": (0.5, 1.0, 1.5, 2.0, 3.0),
}


def _spec_by_name(name: str) -> SweepSpec:
    for spec in SWEEP_SPECS:
        if spec.name == name:
            return spec
    raise KeyError(name)


def _pair_grid(name: str, base_params: dict) -> list[float]:
    if name in _PAIR_GRID_ABS:
        return sorted(_PAIR_GRID_ABS[name])
    return sorted(round(m * float(base_params[name]), 6) for m in _PAIR_GRID_REL[name])


# ---------------------------------------------------------------------------
# sweep 実行
# ---------------------------------------------------------------------------


def _rollout_rmse(
    data: dict,
    t0_ns: int,
    params: dict,
    horizons: tuple[int, ...],
    stride: int,
    gt: dict | None,
) -> dict[int, dict[str, float]]:
    """1 グリッド点の rollout を回し horizon 別 RMSE を返す。"""
    df_roll = s5.run_rollout(
        data, t0_ns, params, _MODEL_TYPE, horizons=horizons, stride=stride, gt=gt
    )
    return rmse_by_horizon(df_roll)


def _rmse_row(rmse_at_h: dict[str, float]) -> dict:
    return {
        "yaw_rmse_deg": rmse_at_h["yaw"],
        "pos_rmse_cm": rmse_at_h["pos"],
        "lat_rmse_cm": rmse_at_h["lat"],
        "long_rmse_cm": rmse_at_h["long"],
        "steer_rmse_deg": rmse_at_h["steer"],
        "vx_rmse_ms": rmse_at_h["vx"],
        "ax_rmse_ms2": rmse_at_h["ax"],
    }


def run_param_sweep(
    data: dict,
    t0_ns: int,
    base_params: dict,
    spec: SweepSpec,
    grid: list[float],
    horizons: tuple[int, ...],
    stride: int,
    gt: dict | None = None,
) -> pd.DataFrame:
    """spec のグリッドを sweep し horizon 別 RMSE を集計した DataFrame を返す。"""
    # GT がパラメータ非依存なら 1 回だけ計算して共有 (run_rollout docstring 参照)
    if gt is None and not spec.affects_gt:
        gt = s5._prepare_gt(data, t0_ns, base_params)
    h_max = max(horizons)

    rows: list[dict] = []
    for value in grid:
        params = dict(base_params)
        params[spec.name] = value
        rmse = _rollout_rmse(data, t0_ns, params, horizons, stride, gt)
        for horizon in horizons:
            rows.append({"value": value, "horizon": int(horizon), **_rmse_row(rmse[horizon])})
        print(
            f"  {spec.name}={value:g}: "
            f"N{h_max} yaw={rmse[h_max]['yaw']:.3f}deg pos={rmse[h_max]['pos']:.2f}cm"
        )
    return pd.DataFrame(rows)


def identify(res: pd.DataFrame, spec: SweepSpec, h_max: int) -> dict:
    """最大 horizon の同定メトリクス RMSE を最小化する値 (グリッド + 放物線)。"""
    col = _METRIC_INFO[spec.metric][2]
    at = res[res["horizon"] == h_max].sort_values("value").reset_index(drop=True)
    xs = at["value"].tolist()
    ys = at[col].tolist()
    grid_best_i = int(np.argmin(ys))
    return {
        "horizon": h_max,
        "metric_col": col,
        "grid_value": float(xs[grid_best_i]),
        "grid_rmse": float(ys[grid_best_i]),
        "parabolic_value": parabolic_min(xs, ys),
        "at_edge": grid_best_i in (0, len(xs) - 1),
    }


# 端拡張: 1 回あたりの追加点数と最大拡張回数
_EXTEND_POINTS = 2
_MAX_EXTENSIONS = 3


def _extension_points(grid: list[float], low_edge: bool, bounds: tuple[float, float]) -> list[float]:
    """グリッド端の外側に追加する点を生成する。

    正値のみのグリッドは端の隣接比を保つ等比、それ以外 (0 や負を含む) は端の
    間隔を保つ等差で外挿する。spec.bounds を超える点は捨てる。
    """
    lo_b, hi_b = bounds
    pts: list[float] = []
    if low_edge:
        if all(v > 0 for v in grid) and grid[1] > 0:
            ratio = grid[0] / grid[1]  # < 1
            v = grid[0]
            for _ in range(_EXTEND_POINTS):
                v = round(v * ratio, 6)
                pts.append(v)
        else:
            step = grid[1] - grid[0]
            v = grid[0]
            for _ in range(_EXTEND_POINTS):
                v = round(v - step, 6)
                pts.append(v)
        pts = [v for v in pts if v >= lo_b]
    else:
        if all(v > 0 for v in grid) and grid[-2] > 0:
            ratio = grid[-1] / grid[-2]  # > 1
            v = grid[-1]
            for _ in range(_EXTEND_POINTS):
                v = round(v * ratio, 6)
                pts.append(v)
        else:
            step = grid[-1] - grid[-2]
            v = grid[-1]
            for _ in range(_EXTEND_POINTS):
                v = round(v + step, 6)
                pts.append(v)
        pts = [v for v in pts if v <= hi_b]
    return pts


def sweep_with_extension(
    data: dict,
    t0_ns: int,
    base_params: dict,
    spec: SweepSpec,
    grid: list[float],
    horizons: tuple[int, ...],
    stride: int,
) -> tuple[pd.DataFrame, dict, list[float]]:
    """グリッド sweep を実行し、最小が端なら spec.bounds 内で自動拡張して再同定する。

    「グリッド端 (範囲拡大推奨)」を人手で潰さなくて済むよう、端方向へ
    _EXTEND_POINTS 点ずつ最大 _MAX_EXTENSIONS 回外挿する (等比 or 等差)。
    bounds に到達してなお端の場合は at_edge のまま返す (物理的に意味のある
    範囲の端であることが summary の注で分かる)。
    """
    gt = None if spec.affects_gt else s5._prepare_gt(data, t0_ns, base_params)
    h_max = max(horizons)
    res = run_param_sweep(data, t0_ns, base_params, spec, grid, horizons, stride, gt=gt)
    ident = identify(res, spec, h_max)
    grid = sorted(grid)

    for _ in range(_MAX_EXTENSIONS):
        if not ident["at_edge"]:
            break
        low_edge = ident["grid_value"] == grid[0]
        new_pts = _extension_points(grid, low_edge, spec.bounds)
        new_pts = [v for v in new_pts if v not in grid]
        if not new_pts:
            break  # bounds に到達
        print(f"  [端拡張] {spec.name}: {'下' if low_edge else '上'}方向へ {new_pts} を追加")
        res_ext = run_param_sweep(
            data, t0_ns, base_params, spec, new_pts, horizons, stride, gt=gt
        )
        res = pd.concat([res, res_ext], ignore_index=True)
        grid = sorted(grid + new_pts)
        ident = identify(res, spec, h_max)

    return res, ident, grid


def run_pair_sweep(
    data: dict,
    t0_ns: int,
    base_params: dict,
    name_a: str,
    name_b: str,
    horizons: tuple[int, ...],
    stride: int,
) -> pd.DataFrame:
    """2 パラメータの直積グリッドを sweep し最大 horizon の RMSE を集計する。"""
    spec_a, spec_b = _spec_by_name(name_a), _spec_by_name(name_b)
    grid_a = _pair_grid(name_a, base_params)
    grid_b = _pair_grid(name_b, base_params)
    h_max = max(horizons)
    affects_gt = spec_a.affects_gt or spec_b.affects_gt
    gt = None if affects_gt else s5._prepare_gt(data, t0_ns, base_params)

    rows: list[dict] = []
    for vb in grid_b:
        for va in grid_a:
            params = dict(base_params)
            params[name_a] = va
            params[name_b] = vb
            rmse = _rollout_rmse(data, t0_ns, params, horizons, stride, gt)
            rows.append({name_a: va, name_b: vb, **_rmse_row(rmse[h_max])})
        print(f"  {name_b}={vb:g}: {name_a} {len(grid_a)} 点完了")
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# 実機ログ根拠プロット (evidence)
# ---------------------------------------------------------------------------
# 各パラメータについて、sweep (モデル経由の同定) と独立に実機ログから直接観察できる
# 物理関係を描く。sweep 同定値とのクロスチェックに使う。
# ev: main() が _prepare_gt から構築する dict
#   t / vx / wz / steer (実測ステア) / ax / steer_des / accel_des (cmd タイムスタンプ上) / wb


def _fit_line(x: np.ndarray, y: np.ndarray) -> np.ndarray | None:
    """有効点で 1 次フィット (slope, intercept)。点数不足なら None。"""
    valid = np.isfinite(x) & np.isfinite(y)
    if valid.sum() < 10:
        return None
    return np.polyfit(x[valid], y[valid], 1)


def _ident_value(ident: dict) -> float:
    v = ident["parabolic_value"]
    return v if v is not None else ident["grid_value"]


_LS_DASH = {"-": "solid", "--": "dash", "-.": "dashdot", ":": "dot"}


def _etitle(title: str, source: str) -> str:
    """根拠パネルのタイトル（本文 + グレー小字のデータソース注）を plotly 用に組む。"""
    return f"{title.replace(chr(10), '<br>')}<br><sub>{source}</sub>"


def _scatter_fit_spec(
    x, y, xlabel, ylabel, title, source,
    ident_slope: float | None = None, ident_label: str = "",
    ref_slope: float | None = None, ref_label: str = "",
) -> tuple[dict, np.ndarray | None]:
    """散布 + 実機 1 次フィット + (任意) 同定/参照の傾き線の panel-spec を返す。fit 係数も返す。"""
    xs = np.linspace(np.nanmin(x), np.nanmax(x), 50)
    coef = _fit_line(x, y)
    traces = [go.Scatter(x=x, y=y, mode="markers", showlegend=False, hoverinfo="skip",
                         marker=dict(size=3, color="#4472C4", opacity=0.3))]
    if coef is not None:
        traces.append(go.Scatter(x=xs, y=np.polyval(coef, xs), mode="lines",
                                 name=f"実機 fit: 傾き={coef[0]:.4g}", line=dict(color="black", width=2)))
    if ident_slope is not None:
        traces.append(go.Scatter(x=xs, y=ident_slope * xs, mode="lines", name=ident_label,
                                 line=dict(color="red", width=1.5, dash="dash")))
    if ref_slope is not None:
        traces.append(go.Scatter(x=xs, y=ref_slope * xs, mode="lines", name=ref_label,
                                 line=dict(color="gray", width=1.5, dash="dot")))
    spec = {"title": _etitle(title, source), "xlabel": xlabel, "ylabel": ylabel,
            "traces": traces, "hlines": [(0, "black")]}
    return spec, coef


def _lag_corr_spec(ev: dict, u: np.ndarray, y: np.ndarray, title: str, source: str,
                   marks: list[tuple[float, str, str, str]] = (),
                   derivative: bool = True, max_lag: float = 0.6) -> tuple[dict, float]:
    """指令/実測の相互相関 vs ラグの panel-spec を返す。ピーク位置 [s] も返す。"""
    t = ev["t"]
    dt = float(np.median(np.diff(t)))
    n_lag = max(1, int(max_lag / dt))
    a0 = np.gradient(u, t) if derivative else np.asarray(u, dtype=float)
    b0 = np.gradient(y, t) if derivative else np.asarray(y, dtype=float)
    a0 = a0 - np.nanmean(a0)
    b0 = b0 - np.nanmean(b0)
    lags, corrs = [], []
    for k in range(n_lag + 1):
        a = a0[: len(a0) - k] if k else a0
        b = b0[k:]
        corrs.append(np.corrcoef(a, b)[0, 1] if len(a) > 10 else np.nan)
        lags.append(k * dt)
    lags = np.asarray(lags)
    corrs = np.asarray(corrs)
    peak = float(lags[int(np.nanargmax(corrs))])
    spec = {
        "title": _etitle(title, source),
        "xlabel": "ラグ τ [s] (指令を τ 遅らせたときの相関)",
        "ylabel": "相互相関 (微分同士)" if derivative else "相互相関 (信号同士)",
        "traces": [go.Scatter(x=lags, y=corrs, mode="lines+markers", showlegend=False,
                              line=dict(color="#4472C4"), marker=dict(size=4))],
        "vlines": [(peak, "black", "solid", f"相関ピーク τ={peak:.3f}s")]
        + [(v, c, _LS_DASH.get(ls, "dash"), lbl) for v, c, ls, lbl in marks],
    }
    return spec, peak


def _ev_k_us(ev, ident, base_value) -> dict:
    """k_us: tanδ − L·wz/vx vs 横加速度 wz·vx。モデル定義の逆算で傾き = k_us。"""
    mask = ev["vx"] > 2.0
    vx, wz, steer = ev["vx"][mask], ev["wz"][mask], ev["steer"][mask]
    x = wz * vx
    y = np.tan(steer) - ev["wb"] * wz / vx
    spec, coef = _scatter_fit_spec(
        x, y, "横加速度 wz·vx [m/s²]", "tan(δ実測) − L·wz/vx [rad]",
        "実機ログ根拠: understeer 勾配 (傾き = k_us)",
        "実測: steering_status × kinematic_state (vx>2)  ω=vx·tanδ/(L+k_us·vx²) の逆算",
        ident_slope=_ident_value(ident), ident_label=f"sweep 同定 k_us={_ident_value(ident):.4g}",
        ref_slope=base_value, ref_label=f"仕様値 {base_value:.4g}" if base_value is not None else "",
    )
    return {**spec, "value": float(coef[0]) if coef is not None else None,
            "desc": "understeer 勾配 fit (横加速度 vs ステア残差)"}


def _ev_steer_scaling(ev, ident, base_value) -> dict:
    """scaling: 指令 vs 実測ステア。傾き = 実効スケーリング (遅れの影響は含む)。"""
    mask = ev["vx"] > 1.0
    x = np.degrees(ev["steer_des"][mask])
    y = np.degrees(ev["steer"][mask])
    spec, coef = _scatter_fit_spec(
        x, y, "指令ステア角 [deg]", "実測ステア角 [deg]",
        "実機ログ根拠: 指令 vs 実測ステア (傾き = scaling)",
        "実測: steering_status  指令: control_cmd (vx>1; 応答遅れ分の散らばりを含む)",
        ident_slope=_ident_value(ident), ident_label=f"sweep 同定 scaling={_ident_value(ident):.4g}",
        ref_slope=base_value, ref_label=f"仕様値 {base_value:.4g}",
    )
    return {**spec, "value": float(coef[0]) if coef is not None else None,
            "desc": "指令→実測ステア fit 傾き"}


def _ev_steer_tc(ev, ident, base_value) -> dict:
    """steer_tc: 1 次遅れ dδ/dt=(u−δ)/tc → (u−δ) vs dδ/dt の傾き = 1/tc。"""
    t = ev["t"]
    x = np.degrees(ev["steer_des"] - ev["steer"])
    y = np.degrees(np.gradient(ev["steer"], t))
    spec, coef = _scatter_fit_spec(
        x, y, "指令 − 実測 ステア [deg]", "d(実測ステア)/dt [deg/s]",
        "実機ログ根拠: 1次遅れ応答 (傾き = 1/tc)",
        "実測: steering_status  指令: control_cmd (むだ時間は未補正)",
    )
    tc_fit = None
    if coef is not None and coef[0] > 1e-6:
        tc_fit = 1.0 / coef[0]
        spec["traces"].append(go.Scatter(
            x=[None], y=[None], mode="lines",
            name=f"→ tc_fit ≈ {tc_fit:.3f}s (sweep 同定≈{_ident_value(ident):.3f}s / 仕様 {base_value:.3f}s)",
        ))
    return {**spec, "value": tc_fit, "desc": "1次遅れ勾配 fit (Δsteer vs dδ/dt)"}


def _first_order_filter(u: np.ndarray, t: np.ndarray, tc: float) -> np.ndarray:
    """一様 dt 近似の離散 1 次遅れフィルタ y' = (u − y)/tc。"""
    dt = float(np.median(np.diff(t)))
    alpha = dt / (tc + dt)
    out = np.empty_like(u, dtype=float)
    out[0] = u[0]
    for i in range(1, len(u)):
        out[i] = out[i - 1] + alpha * (u[i - 1] - out[i - 1])
    return out


def _ev_steer_tc_corr(ev, ident, base_value) -> dict:
    """steer_tc 第2根拠: 相関係数解析（1次遅れフィルタ tc を走査して相関最大の tc を探す）。"""
    t, u, y = ev["t"], ev["steer_des"], ev["steer"]
    dt = float(np.median(np.diff(t)))
    delay_spec = float(ev["params"].get("steer_time_delay", 0.0))
    k = int(round(delay_spec / dt))
    u_d = np.concatenate([np.full(k, u[0]), u[: len(u) - k]]) if k > 0 else u

    def _highpass(x: np.ndarray, win_s: float = 1.0) -> np.ndarray:
        w = max(3, int(win_s / dt))
        return x - pd.Series(x).rolling(w, center=True, min_periods=1).mean().to_numpy()

    y_hp = _highpass(y)
    tcs = np.geomspace(0.02, 1.5, 30)
    corrs = np.asarray([
        float(np.corrcoef(_highpass(_first_order_filter(u_d, t, float(tc))), y_hp)[0, 1])
        for tc in tcs
    ])
    tc_best = float(tcs[int(np.nanargmax(corrs))])
    vlines = [(tc_best, "black", "solid", f"相関最大 tc≈{tc_best:.3f}s"),
              (_ident_value(ident), "red", "dash", f"sweep 同定≈{_ident_value(ident):.3f}s")]
    if base_value is not None:
        vlines.append((base_value, "gray", "dot", f"仕様値 {base_value:.3f}s"))
    spec = {
        "title": _etitle(
            "実機ログ根拠: 相関係数解析 (1次遅れフィルタ tc を走査)\n"
            "sweep 同定との乖離 = tc が操舵以外の遅れを代理吸収している示唆",
            "実測: steering_status  指令: control_cmd (むだ時間=仕様値で遅延、移動平均1.0s除去済み)"),
        "xlabel": "tc 候補 [s] (log)", "ylabel": "corr( 1次遅れフィルタ済み指令, 実測 ) [HP 1.0s]",
        "log_x": True,
        "traces": [go.Scatter(x=tcs, y=corrs, mode="lines+markers", showlegend=False,
                              line=dict(color="#4472C4"), marker=dict(size=4))],
        "vlines": vlines,
    }
    return {**spec, "value": tc_best, "desc": "相関係数解析 (1次遅れフィルタ tc 走査の相関最大)"}


def _ev_steer_delay(ev, ident, base_value) -> dict:
    marks = [(_ident_value(ident), "red", "--", f"sweep 同定≈{_ident_value(ident):.3f}s")]
    if base_value is not None:
        marks.append((base_value, "gray", ":", f"仕様値 {base_value:.3f}s"))
    spec, peak = _lag_corr_spec(
        ev, ev["steer_des"], ev["steer"],
        "実機ログ根拠: 指令→実測ステアのラグ相関",
        "実測: steering_status  指令: control_cmd (微分同士の相互相関)", marks=marks,
    )
    return {**spec, "value": peak, "desc": "ラグ相関ピーク (≈むだ時間; tc 成分を一部含む)"}


def _ev_steer_bias(ev, ident, base_value) -> dict:
    """bias: 直進・低横加速度域の bicycle 残差ヒストグラム。平均 ≈ 実効バイアス。"""
    ay = ev["wz"] * ev["vx"]
    mask = (ev["vx"] > 2.0) & (np.abs(ay) < 0.3)
    resid = np.tan(ev["steer"][mask]) - ev["wb"] * ev["wz"][mask] / ev["vx"][mask]
    resid = resid[np.isfinite(resid)]
    mean = float(np.mean(resid)) if len(resid) else float("nan")
    vlines = [(mean, "black", "solid", f"実機平均={mean:.4g} rad"),
              (_ident_value(ident), "red", "dash", f"sweep 同定≈{_ident_value(ident):.4g}")]
    if base_value is not None:
        vlines.append((base_value, "gray", "dot", f"仕様値 {base_value:.4g}"))
    spec = {
        "title": _etitle("実機ログ根拠: 直進域ステア残差 (平均 ≈ bias)",
                         "実測: steering_status × kinematic_state"),
        "xlabel": "tan(δ実測) − L·wz/vx [rad] (直進域 |ay|<0.3, vx>2)", "ylabel": "頻度",
        "traces": [go.Histogram(x=resid, nbinsx=60, marker=dict(color="#4472C4"),
                                opacity=0.7, showlegend=False)],
        "vlines": vlines,
    }
    return {**spec, "value": mean if np.isfinite(mean) else None, "desc": "直進域ステア残差の平均"}


def _ev_steer_dead_band(ev, ident, base_value) -> dict:
    """dead_band: 小指令域の指令 vs 実測 (不感帯 = 中央の平坦域)。"""
    lim = 1.5  # [deg]
    x_all = np.degrees(ev["steer_des"])
    mask = (np.abs(x_all) < lim) & (ev["vx"] > 1.0)
    x = x_all[mask]
    y = np.degrees(ev["steer"][mask])
    xs = np.linspace(-lim, lim, 50)
    db_deg = np.degrees(abs(_ident_value(ident)))
    spec = {
        "title": _etitle("実機ログ根拠: 小指令域の応答 (平坦域 = 不感帯)",
                         "実測: steering_status  指令: control_cmd (vx>1)"),
        "xlabel": "指令ステア角 [deg] (±1.5° 拡大)", "ylabel": "実測ステア角 [deg]",
        "traces": [
            go.Scatter(x=x, y=y, mode="markers", showlegend=False, hoverinfo="skip",
                       marker=dict(size=4, color="#4472C4", opacity=0.4)),
            go.Scatter(x=xs, y=xs, mode="lines", name="y = x (不感帯なし)",
                       line=dict(color="gray", width=1.2, dash="dot")),
            go.Scatter(x=[None], y=[None], mode="lines",
                       name=f"sweep 同定 ±{abs(_ident_value(ident)):.4g} rad (±{db_deg:.2f}°)",
                       line=dict(color="red")),
        ],
        "vrects": [(-db_deg, db_deg, "red")],
    }
    return {**spec, "value": None, "desc": "小指令域の平坦域 (視覚確認; 数値推定なし)"}


def _ev_acc_tc(ev, ident, base_value) -> dict:
    t = ev["t"]
    x = ev["accel_des"] - ev["ax"]
    y = np.gradient(ev["ax"], t)
    spec, coef = _scatter_fit_spec(
        x, y, "指令 − 実測 加速度 [m/s²]", "d(実測加速度)/dt [m/s³]",
        "実機ログ根拠: 1次遅れ応答 (傾き = 1/tc)",
        "実測: localization/acceleration  指令: control_cmd (むだ時間は未補正)",
    )
    tc_fit = None
    if coef is not None and coef[0] > 1e-6:
        tc_fit = 1.0 / coef[0]
        spec["traces"].append(go.Scatter(
            x=[None], y=[None], mode="lines",
            name=f"→ tc_fit ≈ {tc_fit:.3f}s (sweep 同定≈{_ident_value(ident):.3f}s / 仕様 {base_value:.3f}s)",
        ))
    return {**spec, "value": tc_fit, "desc": "1次遅れ勾配 fit (Δacc vs da/dt; 散布大のため参考値)"}


def _ev_acc_tc_corr(ev, ident, base_value) -> dict:
    """acc_tc 第2根拠: 相関係数解析（_ev_steer_tc_corr の加速度版。1次遅れフィルタ tc を走査）。"""
    t, u, y = ev["t"], ev["accel_des"], ev["ax"]
    dt = float(np.median(np.diff(t)))
    delay_spec = float(ev["params"].get("acc_time_delay", 0.0))
    k = int(round(delay_spec / dt))
    u_d = np.concatenate([np.full(k, u[0]), u[: len(u) - k]]) if k > 0 else u

    def _highpass(x: np.ndarray, win_s: float = 1.0) -> np.ndarray:
        w = max(3, int(win_s / dt))
        return x - pd.Series(x).rolling(w, center=True, min_periods=1).mean().to_numpy()

    y_hp = _highpass(y)
    tcs = np.geomspace(0.02, 1.5, 30)
    corrs = np.asarray([
        float(np.corrcoef(_highpass(_first_order_filter(u_d, t, float(tc))), y_hp)[0, 1])
        for tc in tcs
    ])
    tc_best = float(tcs[int(np.nanargmax(corrs))])
    vlines = [(tc_best, "black", "solid", f"相関最大 tc≈{tc_best:.3f}s"),
              (_ident_value(ident), "red", "dash", f"sweep 同定≈{_ident_value(ident):.3f}s")]
    if base_value is not None:
        vlines.append((base_value, "gray", "dot", f"仕様値 {base_value:.3f}s"))
    spec = {
        "title": _etitle(
            "実機ログ根拠: 相関係数解析 (1次遅れフィルタ tc を走査)\n"
            "sweep 同定との乖離 = tc が加速以外の遅れを代理吸収している示唆",
            "実測: localization/acceleration  指令: control_cmd (むだ時間=仕様値で遅延、移動平均1.0s除去済み)"),
        "xlabel": "tc 候補 [s] (log)", "ylabel": "corr( 1次遅れフィルタ済み指令, 実測 ) [HP 1.0s]",
        "log_x": True,
        "traces": [go.Scatter(x=tcs, y=corrs, mode="lines+markers", showlegend=False,
                              line=dict(color="#4472C4"), marker=dict(size=4))],
        "vlines": vlines,
    }
    return {**spec, "value": tc_best, "desc": "相関係数解析 (1次遅れフィルタ tc 走査の相関最大)"}


def _ev_acc_delay(ev, ident, base_value) -> dict:
    marks = [(_ident_value(ident), "red", "--", f"sweep 同定≈{_ident_value(ident):.3f}s")]
    if base_value is not None:
        marks.append((base_value, "gray", ":", f"仕様値 {base_value:.3f}s"))
    spec, peak = _lag_corr_spec(
        ev, ev["accel_des"], ev["ax"],
        "実機ログ根拠: 指令→実測加速度のラグ相関",
        "実測: localization/acceleration  指令: control_cmd (微分同士の相互相関)", marks=marks,
    )
    return {**spec, "value": peak, "desc": "ラグ相関ピーク (≈むだ時間; tc 成分を一部含む)"}


def _ev_acc_scaling(ev, ident, base_value) -> dict:
    spec, coef = _scatter_fit_spec(
        ev["accel_des"], ev["ax"], "指令加速度 [m/s²]", "実測加速度 [m/s²]",
        "実機ログ根拠: 指令 vs 実測加速度 (傾き = scaling)",
        "実測: localization/acceleration  指令: control_cmd (応答遅れ分の散らばりを含む)",
        ident_slope=_ident_value(ident), ident_label=f"sweep 同定 scaling={_ident_value(ident):.4g}",
        ref_slope=base_value, ref_label=f"仕様値 {base_value:.4g}",
    )
    return {**spec, "value": float(coef[0]) if coef is not None else None,
            "desc": "指令→実測加速度 fit 傾き (応答遅れの散布を含む参考値)"}


# パラメータ名 → 根拠プロット関数 (ax, ev, ident, base_value) のリスト (パネル数分)
_EVIDENCE_PLOTS: dict[str, list] = {
    "k_us": [_ev_k_us],
    "debug_steer_scaling_factor": [_ev_steer_scaling],
    "steer_time_constant": [_ev_steer_tc, _ev_steer_tc_corr],
    "steer_time_delay": [_ev_steer_delay],
    "steer_bias": [_ev_steer_bias],
    "steer_dead_band": [_ev_steer_dead_band],
    "acc_time_constant": [_ev_acc_tc, _ev_acc_tc_corr],
    "acc_time_delay": [_ev_acc_delay],
    "debug_acc_scaling_factor": [_ev_acc_scaling],
}


def build_evidence_data(g: dict, base_params: dict) -> dict:
    """_prepare_gt の出力から根拠プロット用の実機系列を束ねる。"""
    return {
        "t": g["t_cmd"],
        "vx": g["gt_vx"],
        "wz": g["gt_wz"],
        "steer": g["gt_steer"],
        "ax": g["gt_ax"],
        "steer_des": g["df_cmd"]["steer_des"].to_numpy(dtype=float),
        "accel_des": g["df_cmd"]["accel_des"].to_numpy(dtype=float),
        "wb": float(base_params["wheelbase"]),
        "params": dict(base_params),  # 根拠プロットが他パラメータの仕様値を参照する用
    }


# ---------------------------------------------------------------------------
# プロット
# ---------------------------------------------------------------------------


def plot_sweep(
    res: pd.DataFrame,
    spec: SweepSpec,
    identified: dict,
    base_value: float | None,
    params: dict,
    out_path: Path,
    evidence: list | None = None,
) -> list[dict]:
    """sweep パネル (同定メトリクス / 副メトリクス) + 実機ログ根拠パネル群を 1 図に。

    evidence: 各根拠パネルの panel-spec dict（_ev_* が返す。value/desc も含む）のリスト。
    sweep はモデル経由の同定、根拠パネルは実機ログの直接観察 — 両者の一致が
    同定の信頼性のクロスチェックになる。ev_results（value/desc）を返す。
    """
    metric_label, _, _ = _METRIC_INFO[spec.metric]
    horizons = sorted(res["horizon"].unique())
    evidence = evidence or []
    sweep_metrics = [spec.metric] + [m for m in spec.secondary_metrics if m != spec.metric]
    ident = _ident_value(identified)
    colors = viridis_colors(len(horizons))

    panels: list[dict] = []
    for mi, metric in enumerate(sweep_metrics):
        m_label, m_unit, m_col = _METRIC_INFO[metric]
        traces = [
            go.Scatter(
                x=res[res["horizon"] == h].sort_values("value")["value"].to_numpy(),
                y=res[res["horizon"] == h].sort_values("value")[m_col].to_numpy(),
                mode="lines+markers", name=f"N={h}", legendgroup=f"N{h}", showlegend=(mi == 0),
                line=dict(color=colors[idx]), marker=dict(size=5),
            )
            for idx, h in enumerate(horizons)
        ]
        vlines = [(ident, "red", "dash", f"identified ≈{ident:.4g}")]
        if base_value is not None:
            vlines.append((base_value, "gray", "dot", f"仕様値 {base_value:.4g}"))
        title = f"{m_label} vs {spec.name}" + (" (同定メトリクス)" if metric == spec.metric else "")
        panels.append({
            "title": title, "xlabel": f"{spec.name} [{spec.unit}]", "ylabel": f"{m_label} [{m_unit}]",
            "traces": traces, "vlines": vlines,
        })
    panels.extend(evidence)  # 根拠パネル（panel-spec dict）をそのまま並置

    fig = build_fig_sweep(
        panels, params=params,
        title=f"{spec.label} ({spec.name}) スイープ同定 (free-running rollout, "
        f"同定 = N={identified['horizon']} の {metric_label} 最小化)",
    )
    write_fig_json(fig, out_path)
    return [{"value": e.get("value"), "desc": e.get("desc")} for e in evidence]


def plot_pair_sweep(
    res: pd.DataFrame,
    name_a: str,
    name_b: str,
    metric: str,
    h_max: int,
    base_params: dict,
    out_path: Path,
) -> None:
    """2D グリッドの RMSE ヒートマップ (最小点マーカー + セル注釈付き)。"""
    metric_label, metric_unit, metric_col = _METRIC_INFO[metric]
    grid_a = sorted(res[name_a].unique())
    grid_b = sorted(res[name_b].unique())
    mat = np.full((len(grid_b), len(grid_a)), np.nan)
    for _, row in res.iterrows():
        mat[grid_b.index(row[name_b]), grid_a.index(row[name_a])] = row[metric_col]
    min_ij = tuple(int(v) for v in np.unravel_index(np.nanargmin(mat), mat.shape))

    def _base_str(name: str) -> str:
        v = _spec_by_name(name).base_value(base_params)
        return f"{v:.4g}" if v is not None else "—"

    fig = build_fig_pair_sweep(
        mat, grid_a, grid_b, name_a, name_b, min_ij,
        metric_label=metric_label, metric_unit=metric_unit, h_max=h_max,
        base_str_a=_base_str(name_a), base_str_b=_base_str(name_b),
    )
    write_fig_json(fig, out_path)


# ---------------------------------------------------------------------------
# サマリ
# ---------------------------------------------------------------------------


def _rmse_at(res: pd.DataFrame, ident: dict, value: float | None) -> float | None:
    """最大 horizon の同定メトリクス曲線上で value 位置の RMSE を線形補間で返す。"""
    if value is None:
        return None
    at = res[res["horizon"] == ident["horizon"]].sort_values("value")
    return float(np.interp(value, at["value"].to_numpy(), at[ident["metric_col"]].to_numpy()))


def _improvement_pct(rec: dict) -> float | None:
    """仕様値 RMSE → 同定値 RMSE の改善率 [%] (仕様値が不明なら None)。"""
    rmse_spec = rec["rmse_at_spec"]
    if rmse_spec is None or rmse_spec <= 0:
        return None
    return (rmse_spec - rec["ident"]["grid_rmse"]) / rmse_spec * 100.0


# 同定メトリクス別の系統色 (オーバービュー図と summary の対応用)
_METRIC_COLORS = {
    "yaw": "#1f77b4", "lat": "#2ca02c", "long": "#ff7f0e", "pos": "#9467bd",
    "vx": "#d62728", "ax": "#8c564b",
}


def plot_sweep_overview(records: list[dict], h_max: int, params: dict, out_path: Path) -> None:
    """全パラメータの感度オーバービュー (2 パネル)。

    左: 改善率ランキング — 仕様値の RMSE に対し同定値でどれだけ下がるか [%]。
        大きいほど「そのパラメータを直すと効く」。色 = 同定メトリクス系統。
    右: 正規化 RMSE カーブ重ね描き — x はグリッド範囲を [0,1] に正規化、
        y は仕様値 RMSE 比。平坦 = 非感度、谷 = 同定可能、単調 = グリッド端。
        ○ = 仕様値位置 (y=1)、▼ = グリッド最小。
    """
    recs = [r for r in records if _improvement_pct(r) is not None]
    if not recs:
        return
    recs = sorted(recs, key=_improvement_pct)

    # 左: 改善率ランキング barh 用データ（改善率昇順 = 上が最大）
    bars: list[dict] = []
    for r in recs:
        ident = r["ident"]
        v = ident["parabolic_value"] if ident["parabolic_value"] is not None else ident["grid_value"]
        edge = " (端)" if ident["at_edge"] else ""
        bars.append({
            "name": r["spec"].name,
            "pct": _improvement_pct(r),
            "color": _METRIC_COLORS[r["spec"].metric],
            "text": f"同定≈{v:.4g}{edge}",
        })

    # 右: 正規化カーブ用データ（param ごとに [0,1] 正規化、tab10 で色分け）
    _TAB10 = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
              "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]
    curves: list[dict] = []
    for i, r in enumerate(reversed(recs)):  # 改善率の大きい順に手前へ
        res, ident = r["res"], r["ident"]
        at = res[res["horizon"] == h_max].sort_values("value")
        xs = at["value"].to_numpy()
        ys = at[ident["metric_col"]].to_numpy()
        span = xs.max() - xs.min()
        if span <= 0 or r["rmse_at_spec"] is None or r["rmse_at_spec"] <= 0:
            continue
        xn = (xs - xs.min()) / span
        yn = ys / r["rmse_at_spec"]
        mi = int(np.argmin(yn))
        curves.append({
            "name": r["spec"].name, "color": _TAB10[i % 10],
            "xn": xn, "yn": yn, "bx": float((r["base_value"] - xs.min()) / span),
            "min_x": float(xn[mi]), "min_y": float(yn[mi]),
        })

    fig = build_fig_sweep_overview(bars, curves, h_max=h_max)
    write_fig_json(fig, out_path)
    print(f"  Saved: {out_path}")


def write_summary(
    records: list[dict],
    horizons: tuple[int, ...],
    stride: int,
    out_path: Path,
) -> None:
    """全パラメータの同定結果を Markdown 表に集約する (改善率の大きい順)。"""
    lines = [
        "# param sweep summary\n",
        f"free-running rollout (horizons={list(horizons)}, stride={stride}) の最大 horizon "
        "終端誤差 RMSE を最小化するパラメータ値。グリッド最小 + 近傍 3 点の放物線フィット。"
        "**改善率の大きい順** (= 直すと効く順) に並べる。\n",
        "",
        metrics_description_md(),
        "",
        "| param | 仕様値 | 同定値 (グリッド / 放物線) | Δ(同定−仕様) | 同定メトリクス | "
        "RMSE@仕様値 → @同定 | 改善率 | 注 |",
        "|---|---:|---:|---:|---|---:|---:|---|",
    ]
    def _sort_key(r: dict) -> float:
        pct = _improvement_pct(r)
        return -pct if pct is not None else 1e9  # 改善率降順、不明 (仕様値なし) は末尾

    order = sorted(records, key=_sort_key)
    for rec in order:
        spec, ident, base_value = rec["spec"], rec["ident"], rec["base_value"]
        metric_label, metric_unit, _ = _METRIC_INFO[spec.metric]
        parab = ident["parabolic_value"]
        best = parab if parab is not None else ident["grid_value"]
        base_str = f"{base_value:.4g}" if base_value is not None else "—"
        parab_str = f"{parab:.4g}" if parab is not None else "—"
        delta_str = f"{best - base_value:+.4g}" if base_value is not None else "—"
        rmse_spec = rec["rmse_at_spec"]
        rmse_str = (
            f"{rmse_spec:.4f} → {ident['grid_rmse']:.4f} {metric_unit}"
            if rmse_spec is not None else f"— → {ident['grid_rmse']:.4f} {metric_unit}"
        )
        pct = _improvement_pct(rec)
        pct_str = f"{pct:.1f}%" if pct is not None else "—"
        note = "グリッド端 (自動拡張後も端 = bounds 限界)" if ident["at_edge"] else ""
        lines.append(
            f"| {spec.name} ({spec.label}) | {base_str} | "
            f"{ident['grid_value']:.4g} / {parab_str} | {delta_str} | "
            f"{metric_label}@N{ident['horizon']} | {rmse_str} | {pct_str} | {note} |"
        )
    lines += [
        "",
        "> 各パラメータは他を仕様値に固定した 1 次元 sweep (one-at-a-time)。改善率は同一メトリクス内"
        "での相対値であり、メトリクスが異なる行同士の直接比較には注意。パラメータ間の相関は "
        "2D ペア sweep (pair_*.svg) を、全体俯瞰は感度オーバービュー (_overview_sensitivity.svg) を参照。",
        "",
    ]
    lines += _build_discussion(records)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"  Saved: {out_path}")


def _judge_evidence(sweep_v: float, ev_v: float | None, spec_v: float | None) -> str:
    """sweep 同定値と実機ログ直接推定の整合判定 (仕様値からの偏差比 r = Δ直接/Δsweep)。"""
    if ev_v is None:
        return "—"
    ref = spec_v if spec_v is not None else 0.0
    d_s = sweep_v - ref
    d_e = ev_v - ref
    # sweep がほぼ仕様値なら偏差比 r が数値的に暴れるため先に判定する
    eps = 0.10 * max(abs(ref), abs(sweep_v), 1e-9)
    if abs(d_s) <= eps:
        return "sweep は仕様値近傍 (調整不要の示唆)"
    r = d_e / d_s
    if 0.5 <= r <= 2.0:
        return "**整合** — 物理パラメータの補正として妥当"
    if 0.25 <= r < 0.5 or 2.0 < r <= 4.0:
        return "弱い整合 — 方向は同じだが大きさが乖離"
    return "**乖離** — sweep 値はモデル外誤差の代理吸収の疑い (物理値は直接推定側を参照)"


def _build_discussion(records: list[dict]) -> list[str]:
    """sweep 同定値 vs 実機ログ直接推定の比較表と注意点を自動生成する。

    根拠パネル (evidence) が返す直接推定値はデータから計算した一次情報。
    rollout sweep の同定値が直接推定と乖離する場合、その値はモデル外の誤差
    (例: bicycle モデルに無いヨー応答遅れ) を代理吸収した「実効値」であり、
    物理パラメータの修正としてそのまま採用すべきでないことを表で明示する。
    """
    lines = [
        "## 考察: sweep 同定値 vs 実機ログ直接推定\n",
        "rollout sweep (モデル経由の最適化) と、根拠パネルの実機ログ直接推定 (モデル非経由) の比較。"
        "両者が整合するパラメータは物理的な補正として信頼でき、乖離するパラメータの sweep 値は"
        "「rollout 誤差を最小化する実効値」(他の未モデル化誤差の代理吸収) として扱うこと。\n",
        "",
        "| param | sweep 同定値 | 実機直接推定 | 推定方法 | 判定 |",
        "|---|---:|---:|---|---|",
    ]
    n_diverge = 0
    for rec in records:
        spec, ident = rec["spec"], rec["ident"]
        sweep_v = ident["parabolic_value"]
        if sweep_v is None:
            sweep_v = ident["grid_value"]
        for ev in rec.get("evidence") or []:
            ev_v = ev.get("value")
            judge = _judge_evidence(sweep_v, ev_v, rec["base_value"])
            if "乖離" in judge and "弱い" not in judge:
                n_diverge += 1
            ev_str = f"{ev_v:.4g}" if ev_v is not None else "—"
            lines.append(
                f"| {spec.name} | {sweep_v:.4g} | {ev_str} | {ev['desc']} | {judge} |"
            )
    notes = [
        "",
        "### 注意点 (自動生成)",
        "- 判定は仕様値からの偏差比 r = (直接推定 − 仕様値)/(sweep 同定値 − 仕様値) による"
        "目安 (0.5≤r≤2 で整合、sweep 偏差が仕様値の 10% 以下なら仕様値近傍)。"
        "最終判断は各 sweep 図の根拠パネルを参照。",
        "- ステア系パラメータの sweep 図にはステア RMSE パネルを併記している。"
        "同定メトリクス (yaw 等) が改善してもステア RMSE が悪化する場合、"
        "その同定値はステア応答以外の遅れの代理吸収である。",
    ]
    edge_params = [r["spec"].name for r in records if r["ident"]["at_edge"]]
    if edge_params:
        notes.append(
            f"- bounds 限界でも端のパラメータ: {', '.join(edge_params)} — "
            "物理的に意味のある範囲では最小に到達せず。モデル構造側の検討を推奨。"
        )
    if n_diverge:
        notes.append(
            f"- 「乖離」判定が {n_diverge} 件。これらを sim パラメータとして採用する場合は"
            "実効値である (物理値ではない) ことを認識した上で、結合探索 (相関考慮) で決めること。"
        )
    return lines + notes + [""]


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------


def _parse_grid_overrides(items: list[str]) -> dict[str, list[float]]:
    """--grid name=v1,v2,... (複数指定可) をパースする。"""
    out: dict[str, list[float]] = {}
    for item in items:
        name, _, values = item.partition("=")
        if not values:
            raise ValueError(f"--grid の形式が不正です: {item!r} (name=v1,v2,... を期待)")
        out[name.strip()] = sorted(float(v) for v in values.split(",") if v.strip())
    return out


def _parse_metric_overrides(items: list[str]) -> dict[str, str]:
    """--metric name=metric (複数指定可) をパースする。metric は _METRIC_INFO のキー。"""
    out: dict[str, str] = {}
    for item in items:
        name, _, metric = item.partition("=")
        metric = metric.strip()
        if metric not in _METRIC_INFO:
            raise ValueError(
                f"--metric の値が不正です: {item!r} "
                f"(name={'|'.join(_METRIC_INFO)} を期待)"
            )
        out[name.strip()] = metric
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="車両モデルパラメータ sweep 同定 (rollout)")
    add_common_cli_arguments(parser)
    all_names = [s.name for s in SWEEP_SPECS] + [f"pair_{a}_{b}" for a, b, _ in PAIR_SPECS]
    parser.add_argument(
        "--params",
        default="all",
        help=f"カンマ区切りの対象 (all または {','.join(all_names)} から選択)",
    )
    parser.add_argument(
        "--grid",
        action="append",
        default=[],
        metavar="NAME=V1,V2,...",
        help="グリッド上書き (複数指定可。例: --grid k_us=0,0.01,0.02)",
    )
    parser.add_argument(
        "--metric",
        action="append",
        default=[],
        metavar="NAME=METRIC",
        help=(
            "同定メトリクス上書き (複数指定可。METRIC は "
            f"{'|'.join(_METRIC_INFO)} から選択。例: --metric acc_time_constant=ax)"
        ),
    )
    parser.add_argument("--horizons", default="2,5,10,20,40", help="カンマ区切りの rollout horizon")
    parser.add_argument("--stride", type=int, default=5, help="開始点サンプリング間隔 [step]")
    args = parser.parse_args()

    cfg = build_runtime_config(args, default_base_dir=Path.cwd())
    s5.LITE_DIR = cfg.lite_dir

    # bag 解決は lib._io.resolve_lite_bag に集約済み（step5/8/9/10 と同方式）
    real_mcap = resolve_lite_bag(cfg.lite_dir, "real")
    if real_mcap is None:
        print(f"ERROR: real lite bag が見つかりません: {cfg.lite_dir}", file=sys.stderr)
        sys.exit(1)
    print(f"Loading: {real_mcap}")
    data = s5.load_real_bag(real_mcap)
    t0_ns = s5.find_autonomous_start(data)

    base_params = s5._build_params(cfg)  # wheelbase=cfg.wheelbase_sim, steer_bias/sub_dt は仕様値
    s5.SUB_DT = base_params["sub_dt"]  # run_rollout / _prepare_gt が参照する積分刻み
    horizons = tuple(int(x) for x in args.horizons.split(",") if x.strip())
    h_max = max(horizons)
    grid_overrides = _parse_grid_overrides(args.grid)
    metric_overrides = _parse_metric_overrides(args.metric)

    targets = (
        all_names if args.params.strip() == "all"
        else [t.strip() for t in args.params.split(",") if t.strip()]
    )
    unknown = [t for t in targets if t not in all_names]
    if unknown:
        print(f"ERROR: 未知の --params: {unknown} (候補: {all_names})", file=sys.stderr)
        sys.exit(2)

    out_dir = cfg.out_dir / "param_sweep"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"horizons={horizons}, stride={args.stride}, targets={targets}")

    # 実機ログ根拠プロット用の系列 (仕様値パラメータで GT を 1 回構築して共有)
    ev = build_evidence_data(s5._prepare_gt(data, t0_ns, base_params), base_params)

    # --- 1D sweep ---
    records: list[dict] = []
    for spec in SWEEP_SPECS:
        if spec.name not in targets:
            continue
        if spec.name in metric_overrides:
            # frozen dataclass のため replace で差し替え (identify/plot_sweep/summary に伝播)
            spec = replace(spec, metric=metric_overrides[spec.name])
        grid = grid_overrides.get(spec.name) or spec.grid(base_params)
        base_value = spec.base_value(base_params)
        print(f"\n=== sweep: {spec.name} ({spec.label})  grid={grid} ===")
        res, ident, grid = sweep_with_extension(
            data, t0_ns, base_params, spec, grid, horizons, args.stride
        )
        res.to_csv(out_dir / f"{spec.name}_sweep.csv", index=False)
        # 各根拠 _ev_* は panel-spec dict（value/desc 同梱）を返す
        evidence = [fn(ev, ident, base_value) for fn in _EVIDENCE_PLOTS.get(spec.name, [])]
        ev_results = plot_sweep(res, spec, ident, base_value, base_params,
                                out_dir / f"{spec.name}_sweep", evidence=evidence)
        records.append({
            "spec": spec,
            "res": res,
            "ident": ident,
            "base_value": base_value,
            "rmse_at_spec": _rmse_at(res, ident, base_value),
            "evidence": ev_results,
        })

        metric_label, _, _ = _METRIC_INFO[spec.metric]
        parab = ident["parabolic_value"]
        print(f"  同定 ({metric_label} 最小化 @ N={h_max}): "
              f"グリッド={ident['grid_value']:.4g}"
              + (f", 放物線≈{parab:.4g}" if parab is not None else " (放物線: 端のため範囲外)"))
        if ident["at_edge"]:
            print(f"  [WARN] 自動拡張後も最小がグリッド端 ({spec.name}={ident['grid_value']:.4g})。"
                  "bounds の端 (物理的に意味のある範囲の限界) に到達。")

    # --- 2D pair sweep ---
    for name_a, name_b, metric in PAIR_SPECS:
        key = f"pair_{name_a}_{name_b}"
        if key not in targets:
            continue
        print(f"\n=== 2D sweep: {name_a} × {name_b} ===")
        res = run_pair_sweep(data, t0_ns, base_params, name_a, name_b, horizons, args.stride)
        res.to_csv(out_dir / f"{key}.csv", index=False)
        plot_pair_sweep(res, name_a, name_b, metric, h_max, base_params, out_dir / f"{key}.svg")

    if records:
        # ファイル名先頭の "_" は意図的: step11 はパス昇順で図を並べるため、
        # オーバービューがセクション内の先頭 (個別 sweep 図より前) に来る。
        plot_sweep_overview(records, h_max, base_params, out_dir / "_overview_sensitivity.svg")
        write_summary(records, horizons, args.stride, out_dir / "param_sweep_summary.md")

    print(f"\n完了。出力先: {out_dir}")


if __name__ == "__main__":
    main()
