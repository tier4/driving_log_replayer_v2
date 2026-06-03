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

縦方向の brake_time_constant は減速・ギア拘束を含む別モデルでの発進フィットが
必要なため Stage 9 (step9_identify_brake) が担当する。

evaluator_node が Stage 6 の後に env (BEST_MODEL_BASE_DIR) のみで実行する (追加設定不要)。
手動実行・対象/グリッド変更も可能:
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.step7_sweep_params \
        --base-dir <out_dir> \
        [--params k_us,pair_k_us_steer_time_constant] \
        [--grid k_us=0,0.01,0.02 --grid steer_time_constant=0.1,0.2,0.4] \
        [--horizons 2,5,10,20] [--stride 5]

出力: <base-dir>/comparison/param_sweep/
    <name>_sweep.{csv,svg} (各パラメータ), pair_<a>_<b>.{csv,svg} (2D),
    param_sweep_summary.md (同定値一覧)
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from . import step5_analyze_nstep as s5
from .lib._io import resolve_lite_bag
from .lib._nstep_common import parabolic_min, rmse_by_horizon
from .lib._params_utils import add_params_annotation, setup_jp_font
from .lib._runtime_config import add_common_cli_arguments, build_runtime_config

setup_jp_font()

_MODEL_TYPE = "delay_steer_acc_geared_wo_fall_guard"  # 全 dynamics パラメータを持つ対応モデル

# 同定メトリクス (rmse_by_horizon のキー) → (表示名, 単位, res 列名)
_METRIC_INFO = {
    "yaw": ("yaw RMSE", "deg", "yaw_rmse_deg"),
    "lat": ("横方向 RMSE", "cm", "lat_rmse_cm"),
    "long": ("縦方向 RMSE", "cm", "long_rmse_cm"),
    "pos": ("位置 RMSE", "cm", "pos_rmse_cm"),
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
    ),
    SweepSpec(
        "steer_time_constant", "ステア時定数", "s", "yaw",
        grid_rel=(0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0),
    ),
    SweepSpec(
        "steer_time_delay", "ステアむだ時間", "s", "yaw",
        grid_rel=(0.0, 0.5, 1.0, 1.5, 2.0, 3.0),
        affects_gt=True,  # delay queue 長 → 過去 cmd 窓の拡張幅が変わる
    ),
    SweepSpec(
        "steer_bias", "ステアバイアス", "rad", "lat",
        grid_abs=(-0.01, -0.005, -0.002, -0.001, 0.0, 0.0005, 0.001, 0.002, 0.005, 0.01),
    ),
    SweepSpec(
        "steer_dead_band", "ステア不感帯", "rad", "lat",
        grid_abs=(0.0, 0.001, 0.002, 0.005, 0.01, 0.02),
    ),
    SweepSpec(
        "debug_steer_scaling_factor", "ステアスケーリング", "-", "yaw",
        grid_abs=(0.80, 0.85, 0.90, 0.95, 1.0, 1.05, 1.10, 1.15, 1.20),
        default=1.0,
    ),
    # --- 縦方向系 (rollout は加減速の全区間を含むため発進フィットより拘束が広い) ---
    SweepSpec(
        "acc_time_constant", "加速度時定数", "s", "long",
        grid_rel=(0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0),
    ),
    SweepSpec(
        "acc_time_delay", "加速度むだ時間", "s", "long",
        grid_rel=(0.0, 0.5, 1.0, 1.5, 2.0, 3.0),
        affects_gt=True,
    ),
    SweepSpec(
        "debug_acc_scaling_factor", "加速度スケーリング", "-", "long",
        grid_abs=(0.80, 0.85, 0.90, 0.95, 1.0, 1.05, 1.10, 1.15, 1.20),
        default=1.0,
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
    }


def run_param_sweep(
    data: dict,
    t0_ns: int,
    base_params: dict,
    spec: SweepSpec,
    grid: list[float],
    horizons: tuple[int, ...],
    stride: int,
) -> pd.DataFrame:
    """spec のグリッドを sweep し horizon 別 RMSE を集計した DataFrame を返す。"""
    # GT がパラメータ非依存なら 1 回だけ計算して共有 (run_rollout docstring 参照)
    gt = None if spec.affects_gt else s5._prepare_gt(data, t0_ns, base_params)
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


def _scatter_fit(
    ax, x, y, xlabel, ylabel, title, source,
    ident_slope: float | None = None,
    ident_label: str = "",
    ref_slope: float | None = None,
    ref_label: str = "",
) -> None:
    """散布 + 実機 1 次フィット + (任意) 同定値/参照の傾き線。"""
    ax.scatter(x, y, s=3, alpha=0.3, color="#4472C4", rasterized=True)
    xs = np.linspace(np.nanmin(x), np.nanmax(x), 50)
    coef = _fit_line(x, y)
    if coef is not None:
        ax.plot(xs, np.polyval(coef, xs), "k-", lw=2,
                label=f"実機 fit: 傾き={coef[0]:.4g}")
    if ident_slope is not None:
        ax.plot(xs, ident_slope * xs, "r--", lw=1.5, label=ident_label)
    if ref_slope is not None:
        ax.plot(xs, ref_slope * xs, color="gray", ls=":", lw=1.5, label=ref_label)
    ax.axhline(0, color="black", lw=0.6)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    _set_evidence_title(ax, title, source)
    ax.grid(True, lw=0.5, alpha=0.6)
    ax.legend(fontsize=8)


def _set_evidence_title(ax, title: str, source: str) -> None:
    ax.set_title(title, fontsize=10, pad=16)
    ax.text(0.5, 1.0, source, transform=ax.transAxes, fontsize=6.5,
            ha="center", va="bottom", color="#888888", clip_on=False)


def _lag_corr_plot(ax, ev: dict, u: np.ndarray, y: np.ndarray, title: str, source: str,
                   marks: list[tuple[float, str, str, str]] = (),
                   derivative: bool = True, max_lag: float = 0.6) -> float:
    """指令/実測の相互相関 vs ラグを描き、ピーク位置 [s] を返す。

    derivative=True: 微分同士の相関 — ピーク ≈ 純むだ時間 (波形の立ち上がり整合)。
    derivative=False: 信号同士の相関 — 1 次遅れの平滑化も含むため
                      ピーク ≈ むだ時間 + 時定数相当の実効遅れ (入力波形依存の目安)。
    marks: 縦線 (value, color, linestyle, label) のリスト。
    """
    t = ev["t"]
    dt = float(np.median(np.diff(t)))
    n_lag = max(1, int(max_lag / dt))
    if derivative:
        a0 = np.gradient(u, t)
        b0 = np.gradient(y, t)
    else:
        a0 = np.asarray(u, dtype=float)
        b0 = np.asarray(y, dtype=float)
    a0 = a0 - np.nanmean(a0)
    b0 = b0 - np.nanmean(b0)
    lags, corrs = [], []
    for k in range(n_lag + 1):
        a = a0[: len(a0) - k] if k else a0
        b = b0[k:]
        c = np.corrcoef(a, b)[0, 1] if len(a) > 10 else np.nan
        lags.append(k * dt)
        corrs.append(c)
    lags = np.asarray(lags)
    corrs = np.asarray(corrs)
    ax.plot(lags, corrs, "o-", color="#4472C4", ms=3)
    peak = float(lags[int(np.nanargmax(corrs))])
    ax.axvline(peak, color="black", lw=1.5, label=f"相関ピーク τ={peak:.3f}s")
    for value, color, ls, label in marks:
        ax.axvline(value, color=color, ls=ls, lw=1.2, label=label)
    ax.set_xlabel("ラグ τ [s] (指令を τ 遅らせたときの相関)")
    ax.set_ylabel("相互相関 (微分同士)" if derivative else "相互相関 (信号同士)")
    _set_evidence_title(ax, title, source)
    ax.grid(True, lw=0.5, alpha=0.6)
    ax.legend(fontsize=8)
    return peak


def _ev_k_us(ax, ev, ident, base_value) -> None:
    """k_us: tanδ − L·wz/vx vs 横加速度 wz·vx。モデル定義の逆算で傾き = k_us。"""
    mask = ev["vx"] > 2.0
    vx, wz, steer = ev["vx"][mask], ev["wz"][mask], ev["steer"][mask]
    x = wz * vx  # 横加速度 ay [m/s²]
    y = np.tan(steer) - ev["wb"] * wz / vx  # [rad]
    _scatter_fit(
        ax, x, y,
        "横加速度 wz·vx [m/s²]", "tan(δ実測) − L·wz/vx [rad]",
        "実機ログ根拠: understeer 勾配 (傾き = k_us)",
        "実測: steering_status × kinematic_state (vx>2)  ω=vx·tanδ/(L+k_us·vx²) の逆算",
        ident_slope=_ident_value(ident), ident_label=f"sweep 同定 k_us={_ident_value(ident):.4g}",
        ref_slope=base_value, ref_label=f"仕様値 {base_value:.4g}" if base_value is not None else "",
    )


def _ev_steer_scaling(ax, ev, ident, base_value) -> None:
    """scaling: 指令 vs 実測ステア。傾き = 実効スケーリング (遅れの影響は含む)。"""
    mask = ev["vx"] > 1.0
    x = np.degrees(ev["steer_des"][mask])
    y = np.degrees(ev["steer"][mask])
    _scatter_fit(
        ax, x, y,
        "指令ステア角 [deg]", "実測ステア角 [deg]",
        "実機ログ根拠: 指令 vs 実測ステア (傾き = scaling)",
        "実測: steering_status  指令: control_cmd (vx>1; 応答遅れ分の散らばりを含む)",
        ident_slope=_ident_value(ident),
        ident_label=f"sweep 同定 scaling={_ident_value(ident):.4g}",
        ref_slope=base_value, ref_label=f"仕様値 {base_value:.4g}",
    )


def _ev_steer_tc(ax, ev, ident, base_value) -> None:
    """steer_tc: 1 次遅れ dδ/dt=(u−δ)/tc → (u−δ) vs dδ/dt の傾き = 1/tc。"""
    t = ev["t"]
    x = np.degrees(ev["steer_des"] - ev["steer"])
    y = np.degrees(np.gradient(ev["steer"], t))
    _scatter_fit(
        ax, x, y,
        "指令 − 実測 ステア [deg]", "d(実測ステア)/dt [deg/s]",
        "実機ログ根拠: 1次遅れ応答 (傾き = 1/tc)",
        "実測: steering_status  指令: control_cmd (むだ時間は未補正)",
    )
    coef = _fit_line(x, y)
    if coef is not None and coef[0] > 1e-6:
        ax.plot([], [], " ", label=f"→ tc_fit ≈ {1.0 / coef[0]:.3f}s "
                f"(sweep 同定≈{_ident_value(ident):.3f}s / 仕様 {base_value:.3f}s)")
        ax.legend(fontsize=8)


def _first_order_filter(u: np.ndarray, t: np.ndarray, tc: float) -> np.ndarray:
    """一様 dt 近似の離散 1 次遅れフィルタ y' = (u − y)/tc。"""
    dt = float(np.median(np.diff(t)))
    alpha = dt / (tc + dt)
    out = np.empty_like(u, dtype=float)
    out[0] = u[0]
    for i in range(1, len(u)):
        out[i] = out[i - 1] + alpha * (u[i - 1] - out[i - 1])
    return out


def _ev_steer_tc_corr(ax, ev, ident, base_value) -> None:
    """steer_tc 第2根拠: 相関係数解析。

    tc 候補ごとに「むだ時間(仕様)で遅延 + 1 次遅れフィルタした指令」を作り、実測ステアとの
    相関係数を計算する (低周波の操舵波形は tc に非感度なため、移動平均 1.0s を除去した
    ハイパス成分同士で相関を取る)。相関最大の tc が「指令→実測ステア」の直接追従としての
    実効時定数。sweep 同定値との乖離は、sweep の tc が操舵応答以外の遅れ (ヨー応答等) を
    代理吸収していることを示唆する。
    """
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

    ax.semilogx(tcs, corrs, "o-", color="#4472C4", ms=3)
    tc_best = float(tcs[int(np.nanargmax(corrs))])
    ax.axvline(tc_best, color="black", lw=1.5, label=f"相関最大 tc≈{tc_best:.3f}s")
    ax.axvline(_ident_value(ident), color="red", ls="--", lw=1.2,
               label=f"sweep 同定≈{_ident_value(ident):.3f}s")
    if base_value is not None:
        ax.axvline(base_value, color="gray", ls=":", lw=1.2, label=f"仕様値 {base_value:.3f}s")
    ax.set_xlabel("tc 候補 [s] (log)")
    ax.set_ylabel("corr( 1次遅れフィルタ済み指令, 実測 ) [HP 1.0s]")
    _set_evidence_title(
        ax, "実機ログ根拠: 相関係数解析 (1次遅れフィルタ tc を走査)\n"
            "sweep 同定との乖離 = tc が操舵以外の遅れを代理吸収している示唆",
        "実測: steering_status  指令: control_cmd (むだ時間=仕様値で遅延、移動平均1.0s除去済み)",
    )
    ax.grid(True, lw=0.5, alpha=0.6, which="both")
    ax.legend(fontsize=8)


def _ev_steer_delay(ax, ev, ident, base_value) -> None:
    marks = [(_ident_value(ident), "red", "--", f"sweep 同定≈{_ident_value(ident):.3f}s")]
    if base_value is not None:
        marks.append((base_value, "gray", ":", f"仕様値 {base_value:.3f}s"))
    _lag_corr_plot(
        ax, ev, ev["steer_des"], ev["steer"],
        "実機ログ根拠: 指令→実測ステアのラグ相関",
        "実測: steering_status  指令: control_cmd (微分同士の相互相関)",
        marks=marks,
    )


def _ev_steer_bias(ax, ev, ident, base_value) -> None:
    """bias: 直進・低横加速度域の bicycle 残差ヒストグラム。平均 ≈ 実効バイアス。"""
    ay = ev["wz"] * ev["vx"]
    mask = (ev["vx"] > 2.0) & (np.abs(ay) < 0.3)
    resid = np.tan(ev["steer"][mask]) - ev["wb"] * ev["wz"][mask] / ev["vx"][mask]
    resid = resid[np.isfinite(resid)]
    ax.hist(resid, bins=60, color="#4472C4", alpha=0.7)
    mean = float(np.mean(resid)) if len(resid) else float("nan")
    ax.axvline(mean, color="black", lw=2, label=f"実機平均={mean:.4g} rad")
    ax.axvline(_ident_value(ident), color="red", ls="--", lw=1.5,
               label=f"sweep 同定≈{_ident_value(ident):.4g}")
    if base_value is not None:
        ax.axvline(base_value, color="gray", ls=":", lw=1.5, label=f"仕様値 {base_value:.4g}")
    ax.set_xlabel("tan(δ実測) − L·wz/vx [rad] (直進域 |ay|<0.3, vx>2)")
    ax.set_ylabel("頻度")
    _set_evidence_title(ax, "実機ログ根拠: 直進域ステア残差 (平均 ≈ bias)",
                        "実測: steering_status × kinematic_state")
    ax.grid(True, lw=0.5, alpha=0.6)
    ax.legend(fontsize=8)


def _ev_steer_dead_band(ax, ev, ident, base_value) -> None:
    """dead_band: 小指令域の指令 vs 実測 (不感帯 = 中央の平坦域)。"""
    lim = 1.5  # [deg]
    x_all = np.degrees(ev["steer_des"])
    mask = (np.abs(x_all) < lim) & (ev["vx"] > 1.0)
    x = x_all[mask]
    y = np.degrees(ev["steer"][mask])
    ax.scatter(x, y, s=4, alpha=0.4, color="#4472C4", rasterized=True)
    xs = np.linspace(-lim, lim, 50)
    ax.plot(xs, xs, color="gray", ls=":", lw=1.2, label="y = x (不感帯なし)")
    db_deg = np.degrees(abs(_ident_value(ident)))  # 同定値 [rad] を度軸に変換
    ax.axvspan(-db_deg, db_deg, color="red", alpha=0.10,
               label=f"sweep 同定 ±{np.radians(db_deg):.4g} rad (±{db_deg:.2f}°)")
    ax.set_xlabel("指令ステア角 [deg] (±1.5° 拡大)")
    ax.set_ylabel("実測ステア角 [deg]")
    _set_evidence_title(ax, "実機ログ根拠: 小指令域の応答 (平坦域 = 不感帯)",
                        "実測: steering_status  指令: control_cmd (vx>1)")
    ax.grid(True, lw=0.5, alpha=0.6)
    ax.legend(fontsize=8)


def _ev_acc_tc(ax, ev, ident, base_value) -> None:
    t = ev["t"]
    x = ev["accel_des"] - ev["ax"]
    y = np.gradient(ev["ax"], t)
    _scatter_fit(
        ax, x, y,
        "指令 − 実測 加速度 [m/s²]", "d(実測加速度)/dt [m/s³]",
        "実機ログ根拠: 1次遅れ応答 (傾き = 1/tc)",
        "実測: localization/acceleration  指令: control_cmd (むだ時間は未補正)",
    )
    coef = _fit_line(x, y)
    if coef is not None and coef[0] > 1e-6:
        ax.plot([], [], " ", label=f"→ tc_fit ≈ {1.0 / coef[0]:.3f}s "
                f"(sweep 同定≈{_ident_value(ident):.3f}s / 仕様 {base_value:.3f}s)")
        ax.legend(fontsize=8)


def _ev_acc_delay(ax, ev, ident, base_value) -> None:
    marks = [(_ident_value(ident), "red", "--", f"sweep 同定≈{_ident_value(ident):.3f}s")]
    if base_value is not None:
        marks.append((base_value, "gray", ":", f"仕様値 {base_value:.3f}s"))
    _lag_corr_plot(
        ax, ev, ev["accel_des"], ev["ax"],
        "実機ログ根拠: 指令→実測加速度のラグ相関",
        "実測: localization/acceleration  指令: control_cmd (微分同士の相互相関)",
        marks=marks,
    )


def _ev_acc_scaling(ax, ev, ident, base_value) -> None:
    x = ev["accel_des"]
    y = ev["ax"]
    _scatter_fit(
        ax, x, y,
        "指令加速度 [m/s²]", "実測加速度 [m/s²]",
        "実機ログ根拠: 指令 vs 実測加速度 (傾き = scaling)",
        "実測: localization/acceleration  指令: control_cmd (応答遅れ分の散らばりを含む)",
        ident_slope=_ident_value(ident),
        ident_label=f"sweep 同定 scaling={_ident_value(ident):.4g}",
        ref_slope=base_value, ref_label=f"仕様値 {base_value:.4g}",
    )


# パラメータ名 → 根拠プロット関数 (ax, ev, ident, base_value) のリスト (パネル数分)
_EVIDENCE_PLOTS: dict[str, list] = {
    "k_us": [_ev_k_us],
    "debug_steer_scaling_factor": [_ev_steer_scaling],
    "steer_time_constant": [_ev_steer_tc, _ev_steer_tc_corr],
    "steer_time_delay": [_ev_steer_delay],
    "steer_bias": [_ev_steer_bias],
    "steer_dead_band": [_ev_steer_dead_band],
    "acc_time_constant": [_ev_acc_tc],
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
) -> None:
    """sweep 2 パネル (同定メトリクス / 位置 RMSE) + 実機ログ根拠パネル群。

    evidence: 第 3 パネル以降を描く callable (ax を受け取る) のリスト。None/空なら 2 パネル。
    sweep はモデル経由の同定、根拠パネルは実機ログの直接観察 — 両者の一致が
    同定の信頼性のクロスチェックになる。
    """
    metric_label, metric_unit, metric_col = _METRIC_INFO[spec.metric]
    horizons = sorted(res["horizon"].unique())
    evidence = evidence or []
    n_panels = 2 + len(evidence)
    fig, axes = plt.subplots(1, n_panels, figsize=(6.5 * n_panels, 5))
    ax1, ax2 = axes[0], axes[1]
    cmap = plt.get_cmap("viridis")
    for idx, horizon in enumerate(horizons):
        sub = res[res["horizon"] == horizon].sort_values("value")
        color = cmap(idx / max(1, len(horizons) - 1))
        v = sub["value"].to_numpy()
        ax1.plot(v, sub[metric_col].to_numpy(), "o-", color=color, label=f"N={horizon}")
        ax2.plot(v, sub["pos_rmse_cm"].to_numpy(), "o-", color=color, label=f"N={horizon}")
    ident = identified["parabolic_value"]
    if ident is None:
        ident = identified["grid_value"]
    for ax, ylab, title in (
        (ax1, f"{metric_label} [{metric_unit}]", f"{metric_label} vs {spec.name}"),
        (ax2, "位置 RMSE [cm]", f"位置 RMSE vs {spec.name}"),
    ):
        ax.axvline(ident, color="red", ls="--", lw=1.2, label=f"identified ≈{ident:.4g}")
        if base_value is not None:
            ax.axvline(base_value, color="gray", ls=":", lw=1.2,
                       label=f"仕様値 {base_value:.4g}")
        ax.set_xlabel(f"{spec.name} [{spec.unit}]")
        ax.set_ylabel(ylab)
        ax.set_title(title, fontsize=10)
        ax.grid(True, lw=0.5, alpha=0.6)
        ax.legend(fontsize=8)
    for i, ev_fn in enumerate(evidence):
        ev_fn(axes[2 + i])
    fig.suptitle(
        f"{spec.label} ({spec.name}) スイープ同定 (free-running rollout, "
        f"同定 = N={identified['horizon']} の {metric_label} 最小化)",
        fontsize=12,
    )
    add_params_annotation(fig, params)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


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

    fig, ax = plt.subplots(figsize=(8.5, 6))
    im = ax.imshow(mat, cmap="YlOrRd", aspect="auto", origin="lower")
    ax.set_xticks(range(len(grid_a)), [f"{v:g}" for v in grid_a])
    ax.set_yticks(range(len(grid_b)), [f"{v:g}" for v in grid_b])
    ax.set_xlabel(name_a)
    ax.set_ylabel(name_b)
    # セル注釈: 背景の濃淡で文字色を切り替える
    thresh = np.nanmin(mat) + (np.nanmax(mat) - np.nanmin(mat)) * 0.6
    for i in range(len(grid_b)):
        for j in range(len(grid_a)):
            if np.isfinite(mat[i, j]):
                ax.text(j, i, f"{mat[i, j]:.3f}", ha="center", va="center", fontsize=8,
                        color="white" if mat[i, j] > thresh else "black")
    mi, mj = np.unravel_index(np.nanargmin(mat), mat.shape)
    ax.plot(mj, mi, "o", ms=14, mfc="none", mec="blue", mew=2,
            label=f"最小 ({name_a}={grid_a[mj]:g}, {name_b}={grid_b[mi]:g})")
    ax.legend(fontsize=9, loc="upper right")
    fig.colorbar(im, ax=ax, label=f"{metric_label} [{metric_unit}] @ N={h_max}")
    def _base_str(name: str) -> str:
        v = _spec_by_name(name).base_value(base_params)
        return f"{v:.4g}" if v is not None else "—"

    fig.suptitle(
        f"2D スイープ: {name_a} × {name_b} ({metric_label} @ N={h_max})\n"
        f"仕様値: {name_a}={_base_str(name_a)}, {name_b}={_base_str(name_b)}",
        fontsize=11,
    )
    add_params_annotation(fig, base_params)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path}")


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
_METRIC_COLORS = {"yaw": "#1f77b4", "lat": "#2ca02c", "long": "#ff7f0e", "pos": "#9467bd"}


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

    fig, (ax_bar, ax_curve) = plt.subplots(1, 2, figsize=(15, max(5, 0.55 * len(recs) + 2)))

    # --- 左: 改善率ランキング (barh, 改善率昇順 = 上が最大) ---
    names = [r["spec"].name for r in recs]
    pcts = [_improvement_pct(r) for r in recs]
    colors = [_METRIC_COLORS[r["spec"].metric] for r in recs]
    bars = ax_bar.barh(range(len(recs)), pcts, color=colors, alpha=0.85)
    ax_bar.set_yticks(range(len(recs)), names)
    for i, (r, b) in enumerate(zip(recs, bars)):
        ident = r["ident"]
        v = ident["parabolic_value"] if ident["parabolic_value"] is not None else ident["grid_value"]
        edge = " (端)" if ident["at_edge"] else ""
        ax_bar.text(b.get_width() + 0.3, i, f"同定≈{v:.4g}{edge}", va="center", fontsize=8)
    ax_bar.set_xlabel(f"RMSE 改善率 [%] (仕様値 → 同定値, N={h_max})")
    ax_bar.set_title("感度ランキング: どのパラメータを直すと効くか", fontsize=11)
    ax_bar.grid(True, axis="x", lw=0.5, alpha=0.6)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c, alpha=0.85)
               for m, c in _METRIC_COLORS.items() if any(r["spec"].metric == m for r in recs)]
    labels = [_METRIC_INFO[m][0] for m in _METRIC_COLORS
              if any(r["spec"].metric == m for r in recs)]
    ax_bar.legend(handles, labels, fontsize=8, loc="lower right")

    # --- 右: 正規化カーブ (param ごとに色分け) ---
    cmap = plt.get_cmap("tab10")
    for i, r in enumerate(reversed(recs)):  # 改善率の大きい順に手前へ
        spec, res, ident = r["spec"], r["res"], r["ident"]
        at = res[res["horizon"] == h_max].sort_values("value")
        xs = at["value"].to_numpy()
        ys = at[ident["metric_col"]].to_numpy()
        span = xs.max() - xs.min()
        if span <= 0 or r["rmse_at_spec"] is None or r["rmse_at_spec"] <= 0:
            continue
        xn = (xs - xs.min()) / span
        yn = ys / r["rmse_at_spec"]
        color = cmap(i % 10)
        ax_curve.plot(xn, yn, "-", color=color, lw=1.5, label=spec.name)
        # 仕様値位置 (y=1) と グリッド最小
        bx = (r["base_value"] - xs.min()) / span
        ax_curve.plot([bx], [1.0], "o", color=color, ms=6, mec="white", mew=0.8)
        mi = int(np.argmin(yn))
        ax_curve.plot([xn[mi]], [yn[mi]], "v", color=color, ms=7, mec="white", mew=0.8)
    ax_curve.axhline(1.0, color="black", lw=0.8, ls=":")
    ax_curve.set_xlabel("グリッド位置 (パラメータごとに [0,1] 正規化)")
    ax_curve.set_ylabel("RMSE / RMSE@仕様値")
    ax_curve.set_title("正規化 RMSE カーブ (平坦=非感度 / 谷=同定可能 / 単調=端)", fontsize=11)
    ax_curve.grid(True, lw=0.5, alpha=0.6)
    ax_curve.legend(fontsize=8, ncol=2)

    fig.suptitle(
        f"パラメータ sweep 感度オーバービュー (N={h_max} 終端誤差, ○=仕様値, ▼=最小)",
        fontsize=12,
    )
    add_params_annotation(fig, params)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
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
        note = "グリッド端 (範囲拡大推奨)" if ident["at_edge"] else ""
        lines.append(
            f"| {spec.name} ({spec.label}) | {base_str} | "
            f"{ident['grid_value']:.4g} / {parab_str} | {delta_str} | "
            f"{metric_label}@N{ident['horizon']} | {rmse_str} | {pct_str} | {note} |"
        )
    lines += [
        "",
        "> 各パラメータは他を仕様値に固定した 1 次元 sweep (one-at-a-time)。改善率は同一メトリクス内"
        "での相対値であり、メトリクスが異なる行同士の直接比較には注意。パラメータ間の相関は "
        "2D ペア sweep (pair_*.svg) を、全体俯瞰は感度オーバービュー (_overview_sensitivity.svg) を参照。"
        "brake_time_constant は減速モデルが別実装のため Stage 9 (brake_sweep/) が担当。",
        "",
    ]
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"  Saved: {out_path}")


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
    parser.add_argument("--horizons", default="2,5,10,20", help="カンマ区切りの rollout horizon")
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
        grid = grid_overrides.get(spec.name) or spec.grid(base_params)
        base_value = spec.base_value(base_params)
        print(f"\n=== sweep: {spec.name} ({spec.label})  grid={grid} ===")
        res = run_param_sweep(data, t0_ns, base_params, spec, grid, horizons, args.stride)
        ident = identify(res, spec, h_max)
        res.to_csv(out_dir / f"{spec.name}_sweep.csv", index=False)
        evidence = [
            (lambda ax, _fn=fn, _i=ident, _b=base_value: _fn(ax, ev, _i, _b))
            for fn in _EVIDENCE_PLOTS.get(spec.name, [])
        ]
        plot_sweep(res, spec, ident, base_value, base_params,
                   out_dir / f"{spec.name}_sweep.svg", evidence=evidence)
        records.append({
            "spec": spec,
            "res": res,
            "ident": ident,
            "base_value": base_value,
            "rmse_at_spec": _rmse_at(res, ident, base_value),
        })

        metric_label, _, _ = _METRIC_INFO[spec.metric]
        parab = ident["parabolic_value"]
        print(f"  同定 ({metric_label} 最小化 @ N={h_max}): "
              f"グリッド={ident['grid_value']:.4g}"
              + (f", 放物線≈{parab:.4g}" if parab is not None else " (放物線: 端のため範囲外)"))
        if ident["at_edge"]:
            print(f"  [WARN] 最小がグリッド端 ({spec.name}={ident['grid_value']:.4g})。"
                  "真の最小はこの外側の可能性。--grid で範囲を広げて再実行を推奨。")

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
