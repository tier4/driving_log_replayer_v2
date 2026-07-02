"""軌跡再生ビューア用の payload 組み立て純関数.

`_playback_viewer.py` の自己完結 HTML テンプレートから、データ整形と JSON payload
構築を切り出す。呼び出し側はこのモジュールの `build_playback_payload()` を使い、
`_playback_viewer.py` は HTML 書き出しに専念する。
"""

from __future__ import annotations

import numpy as np
import pandas as pd

from ._map import map_ways_in_bbox

# 再生データのリサンプリングレート [Hz]。上げると滑らかになるが HTML サイズが増える。
PLAYBACK_RATE_HZ = 10.0
# 地図クリップ・全体表示のマージン [m]（plot_trajectory と同じ値）。
MAP_MARGIN_M = 30.0
# ベースライン (位置同期モードの基準) とする run ラベル。
BASELINE_LABEL = "実機"
# DP 計画軌跡の間引き設定 (HTML サイズ抑制)。フレームは最小間隔 [s]、点は stride 間引き。
DP_FRAME_MIN_DT = 0.5
DP_POINT_STRIDE = 3
# 角速度(yaw rate)指令を自転車モデル wz=v·tanδ/L で導出する際の wheelbase [m]
# (multi_dataset_tune.WHEELBASE と整合)。slip/k_us を無視する近似のため要注記。
PLAYBACK_WHEELBASE_M = 4.76012


def _round_list(arr: np.ndarray, ndigits: int) -> list[float]:
    """JSON サイズ削減のための一括丸め。"""
    return [round(float(v), ndigits) for v in arr]


def _resample_channel(
    df: pd.DataFrame | None, col: str, t_grid: np.ndarray, offset: float
) -> tuple[np.ndarray, np.ndarray]:
    """1 信号 (df[col]) を共有 t グリッドへ線形補間し、(値, 有効マスク) を返す。

    `offset` (= run の発進時刻) を t から引いてから載せる (_resample_run と同基準)。
    ソース時刻範囲の外側 (左右) は補間が端値ホールドになり実測でない区間なので、
    マスク False とする (呼び出し側で JSON null にしてプロット線・ドットを止める)。
    df が無い/空/列なしなら全点無効。
    """
    if df is None or df.empty or col not in df.columns or "t" not in df.columns:
        return np.zeros_like(t_grid), np.zeros(t_grid.shape, dtype=bool)
    t = df["t"].to_numpy(dtype=float) - offset
    y = df[col].to_numpy(dtype=float)
    finite = np.isfinite(t) & np.isfinite(y)
    if finite.sum() < 1:
        return np.zeros_like(t_grid), np.zeros(t_grid.shape, dtype=bool)
    t, y = t[finite], y[finite]
    vals = np.interp(t_grid, t, y)
    valid = (t_grid >= t[0] - 1e-9) & (t_grid <= t[-1] + 1e-9)
    return vals, valid


def _channel_jsonlist(
    vals: np.ndarray, valid: np.ndarray, ndigits: int
) -> list[float | None]:
    """(値, 有効マスク) を JSON 用リストへ。無効点・非有限は None (= JS の null)。"""
    return [
        round(float(v), ndigits) if (ok and np.isfinite(v)) else None
        for v, ok in zip(vals, valid, strict=True)
    ]


def _resample_run(
    kin: pd.DataFrame, vel: pd.DataFrame, t_grid: np.ndarray, offset: float = 0.0
) -> dict:
    """
    1 run の kinematic / velocity を共有 t グリッドへリサンプリングする。

    `offset` (= その run の発進時刻 t_launch) を t から引いてから共有グリッドに載せる。
    これにより全 run が発進時刻 (t=0) で揃い、実機の初期停止が長くても再生がズレない。

    - yaw は ±π wrap で補間が暴れるため `np.unwrap` 後に補間する。
    - run の最終時刻より後のグリッド点は `np.interp` が端値ホールドする。
      `n_valid` (最終有効グリッド点数) を併記し、それ以降は v=0 に強制する
      （停止車両に速度矢印が出ないように。JS 側は ended 表示にも使う）。
    - 弧長 s はグリッド化後の座標から累積。端値ホールド区間は ds=0 で plateau し、
      単調非減少が保たれる（JS の二分探索の前提）。
    """
    t = kin["t"].to_numpy(dtype=float) - offset
    x = np.interp(t_grid, t, kin["x"].to_numpy(dtype=float))
    y = np.interp(t_grid, t, kin["y"].to_numpy(dtype=float))
    yaw = np.interp(t_grid, t, np.unwrap(kin["yaw"].to_numpy(dtype=float)))
    if vel is not None and not vel.empty:
        v = np.interp(
            t_grid,
            vel["t"].to_numpy(dtype=float) - offset,
            vel["lon_vel"].to_numpy(dtype=float),
        )
    else:
        v = np.zeros_like(t_grid)
    n_valid = int(np.searchsorted(t_grid, t[-1], side="right"))
    n_valid = max(n_valid, 1)
    v[n_valid:] = 0.0
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return {
        "x": _round_list(x, 2),
        "y": _round_list(y, 2),
        "yaw": _round_list(yaw, 3),
        "v": _round_list(v, 2),
        "s": _round_list(s, 2),
        "n_valid": n_valid,
        "s_total": round(float(s[n_valid - 1]), 2),
    }


def _build_channels(d: dict, t_grid: np.ndarray, offset: float) -> dict:
    """1 run の同期プロット用信号群を共有 t グリッドへ載せる。

    返すキー (各 list は t_grid と同長、無効点は None):
      lon_vel / cmd_vel   縦速度・指令速度 [m/s]
      vy                  横速度 [m/s] (twist.linear.y, base_link 系・実測)
      accel / cmd_accel   縦加速度・指令 [m/s²]
      ay                  横加速度 [m/s²] (= v_lon·wz・遠心加速度、sim と同式)
      slope_acc           勾配加速度 [m/s²] (= 9.81·sin(pitch)、登り<0・縦横モデルの重力項用)
      steer / cmd_steer   ステア角・指令 [deg]
      wz                  角速度 yaw rate [rad/s] (twist.angular.z・実測)
      cmd_wz              角速度指令 [rad/s] = cmd_vel·tan(cmd_steer)/L (自転車近似)
    """
    vel, accel, steer, cmd, kin = (
        d.get("velocity"), d.get("accel"), d.get("steering"), d.get("cmd"), d.get("kinematic"),
    )

    lon_vel = _resample_channel(vel, "lon_vel", t_grid, offset)
    cmd_vel = _resample_channel(cmd, "cmd_vel", t_grid, offset)
    vy = _resample_channel(kin, "vy", t_grid, offset)
    acc = _resample_channel(accel, "accel", t_grid, offset)
    cmd_acc = _resample_channel(cmd, "cmd_accel", t_grid, offset)
    steer_rad = _resample_channel(steer, "steer", t_grid, offset)
    cmd_steer_rad = _resample_channel(cmd, "cmd_steer", t_grid, offset)
    wz = _resample_channel(kin, "wz", t_grid, offset)
    # 横加速度 (遠心加速度) a_y = v_lon·wz。localization/acceleration の横成分は常に 0 のため
    # 実測の縦速度・ヨーレートから sim 側 (out.ay = vv·om) と同式で算出する。両信号が有効な点のみ。
    ay = (lon_vel[0] * wz[0], lon_vel[1] & wz[1])
    # 勾配加速度 (重力分力) = 9.81·sin(pitch)。pitch は nose-down 正なので登り(nose-up)で負=減速。
    # simple_planning_simulator の acc_by_slope=-9.81·sin(slope_angle登り正) と同符号。縦モデルの a_target に
    # c_slope 倍して加算する (_model_viewer)。physical な参照量としてパネルにも表示。
    pitch_ch = _resample_channel(kin, "pitch", t_grid, offset)
    slope_acc = (9.81 * np.sin(pitch_ch[0]), pitch_ch[1])

    # 角速度指令 (自転車近似): cmd_vel·tan(cmd_steer)/L。両信号が有効な点のみ。
    cmd_wz_vals = cmd_vel[0] * np.tan(cmd_steer_rad[0]) / PLAYBACK_WHEELBASE_M
    cmd_wz_valid = cmd_vel[1] & cmd_steer_rad[1]

    return {
        "lon_vel": _channel_jsonlist(*lon_vel, 2),
        "cmd_vel": _channel_jsonlist(*cmd_vel, 2),
        "vy": _channel_jsonlist(*vy, 3),
        "accel": _channel_jsonlist(*acc, 3),
        "cmd_accel": _channel_jsonlist(*cmd_acc, 3),
        "ay": _channel_jsonlist(*ay, 3),
        "slope_acc": _channel_jsonlist(*slope_acc, 3),
        "steer": _channel_jsonlist(np.degrees(steer_rad[0]), steer_rad[1], 2),
        "cmd_steer": _channel_jsonlist(np.degrees(cmd_steer_rad[0]), cmd_steer_rad[1], 2),
        "wz": _channel_jsonlist(*wz, 4),
        "cmd_wz": _channel_jsonlist(cmd_wz_vals, cmd_wz_valid, 4),
    }


def _compact_dp_frames(frames: list[dict], offset: float = 0.0) -> dict:
    """DP 計画軌跡フレームを間引き・丸めして JSON 埋め込み用に圧縮する。

    frames: step4 `_load_dp_trajectories` の出力 ({"t", "x", "y"} のリスト、t 昇順)。
    `offset` (= その run の発進時刻) を t から引き、発進前 (t<0) のフレームは落とす。
    戻り値: {"t": [...], "x": [[...]...], "y": [[...]...]} (保持フレームのみ)。
    点列は DP_POINT_STRIDE 間引き + 終端点保持 (計画長が見た目で縮まないように)。
    """
    out_t: list[float] = []
    out_x: list[list[float]] = []
    out_y: list[list[float]] = []
    last_t = -1e9
    for f in frames:
        t = float(f["t"]) - offset
        if t < 0 or t - last_t < DP_FRAME_MIN_DT:
            continue
        last_t = t
        xs = list(f["x"][::DP_POINT_STRIDE])
        ys = list(f["y"][::DP_POINT_STRIDE])
        if (len(f["x"]) - 1) % DP_POINT_STRIDE != 0:
            xs.append(f["x"][-1])
            ys.append(f["y"][-1])
        out_t.append(round(t, 2))
        out_x.append([round(float(v), 2) for v in xs])
        out_y.append([round(float(v), 2) for v in ys])
    return {"t": out_t, "x": out_x, "y": out_y}


def _build_metrics_payload(metrics: dict | None) -> dict | None:
    """compute_closed_loop_metrics の結果から JS 表示用の最小ペイロードを抽出する。

    ビューア凡例のクローズループ指標パネルに表示する値だけを渡す。
    metrics が None / runs が空のときは None を返す（パネル非表示）。
    """
    if not metrics:
        return None
    runs_out = {}
    for label, rec in (metrics.get("runs") or {}).items():
        runs_out[label] = {
            "completion_pct": rec.get("completion_pct"),
            "s2r_mean_m": rec.get("s2r_mean_m"),
            "s2r_max_m": rec.get("s2r_max_m"),
            "r2s_mean_m": rec.get("r2s_mean_m"),
            "r2s_max_m": rec.get("r2s_max_m"),
            "vel_rmse_mps": rec.get("vel_rmse_mps"),
            "steer_rmse_deg": rec.get("steer_rmse_deg"),
            "status": rec.get("status"),
        }
    real_rec = metrics.get("real") or {}
    return {
        "real_dist_m": metrics.get("real_dist_m"),
        "real_rmse": {
            "vel_rmse_mps": real_rec.get("vel_rmse_mps"),
            "steer_rmse_deg": real_rec.get("steer_rmse_deg"),
        },
        "runs": runs_out,
    }


def build_playback_payload(
    data: dict,
    map_ways: list | None,
    rate_hz: float = PLAYBACK_RATE_HZ,
    map_margin_m: float = MAP_MARGIN_M,
    title: str = "",
    metrics: dict | None = None,
) -> dict | None:
    """
    step4 の `loaded` dict から埋め込み用 payload を構築する。

    Args:
        data: step4 main() の loaded。label -> {"kinematic": df(t,x,y,yaw),
              "velocity": df(t,lon_vel), "color": CSS色, ...}
        map_ways: `load_map_ways()` の戻り値 (None なら地図なし)。
        rate_hz: リサンプリングレート。
        map_margin_m: bbox マージン。
        title: ビューア上部に表示するシナリオ名。

    Returns:
        JSON 化可能な dict。kinematic が全 run 空なら None。
    """
    kins = {lbl: d["kinematic"] for lbl, d in data.items() if not d["kinematic"].empty}
    if not kins:
        return None

    # 各 run を発進時刻 (t_launch) で相対化して共有グリッドへ載せる。グリッドは発進 (t=0) から
    # 最長の発進後継続時間まで。実機の初期停止が長くても全 run が発進で揃う (実機だけ遅れない)。
    def _offset(lbl: str) -> float:
        return float(data[lbl].get("t_launch", 0.0))

    t_max = max(float(k["t"].iloc[-1]) - _offset(lbl) for lbl, k in kins.items())
    n = int(np.floor(t_max * rate_hz)) + 1
    t_grid = np.arange(n) / rate_hz

    runs = []
    for label, d in data.items():
        kin = d["kinematic"]
        if kin.empty:
            continue
        offset = _offset(label)
        run = _resample_run(kin, d.get("velocity"), t_grid, offset)
        run["label"] = label
        run["color"] = str(d["color"])
        run["is_baseline"] = label == BASELINE_LABEL
        run["dp"] = _compact_dp_frames(d.get("dp_traj") or [], offset)
        # 同期時系列プロット用の信号群 (実測 + 指令)。t_grid 上・無効点は null。
        run["ch"] = _build_channels(d, t_grid, offset)
        runs.append(run)
    if not any(r["is_baseline"] for r in runs):
        runs[0]["is_baseline"] = True

    all_xy = np.concatenate([k[["x", "y"]].to_numpy(dtype=float) for k in kins.values()])
    x_min, y_min = all_xy.min(axis=0) - map_margin_m
    x_max, y_max = all_xy.max(axis=0) + map_margin_m
    bbox = [
        round(float(x_min), 1),
        round(float(y_min), 1),
        round(float(x_max), 1),
        round(float(y_max), 1),
    ]

    lanelets: list[list[list[float]]] = []
    if map_ways:
        for pts in map_ways_in_bbox(map_ways, (x_min, x_max), (y_min, y_max)):
            lanelets.append([[round(float(px), 2), round(float(py), 2)] for px, py in pts])

    payload: dict = {
        "title": title,
        "rate_hz": rate_hz,
        "n": n,
        "bbox": bbox,
        "lanelets": lanelets,
        "runs": runs,
    }
    m = _build_metrics_payload(metrics)
    if m is not None:
        payload["metrics"] = m
    return payload
