#!/usr/bin/env python3
"""完全データドリブン線形/アフィン ARX 車両モデル (learned_arx) の係数を pooled ridge 最小二乗で同定する。

delay_steer_acc_geared_wo_fall_guard (物理: 自転車+understeer・1 次遅延) と後で比較し、
「物理モデルに足りない応答項」をデータドリブンに読み取る目的の新モデル用。学習対象は
縦加速度応答 ax と yaw-rate 応答 wz の 2 チャネルのみ (位置/yaw の運動学積分は物理モデルと共有)。

離散 ARX (C++ SimModelLearnedArx と同一定義、Δ=SUB_DT):
    ax[k+1] = a0 + a1*ax[k] + a2*accel_cmd_delayed[k] + a3*vx[k] + a4*vx[k]^2
    wz[k+1] = b0 + b1*wz[k] + b2*steer_cmd_delayed[k] + b3*vx[k]*steer_cmd_delayed[k] + b4*vx[k]

【derivative-form での同定 (重要)】
制御コマンドの配信周期 (実測 ~0.030s) はシミュレータの SUB_DT (1/30s≈0.0333s) と一致しないため、
素朴な離散 fit (ax[k+1]=... を cmd サンプル間で当てる) は時定数を ~10% 取り違える。C++ は
d(ax)/dt = (a0 + (a1-1)*ax + ...)/Δ を Δ=SUB_DT で割って積分し、cmd 区間 iv (<SUB_DT) では
1 fractional step で状態を B*(iv/Δ) だけ進める (B = 角括弧)。よって測定差分から
    B = Δ * (y[k+1]-y[k]) / iv      (Δ=SUB_DT)
を equation-error ターゲットとして特徴量 [1, y, cmd_delayed, vx, ...] に最小二乗回帰すれば、
rollout の積分と厳密に整合する離散係数 [a0..a4] が得られる (iv の周期ずれを正しく吸収)。

【pooled robust fit】
best_normal の anti-overfit 思想を踏襲し、全データセットの遷移を 1 つの design 行列にプールして
ridge (列標準化・intercept 非正則化) で解く。per-dataset fit はしない。評価は別途
multi_dataset_tune (make local_multidataset_run) が同一 baseline 正規化 N-step 誤差で行う。

注: LS は 1-step equation-error を最小化、評価は free-running N-step 誤差なので両者は別物
(これは閉形式・SGD 不要の代償で正常)。本スクリプトの主成果は係数値そのもの (a0/b0 アフィン、
a4 vx^2、b3 vx*steer、a3/b4 vx 線形) で、これが「物理モデルに足りない応答項」を表す。

使い方:
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.arx_fit \
        --collection-dir driving_log_replayer_v2/real_log_sim_comparison/sample/multi \
        --out driving_log_replayer_v2/real_log_sim_comparison/sample/arx_coeffs.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

from . import step5_analyze_nstep as s5
from .lib._io import resolve_lite_bag
from .multi_dataset_tune import WHEELBASE, _discover

# rollout (run_rollout) と同じ行除外ガード。
VX_MIN = 0.5            # 低速 (kinematic steer 逆算が不安定) は除外
IV_LO_FACTOR = 0.3      # iv が SUB_DT の 0.3 倍未満は異常
IV_HI_FACTOR = 3.0      # iv が SUB_DT の 3 倍超は異常 (欠損/ジャンプ)

# 縦/横の特徴量ラベル (係数解釈の出力用)。
# 注: learned_arx は 6 係数/ch。ax は throttle/brake 分離、wz は understeer k_us を持つが
# k_us は非線形 (vx² 分母) で線形 LS では同定できないため本スクリプトは 0 固定で出力する
# (k_us / 非線形項の最適化は arx_optuna.py が担当)。
_AX_FEATURES = [
    "a0 (affine)", "a1 (AR ax)", "a2_thr (accel>=0)", "a2_brk (accel<0)", "a3 (vx)", "a4 (vx^2)"
]
_WZ_FEATURES = ["b0 (affine)", "b1 (AR wz)", "b2 (steer_cmd)", "b3 (vx*steer)", "b4 (vx)"]


def _delayed(t_cmd: np.ndarray, q: int, t_full: np.ndarray, val_full: np.ndarray, sub_dt: float):
    """delay queue 先頭が k→k+1 ステップで適用する遅延コマンド (= q*sub_dt 過去) を補間。

    step5._delay_history と同じく実効遅延 q*SUB_DT (round(delay/SUB_DT)*SUB_DT) でシフトする。
    """
    return np.interp(
        t_cmd - q * sub_dt, t_full, val_full, left=val_full[0], right=val_full[-1]
    )


def build_rows(ds_dirs: list[tuple[str, Path]]):
    """全データセットをプールし design 行列 (X_ax, y_ax, X_wz, y_wz) と採用行数を返す。"""
    Xax: list[list[float]] = []
    yax: list[float] = []
    Xwz: list[list[float]] = []
    ywz: list[float] = []
    used: list[tuple[str, int]] = []
    for ds_id, lite_dir in ds_dirs:
        s5.LITE_DIR = lite_dir
        real = resolve_lite_bag(lite_dir, "real")
        if real is None:
            print(f"[WARN] real.lite が見つかりません: {lite_dir}", file=sys.stderr)
            continue
        try:
            data = s5.load_real_bag(real)
            t0 = s5.find_autonomous_start(data)
            base = s5._build_params()
            base["wheelbase"] = WHEELBASE
            s5.SUB_DT = base["sub_dt"]
            g = s5._prepare_gt(data, t0, base)
        except Exception as e:  # noqa: BLE001
            print(f"[SKIP] {ds_id}: ロード失敗 ({type(e).__name__}: {e})", file=sys.stderr)
            continue

        dt = s5.SUB_DT
        t = g["t_cmd"]
        vx = g["gt_vx"]
        ax = g["gt_ax"]
        wz = g["gt_wz"]
        q_acc = round(base["acc_time_delay"] / dt)
        q_steer = round(base["steer_time_delay"] / dt)
        u_acc = _delayed(t, q_acc, g["t_cmd_full"], g["accel_des_full"], dt)
        u_steer = _delayed(t, q_steer, g["t_cmd_full"], g["steer_des_full"], dt)

        cnt = 0
        for k in range(len(t) - 1):
            iv = t[k + 1] - t[k]
            if iv <= IV_LO_FACTOR * dt or iv >= IV_HI_FACTOR * dt:
                continue
            if vx[k] < VX_MIN:
                continue
            scale = dt / iv  # B = Δ/iv * (y[k+1]-y[k]); rollout の 1 fractional step と厳密一致
            # ax は throttle/brake 分離 (max(u,0), min(u,0))
            Xax.append([1.0, ax[k], max(u_acc[k], 0.0), min(u_acc[k], 0.0), vx[k], vx[k] * vx[k]])
            yax.append(scale * (ax[k + 1] - ax[k]))
            Xwz.append([1.0, wz[k], u_steer[k], vx[k] * u_steer[k], vx[k]])
            ywz.append(scale * (wz[k + 1] - wz[k]))
            cnt += 1
        used.append((ds_id, cnt))

    return (
        np.asarray(Xax, dtype=float),
        np.asarray(yax, dtype=float),
        np.asarray(Xwz, dtype=float),
        np.asarray(ywz, dtype=float),
        used,
    )


def ridge_fit(X: np.ndarray, y: np.ndarray, lam: float):
    """intercept(列0)非正則化・残り列を標準化した ridge で B-target を fit し、離散係数を返す。

    モデル: B = g0 + w1*x1 + w2*x2 + w3*x3 + w4*x4  (X = [1, x1, x2, x3, x4])。
    返り値: (coeffs, r2)  coeffs = [c0, c1, c2, c3, c4] (離散 ARX; c1 は AR 係数 = g1+1)。
    """
    F = X[:, 1:]  # 非定数特徴 4 列
    mu = F.mean(axis=0)
    sd = F.std(axis=0)
    sd = np.where(sd == 0.0, 1.0, sd)
    Fs = (F - mu) / sd
    my = y.mean()
    yc = y - my
    n = len(y)
    # 正規方程式を n で正規化し、ridge λ を単位分散特徴に対して意味のあるスケールにする。
    A = Fs.T @ Fs / n + lam * np.eye(Fs.shape[1])
    w_s = np.linalg.solve(A, Fs.T @ yc / n)
    w = w_s / sd
    g0 = my - mu @ w
    g = np.concatenate([[g0], w])  # B = g·[1,x1..x4] (= [a0, a1-1, a2, a3, a4])

    # equation-error R^2
    pred = X @ g
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    coeffs = g.copy()
    coeffs[1] += 1.0  # g1 = a1-1 -> a1
    return coeffs, r2


def _stabilize(coeffs: np.ndarray, ar_label: str) -> np.ndarray:
    """AR 係数 (index 1) が |.|>=1 だと free-running で発散するため [-0.999,0.999] にクランプ。

    equation-error LS は不安定極を返し得る。クランプは fallback で、発動時は警告する。
    """
    out = coeffs.copy()
    if abs(out[1]) >= 1.0:
        clamped = float(np.clip(out[1], -0.999, 0.999))
        print(
            f"[WARN] {ar_label} AR 係数 {out[1]:.4f} が |.|>=1 (発散リスク)。{clamped:.4f} にクランプ。",
            file=sys.stderr,
        )
        out[1] = clamped
    return out


def _fmt_coeffs(name: str, coeffs: np.ndarray, labels: list[str], r2: float) -> str:
    body = "\n".join(f"    {lab:18s} = {c:+.6g}" for lab, c in zip(labels, coeffs))
    return f"{name}  (equation-error R^2 = {r2:.4f})\n{body}"


def main() -> None:
    ap = argparse.ArgumentParser(description="ARX 係数の pooled ridge 最小二乗同定")
    ap.add_argument(
        "--collection-dir",
        default=str(Path(__file__).parent / "sample" / "multi"),
        help="収集済み <dataset_id>/real.lite を含むディレクトリ",
    )
    ap.add_argument(
        "--lite-dir",
        action="append",
        default=[],
        metavar="DATASET_ID=LITE_DIR",
        help="収集を使わず直接指定 (複数可)",
    )
    ap.add_argument("--ridge", type=float, default=1e-3, help="ridge λ (単位分散特徴に対する)")
    ap.add_argument("--out", default="", help="係数 YAML 出力先 (省略時は stdout のみ)")
    args = ap.parse_args()

    if args.lite_dir:
        ds_dirs = []
        for spec in args.lite_dir:
            ds_id, raw = spec.split("=", 1)
            ds_dirs.append((ds_id.strip(), Path(raw.strip())))
    else:
        ds_dirs = _discover(Path(args.collection_dir))
    if not ds_dirs:
        print(f"ERROR: real.lite が見つかりません: {args.collection_dir}", file=sys.stderr)
        sys.exit(1)
    print(f"datasets: {[d for d, _ in ds_dirs]}")

    Xax, yax, Xwz, ywz, used = build_rows(ds_dirs)
    n_rows = len(yax)
    print("\n## pooled design 行数 (dataset 別採用遷移数)")
    for ds_id, cnt in used:
        print(f"  {ds_id}: {cnt}")
    print(f"  合計: {n_rows} 行 (ridge λ={args.ridge})")
    if n_rows < 50:
        print(f"ERROR: 採用行が少なすぎます ({n_rows})", file=sys.stderr)
        sys.exit(1)

    ax_coeffs, ax_r2 = ridge_fit(Xax, yax, args.ridge)
    wz_coeffs, wz_r2 = ridge_fit(Xwz, ywz, args.ridge)

    # --- 安定性ゲート: AR 係数 |.|<1 ---
    ax_coeffs = _stabilize(ax_coeffs, "ax")  # len 6: [a0,a1,a2_thr,a2_brk,a3,a4]
    wz_coeffs = _stabilize(wz_coeffs, "wz")  # len 5: [b0,b1,b2,b3,b4]
    # learned_arx の wz は 6 係数目に understeer k_us を持つが、非線形 (vx² 分母) で線形 LS では
    # 同定できないため 0 を付与する (k_us の最適化は arx_optuna.py)。
    wz_coeffs = np.append(wz_coeffs, 0.0)  # -> [b0,b1,b2,b3,b4,k_us=0]

    print("\n## 同定された離散 ARX 係数")
    print(_fmt_coeffs("ax[k+1] = a0 + a1*ax + a2_thr*max(u,0) + a2_brk*min(u,0) + a3*vx + a4*vx^2",
                      ax_coeffs, _AX_FEATURES, ax_r2))
    print(_fmt_coeffs("wz[k+1] = b0 + b1*wz + b2*steer + b3*vx*steer + b4*vx  (+ k_us=0; LS 不可)",
                      wz_coeffs[:5], _WZ_FEATURES, wz_r2))

    print("\n## 物理モデル (delay_steer_acc_geared_wo_fall_guard) に足りない応答項の読み取り")
    print(f"  縦アフィン a0 = {ax_coeffs[0]:+.5g}  (転がり抵抗/勾配オフセット; 物理は 0 相当)")
    print(f"  縦 throttle ゲイン a2_thr = {ax_coeffs[2]:+.5g}, brake ゲイン a2_brk = {ax_coeffs[3]:+.5g}")
    print(f"  縦 vx 線形 a3 = {ax_coeffs[4]:+.5g}, vx^2 a4 = {ax_coeffs[5]:+.5g}  (速度依存抵抗; 物理は無)")
    print(f"  横アフィン b0 = {wz_coeffs[0]:+.5g}, vx 線形 b4 = {wz_coeffs[4]:+.5g}  (drift/bias; 物理は無)")
    print(f"  横 vx*steer b3 = {wz_coeffs[3]:+.5g}  (bicycle 主項 vx*δ/L に対応)")
    print(f"  ax AR 時定数相当 tau_ax ≈ {-s5.SUB_DT / np.log(abs(ax_coeffs[1])):.4f}s "
          f"(a1={ax_coeffs[1]:.4f})" if 0 < abs(ax_coeffs[1]) < 1 else "")
    print(f"  wz AR 時定数相当 tau_wz ≈ {-s5.SUB_DT / np.log(abs(wz_coeffs[1])):.4f}s "
          f"(b1={wz_coeffs[1]:.4f})" if 0 < abs(wz_coeffs[1]) < 1 else "")

    ax_list = "[" + ", ".join(f"{c:.8g}" for c in ax_coeffs) + "]"
    wz_list = "[" + ", ".join(f"{c:.8g}" for c in wz_coeffs) + "]"
    yaml_block = f"arx_ax_coeffs: {ax_list}\narx_wz_coeffs: {wz_list}\n"
    print("\n## cases.yaml / simulator_model.param.yaml に貼り付け")
    print(yaml_block)

    if args.out:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(yaml_block, encoding="utf-8")
        print(f"書き出し: {out_path}")


if __name__ == "__main__":
    main()
