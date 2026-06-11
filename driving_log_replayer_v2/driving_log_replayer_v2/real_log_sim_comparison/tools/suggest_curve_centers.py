#!/usr/bin/env python3
"""
実機 bag の /localization/kinematic_state から旋回点（高 yaw-rate 点）を抽出し、
curve_config YAML の curve_centers 記入を補助する。

Usage:
    python3 tools/suggest_curve_centers.py --bag <bag_dir_or_mcap> [オプション]

例:
    # 課題 A（ロータリー）の dataset を make issue_scenario で構築した後:
    python3 tools/suggest_curve_centers.py \\
        --bag work/dataset/<uuid>/input_bag \\
        --top 5 --min-gap 20

出力例:
    top 5 高 yaw-rate 点（yaw_rate_deg_s > 5.0 の区間を除外して重複排除）:
    +---------+--------+----------+----------+----------+
    | rank    | t [s]  | cx       | cy       | yaw_rate |
    +---------+--------+----------+----------+----------+
    | 1       | 34.1   | 89421.3  | 43128.7  | 18.3 °/s |
    | 2       | 67.8   | 89302.1  | 43086.4  | 12.1 °/s |
    +---------+--------+----------+----------+----------+

    curve_config YAML 記入例:
    curve_centers:
      - {label: "カーブ①", cx: 89421, cy: 43129, margin: 40}
      - {label: "カーブ②", cx: 89302, cy: 43086, margin: 40}

cx/cy は /localization/kinematic_state の pose.pose.position.x/y（地図座標）。
"""

import argparse
import sys
from pathlib import Path

import numpy as np

# パッケージが colcon install 済みの場合はモジュールパスが解決される。
# スクリプト単体実行の場合は sys.path を補完する。
_RLSC_ROOT = Path(__file__).resolve().parent.parent
if str(_RLSC_ROOT) not in sys.path:
    sys.path.insert(0, str(_RLSC_ROOT))

from lib._io import load_kinematic  # noqa: E402


def suggest_curve_centers(
    bag_path: Path,
    top: int = 5,
    min_yaw_rate_deg: float = 5.0,
    min_gap_s: float = 10.0,
) -> list[dict]:
    """旋回点の map 座標リストを返す。

    Args:
        bag_path: 実機 bag ディレクトリ（input_bag/ など）または単一 .mcap ファイル。
        top: 返す上位点の数。
        min_yaw_rate_deg: この値（deg/s）を超える旋回区間のみ候補とする。
        min_gap_s: 同じ旋回区間の重複ピークを排除する最小時間間隔 [s]。

    Returns:
        [{"rank": int, "t_s": float, "cx": float, "cy": float, "yaw_rate_deg": float}, ...]
        降順ソート済み。
    """
    df = load_kinematic(bag_path)
    if df.empty:
        print("[WARN] kinematic_state が取得できませんでした。トピックが存在するか確認してください。",
              file=sys.stderr)
        return []

    t_s = (df["t_ns"].to_numpy() - df["t_ns"].iloc[0]) / 1e9
    yaw_rate = df["wz"].to_numpy()  # rad/s (base_link angular.z)
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()

    yaw_rate_deg = np.abs(np.degrees(yaw_rate))
    above = yaw_rate_deg >= min_yaw_rate_deg

    if not above.any():
        print(f"[INFO] yaw_rate >= {min_yaw_rate_deg} deg/s の区間が見つかりませんでした。"
              "--min-yaw-rate を下げてみてください。", file=sys.stderr)
        return []

    # 候補インデックスを降順ソートし、min_gap で重複除去
    candidates = np.where(above)[0]
    order = candidates[np.argsort(yaw_rate_deg[candidates])[::-1]]

    selected = []
    for idx in order:
        if len(selected) >= top:
            break
        t_c = t_s[idx]
        if any(abs(t_c - t_s[s]) < min_gap_s for s in selected):
            continue
        selected.append(idx)

    results = []
    for rank, idx in enumerate(selected, 1):
        results.append({
            "rank": rank,
            "t_s": float(t_s[idx]),
            "cx": float(x[idx]),
            "cy": float(y[idx]),
            "yaw_rate_deg": float(yaw_rate_deg[idx]),
        })
    return results


def _print_table(rows: list[dict]) -> None:
    if not rows:
        print("（候補なし）")
        return
    header = f"{'rank':>4}  {'t [s]':>7}  {'cx':>10}  {'cy':>10}  {'yaw_rate':>12}"
    sep = "-" * len(header)
    print(sep)
    print(header)
    print(sep)
    for r in rows:
        print(f"{r['rank']:>4}  {r['t_s']:>7.1f}  {r['cx']:>10.1f}  {r['cy']:>10.1f}  "
              f"{r['yaw_rate_deg']:>9.1f} °/s")
    print(sep)


def _print_yaml_snippet(rows: list[dict]) -> None:
    if not rows:
        return
    print()
    print("curve_config YAML 記入例（cx/cy は整数に丸めた概算値）:")
    print("  curve_centers:")
    for r in rows:
        cx_i = int(round(r["cx"]))
        cy_i = int(round(r["cy"]))
        label = f"カーブ{r['rank']}"
        print(f"    - {{label: \"{label}\", cx: {cx_i}, cy: {cy_i}, margin: 40}}")
    print()
    print("  # margin は旋回半径に応じて調整すること（通常 20〜60）。")
    print("  # curve_config YAML の cx/cy を上記で更新してから make local_cloud_run を実行する。")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--bag", required=True, type=Path,
        help="実機 bag ディレクトリ（input_bag/ など）または単一 .mcap ファイル",
    )
    parser.add_argument(
        "--top", type=int, default=5,
        help="出力する上位点の数 (デフォルト: 5)",
    )
    parser.add_argument(
        "--min-yaw-rate", type=float, default=5.0,
        metavar="DEG_S",
        help="旋回検出の最低 yaw-rate しきい値 [deg/s] (デフォルト: 5.0)",
    )
    parser.add_argument(
        "--min-gap", type=float, default=10.0,
        metavar="SEC",
        help="同一旋回区間の重複排除のための最小時間間隔 [s] (デフォルト: 10.0)",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag)
    if not bag_path.exists():
        print(f"[ERROR] bag パスが存在しません: {bag_path}", file=sys.stderr)
        sys.exit(1)

    print(f"[INFO] bag: {bag_path}")
    print(f"[INFO] 上位 {args.top} 点 (min_yaw_rate={args.min_yaw_rate} deg/s, "
          f"min_gap={args.min_gap} s)")
    print()

    rows = suggest_curve_centers(
        bag_path,
        top=args.top,
        min_yaw_rate_deg=args.min_yaw_rate,
        min_gap_s=args.min_gap,
    )

    print(f"高 yaw-rate 旋回点 top {args.top}（降順）:")
    _print_table(rows)
    _print_yaml_snippet(rows)


if __name__ == "__main__":
    main()
