#!/usr/bin/env python3
"""複数 run の metrics_closed_loop.json を読み、実機との再現率を横断表示する。

使い方:
    python3 tools/repro_compare.py LABEL=OUT_DIR [LABEL=OUT_DIR ...]
        OUT_DIR は sample/out/<ts> (result_archive/.../comparison/metrics_closed_loop.json を内包)

再現率の見方:
    sim mean_speed / elapsed が実機に近いほど時間・速度の再現が良い。
    vel_rmse / steer_rmse は指令追従 (車両モデル忠実度)、s2r_mean は経路追従、
    completion% は走行距離比 (vicinity 許容により ~90% が上限のことがある)。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path


def load(out_dir: str) -> dict:
    p = Path(out_dir)
    cands = list(p.glob("result_archive/*/comparison/metrics_closed_loop.json"))
    if not cands:
        cands = list(p.glob("**/metrics_closed_loop.json"))
    if not cands:
        raise FileNotFoundError(f"metrics_closed_loop.json が見つかりません: {out_dir}")
    return json.load(open(cands[0]))


def main() -> None:
    runs = []
    for arg in sys.argv[1:]:
        label, _, path = arg.partition("=")
        runs.append((label, load(path)))

    for label, d in runs:
        s = d["real"]["summary"]
        print(
            f"\n### {label}: {d.get('scenario_name','?')}\n"
            f"  実機: dist={s['dist']:.1f}m elapsed={s['elapsed']:.1f}s "
            f"mean_speed={s['mean_speed']:.2f} cruise={s['cruise']:.2f} vmax={s['vmax']:.1f}"
        )
        hdr = (f"  {'model':<12}{'compl%':>8}{'elapsed':>9}{'meanV':>7}"
               f"{'ΔmeanV%':>9}{'s2r_m':>7}{'r2s_m':>7}{'velRMSE':>8}{'steerRMSE':>10}")
        print(hdr)
        print("  " + "-" * (len(hdr) - 2))
        real_mv = s["mean_speed"]
        for m, r in d["runs"].items():
            rs = r["summary"]
            dmv = (rs["mean_speed"] - real_mv) / real_mv * 100.0 if real_mv else float("nan")
            def g(k):
                v = r.get(k)
                return v if v is not None else float("nan")
            print(f"  {m:<12}{g('completion_pct'):>8.1f}{rs['elapsed']:>9.1f}"
                  f"{rs['mean_speed']:>7.2f}{dmv:>+9.1f}{g('s2r_mean_m'):>7.2f}"
                  f"{g('r2s_mean_m'):>7.2f}{g('vel_rmse_mps'):>8.2f}{g('steer_rmse_deg'):>10.2f}")


if __name__ == "__main__":
    main()
