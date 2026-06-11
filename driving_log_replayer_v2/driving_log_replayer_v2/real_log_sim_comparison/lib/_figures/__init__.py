"""図スペック（plotly Figure）を組む純関数パッケージ.

matplotlib SVG を廃した新方式（commit 59fef44 置換）の描画層。各 build_fig_* は
データ整形済みの配列/DataFrame を受けて `go.Figure` を返す純関数で、ROS 非依存
（notebook の素 kernel からも import 可）。各解析ステップ（ROS 依存）はデータを
整形してここに渡し、`lib._fig_io.write_fig_json` で `<stem>.fig.json` に吐く。
step11 は単一 HTML、step12 は notebook で同じ図スペックを描画する。
"""

from __future__ import annotations

from ._closed_loop import (
    build_fig_vs_distance,
)
from ._cases import (
    build_fig_cascade_error_overlay,
    build_fig_error_growth_overlay,
    build_fig_error_timeseries_overlay,
    build_fig_growth_relative,
    build_fig_rmse_heatmap,
)
from ._cross_dataset import (
    build_fig_coverage_overview,
    build_fig_cross_closed_loop_heatmap,
    build_fig_cross_normalized_bars,
    build_fig_cross_nstep_heatmap,
    build_fig_loo_stability,
)
from ._dp import (
    build_fig_dp_real_vs_sim,
    build_fig_dp_vs_actual,
    build_fig_dp_vs_final_traj,
)
from ._sweep import build_fig_sweep, build_fig_sweep_overview
from ._nstep import build_fig_overview

__all__ = [
    "build_fig_cascade_error_overlay",
    "build_fig_coverage_overview",
    "build_fig_cross_closed_loop_heatmap",
    "build_fig_cross_normalized_bars",
    "build_fig_cross_nstep_heatmap",
    "build_fig_loo_stability",
    "build_fig_dp_real_vs_sim",
    "build_fig_dp_vs_actual",
    "build_fig_dp_vs_final_traj",
    "build_fig_error_growth_overlay",
    "build_fig_error_timeseries_overlay",
    "build_fig_growth_relative",
    "build_fig_rmse_heatmap",
    "build_fig_overview",
    "build_fig_sweep",
    "build_fig_sweep_overview",
    "build_fig_vs_distance",
]
