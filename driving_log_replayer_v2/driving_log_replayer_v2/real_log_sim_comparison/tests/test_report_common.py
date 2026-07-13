from __future__ import annotations

import pandas as pd

from driving_log_replayer_v2.real_log_sim_comparison import physical_validity_report


def test_report_skips_false_score_mismatch_across_split_scopes() -> None:
    row = {
        "h": 10,
        "p14_yaw": 1.0,
        "p14_lat": 2.0,
        "p14_long": 3.0,
        "p14_vx": 0.4,
        "bl_yaw": 1.1,
        "bl_lat": 2.1,
        "bl_long": 3.1,
        "bl_vx": 0.5,
    }
    rendered = physical_validity_report._build_sec_deviation(
        pd.DataFrame([row]),
        1,
        recomputed_score=12.0,
        expected_score=None,
        score_name="robust_score",
        score_scope="validation",
    )

    assert "validation" in rendered
    assert "再現比較は行わない" in rendered
    assert "⚠ 不整合" not in rendered


def test_report_uses_actual_dataset_count_in_metric_explanation() -> None:
    rendered = physical_validity_report._build_sec_metrics(
        baseline_score=10.0,
        phase14_score=9.0,
        n_dataset=372,
    )

    assert "全 372 データセット" in rendered
    assert "650 データセット" not in rendered
