"""Plotly figures for physical-validity time series.

The builders accept both the current rich row schema and the compact schema
used by older cached reports.
"""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go


def _values(row: dict, *names: str, default: str | None = None) -> list:
    for name in names:
        if name in row:
            return list(row[name])
    if default and isinstance(row.get(default), dict):
        value = row[default]
        if value:
            return list(next(iter(value.values())))
    return []


def build_fig_cross_long(rows: list[dict], cross_fit: dict | None = None) -> go.Figure:
    fig = go.Figure()
    shown: set[str] = set()
    def add(x, y, name, **kwargs):
        if len(y) == 0:
            return
        fig.add_trace(go.Scatter(x=x[:len(y)], y=y, name=name, showlegend=name not in shown, **kwargs))
        shown.add(name)
    for row in rows:
        t = _values(row, "t")
        if not t:
            continue
        cmd = _values(row, "a_cmd_raw", "a_cmd")
        actual = _values(row, "a_act_raw", "a_act")
        fitted = _values(row, "a_sim_cross_raw", "a_sim_cross", "a_sim_fit")
        add(t, actual, "実測: a_report / dot_a_real")
        add(t, cmd, "指令: a_cmd / dot_a_cmd")
        if fitted:
            label = "横断同定: a_sim / dot_a_sim"
            if cross_fit:
                label += f" (τ={float(cross_fit.get('tau', 0.0)):.3f}s, T={float(cross_fit.get('delay', 0.0)):.3f}s)"
            add(t, fitted, label)
        tuned = row.get("a_sim_models_raw", row.get("a_sim_models"))
        if isinstance(tuned, dict):
            for key, values in tuned.items():
                add(t, list(values), f"{key}: a_sim / dot_a_sim (チューニング値)")
        vx = _values(row, "vx")
        if vx:
            add(t, vx, "車速 v_x", yaxis="y2")
        add(t, _values(row, "mask_dyn"), "同定対象区間", yaxis="y3")
    fig.update_layout(height=1800, xaxis_title="時間 [s]", yaxis_title="加速度 [m/s²]", yaxis2={"overlaying": "y", "side": "right"})
    return fig


def build_fig_cross_steer(rows: list[dict]) -> go.Figure:
    fig = go.Figure()
    shown: set[str] = set()
    def add(x, y, name, **kwargs):
        if len(y) == 0:
            return
        fig.add_trace(go.Scatter(x=x[:len(y)], y=y, name=name, showlegend=name not in shown, **kwargs))
        shown.add(name)
    for row in rows:
        t = _values(row, "t")
        if not t:
            continue
        cmd = _values(row, "d_cmd")
        actual = _values(row, "d_act_raw", "d_act")
        fitted = _values(row, "d_sim_fit")
        if cmd:
            add(t, cmd, "指令 δ_cmd")
        if actual:
            add(t, actual, "実測: δ_act / dot_δ_act")
        tuned = row.get("d_sim_models_raw", row.get("d_sim_models"))
        if isinstance(tuned, dict):
            for key, values in tuned.items():
                add(t, list(values), f"{key}: δ_sim / dot_δ_sim (チューニング値)")
        vx = _values(row, "vx")
        if vx:
            add(t, vx, "車速 v_x", yaxis="y2")
        mask = _values(row, "mask_dyn")
        if mask:
            add(t, np.asarray(mask, dtype=float), "同定対象区間", yaxis="y3")
    fig.update_layout(height=1800, xaxis_title="時間 [s]", yaxis_title="操舵角 [rad]", yaxis2={"overlaying": "y", "side": "right"}, yaxis3={"visible": False})
    return fig
