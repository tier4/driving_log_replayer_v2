"""sim_runs.yaml の dataclass + loader (Stage 3 用、cases.yaml と同型).

sim_runs.yaml は scenario_test_runner.launch.py を起動するための設定を
複数 run 並べる。Stage 3 (step3_run_sims) は --run-tag で対象 1 entry を指定。
Stage 4 (step4_compare_logs) は全 tag を lite/<tag>.lite/ から読んで N-way 比較。

schema:

    sim_runs:
      - tag: <str>                   # 出力 lite/<tag>.lite/ + LOGS dict キー
        vehicle_model: <str>         # scenario_test_runner.launch の vehicle_model:= 引数
        sensor_model: <str>          # 同 sensor_model:=
        initialize_duration: <int>   # 同 initialize_duration:= [s] (任意、既定 100)
        architecture_type: <str>     # 同 architecture_type:= (任意、既定 awf/universe/20250130)
        godot_executable: <str>      # vehicle_model が *_godot のとき必須
        timeout_s: <int>             # step3_run_sims subprocess timeout [s] (任意、既定 600)
        dp_model_dir: <str>          # DiffusionPlanner モデルディレクトリ (任意・ローカル既存配置用)。
                                     # diffusion_planner.onnx と args.json を含む。
                                     # step3_run_sims が env DIFFUSION_PLANNER_ONNX_PATH /
                                     # DIFFUSION_PLANNER_ARGS_PATH に設定して launch に渡す
                                     # (autoware_launch の diffusion_planner.param.yaml が
                                     # $(env ...) 置換で参照)。未指定なら Autoware 既定モデル。
        dp_model_release: <str>      # Web.Auto ML パッケージの release 名 (任意・自動 pull 用)。
                                     # 指定すると step3_run_sims が webauto ml package-release で
                                     # search→pull し、取得した dir を dp_model_dir として使う。
                                     # dp_model_dir と同時指定は不可 (どちらか一方)。
        dp_model_package: <str>      # 上記 release の package 名 (任意、既定 diffusion_planner_for_x2_exp)。
                                     # dp_model_release 指定時のみ有効。
        params: {<key>: <scalar>}    # simulator_model パラメータ上書き (任意、cases.yaml と同型)
                                     # 例 {k_us: 0.020}。step3_run_sims が
                                     # simple_sensor_simulator.<key>:=<value> として launch に渡し、
                                     # description の simulator_model.param.yaml を上書きする。
                                     # description パッケージを増やさず dynamics 変種を作れる。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
import re
import warnings
from typing import Any

import yaml


# tag は OUT_DIR = .../lite/<tag>.lite/ に直接連結されるため安全文字のみ許可。
_TAG_PATTERN = re.compile(r"^[A-Za-z0-9_\-.]+$")

# sim_runs[*] で受け付ける既知キー (typo 検出用)
_KNOWN_KEYS: frozenset[str] = frozenset({
    "tag", "vehicle_model", "sensor_model",
    "initialize_duration", "architecture_type",
    "godot_executable", "timeout_s",
    "dp_model_dir", "dp_model_release", "dp_model_package", "params",
})

# 自動 pull 時の既定 package 名 (Web.Auto の DiffusionPlanner 実験用パッケージ)
_DEFAULT_DP_PACKAGE = "diffusion_planner_for_x2_exp"


@dataclass
class SimRunSpec:
    """1 sim run の定義 (sim_runs.yaml の sim_runs[i])."""

    tag: str
    vehicle_model: str
    sensor_model: str = "aip_x2_gen2"
    initialize_duration: int = 100
    architecture_type: str = "awf/universe/20250130"
    godot_executable: str | None = None
    timeout_s: int = 600
    dp_model_dir: str | None = None
    dp_model_release: str | None = None
    dp_model_package: str | None = None
    params: dict[str, float | int | bool | str] = field(default_factory=dict)


@dataclass
class SimRunsConfig:
    runs: list[SimRunSpec]

    def find_run(self, tag: str) -> SimRunSpec:
        for r in self.runs:
            if r.tag == tag:
                return r
        raise KeyError(
            f"tag={tag!r} が sim_runs.yaml に見つかりません。"
            f"定義済 tag: {[r.tag for r in self.runs]}"
        )

    @property
    def tags(self) -> list[str]:
        return [r.tag for r in self.runs]


def load_sim_runs_config(path: str | Path) -> SimRunsConfig:
    """sim_runs.yaml を読み込み SimRunsConfig を返す。

    必須: sim_runs (≥1 要素、各 tag 一意 / 安全文字、vehicle_model 指定)。
    typo 検出: 各 entry に _KNOWN_KEYS 外のキーがあれば warn。
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"sim_runs.yaml が見つかりません: {p}")

    with p.open(encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}

    raw_runs = doc.get("sim_runs")
    if not raw_runs or not isinstance(raw_runs, list):
        raise ValueError(f"{p}: 'sim_runs:' が空またはリストではありません")

    runs: list[SimRunSpec] = []
    seen_tags: set[str] = set()
    for i, entry in enumerate(raw_runs):
        if not isinstance(entry, dict):
            raise ValueError(f"{p}: sim_runs[{i}] が dict ではありません")
        tag = entry.get("tag")
        if not tag or not isinstance(tag, str):
            raise ValueError(f"{p}: sim_runs[{i}].tag が未指定または文字列ではありません")
        if not _TAG_PATTERN.match(tag):
            raise ValueError(
                f"{p}: sim_runs[{i}].tag={tag!r} は安全文字 [A-Za-z0-9_.-] のみ許可"
            )
        if tag in seen_tags:
            raise ValueError(f"{p}: tag={tag!r} が重複しています")
        seen_tags.add(tag)

        vehicle_model = entry.get("vehicle_model")
        if not vehicle_model or not isinstance(vehicle_model, str):
            raise ValueError(
                f"{p}: sim_runs[{i}].vehicle_model が未指定または文字列ではありません"
            )

        unknown = set(entry.keys()) - _KNOWN_KEYS
        if unknown:
            warnings.warn(
                f"{p}: sim_runs[{i}] に未知キー {sorted(unknown)} が含まれます "
                f"(typo の可能性。既知キー: {sorted(_KNOWN_KEYS)})"
            )

        # _godot サフィックスかつ godot_executable 未指定なら警告
        if vehicle_model.endswith("_godot") and not entry.get("godot_executable"):
            warnings.warn(
                f"{p}: sim_runs[{i}] vehicle_model={vehicle_model!r} だが "
                "godot_executable が未指定 (scenario_test_runner.launch の既定パスにフォールバック)"
            )

        # godot_executable は `${HOME}` / `~` を展開 (ros2 launch は展開しないため)
        godot_exe = entry.get("godot_executable")
        if godot_exe:
            godot_exe = os.path.expandvars(os.path.expanduser(str(godot_exe)))

        # dp_model_dir も `${VAR}` / `~` を展開 (env 経由で DP モデルパスに渡すため)
        dp_model_dir = entry.get("dp_model_dir")
        if dp_model_dir:
            dp_model_dir = os.path.expandvars(os.path.expanduser(str(dp_model_dir)))

        # dp_model_release: 自動 pull 用。dp_model_dir と排他。package は既定あり。
        dp_model_release = entry.get("dp_model_release")
        dp_model_package = entry.get("dp_model_package")
        if dp_model_release and dp_model_dir:
            raise ValueError(
                f"{p}: sim_runs[{i}] dp_model_dir と dp_model_release は同時指定できません "
                "(どちらか一方)"
            )
        if dp_model_package and not dp_model_release:
            raise ValueError(
                f"{p}: sim_runs[{i}] dp_model_package は dp_model_release 指定時のみ有効です"
            )
        if dp_model_release:
            dp_model_release = str(dp_model_release)
            dp_model_package = str(dp_model_package) if dp_model_package else _DEFAULT_DP_PACKAGE

        # params: simulator_model パラメータ上書き (スカラのみ)。
        # launch に simple_sensor_simulator.<key>:=<value> として渡る。
        raw_params = entry.get("params") or {}
        if not isinstance(raw_params, dict):
            raise ValueError(f"{p}: sim_runs[{i}].params が dict ではありません")
        params: dict[str, float | int | bool | str] = {}
        for k, v in raw_params.items():
            if not isinstance(v, (int, float, bool, str)):
                raise ValueError(
                    f"{p}: sim_runs[{i}].params.{k} はスカラ (int/float/bool/str) のみ可 "
                    f"(got {type(v).__name__})"
                )
            params[str(k)] = v

        runs.append(SimRunSpec(
            tag=tag,
            vehicle_model=vehicle_model,
            sensor_model=entry.get("sensor_model", "aip_x2_gen2"),
            initialize_duration=int(entry.get("initialize_duration", 100)),
            architecture_type=entry.get("architecture_type", "awf/universe/20250130"),
            godot_executable=godot_exe,
            timeout_s=int(entry.get("timeout_s", 600)),
            dp_model_dir=dp_model_dir,
            dp_model_release=dp_model_release,
            dp_model_package=dp_model_package,
            params=params,
        ))

    return SimRunsConfig(runs=runs)
