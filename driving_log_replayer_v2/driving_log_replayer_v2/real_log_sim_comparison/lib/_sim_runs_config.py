"""sim_runs の dataclass + loader (scenario.yaml 経由).

設定は scenario.yaml の Conditions.models / sim_runs に統合された。
load_sim_runs_config() は scenario.yaml のパスを受け取り、_models_config.load_models_doc()
に委譲して SimRunsConfig を構築する。

vehicle_model_type → sim enum 注入:
  models エントリに vehicle_model_type が設定されている場合、_VEHICLE_MODEL_TYPE_ENUM の
  マッピングに従い大文字 enum を params["vehicle_model_type"] として注入する。これにより
  step3_run_sims が simple_sensor_simulator.vehicle_model_type:=<ENUM> として launch に渡す
  (例: taiga_dyn → TAIGA_DYN)。

schema (scenario.yaml の Conditions ブロック):

    models:
      <name>:
        vehicle_model: <str>           # closed-loop description パッケージ名 (必須)
        vehicle_model_type: <str>      # open-loop クラス名。sim では enum として params 注入 (任意)
        params: {<key>: <scalar>}      # simulator_model パラメータ上書き (任意)
        architecture_type: <str>       # 既定 awf/universe/20250130
        sensor_model: <str>            # 既定 aip_x2_gen2
        initialize_duration: <int>     # 既定 100 [s]
        godot_executable: <str>        # vehicle_model が *_godot のとき必須
        timeout_s: <int>               # 既定 600 [s]
        dp_model_dir: <str>            # DP モデルディレクトリ (任意)
        dp_model_release: <str>        # Web.Auto release 名 (任意・自動 pull)
        dp_model_package: <str>        # dp_model_release 指定時の package 名

    sim_runs: [<name>, ...]            # vehicle_model が必須
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ._models_config import _VEHICLE_MODEL_TYPE_ENUM, load_models_doc


@dataclass
class SimRunSpec:
    """1 sim run の定義 (closed-loop シム実行用)."""

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
            f"tag={tag!r} が Conditions.sim_runs に見つかりません。"
            f"定義済 tag: {[r.tag for r in self.runs]}"
        )

    @property
    def tags(self) -> list[str]:
        return [r.tag for r in self.runs]


def load_sim_runs_config(scenario_path: str | Path) -> SimRunsConfig:
    """scenario.yaml を読み込み SimRunsConfig を返す。

    Args:
        scenario_path: scenario.yaml のパス (Conditions.models / sim_runs を含む)

    Returns:
        SimRunsConfig: sim_runs リスト

    Raises:
        FileNotFoundError: scenario.yaml が存在しない
        ValueError: models / sim_runs の定義が不正、params がスカラでない
    """
    doc = load_models_doc(scenario_path)
    p = Path(scenario_path)

    runs: list[SimRunSpec] = []
    for name in doc.sim_runs_list:
        m = doc.models[name]
        # vehicle_model は load_models_doc で None でないことが保証済み

        # vehicle_model_type → sim enum を params に注入
        # (step3_run_sims が simple_sensor_simulator.vehicle_model_type:=<ENUM> で渡す)
        # params に既に vehicle_model_type が明示されている場合はそちらを優先する。
        params: dict[str, float | int | bool | str] = {}
        for k, v in m.params.items():
            if not isinstance(v, (int, float, bool, str)):
                raise ValueError(
                    f"{p}: models.{name}.params.{k} はスカラ (int/float/bool/str) のみ可 "
                    f"(got {type(v).__name__})"
                )
            params[str(k)] = v

        if m.vehicle_model_type and "vehicle_model_type" not in params:
            params["vehicle_model_type"] = _VEHICLE_MODEL_TYPE_ENUM[m.vehicle_model_type]

        runs.append(SimRunSpec(
            tag=name,
            vehicle_model=m.vehicle_model,  # type: ignore[arg-type]
            sensor_model=m.sensor_model,
            initialize_duration=m.initialize_duration,
            architecture_type=m.architecture_type,
            godot_executable=m.godot_executable,
            timeout_s=m.timeout_s,
            dp_model_dir=m.dp_model_dir,
            dp_model_release=m.dp_model_release,
            dp_model_package=m.dp_model_package,
            params=params,
        ))

    return SimRunsConfig(runs=runs)
