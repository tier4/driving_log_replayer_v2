"""Minimal model configuration used by the reidentify pipeline."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Any

import yaml

from .settings import BASELINE_MODEL_NAME
from .settings import TARGET_MODEL_NAME
from .settings import TARGET_MODEL_TYPE
from .settings import TUNED_MODEL_DISPLAY_NAME
from ..lib._accel_source import normalize_accel_source
from ..lib._steer_source import normalize_steer_source
from ..lib._vehicle_models import SUPPORTED_VMT

_SAFE_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")

# fit の direct-fit ステージ順 (連続プレフィックスの基準)。merge はこれらの後に任意で続く。
DIRECT_FIT_STAGES = ("lon", "steer", "xy")
FIT_STAGES = (*DIRECT_FIT_STAGES, "merge")
# release.model にリテラルで指定できる「fit 出力」の名前 (scenario ケースではない)。
RELEASE_TUNED_NAME = "tuned"


@dataclass(frozen=True)
class ModelSpec:
    name: str
    vehicle_model_type: str
    acceleration_source: str
    steering_source: str
    params: dict


@dataclass(frozen=True)
class ReleaseSpec:
    """リリース対象 ({model, version})。model は scenario ケース名または "tuned"。"""

    model: str
    version: int


@dataclass(frozen=True)
class FitSpec:
    """fit 対象ケース (target) と実行するステージ (連続プレフィックス + 任意 merge)。"""

    target: str
    stages: tuple[str, ...]

    @property
    def enabled(self) -> bool:
        return bool(self.stages)

    @property
    def direct_stages(self) -> tuple[str, ...]:
        return tuple(stage for stage in self.stages if stage in DIRECT_FIT_STAGES)

    @property
    def has_merge(self) -> bool:
        return "merge" in self.stages


@dataclass(frozen=True)
class ModelConfig:
    models: dict[str, ModelSpec]
    comparison_models: tuple[str, ...]
    fit: FitSpec
    release: ReleaseSpec | None = None
    plot_datasets: tuple[str, ...] = ()

    def find_case(self, name: str) -> ModelSpec:
        try:
            return self.models[name]
        except KeyError as exc:
            raise ValueError(
                f"model {name!r} が scenario にありません。定義済み: {sorted(self.models)}"
            ) from exc


def load_model_config(path: str | Path) -> ModelConfig:
    """Load model data and the physical-validity comparison contract."""
    scenario = Path(path)
    if not scenario.is_file():
        raise FileNotFoundError(f"scenario が見つかりません: {scenario}")
    document = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{scenario}: top-level は mapping である必要があります")
    evaluation = document.get("Evaluation")
    if not isinstance(evaluation, dict):
        raise ValueError(f"{scenario}: Evaluation は mapping である必要があります")
    conditions = evaluation.get("Conditions")
    if not isinstance(conditions, dict):
        raise ValueError(f"{scenario}: Evaluation.Conditions は mapping である必要があります")
    raw_models = conditions.get("models")
    if not isinstance(raw_models, dict) or not raw_models:
        raise ValueError(f"{scenario}: Evaluation.Conditions.models が必要です")

    models: dict[str, ModelSpec] = {}
    for raw_name, raw_spec in raw_models.items():
        name = str(raw_name)
        if not _SAFE_NAME.fullmatch(name):
            raise ValueError(f"{scenario}: 不正な model 名です: {name!r}")
        if not isinstance(raw_spec, dict):
            raise ValueError(f"{scenario}: models.{name} は mapping である必要があります")
        model_type = str(raw_spec.get("vehicle_model_type") or "")
        if model_type not in SUPPORTED_VMT:
            raise ValueError(
                f"{scenario}: models.{name}.vehicle_model_type={model_type!r} は未対応です"
            )
        params = raw_spec.get("params") or {}
        if not isinstance(params, dict):
            raise ValueError(f"{scenario}: models.{name}.params は mapping である必要があります")
        models[name] = ModelSpec(
            name=name,
            vehicle_model_type=model_type,
            acceleration_source=normalize_accel_source(
                raw_spec.get("acceleration_source"), default="accel"
            ),
            steering_source=normalize_steer_source(
                raw_spec.get("steering_source"), default="steer"
            ),
            params=dict(params),
        )

    if BASELINE_MODEL_NAME not in models:
        raise ValueError(f"{scenario}: models に必須定義がありません: [{BASELINE_MODEL_NAME!r}]")

    fit = _parse_fit(scenario, conditions, models)

    # wheelbase は baseline に常に、fit 有効時は fit 対象ケースにも正の値が必要。
    wheelbase_required = {BASELINE_MODEL_NAME}
    if fit.enabled:
        wheelbase_required.add(fit.target)
    for name in sorted(wheelbase_required):
        wheelbase = models[name].params.get("wheelbase")
        valid_wheelbase = (
            isinstance(wheelbase, (int, float))
            and not isinstance(wheelbase, bool)
            and math.isfinite(float(wheelbase))
            and float(wheelbase) > 0.0
        )
        if not valid_wheelbase:
            raise ValueError(f"{scenario}: models.{name}.params.wheelbase に正の数値が必要です")
    raw_comparison_models = conditions.get("comparison_models")
    if not isinstance(raw_comparison_models, list) or not raw_comparison_models:
        raise ValueError(f"{scenario}: Evaluation.Conditions.comparison_models が必要です")
    comparison_models = tuple(str(name) for name in raw_comparison_models)
    if len(set(comparison_models)) != len(comparison_models):
        raise ValueError(f"{scenario}: comparison_models に重複があります")
    undefined = [name for name in comparison_models if name not in models]
    if undefined:
        raise ValueError(f"{scenario}: comparison_models に未定義モデルがあります: {undefined}")
    if BASELINE_MODEL_NAME not in comparison_models:
        raise ValueError(
            f"{scenario}: comparison_models に必須モデルがありません: [{BASELINE_MODEL_NAME!r}]"
        )

    release = _parse_release(scenario, conditions, models, fit)
    plot_datasets = _parse_plot_dataset(scenario, conditions)
    return ModelConfig(
        models=models,
        comparison_models=comparison_models,
        fit=fit,
        release=release,
        plot_datasets=plot_datasets,
    )


def _parse_fit(
    scenario: Path, conditions: dict, models: dict[str, ModelSpec],
) -> FitSpec:
    """
    任意の Evaluation.Conditions.fit ({target, stages}) を検証して返す。

    stages は {lon, steer, xy, merge} の部分集合で、direct-fit (lon/steer/xy) は
    その順序の「連続する先頭部分」でなければならない (歯抜け不可)。merge は任意で後続する。
    省略時は全ステージ (lon,steer,xy,merge)、target 既定は TARGET_MODEL_NAME。
    """
    raw_fit = conditions.get("fit")
    target = TARGET_MODEL_NAME
    stages: tuple[str, ...] = FIT_STAGES
    if raw_fit is not None:
        if not isinstance(raw_fit, dict):
            raise ValueError(f"{scenario}: fit は mapping である必要があります")
        if raw_fit.get("target") is not None:
            target = str(raw_fit["target"])
        if "stages" in raw_fit:
            stages = _normalize_fit_stages(scenario, raw_fit["stages"])

    if stages:
        if target not in models:
            raise ValueError(
                f"{scenario}: fit.target={target!r} が models にありません。"
                f"定義済み: {sorted(models)}"
            )
        if models[target].vehicle_model_type != TARGET_MODEL_TYPE:
            raise ValueError(
                f"{scenario}: fit.target={target!r} の vehicle_model_type は "
                f"{TARGET_MODEL_TYPE!r} である必要があります"
            )
    return FitSpec(target=target, stages=stages)


def _normalize_fit_stages(scenario: Path, raw_stages: Any) -> tuple[str, ...]:
    if not isinstance(raw_stages, list):
        raise ValueError(f"{scenario}: fit.stages はリストが必要です: {raw_stages!r}")
    seen: list[str] = []
    for entry in raw_stages:
        name = str(entry)
        if name not in FIT_STAGES:
            raise ValueError(
                f"{scenario}: fit.stages の不正な要素 {name!r} (許可: {list(FIT_STAGES)})"
            )
        if name in seen:
            raise ValueError(f"{scenario}: fit.stages に重複があります: {name!r}")
        seen.append(name)
    direct = [stage for stage in seen if stage in DIRECT_FIT_STAGES]
    expected_prefix = list(DIRECT_FIT_STAGES[: len(direct)])
    if sorted(direct, key=DIRECT_FIT_STAGES.index) != direct or direct != expected_prefix:
        raise ValueError(
            f"{scenario}: fit.stages の direct-fit は {list(DIRECT_FIT_STAGES)} の"
            f"連続する先頭部分である必要があります (歯抜け不可): {direct}"
        )
    # 正規化: direct-fit を規定順に並べ、merge があれば末尾へ。
    ordered = list(expected_prefix)
    if "merge" in seen:
        ordered.append("merge")
    return tuple(ordered)


def _parse_release(
    scenario: Path, conditions: dict, models: dict[str, ModelSpec], fit: FitSpec,
) -> ReleaseSpec | None:
    """
    任意の Evaluation.Conditions.release ({model, version}) を検証して返す。

    release ステージは指定ケース (または fit 出力 "tuned") を v{version} スロットへ
    リリースする。未指定時は release ステージをスキップする (自動リリースはしない)。
    model には scenario ケース名、または fit 出力を指すリテラル "tuned" を指定できる。
    """
    raw_release = conditions.get("release")
    if raw_release is None:
        return None
    if not isinstance(raw_release, dict):
        raise ValueError(f"{scenario}: release は mapping である必要があります")
    model = str(raw_release.get("model") or "")
    if model == RELEASE_TUNED_NAME:
        if not fit.enabled:
            raise ValueError(
                f"{scenario}: release.model={RELEASE_TUNED_NAME!r} は fit が有効"
                " (fit.stages が非空) な場合のみ指定できます"
            )
    elif model not in models:
        raise ValueError(
            f"{scenario}: release.model={model!r} が models にありません。"
            f'定義済み: {sorted(models)} (または fit 出力を指す "{RELEASE_TUNED_NAME}")'
        )
    elif models[model].vehicle_model_type != TARGET_MODEL_TYPE:
        raise ValueError(
            f"{scenario}: release.model={model!r} の vehicle_model_type は "
            f"{TARGET_MODEL_TYPE!r} である必要があります"
        )
    version = raw_release.get("version")
    if isinstance(version, bool) or not isinstance(version, int) or version < 1:
        raise ValueError(f"{scenario}: release.version は正の整数が必要です: {version!r}")
    return ReleaseSpec(model=model, version=version)


def _parse_plot_dataset(scenario: Path, conditions: dict) -> tuple[str, ...]:
    """
    任意の Evaluation.Conditions.plot_dataset (dataset-id のリスト) を検証して返す。

    指定時はレポート末尾の時系列診断セクションが対象データセットを順に描画する。
    dataset の実在確認は collection_dir を知るセクション構築側が行う。
    """
    raw_plot_dataset = conditions.get("plot_dataset")
    if raw_plot_dataset is None:
        return ()
    if not isinstance(raw_plot_dataset, list):
        raise ValueError(
            f"{scenario}: plot_dataset は dataset-id のリストが必要です: {raw_plot_dataset!r}"
        )
    for entry in raw_plot_dataset:
        if not isinstance(entry, str) or not _SAFE_NAME.fullmatch(entry):
            raise ValueError(
                f"{scenario}: plot_dataset は英数字・'_.-' のみの dataset-id の"
                f"リストが必要です: {entry!r}"
            )
    if len(set(raw_plot_dataset)) != len(raw_plot_dataset):
        raise ValueError(f"{scenario}: plot_dataset に重複があります: {raw_plot_dataset!r}")
    return tuple(raw_plot_dataset)


def resolve_baseline_model(config: ModelConfig) -> tuple[str, dict, str]:
    baseline = config.models[BASELINE_MODEL_NAME]
    return baseline.vehicle_model_type, dict(baseline.params), baseline.name


def comparison_display_order(config: ModelConfig) -> tuple[str, ...]:
    """レポート/metrics.csv に載せる比較モデルの表示名を順序付きで返す。

    comparison_models の順序を保ちつつ、fit 有効時は fit 対象ケースを "tuned" に
    置換 (フィット結果で評価される) する。fit 出力 "tuned" を比較に載せたい場合は
    fit 対象ケースを comparison_models に明記する (未指定なら比較には出さない)。
    fit 無効時は素の comparison_models。
    """
    return tuple(
        TUNED_MODEL_DISPLAY_NAME if config.fit.enabled and name == config.fit.target else name
        for name in config.comparison_models
    )
