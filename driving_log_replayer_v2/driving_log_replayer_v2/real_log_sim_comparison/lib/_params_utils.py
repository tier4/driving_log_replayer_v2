"""共通ユーティリティ: シミュレータパラメータ (simulator_model/vehicle_info) の読み込み."""

from __future__ import annotations

from pathlib import Path


def _find_desc_dir() -> Path:
    """j6_gen2_description/config のパスを解決する（ROS 非依存環境では寛容にフォールバック）。

    ament_index_python (ROS) が無い環境（report.ipynb を素の Jupyter kernel で開く等）でも
    本モジュールを import できるよう、解決不能時は存在しないパスを返し、load_sim_params の
    既定値フォールバックに委ねる。pipeline (ROS 有) では従来どおり実パスを返す。
    """
    try:
        from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415

        return Path(get_package_share_directory("j6_gen2_description")) / "config"
    except Exception:  # noqa: BLE001  (ament 不在・パッケージ未ビルド等)
        return Path("/nonexistent/j6_gen2_description/config")


_DESC_DIR = _find_desc_dir()
_SIM_YAML = _DESC_DIR / "simulator_model.param.yaml"
_INFO_YAML = _DESC_DIR / "vehicle_info.param.yaml"


def load_sim_params(params_dir: Path | None = None) -> dict:
    """
    simulator_model.param.yaml + vehicle_info.param.yaml から主要パラメータを返す。

    params_dir: vehicle_info.param.yaml と simulator_model.param.yaml が置かれたディレクトリ。
                None の場合は既定の j6_gen2_description/config/ を使用。
    YAML が読めない場合はフォールバック値を使用。
    """
    if params_dir is not None:
        d = Path(params_dir)
        yaml_files = (d / "simulator_model.param.yaml", d / "vehicle_info.param.yaml")
    else:
        yaml_files = (_SIM_YAML, _INFO_YAML)
    params: dict = {}
    for yaml_path in yaml_files:
        try:
            import yaml  # PyYAML or ruamel.yaml

            with open(yaml_path, encoding="utf-8") as f:
                doc = yaml.safe_load(f)
            # ROS 2 YAML: /**:/ros__parameters/ 以下がフラット
            ros_p = (
                doc.get("/**", {}).get("ros__parameters", {})
                or doc.get("/*", {}).get("ros__parameters", {})
                or {}
            )
            params.update(ros_p)
        except Exception:
            pass

    # フォールバック（YAML 読み取り失敗時）
    defaults = {
        "vehicle_model_type": "DELAY_STEER_ACC_GEARED_WO_FALL_GUARD",
        "acc_time_delay": 0.101,
        "acc_time_constant": 0.2589,
        "brake_delay": 0.0685,
        "brake_time_constant": 0.15,
        "steer_time_delay": 0.0315,
        "steer_time_constant": 0.4983,
        "steer_dead_band": 0.0,
        "steer_bias": 0.0005,
        "vel_lim": 50.0,
        "vel_rate_lim": 7.0,
        "steer_lim": 1.0,
        "steer_rate_lim": 5.0,
        "wheel_base": 4.76012,
        "max_steer_angle": 0.640,
    }
    for k, v in defaults.items():
        params.setdefault(k, v)

    return params
