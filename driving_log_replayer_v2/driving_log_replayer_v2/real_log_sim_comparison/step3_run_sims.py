"""Stage 3: 1 sim run の実行 (scenario_test_runner.launch.py 起動 + step1_make_lite --kind sim).

sim_runs.yaml の tag に対応する設定を読み、ros2 launch を subprocess で起動し、
出力 MCAP を step1_make_lite で lite 化する。

Usage:
    python3 -m driving_log_replayer_v2.real_log_sim_comparison.step3_run_sims \\
        --run-tag sim_normal \\
        --scenario <auto_scenario.yaml> \\
        --sim-runs-config <sim_runs.yaml> \\
        --output-lite <lite/sim_normal.lite>

注意:
- nested ros2 launch (外側 evaluator_node の中で本ツールが内側 scenario_test_runner)
  になる場合、呼び出し側 (evaluator_node) で ROS_DOMAIN_ID を切り替えて DDS 競合回避。
- scenario_test_runner.launch.py の output_directory:= 引数が効かない可能性に備え、
  /tmp/scenario_test_runner/ もフォールバック検索する。
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


# scenario_test_runner.launch.py が output_directory 引数を無視する場合の既定出力先
_FALLBACK_SIM_OUT_ROOT = Path("/tmp/scenario_test_runner")


def _build_launch_cmd(run, scenario: Path, output_directory: Path) -> list[str]:
    """scenario_test_runner.launch.py の ros2 launch 起動コマンドを組み立てる.

    注: rviz の起動は Autoware (planning_simulator.launch.xml) 側のメカニズムで、
    scenario_test_runner.launch.py の launch_rviz 引数とは独立に制御される。
    ここでは追加引数を渡さず、Autoware の default 動作に任せる。
    """
    cmd = [
        "ros2", "launch", "scenario_test_runner", "scenario_test_runner.launch.py",
        f"vehicle_model:={run.vehicle_model}",
        f"sensor_model:={run.sensor_model}",
        "record:=true",
        "record_storage_id:=mcap",
        f"architecture_type:={run.architecture_type}",
        f"initialize_duration:={run.initialize_duration}",
        f"scenario:={scenario}",
        f"output_directory:={output_directory}",
    ]
    if run.godot_executable:
        cmd.append(f"godot_executable:={run.godot_executable}")
    # params: simulator_model パラメータ上書き。
    # scenario_test_runner.launch.py が simple_sensor_simulator. 接頭辞の launch 引数を
    # 収集し、simulator_model.param.yaml の後ろに連結する (後勝ち) ため、
    # description パッケージを増やさずに dynamics 変種 (例 k_us) を作れる。
    for key, value in run.params.items():
        cmd.append(f"simple_sensor_simulator.{key}:={value}")
    return cmd


def _find_output_mcap(search_dirs: list[Path]) -> Path | None:
    """指定ディレクトリ群から最新の *.mcap を 1 つ返す."""
    candidates: list[Path] = []
    for d in search_dirs:
        if d.exists():
            candidates.extend(d.rglob("*.mcap"))
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def _run_subprocess(cmd: list[str], timeout: int, env: dict | None = None) -> None:
    """subprocess.run の薄いラッパ. 非 0 終了で RuntimeError."""
    print(f"[step3_run_sims] $ {' '.join(cmd)}", flush=True)
    try:
        result = subprocess.run(cmd, check=False, env=env, timeout=timeout)
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f"timeout {timeout}s: {' '.join(cmd)}") from e
    if result.returncode != 0:
        raise RuntimeError(
            f"command failed (rc={result.returncode}): {' '.join(cmd)}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Stage 3: 1 sim run の実行")
    parser.add_argument("--run-tag", required=True,
                        default=os.environ.get("SIM_RUN_TAG"),
                        help="sim_runs.yaml の対象 tag (env: SIM_RUN_TAG)")
    parser.add_argument("--scenario", required=True,
                        help="OpenSCENARIO yaml パス (Stage 2 で生成)")
    parser.add_argument("--sim-runs-config", required=True,
                        default=os.environ.get("SIM_RUNS_CONFIG_YAML"),
                        help="sim_runs.yaml パス (env: SIM_RUNS_CONFIG_YAML)")
    parser.add_argument("--output-lite", required=True,
                        help="出力先 lite/<tag>.lite/ ディレクトリ")
    parser.add_argument("--reproduce-bag",
                        default=os.environ.get("REPRODUCE_BAG", ""),
                        help="指定すると実機 input_bag の先行車を ego-pose 同期で sim に注入する "
                             "perception_reproducer ノードを sim と並走起動する (env: REPRODUCE_BAG)。"
                             "空なら注入しない。")
    args = parser.parse_args()

    scenario = Path(args.scenario)
    output_lite = Path(args.output_lite)
    output_lite.parent.mkdir(parents=True, exist_ok=True)

    if not scenario.exists():
        print(f"ERROR: scenario yaml が見つかりません: {scenario}", file=sys.stderr)
        sys.exit(1)

    from .lib._sim_runs_config import load_sim_runs_config  # noqa: PLC0415
    try:
        sim_cfg = load_sim_runs_config(args.sim_runs_config)
        run = sim_cfg.find_run(args.run_tag)
    except (FileNotFoundError, ValueError, KeyError) as e:
        print(f"ERROR: sim_runs.yaml: {e}", file=sys.stderr)
        sys.exit(2)

    print(f"[step3_run_sims] tag={run.tag}, vehicle_model={run.vehicle_model}, "
          f"sensor_model={run.sensor_model}, timeout={run.timeout_s}s")

    # 一時 output_directory を作る (隔離 + 衝突回避)
    tmp_root = Path(tempfile.mkdtemp(
        prefix=f"sim_runner_{run.tag}_", dir="/tmp"))
    print(f"[step3_run_sims] tmp output_directory: {tmp_root}")

    reproducer_proc: subprocess.Popen | None = None
    try:
        # perception reproducer を sim と並走起動 (実機先行車を ego-pose 同期注入)。
        # rclpy が late publisher を扱うため sim より先に起動して問題ない (起動時に bag を読み、
        # sim ego topic が出たら購読開始)。同一 ROS_DOMAIN_ID は os.environ 継承で揃う。
        if args.reproduce_bag and Path(args.reproduce_bag).exists():
            repro_cmd = [
                sys.executable, "-m",
                "driving_log_replayer_v2.real_log_sim_comparison.perception_reproducer_node",
                "--bag", str(args.reproduce_bag),
            ]
            print(f"[step3_run_sims] $ {' '.join(repro_cmd)} (並走)", flush=True)
            reproducer_proc = subprocess.Popen(repro_cmd)  # noqa: S603
        elif args.reproduce_bag:
            print(f"[step3_run_sims] WARN: reproduce_bag が見つかりません: {args.reproduce_bag} "
                  "(perception 注入をスキップ)", file=sys.stderr)

        cmd = _build_launch_cmd(run, scenario.resolve(), tmp_root)
        _run_subprocess(cmd, timeout=run.timeout_s)

        # output_directory が効かない可能性に備え両方検索
        mcap = _find_output_mcap([tmp_root, _FALLBACK_SIM_OUT_ROOT])
        if mcap is None:
            raise RuntimeError(
                f"sim 出力 *.mcap が {tmp_root} / {_FALLBACK_SIM_OUT_ROOT} に見つかりません"
            )
        print(f"[step3_run_sims] sim output mcap: {mcap}")

        # step1_make_lite --kind sim で lite 化
        make_lite_cmd = [
            sys.executable, "-m",
            "driving_log_replayer_v2.real_log_sim_comparison.step1_make_lite",
            "--kind", "sim",
            "--input", str(mcap),
            "--output", str(output_lite),
        ]
        _run_subprocess(make_lite_cmd, timeout=300)

        # この sim run が使った DP モデル重み / autoware バージョンを記録 (版差の解釈用)。
        # capture は sim 実行直後 (= 使用した onnx がまだ /opt にある状態) に行う。
        from .lib._provenance import write_provenance  # noqa: PLC0415
        prov = write_provenance(output_lite, extra={
            "tag": run.tag,
            "vehicle_model": run.vehicle_model,
            "architecture_type": run.architecture_type,
            "params": run.params,
        })
        print(f"[step3_run_sims] provenance: DP={prov.get('dp_exp_name')} "
              f"(onnx {prov.get('dp_onnx_sha8')}) / autoware {prov.get('autoware_version')}")

        print(f"[step3_run_sims] Saved: {output_lite}")

    finally:
        if reproducer_proc is not None and reproducer_proc.poll() is None:
            reproducer_proc.terminate()
            try:
                reproducer_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                reproducer_proc.kill()
        shutil.rmtree(tmp_root, ignore_errors=True)
        # フォールバック先もケース毎にクリア (次 run 干渉防止)
        shutil.rmtree(_FALLBACK_SIM_OUT_ROOT, ignore_errors=True)


if __name__ == "__main__":
    main()
