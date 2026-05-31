#!/usr/bin/env bash
# real_log_sim_comparison: 指定 dataset でローカル再現実行するヘルパ。
#
# 目的:
#   - クラウドで観測した挙動 (例: ego 静止) を make local_cloud_run でローカル再現する。
#   - 非対話シェル (TTY なし) では ros2 launch が spurious な SIGINT を受けて起動 ~8秒で
#     落ちるため、`script` で疑似端末(PTY)を割り当てて回避する。
#   - 連続実行で残る /tmp の中間生成物 (converted_scenario / sim_runner_*) を毎回掃除する。
#
# 使い方:
#   ./repro_local_sim.sh [DATASET_UUID] [SIM_RUNS_CSV]
#     DATASET_UUID : 既定 67db293e-903e-4397-b603-be6b88d98be7
#     SIM_RUNS_CSV : 回す sim tag をカンマ区切りで (既定 sim_normal)。
#                    例: sim_normal,sim_perfect  (vehicle_model は tag から自動: sim_godot のみ _godot)
#
# 結果: sample/out/latest/ に result.jsonl / result_archive/ が出力される。
set -euo pipefail

UUID="${1:-67db293e-903e-4397-b603-be6b88d98be7}"
SIM_RUNS_CSV="${2:-sim_normal}"

RLSC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPRO_DIR="/tmp/repro"

echo "[repro] dataset=${UUID} sim_runs=${SIM_RUNS_CSV}"

# --- 1. 前回実行の残骸を掃除 (preprocessor の stale file / 取り残しプロセス) ---
pkill -9 -f "scenario_test_runner|autoware_diffusion_planner|openscenario_interpreter|component_container|simple_sensor_simulator|rosbag2" 2>/dev/null || true
sleep 2
rm -rf /tmp/converted_scenario /tmp/sim_runner_* "${REPRO_DIR}"
mkdir -p "${REPRO_DIR}"

# --- 2. 再現用 scenario / cases / sim_runs を生成 ---
cat > "${REPRO_DIR}/scenario.yaml" <<EOF
ScenarioFormatVersion: 3.0.0
ScenarioName: real_log_sim_comparison
ScenarioDescription: "local repro for ${UUID}"
SensorModel: aip_x2_gen2
VehicleModel: j6_gen2
Evaluation:
  UseCaseName: planning_control
  SubUseCaseName: real_log_sim_comparison
  UseCaseFormatVersion: 0.1.0
  Datasets:
    - ${UUID}:
        VehicleId: default
  Conditions:
    scenario_name: "repro ${UUID}"
    curve_config_yaml: ""
    cases_config: ./cases.yaml
    sim_runs_config: ./sim_runs.yaml
EOF

cat > "${REPRO_DIR}/cases.yaml" <<'EOF'
cases:
  - tag: baseline
    vehicle_model: delay_steer_acc_geared_wo_fall_guard
    params:
      wheelbase: 4.76012
overlay:
  reference_tag: baseline
  plots: [cascade_error, error_timeseries]
EOF

# sim_runs.yaml を SIM_RUNS_CSV から生成 (tag が *godot なら godot バイナリ指定)
{
  echo "sim_runs:"
  IFS=',' read -ra TAGS <<< "${SIM_RUNS_CSV}"
  for tag in "${TAGS[@]}"; do
    case "$tag" in
      *godot)   vm="j6_gen2_godot" ;;
      *perfect) vm="j6_gen2_perfect_tracker" ;;
      *)        vm="j6_gen2" ;;
    esac
    echo "  - tag: ${tag}"
    echo "    vehicle_model: ${vm}"
    echo "    architecture_type: awf/universe/20250130"
    [ "$vm" = "j6_gen2_godot" ] && echo "    godot_executable: /opt/godot_autoware_simulator/godot_autoware_simulator.x86_64"
  done
} > "${REPRO_DIR}/sim_runs.yaml"

# --- 3. PTY 経由で make local_cloud_run を実行 (SIGINT 回避) ---
echo "[repro] launching make local_cloud_run under PTY ..."
script -qec "make -C '${RLSC_DIR}' local_cloud_run LOCAL_SCENARIO='${REPRO_DIR}/scenario.yaml'" "${REPRO_DIR}/run.log"

echo
echo "[repro] === 完了。結果サマリ ==="
LATEST="${RLSC_DIR}/sample/out/latest"
echo "  出力: ${LATEST}"
[ -f "${LATEST}/result.jsonl" ] && { echo -n "  result.jsonl: "; cat "${LATEST}/result.jsonl"; echo; }
REPORT="${LATEST}/result_archive/comparison/report.md"
[ -f "${REPORT}" ] && { echo "  --- report.md (速度統計) ---"; sed -n '/速度統計/,/速度 RMSE/p' "${REPORT}"; }
echo "  生成 scenario の start/goal:"
grep -E "\[step2_bag_to_scenario\] (start|goal)=" "${REPRO_DIR}/run.log" | tail -2 || true
