# driving_log_replayer_v2/Makefile
# real_log_sim_comparison: 実機 / Godot シム / 通常シム の三方比較ワークフロー
#
# 前提:
#   - colcon build 済みで source install/setup.bash が完了していること
#   - 実機 lite (LITE_DIR/real.lite.mcap) は手動で事前生成済みであること
#     または make real を実行して生成すること
#
# 使い方: make -f Makefile <ターゲット>  (DLRv2 リポジトリルートから)
#         make -C <dlrv2_repo_root> <ターゲット>

SHELL := /bin/bash

MAKEFILE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
# WS_ROOT: colcon ワークスペースルート。環境変数で上書き可能。
WS_ROOT ?= $(abspath $(MAKEFILE_DIR)/../../../..)
SETUP_BASH := $(WS_ROOT)/install/setup.bash

WORK_DIR        := $(MAKEFILE_DIR)/real_log_sim_comparison_work
LITE_DIR        := $(WORK_DIR)/lite
COMP_DIR        := $(WORK_DIR)/comparison
LOG_DIR         := $(WORK_DIR)/logs
GODOT_EXECUTABLE := $(HOME)/Downloads/godot_autoware_simulator.x86_64

# シミュレータ出力ルートディレクトリ（scenario_test_runner.py のデフォルト出力先）
SIM_OUT_ROOT := /tmp/scenario_test_runner

PYTHON := python3

# $(1) = vehicle_model, $(2) = 出力 lite mcap パス, $(3) = 追加 launch 引数（省略可）, $(4) = シナリオ YAML 名（省略時: x2_dev_teleport_to_miraikan.yaml）
define run_sim
	@mkdir -p $(LITE_DIR)
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	rm -rf $(SIM_OUT_ROOT)
	source $(SETUP_BASH) && ros2 launch scenario_test_runner scenario_test_runner.launch.py \
	    vehicle_model:=$(1) \
	    sensor_model:=aip_x2_gen2 \
	    record:=true \
	    record_storage_id:=mcap \
	    architecture_type:=awf/universe/20250130 \
	    initialize_duration:=100 \
	    scenario:=$$(ros2 pkg prefix --share scenario_test_runner)/scenario/$(if $(4),$(4),x2_dev_teleport_to_miraikan.yaml) \
	    $(3)
	@_bag=$$(find $(SIM_OUT_ROOT) -name "*.mcap" | head -1) ; \
	    test -n "$$_bag" || { echo "[ERROR] シム出力 (*.mcap) が $(SIM_OUT_ROOT) に見つかりません"; exit 1; } ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.make_lite \
	        --kind sim --input "$$_bag" --output $(2) ; \
	    _rc=$$? ; rm -rf $(SIM_OUT_ROOT) ; exit $$_rc
endef

.PHONY: all _all real sim sim_normal sim_godot sim_curve2 report report_curve2 clean help

help:
	@echo "使い方: make -f Makefile <ターゲット>"
	@echo ""
	@echo "ターゲット:"
	@echo "  all         sim → report を実行し logs/ にタイムスタンプ付きログを保存"
	@echo "  real        実機 MCAP → lite/real.lite.mcap を生成 (REAL_MCAP 変数で入力指定)"
	@echo "  sim_normal  通常シム (best_model) を実行し lite/sim_normal.lite.mcap を更新"
	@echo "  sim_godot   Godot シム (j6_gen2 + simulator:=godot) を実行し lite/sim_godot.lite.mcap を更新"
	@echo "  sim         sim_normal と sim_godot を順次実行"
	@echo "  report      compare_logs を実行してプロット・レポートを生成"
	@echo "  clean       lite/sim_*.lite.mcap と comparison/ を削除（real・logs は保持）"
	@echo ""
	@echo "変数:"
	@echo "  WS_ROOT      colcon ワークスペースルート (デフォルト: $(WS_ROOT))"
	@echo "  REAL_MCAP    実機 MCAP パス (make real 時に使用)"
	@echo "  WORK_DIR     出力ディレクトリ (デフォルト: $(WORK_DIR))"

# ログ付き一括実行
all:
	@mkdir -p $(LOG_DIR)
	@_log="$(LOG_DIR)/$$(date +%Y%m%d_%H%M%S).log"; \
	    echo "=== ログ保存先: $$_log ==="; \
	    $(MAKE) --no-print-directory _all 2>&1 | tee "$$_log"; \
	    exit $${PIPESTATUS[0]}

_all: sim report

# 実機 MCAP → real.lite.mcap（REAL_MCAP 変数で入力を指定）
real:
	@test -n "$(REAL_MCAP)" || { echo "[ERROR] REAL_MCAP 変数を指定してください: make real REAL_MCAP=/path/to/file.mcap"; exit 1; }
	@mkdir -p $(LITE_DIR)
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません"; exit 1; }
	source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.make_lite \
	    --kind real --input "$(REAL_MCAP)" --output $(LITE_DIR)/real.lite.mcap

sim: sim_normal sim_godot

sim_normal:
	$(call run_sim,best_model,$(LITE_DIR)/sim_normal.lite.mcap)

sim_godot:
	$(call run_sim,j6_gen2_godot,$(LITE_DIR)/sim_godot.lite.mcap,godot_executable:=$(GODOT_EXECUTABLE))

sim_curve2:
	$(call run_sim,best_model,$(LITE_DIR)/sim_curve2.lite.mcap,,x2_dev_curve2_start.yaml)

report:
	@test -f $(LITE_DIR)/real.lite.mcap || \
	    { echo "[ERROR] lite/real.lite.mcap がありません。'make real REAL_MCAP=...' を実行してください"; exit 1; }
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません"; exit 1; }
	source $(SETUP_BASH) && BEST_MODEL_BASE_DIR=$(WORK_DIR) \
	    $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.compare_logs

report_curve2:
	@test -f $(LITE_DIR)/sim_curve2.lite.mcap || \
	    { echo "[ERROR] lite/sim_curve2.lite.mcap がありません。'make sim_curve2' を実行してください"; exit 1; }
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません"; exit 1; }
	source $(SETUP_BASH) && BEST_MODEL_BASE_DIR=$(WORK_DIR) \
	    $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.analyze_real_curve2 && \
	    $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.compare_curve2 && \
	    $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.analyze_curve2_per_step && \
	    $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.compare_dp_trajectory

# DP 軌跡比較 (analyze_trajectory + compare_dp_vs_final の統合後継)
.PHONY: report_dp_trajectory
report_dp_trajectory:
	@test -f $(LITE_DIR)/real.lite.mcap || \
	    { echo "[ERROR] lite/real.lite.mcap がありません。'make real REAL_MCAP=...' を実行してください"; exit 1; }
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません"; exit 1; }
	source $(SETUP_BASH) && BEST_MODEL_BASE_DIR=$(WORK_DIR) \
	    $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.compare_dp_trajectory --target both

# 実機単独の発進診断 (sim 不要)
.PHONY: report_departure
report_departure:
	@test -f $(LITE_DIR)/real.lite.mcap || \
	    { echo "[ERROR] lite/real.lite.mcap がありません"; exit 1; }
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません"; exit 1; }
	source $(SETUP_BASH) && BEST_MODEL_BASE_DIR=$(WORK_DIR) \
	    $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.diagnose_departure

clean:
	rm -f $(LITE_DIR)/sim_godot.lite.mcap $(LITE_DIR)/sim_normal.lite.mcap
	rm -rf $(COMP_DIR)
