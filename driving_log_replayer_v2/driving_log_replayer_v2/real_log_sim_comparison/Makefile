# real_log_sim_comparison / Makefile
# 実機 / Godot シム / 通常シム の三方比較ワークフロー
#
# 前提:
#   - 実機 lite ($(LITE_DIR)/real.lite.mcap) は手動で事前生成済みであること
#     (make_lite.py 手順を参照)
#   - ワークスペースは事前にビルド済みで $(SETUP_BASH) が存在すること
#
# 出力先は環境変数 BEST_MODEL_BASE_DIR で上書き可能。
# デフォルト: <pkg>/../../real_log_sim_comparison_work （DLR2 リポ内の作業ディレクトリ）

SHELL                := /bin/bash
PKG_DIR              := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
WS_ROOT              := $(abspath $(PKG_DIR)/../../../../../..)
BEST_MODEL_BASE_DIR  ?= $(abspath $(PKG_DIR)/../../real_log_sim_comparison_work)
export BEST_MODEL_BASE_DIR

LITE_DIR             := $(BEST_MODEL_BASE_DIR)/lite
COMP_DIR             := $(BEST_MODEL_BASE_DIR)/comparison
LOG_DIR              := $(BEST_MODEL_BASE_DIR)/logs
SETUP_BASH           := $(WS_ROOT)/install/setup.bash
GODOT_EXECUTABLE     := $(HOME)/Downloads/godot_autoware_simulator.x86_64

PY_BASE              := python3 -m driving_log_replayer_v2.real_log_sim_comparison

# シミュレータ出力ルートディレクトリ (scenario_test_runner.py のデフォルト出力先)
SIM_OUT_ROOT         := /tmp/scenario_test_runner

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
	    source $(SETUP_BASH) && $(PY_BASE).make_lite --kind sim --input "$$_bag" --output $(2) ; \
	    _rc=$$? ; rm -rf $(SIM_OUT_ROOT) ; exit $$_rc
endef

.PHONY: all _all sim sim_normal sim_godot sim_curve2 report report_curve2 clean help

help:
	@echo "使い方: make <ターゲット>"
	@echo ""
	@echo "ターゲット:"
	@echo "  all           sim → report を実行し $$BEST_MODEL_BASE_DIR/logs/ にタイムスタンプ付きログを保存"
	@echo "  sim_normal    通常シム (best_model) を実行し lite/sim_normal.lite.mcap を更新"
	@echo "  sim_godot     Godot シム (j6_gen2_godot + simulator:=godot) を実行し lite/sim_godot.lite.mcap を更新"
	@echo "  sim           sim_normal と sim_godot を順次実行"
	@echo "  sim_curve2    カーブ② 集中シナリオを実行し lite/sim_curve2.lite.mcap を更新"
	@echo "  report        compare_logs (三方比較) を実行"
	@echo "  report_curve2 カーブ② 比較レポート (analyze_real_curve2 → compare_curve2 → analyze_curve2_per_step)"
	@echo "  clean         lite/sim_*.lite.mcap と comparison/ を削除（real・logs は保持）"
	@echo ""
	@echo "環境変数:"
	@echo "  BEST_MODEL_BASE_DIR  出力先ベースディレクトリ (現在: $(BEST_MODEL_BASE_DIR))"

all:
	@mkdir -p $(LOG_DIR)
	@_log="$(LOG_DIR)/$$(date +%Y%m%d_%H%M%S).log"; \
	    echo "=== ログ保存先: $$_log ==="; \
	    $(MAKE) --no-print-directory _all 2>&1 | tee "$$_log"; \
	    exit $${PIPESTATUS[0]}

_all: sim report

sim: sim_normal sim_godot

sim_normal:
	$(call run_sim,best_model,$(LITE_DIR)/sim_normal.lite.mcap)

sim_godot:
	$(call run_sim,j6_gen2_godot,$(LITE_DIR)/sim_godot.lite.mcap,godot_executable:=$(GODOT_EXECUTABLE))

sim_curve2:
	$(call run_sim,best_model,$(LITE_DIR)/sim_curve2.lite.mcap,,x2_dev_curve2_start.yaml)

report_curve2:
	@test -f $(LITE_DIR)/sim_curve2.lite.mcap || \
	    { echo "[ERROR] $(LITE_DIR)/sim_curve2.lite.mcap がありません。'make sim_curve2' を実行してください"; exit 1; }
	source $(SETUP_BASH) && $(PY_BASE).analyze_real_curve2 && \
	    $(PY_BASE).compare_curve2 && \
	    $(PY_BASE).analyze_curve2_per_step

report:
	@test -f $(LITE_DIR)/real.lite.mcap || \
	    { echo "[ERROR] $(LITE_DIR)/real.lite.mcap がありません。実機 lite を作成してください"; exit 1; }
	@test -f $(LITE_DIR)/sim_normal.lite.mcap || \
	    { echo "[ERROR] $(LITE_DIR)/sim_normal.lite.mcap がありません。'make sim_normal' を実行してください"; exit 1; }
	@test -f $(LITE_DIR)/sim_godot.lite.mcap || \
	    { echo "[ERROR] $(LITE_DIR)/sim_godot.lite.mcap がありません。'make sim_godot' を実行してください"; exit 1; }
	source $(SETUP_BASH) && $(PY_BASE).compare_logs

clean:
	rm -f $(LITE_DIR)/sim_godot.lite.mcap $(LITE_DIR)/sim_normal.lite.mcap
	rm -rf $(COMP_DIR)
