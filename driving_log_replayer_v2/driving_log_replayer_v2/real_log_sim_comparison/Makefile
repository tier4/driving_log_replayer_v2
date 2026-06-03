# real_log_sim_comparison use case 専用 Makefile。
# 同ディレクトリ配下に Python module / evaluator_node / README / sample をすべて集約しており、
# `make local_cloud_run` で 10 段階パイプライン (実機ログ抽出 → scenario 自動生成 →
# sim 実行 (sim_runs.yaml) → 比較解析 → per-step → 集約) をワンコマンド実行する。

SHELL := /bin/bash

MAKEFILE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
# colcon ワークスペース root = 6 階層上
# (.../<ws>/src/simulator/driving_log_replayer_v2/driving_log_replayer_v2/driving_log_replayer_v2/real_log_sim_comparison)
WS_ROOT ?= $(abspath $(MAKEFILE_DIR)/../../../../../..)
SETUP_BASH := $(WS_ROOT)/install/setup.bash

# local_cloud_run: webauto キャッシュの T4 dataset を使ったクラウド相当ローカル実行
# webauto data annotation-dataset pull の出力先は ~/.webauto/data/data/annotation_dataset/<UUID>/<frame>/。
# UUID/frame から t4_dataset_path を解決するロジックは lib/_dataset.py (SSOT) に集約。
# クラウド (Web.Auto) は dataset を固定マウントに事前ステージし同じ t4_dataset_path 契約
# (input_bag/ と map/ を直下に持つ dir) を渡すため、解決方法のみがローカルと異なる。
WEBAUTO_T4_ROOT  ?= $(HOME)/.webauto/data/data/annotation_dataset
LOCAL_SAMPLE_DIR ?= $(MAKEFILE_DIR)/sample
LOCAL_SCENARIO   ?= $(LOCAL_SAMPLE_DIR)/scenario.yaml

PYTHON := python3

.PHONY: help local_cloud_run

help:
	@echo "使い方: make -C $(MAKEFILE_DIR) <ターゲット>"
	@echo ""
	@echo "ターゲット:"
	@echo "  local_cloud_run  webauto キャッシュの T4 dataset を使い 10 段階パイプラインをワンコマンド実行"
	@echo ""
	@echo "  パイプライン構成 (Stage 1〜6):"
	@echo "    1. step1_make_lite        実機 input_bag → lite/real.lite/"
	@echo "    2. step2_bag_to_scenario  lite/real.lite → scenarios/auto_scenario.yaml"
	@echo "    3. step3_run_sims         scenario_test_runner で sim_runs.yaml の各 run を実行 → lite/<tag>.lite/"
	@echo "    4. step4_compare_logs     real + 全 sim run の lite を N-way 比較 (figures/, report.md)"
	@echo "    5. step5_analyze_per_step cases.yaml の各 case (VehicleModel パラメータ) で per-step delta 解析"
	@echo "    6. step6_analyze_cases    per_step/*.csv を集約 (overlay/, cases_summary.md)"
	@echo "    7. step7_identify_kus     real.lite で k_us を rollout sweep 同定 (kus_sweep/)"
	@echo "    8. step8_compare_dp_trajectory  DiffusionPlanner 出力軌跡 real vs sim 比較 (figures/dp_*.png)"
	@echo "    9. step9_identify_brake   real.lite で縦方向 brake_tc を発進フィット同定 (brake_sweep/)"
	@echo "    10. step10_diagnose_curve カーブ/発進区間の乖離を縦横分解診断 (curve_diag/)"
	@echo "    11. step11_build_html_report comparison/ 配下の全プロットを集約 (result_archive/real_log_sim_comparison/index.html)"
	@echo ""
	@echo "変数:"
	@echo "  WS_ROOT          colcon ワークスペースルート (デフォルト: $(WS_ROOT))"
	@echo "  WEBAUTO_T4_ROOT  webauto キャッシュルート (デフォルト: $(WEBAUTO_T4_ROOT))"
	@echo "  LOCAL_SCENARIO   ローカル実行する scenario.yaml (デフォルト: $(LOCAL_SCENARIO))"

# クラウド (Web.Auto evaluator) と同じ scenario.yaml ベースで、
# webauto data annotation-dataset pull --include-intermediate-artifacts で
# ローカルに取得済みの T4 dataset を使って real_log_sim_comparison を実行する。
local_cloud_run:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	@uuid=$$($(PYTHON) -c "import yaml; d=yaml.safe_load(open('$(LOCAL_SCENARIO)')); print(list(d['Evaluation']['Datasets'][0].keys())[0])") ; \
	    test -n "$$uuid" || { echo "[ERROR] scenario.yaml から Datasets UUID を取得できませんでした"; exit 1; } ; \
	    echo "[INFO] Dataset UUID: $$uuid" ; \
	    frame_dir=$$($(PYTHON) $(MAKEFILE_DIR)/lib/_dataset.py --root "$(WEBAUTO_T4_ROOT)" --uuid "$$uuid") || exit 1 ; \
	    echo "[INFO] T4 dataset path: $$frame_dir" ; \
	    out_dir=$(LOCAL_SAMPLE_DIR)/out/$$(date +%Y%m%d_%H%M%S) ; \
	    mkdir -p "$$out_dir" ; \
	    ln -snf "$$(basename $$out_dir)" $(LOCAL_SAMPLE_DIR)/out/latest ; \
	    echo "[INFO] Output: $$out_dir" ; \
	    source $(SETUP_BASH) && ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
	        scenario_path:=$(LOCAL_SCENARIO) \
	        t4_dataset_path:="$$frame_dir" \
	        t4_dataset_id:="$$uuid" \
	        output_dir:="$$out_dir" \
	        with_autoware:=false
