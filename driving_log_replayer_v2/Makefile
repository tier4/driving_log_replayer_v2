# driving_log_replayer_v2/Makefile
# real_log_sim_comparison: 6 段階パイプライン (実機ログ抽出 → scenario 自動生成 →
# sim 実行 (sim_runs.yaml) → 比較解析 → per-step → 集約) を make local_cloud_run に統一。
#
# 旧 make sim_normal / sim_godot / sim / all / _all / sim_curve2 / clean / real /
# report / report_curve2 / report_per_step / report_dp_trajectory / report_departure
# は make local_cloud_run に集約された (Stage 3 の sim_runs.yaml に sim 種別を記述)。

SHELL := /bin/bash

MAKEFILE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
WS_ROOT ?= $(abspath $(MAKEFILE_DIR)/../../../..)
SETUP_BASH := $(WS_ROOT)/install/setup.bash

# local_cloud_run: webauto キャッシュの T4 dataset を使ったクラウド相当ローカル実行
# webauto data annotation-dataset pull の出力先は ~/.webauto/data/data/annotation_dataset/<UUID>/<frame>/。
WEBAUTO_T4_ROOT  ?= $(HOME)/.webauto/data/data/annotation_dataset
LOCAL_SAMPLE_DIR ?= $(MAKEFILE_DIR)/../sample/real_log_sim_comparison_local
LOCAL_SCENARIO   ?= $(LOCAL_SAMPLE_DIR)/scenario.yaml

PYTHON := python3

.PHONY: help local_cloud_run

help:
	@echo "使い方: make -C $(MAKEFILE_DIR) <ターゲット>"
	@echo ""
	@echo "ターゲット:"
	@echo "  local_cloud_run  webauto キャッシュの T4 dataset を使い 6 段階パイプラインをワンコマンド実行"
	@echo ""
	@echo "  パイプライン構成 (Stage 1〜6):"
	@echo "    1. make_lite       実機 input_bag → lite/real.lite/"
	@echo "    2. bag_to_scenario lite/real.lite → scenarios/auto_scenario.yaml"
	@echo "    3. run_sims        scenario_test_runner で sim_runs.yaml の各 run を実行 → lite/<tag>.lite/"
	@echo "    4. compare_logs    real + 全 sim run の lite を N-way 比較 (figures/, report.md)"
	@echo "    5. analyze_per_step cases.yaml の各 case (VehicleModel パラメータ) で per-step delta 解析"
	@echo "    6. analyze_cases   per_step/*.csv を集約 (overlay/, cases_summary.md)"
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
	    dataset_root=$$(find $(WEBAUTO_T4_ROOT) -type d -name "$$uuid" -not -path '*/.lock/*' 2>/dev/null | head -1) ; \
	    if [ -z "$$dataset_root" ]; then \
	        echo "[ERROR] dataset $$uuid が $(WEBAUTO_T4_ROOT) に見つかりません。" ; \
	        echo "        次のコマンドで取得してください:" ; \
	        echo "          webauto data annotation-dataset pull \\" ; \
	        echo "            --project-id x2_dev \\" ; \
	        echo "            --annotation-dataset-id $$uuid \\" ; \
	        echo "            --include-intermediate-artifacts" ; \
	        exit 1 ; \
	    fi ; \
	    frame_dir=$$(find "$$dataset_root" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -1) ; \
	    test -n "$$frame_dir" || { echo "[ERROR] frame ディレクトリが $$dataset_root の下に見つかりません"; exit 1; } ; \
	    echo "[INFO] T4 dataset path: $$frame_dir" ; \
	    if ! find "$$frame_dir/input_bag" -maxdepth 1 \( -name '*.mcap' -o -name '*.db3' \) 2>/dev/null | grep -q . ; then \
	        echo "[ERROR] $$frame_dir/input_bag/ に *.mcap / *.db3 が見つかりません。" ; \
	        echo "        --include-intermediate-artifacts を付けて再 pull してください:" ; \
	        echo "          webauto data annotation-dataset pull \\" ; \
	        echo "            --project-id x2_dev \\" ; \
	        echo "            --annotation-dataset-id $$uuid \\" ; \
	        echo "            --include-intermediate-artifacts" ; \
	        exit 1 ; \
	    fi ; \
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
