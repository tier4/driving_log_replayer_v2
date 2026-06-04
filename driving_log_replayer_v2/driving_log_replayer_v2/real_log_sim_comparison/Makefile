# real_log_sim_comparison use case 専用 Makefile。
# 同ディレクトリ配下に Python module / evaluator_node / README / sample をすべて集約しており、
# `make local_cloud_run` で 10 段階パイプライン (実機ログ抽出 → scenario 自動生成 →
# sim 実行 (sim_runs.yaml) → 比較解析 → N-step オープンループ → 集約) をワンコマンド実行する。

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

.PHONY: help local_cloud_run local_analysis_run local_multidataset_run

help:
	@echo "使い方: make -C $(MAKEFILE_DIR) <ターゲット>"
	@echo ""
	@echo "ターゲット:"
	@echo "  local_cloud_run     webauto キャッシュの T4 dataset を使い 10 段階パイプラインをワンコマンド実行"
	@echo "  local_analysis_run  既存出力 (OUT_DIR; 既定 sample/out/latest) の sim 実行結果を再利用し、"
	@echo "                      解析ステージ (Stage 4〜11) だけを再実行 (解析コード変更後の作り直し用)"
	@echo "  local_multidataset_run  収集済み複数 real.lite (COLLECTION_DIR; 既定 sample/multi) で"
	@echo "                      open-loop N-step rollout を dataset 横断集約 (SEARCH=1 でロバスト同定も実行)"
	@echo ""
	@echo "  パイプライン構成 (Stage 1〜6):"
	@echo "    1. step1_make_lite        実機 input_bag → lite/real.lite/"
	@echo "    2. step2_bag_to_scenario  lite/real.lite → scenarios/auto_scenario.yaml"
	@echo "    3. step3_run_sims         scenario_test_runner で sim_runs.yaml の各 run を実行 → lite/<tag>.lite/"
	@echo "    4. step4_compare_logs     real + 全 sim run の lite を N-way 比較 (figures/, report.md)"
	@echo "    5. step5_analyze_nstep    cases.yaml の各 case (VehicleModel パラメータ) で N-step オープンループ解析"
	@echo "    6. step6_analyze_cases    nstep/*.csv を集約 (overlay/, cases_summary.md)"
	@echo "    7. step7_sweep_params     real.lite で車両モデルパラメータを rollout sweep 同定 (param_sweep/)"
	@echo "    8. step8_compare_dp_trajectory  DiffusionPlanner 出力軌跡 real vs sim 比較 (figures/dp_*.png)"
	@echo "    9. step9_identify_brake   real.lite で縦方向 brake_tc を発進フィット同定 (brake_sweep/)"
	@echo "    10. step10_diagnose_curve カーブ/発進区間の乖離を縦横分解診断 (curve_diag/)"
	@echo "    11. step11_build_html_report comparison/ 配下の全プロットを集約 (result_archive/real_log_sim_comparison/index.html)"
	@echo ""
	@echo "変数:"
	@echo "  WS_ROOT          colcon ワークスペースルート (デフォルト: $(WS_ROOT))"
	@echo "  WEBAUTO_T4_ROOT  webauto キャッシュルート (デフォルト: $(WEBAUTO_T4_ROOT))"
	@echo "  LOCAL_SCENARIO   ローカル実行する scenario.yaml (デフォルト: $(LOCAL_SCENARIO))"
	@echo "  OUT_DIR          local_analysis_run の対象出力ディレクトリ (デフォルト: sample/out/latest)"

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

# 既存の out/<timestamp> (lite/ に real + sim run が揃ったバンドル) を入力に、
# 解析ステージ (Stage 4〜11) だけを再実行する。Stage 1〜3 (lite 抽出・scenario 生成・
# sim 実行) をスキップするため、解析コードの変更を数分で結果に反映できる。
# 地図は local_cloud_run と同じく scenario.yaml の Datasets UUID から webauto キャッシュを
# 解決する (見つからなければ地図背景なしで続行)。
OUT_DIR ?= $(LOCAL_SAMPLE_DIR)/out/latest

local_analysis_run:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	@test -d "$(OUT_DIR)" || \
	    { echo "[ERROR] OUT_DIR が見つかりません: $(OUT_DIR)"; exit 1; }
	@map_osm="" ; \
	    uuid=$$($(PYTHON) -c "import yaml; d=yaml.safe_load(open('$(LOCAL_SCENARIO)')); print(list(d['Evaluation']['Datasets'][0].keys())[0])" 2>/dev/null) ; \
	    if [ -n "$$uuid" ] ; then \
	        frame_dir=$$($(PYTHON) $(MAKEFILE_DIR)/lib/_dataset.py --root "$(WEBAUTO_T4_ROOT)" --uuid "$$uuid" 2>/dev/null) \
	            && test -f "$$frame_dir/map/lanelet2_map.osm" \
	            && map_osm="$$frame_dir/map/lanelet2_map.osm" ; \
	    fi ; \
	    if [ -z "$$map_osm" ] ; then echo "[WARN] 地図を解決できないため地図背景なしで実行します" ; fi ; \
	    echo "[INFO] OUT_DIR: $(OUT_DIR)" ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.run_analysis \
	        --bundle-dir "$(OUT_DIR)" \
	        --scenario "$(LOCAL_SCENARIO)" \
	        --map-osm "$$map_osm"

# 複数データセット横断の open-loop ロバスト評価・同定。
# クラウドは 1 評価ジョブ = 1 データセットのため、各ジョブが出力する real.lite を
# collect_real_lite.py で COLLECTION_DIR に収集してから本ターゲットで dataset 横断集約する。
#   収集例: $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.collect_real_lite \
#              --bundle sample/out/<tsA> --bundle sample/out/<tsB>
# SEARCH=1 でロバスト best_normal の結合探索 (coordinate descent + 直積精密化) も実行する。
COLLECTION_DIR ?= $(LOCAL_SAMPLE_DIR)/multi

local_multidataset_run:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -d "$(COLLECTION_DIR)" || \
	    { echo "[ERROR] COLLECTION_DIR が見つかりません: $(COLLECTION_DIR) (先に collect_real_lite.py で収集してください)"; exit 1; }
	@echo "[INFO] COLLECTION_DIR: $(COLLECTION_DIR)" ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune \
	        --collection-dir "$(COLLECTION_DIR)" \
	        --cases-config "$(LOCAL_SAMPLE_DIR)/cases.yaml" \
	        $(if $(SEARCH),--search,)
