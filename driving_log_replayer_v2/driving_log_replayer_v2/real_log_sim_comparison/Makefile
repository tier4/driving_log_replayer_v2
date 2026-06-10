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

.PHONY: help local_cloud_run local_analysis_run local_batch_run local_cross_analysis_run local_multidataset_run

help:
	@echo "使い方: make -C $(MAKEFILE_DIR) <ターゲット>"
	@echo ""
	@echo "ターゲット:"
	@echo "  local_cloud_run     webauto キャッシュの T4 dataset を使いパイプラインをワンコマンド実行"
	@echo "                      (scenario.yaml の Datasets[0] のみ。複数 dataset は local_batch_run。"
	@echo "                       SKIP_SIM=1 で closed-loop sim を省略)"
	@echo "  local_analysis_run  既存出力 (OUT_DIR; 既定 sample/out/latest) の sim 実行結果を再利用し、"
	@echo "                      解析ステージ (Stage 4〜12) だけを再実行 (解析コード変更後の作り直し用)"
	@echo "  local_batch_run     scenario.yaml の Datasets 全 UUID を順次ローカル実行し、収集 +"
	@echo "                      横断分析 (Stage 13) + マルチ DS report.html まで実行"
	@echo "                      (BATCH_ROOT 既定 sample/out/batch_<ts>; RESUME=1 で再開;"
	@echo "                       SKIP_SIM=1 で closed-loop sim を省略し open-loop 解析のみ)"
	@echo "  local_cross_analysis_run  既存 collection (COLLECTION_DIR; 既定 sample/out/batch_latest) の"
	@echo "                      横断分析 (Stage 13) + マルチ DS report.html だけを再実行"
	@echo "  local_multidataset_run  収集済み複数 real.lite (COLLECTION_DIR) で open-loop ロバスト同定"
	@echo "                      (multi_dataset_tune robust_search)"
	@echo ""
	@echo "  パイプライン構成:"
	@echo "    1. step1_make_lite        実機 input_bag → lite/real.lite/"
	@echo "    2. step2_bag_to_scenario  lite/real.lite → scenarios/auto_scenario.yaml"
	@echo "    3. step3_run_sims         scenario_test_runner で sim_runs.yaml の各 run を実行 → lite/<tag>.lite/"
	@echo "    4. step4_compare_logs     real + 全 sim run の lite を N-way 比較 (figures/, report.md, metrics_closed_loop.json)"
	@echo "    5. step5_analyze_nstep    cases.yaml の各 case (VehicleModel パラメータ) で N-step オープンループ解析"
	@echo "    6. step6_analyze_cases    nstep/*.csv を集約 (overlay/, cases_summary.md, cases_metrics.json)"
	@echo "    7. step7_sweep_params     real.lite で車両モデルパラメータを rollout sweep 同定 (param_sweep/)"
	@echo "    8. step8_compare_dp_trajectory  DiffusionPlanner 出力軌跡 real vs sim 比較 (figures/dp_*)"
	@echo "    10. step10_diagnose_curve カーブ/発進区間の乖離を縦横分解診断 (curve_diag/)"
	@echo "    11. step11_build_html_report 図スペック・Markdown・設定 YAML を単一 report.html に集約"
	@echo "    12. step12_build_notebook 開発者向け report.ipynb 生成"
	@echo "    13. step13_cross_dataset  collection 横断のモデル×データセット分析 (cross_dataset/)"
	@echo ""
	@echo "変数:"
	@echo "  WS_ROOT          colcon ワークスペースルート (デフォルト: $(WS_ROOT))"
	@echo "  WEBAUTO_T4_ROOT  webauto キャッシュルート (デフォルト: $(WEBAUTO_T4_ROOT))"
	@echo "  LOCAL_SCENARIO   ローカル実行する scenario.yaml (デフォルト: $(LOCAL_SCENARIO))"
	@echo "  OUT_DIR          local_analysis_run の対象出力ディレクトリ (デフォルト: sample/out/latest)"
	@echo "  BATCH_ROOT       local_batch_run の出力 root (デフォルト: sample/out/batch_<timestamp>)"
	@echo "  COLLECTION_DIR   local_cross_analysis_run / local_multidataset_run の対象 collection"

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
	    source $(SETUP_BASH) && $(if $(SKIP_SIM),SKIP_SIM=1,) ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
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

# scenario.yaml の Datasets 全 UUID を順次ローカル実行するバッチ。各 dataset の per-dataset
# パイプライン (Stage 1〜12) → collect_datasets 収集 → Stage 13 横断分析 → マルチ DS
# report.html まで一気通貫。失敗 dataset は WARN + collection.yaml に status 記録して継続。
# RESUME=1 で real.lite が揃っている dataset の sim 実行をスキップ (中断再開用)。
# SKIP_SIM=1 で Stage 3 (closed-loop sim 実行) を全 dataset で省略する。closed-loop 比較が
# 不要なとき (open-loop N-step / sweep / カバレッジだけ欲しいとき) のバッチ時間短縮用。
# 後から closed-loop も欲しくなったら SKIP_SIM/RESUME 無しで同じ BATCH_ROOT を再実行する。
BATCH_ROOT ?= $(LOCAL_SAMPLE_DIR)/out/batch_$(shell date +%Y%m%d_%H%M%S)

local_batch_run:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	@echo "[INFO] BATCH_ROOT: $(BATCH_ROOT)" ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.run_batch \
	        --scenario "$(LOCAL_SCENARIO)" \
	        --batch-root "$(BATCH_ROOT)" \
	        --webauto-root "$(WEBAUTO_T4_ROOT)" \
	        $(if $(RESUME),--resume,) \
	        $(if $(SKIP_SIM),--skip-sim,)

# 既存 collection (local_batch_run の出力 or collect_datasets.py の手収集) の横断分析だけを
# 再実行する (Stage 13 + マルチ DS report.html)。per-dataset の sim/解析はスキップするため、
# 横断分析コードの変更を数秒で結果に反映できる (local_analysis_run の横断版)。
COLLECTION_DIR ?= $(LOCAL_SAMPLE_DIR)/out/batch_latest

local_cross_analysis_run:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -d "$(COLLECTION_DIR)" || \
	    { echo "[ERROR] COLLECTION_DIR が見つかりません: $(COLLECTION_DIR) (先に local_batch_run か collect_datasets.py で収集してください)"; exit 1; }
	@echo "[INFO] COLLECTION_DIR: $(COLLECTION_DIR)" ; \
	    source $(SETUP_BASH) \
	    && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.step13_cross_dataset \
	        --collection-dir "$(COLLECTION_DIR)" \
	    && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.step11_build_html_report \
	        --collection-dir "$(COLLECTION_DIR)"

# 複数データセット横断の open-loop ロバスト同定 (multi_dataset_tune robust_search)。
# 収集は local_batch_run か collect_datasets.py で行う:
#   $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.collect_datasets \
#       --bundle sample/out/<tsA> --bundle sample/out/<tsB> --collection-dir <dir>
local_multidataset_run:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -d "$(COLLECTION_DIR)" || \
	    { echo "[ERROR] COLLECTION_DIR が見つかりません: $(COLLECTION_DIR) (先に collect_datasets.py で収集してください)"; exit 1; }
	@echo "[INFO] COLLECTION_DIR: $(COLLECTION_DIR)" ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune \
	        --collection-dir "$(COLLECTION_DIR)" \
	        --cases-config "$(LOCAL_SAMPLE_DIR)/cases.yaml"
