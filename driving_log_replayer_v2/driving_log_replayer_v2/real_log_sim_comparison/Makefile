# real_log_sim_comparison use case 専用 Makefile。
# 同ディレクトリ配下に Python module / evaluator_node / README / sample をすべて集約しており、
# `make local_cloud_run` で 9 段階パイプライン (実機ログ抽出 → scenario 自動生成 →
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

.PHONY: help local_cloud_run local_analysis_run local_batch_run local_cross_analysis_run local_multidataset_run local_multidataset_cloud_run prepare_multidataset issue_scenario collect_raw_rosbags open_loop_tune open_loop_batch

help:
	@echo "使い方: make -C $(MAKEFILE_DIR) <ターゲット>"
	@echo ""
	@echo "ターゲット:"
	@echo "  collect_raw_rosbags  vehicle_id で生rosbagを収集し real.lite を生成 (open-loop 同定用)"
	@echo "                       VEHICLE_ID=<uuid> ROOT=<dir> [DATE_FROM=YYYY-MM-DD] [DATE_TO=YYYY-MM-DD] [LIMIT=20]"
	@echo "  open_loop_tune       軽量ループ (主): 全 real.lite で横断ロバスト同定 (multi_dataset_tune)"
	@echo "                       ROOT=<collect_raw_rosbags の出力 root>"
	@echo "  open_loop_batch      重量ループ (subset 精査): curated subset で full launch + 集約レポート"
	@echo "                       ROOT=<dir> [RESUME=1]"
	@echo "  issue_scenario      MOB rosbag リンクから課題シナリオを一括生成（download.py ラッパー）"
	@echo "                      ROSBAG_ID=<file_id> [TEMPLATE=<scenario_template.yaml>]"
	@echo "                      [PROVENANCE=<説明文>] [START=<ts>] [END=<ts>] [WORK_DIR=work]"
	@echo "                      例: make issue_scenario ROSBAG_ID=f8b5e21f-8361-45a6-9079-05fbf40366a5 \\"
	@echo "                               TEMPLATE=\$$(pwd)/sample/scenario_issue_rotary_left.yaml"
	@echo "  local_cloud_run     webauto キャッシュの T4 dataset を使いパイプラインをワンコマンド実行"
	@echo "                      (scenario.yaml の Datasets[0] のみ。複数 dataset は local_batch_run。"
	@echo "                       SKIP_SIM=1 で closed-loop sim を省略)"
	@echo "  local_analysis_run  既存出力 (OUT_DIR; 既定 sample/out/latest) の sim 実行結果を再利用し、"
	@echo "                      解析ステージ (Stage OL1〜Report HTML) だけを再実行 (解析コード変更後の作り直し用)"
	@echo "  local_batch_run     scenario.yaml の Datasets 全 UUID を順次ローカル実行し、収集 +"
	@echo "                      横断分析 (Stage Cross Dataset) + マルチ DS report.html まで実行"
	@echo "                      (BATCH_ROOT 既定 sample/out/batch_<ts>; RESUME=1 で再開;"
	@echo "                       SKIP_SIM=1 で closed-loop sim を省略し open-loop 解析のみ)"
	@echo "  local_cross_analysis_run  既存 collection (COLLECTION_DIR; 既定 sample/out/batch_latest) の"
	@echo "                      横断分析 (Stage Cross Dataset) + マルチ DS report.html だけを再実行"
	@echo "  local_multidataset_run  収集済み複数 real.lite (COLLECTION_DIR) で open-loop ロバスト同定"
	@echo "                      (multi_dataset_tune robust_search)"
	@echo ""
	@echo "  パイプライン構成:"
	@echo "    0. step0_make_lite        実機 input_bag → lite/real.lite/ (共通基盤)"
	@echo "    - Open-Loop (OL) 系列:"
	@echo "      ol1. step_ol1_analyze_nstep   Conditions.cases の各 case (VehicleModel パラメータ) で N-step オープンループ解析"
	@echo "      ol2. step_ol2_analyze_cases   nstep/*.csv を集約 (overlay/, cases_summary.md, cases_metrics.json)"
	@echo "                                    + real.lite から縦/操舵/横 k_us の物理妥当性を直接同定 (cases/physical_validity/)"
	@echo "    - Closed-Loop (CL) 系列:"
	@echo "      cl1. step_cl1_bag_to_scenario lite/real.lite → scenarios/auto_scenario.yaml"
	@echo "      cl2. step_cl2_run_sims        scenario_test_runner で Conditions.sim_runs の各 run を実行 → lite/<tag>.lite/"
	@echo "      cl3. step_cl3_compare_logs    real + 全 sim run の lite を N-way 比較 (figures/, report.md, metrics_closed_loop.json)"
	@echo "      cl4. step_cl4_compare_dp_trajectory DiffusionPlanner 出力軌跡 real vs sim 比較 (figures/dp_*)"
	@echo "    - レポート・横断分析系列:"
	@echo "      html. step_report_html        図スペック・Markdown・設定 YAML を単一 report.html に集約"
	@echo "      cross.step_cross_dataset      collection 横断のモデル×データセット分析 + 物理妥当性検証の横断集約 (cross_dataset/)"
	@echo ""
	@echo "変数:"
	@echo "  WS_ROOT          colcon ワークスペースルート (デフォルト: $(WS_ROOT))"
	@echo "  WEBAUTO_T4_ROOT  webauto キャッシュルート (デフォルト: $(WEBAUTO_T4_ROOT))"
	@echo "  LOCAL_SCENARIO   ローカル実行する scenario.yaml (デフォルト: $(LOCAL_SCENARIO))"
	@echo "  OUT_DIR          local_analysis_run の対象出力ディレクトリ (デフォルト: sample/out/latest)"
	@echo "  BATCH_ROOT       local_batch_run の出力 root (デフォルト: sample/out/batch_<timestamp>)"
	@echo "  COLLECTION_DIR   local_cross_analysis_run / local_multidataset_run の対象 collection"
	@echo ""
	@echo "  issue_scenario 変数:"
	@echo "  ROSBAG_ID    (必須) webauto rosbag の file_id (MOB リンクの file_id= パラメータ)"
	@echo "  TEMPLATE     課題 scenario テンプレート YAML (デフォルト: sample/scenario.yaml)"
	@echo "  PROVENANCE   scenario の real_provenance に記録する文字列"
	@echo "  START/END    rosbag の時刻スライス (webauto --slicing-start/end-timestamp 形式)"
	@echo "  WORK_DIR     作業ディレクトリ (デフォルト: work)"

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
# 解析ステージ (Stage OL1〜Reports) だけを再実行する。Stage 0〜CL2 (lite 抽出・scenario 生成・
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
# パイプライン (Stage 0〜Report HTML) → collect_datasets 収集 → Stage Cross Dataset 横断分析 → マルチ DS
# report.html まで一気通貫。失敗 dataset は WARN + collection.yaml に status 記録して継続。
# RESUME=1 で real.lite が揃っている dataset の sim 実行をスキップ (中断再開用)。
# SKIP_SIM=1 で Stage 3 (closed-loop sim 実行) を全 dataset で省略する。closed-loop 比較が
# 不要なとき (open-loop N-step / カバレッジだけ欲しいとき) のバッチ時間短縮用。
# 後から closed-loop も欲しくなったら SKIP_SIM/RESUME 無しで同じ BATCH_ROOT を再実行する。
BATCH_ROOT ?= $(LOCAL_SAMPLE_DIR)/out/batch_$(shell date +%Y%m%d_%H%M%S)

# マルチデータセット版のクラウド相当ローカル実行
# 6/16以降のデータセットを抽出し、指定した特定のデータセットのみでクローズドループを実行する
DATE_FROM          ?= 2026-06-16
SRC_DATASET_ROOT   ?= /home/kotaroyoshimoto/data/openloop_j6_15_june
CLOSED_LOOP_UUIDS  ?= 00d08afd-b605-5de1-9db0-51241fa87b18,01ccf59a-bf38-5d62-8a81-be0fe0e24b95
MULTI_BATCH_ROOT   ?= /home/kotaroyoshimoto/data/openloop_j6_16_onwards
N_TRIALS           ?= 50
N_JOBS             ?= 32



prepare_multidataset:
	@echo "[INFO] Preparing datasets starting from $(DATE_FROM) in $(MULTI_BATCH_ROOT)..."
	$(PYTHON) $(MAKEFILE_DIR)/prepare_datasets.py \
	    --src-dir "$(SRC_DATASET_ROOT)/datasets" \
	    --dst-dir "$(MULTI_BATCH_ROOT)/datasets" \
	    --date "$(DATE_FROM)"

local_multidataset_cloud_run: prepare_multidataset
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	@extracted=0 ; \
	if [ -d "$(MULTI_BATCH_ROOT)/datasets" ] && [ -f "$(MULTI_BATCH_ROOT)/manifest.yaml" ]; then \
	    num_extracted=$$(find "$(MULTI_BATCH_ROOT)/datasets" -mindepth 1 -maxdepth 1 -type d | wc -l) ; \
	    if [ "$$num_extracted" -gt 0 ]; then \
	        echo "[INFO] Step 1: Already found $$num_extracted extracted datasets in $(MULTI_BATCH_ROOT)/datasets. Skipping run_batch." ; \
	        extracted=1 ; \
	    fi ; \
	fi ; \
	if [ "$$extracted" -eq 0 ]; then \
	    echo "[INFO] Step 1: Running initial open-loop batch run to collect real.lite across datasets..." ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.run_batch \
	        --scenario "$(LOCAL_SCENARIO)" \
	        --batch-root "$(MULTI_BATCH_ROOT)" \
	        --input-mode raw \
	        --skip-sim \
	        --skip-ol \
	        $(if $(RESUME),--resume,) ; \
	fi
	@echo "[INFO] Step 2: Phase 1 Longitudinal Tuning (Direct Physical Identification)..."
	source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.physical_tuning \
	    --collection-dir "$(MULTI_BATCH_ROOT)" \
	    --scenario "$(LOCAL_SCENARIO)" \
	    --phase 1 \
	    --out "$(MULTI_BATCH_ROOT)/phase1_acc.yaml" \
	    --n-jobs $(N_JOBS)
	@echo "[INFO] Step 3: Phase 2 Lateral/Steer Tuning (Direct Physical Identification)..."
	source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.physical_tuning \
	    --collection-dir "$(MULTI_BATCH_ROOT)" \
	    --scenario "$(LOCAL_SCENARIO)" \
	    --phase 2 \
	    --phase-params "$(MULTI_BATCH_ROOT)/phase1_acc.yaml" \
	    --out "$(MULTI_BATCH_ROOT)/phase2_steer.yaml" \
	    --n-jobs $(N_JOBS)
	@echo "[INFO] Step 4: Phase 0 Integrated Tuning & Validity Report Generation (Warm-Started Optuna)..."
	source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune \
	    --collection-dir "$(MULTI_BATCH_ROOT)" \
	    --scenario "$(LOCAL_SCENARIO)" \
	    --phase 0 \
	    --phase-params "$(MULTI_BATCH_ROOT)/phase2_steer.yaml" \
	    --enqueue-params "$(MULTI_BATCH_ROOT)/phase2_steer.yaml" \
	    --n-trials $(N_TRIALS) \
	    --search-subsample 15 \
	    --jobs $(N_JOBS) \
	    --out "$(MULTI_BATCH_ROOT)/tuned_params.yaml" \
	    --report "$(MULTI_BATCH_ROOT)/tuning_report.html"
	source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.physical_validity_report \
	    --params "$(MULTI_BATCH_ROOT)/tuned_params.yaml" \
	    --collection-dir "$(MULTI_BATCH_ROOT)" \
	    --out "$(MULTI_BATCH_ROOT)/physical_validity_report.html" \
	    --metrics-cache "$(MULTI_BATCH_ROOT)/metrics_cache.csv" \
	    --closed-loop-uuids "$(CLOSED_LOOP_UUIDS)"
	@echo "[INFO] Step 5: Applying tuned parameters to scenario..."
	source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.apply_tuned_params \
	    --scenario "$(LOCAL_SCENARIO)" \
	    --params "$(MULTI_BATCH_ROOT)/tuned_params.yaml" \
	    --out "$(MULTI_BATCH_ROOT)/scenarios/tuned_scenario.yaml"
	@echo "[INFO] Step 6: Running closed-loop batch run using tuned parameters for selected datasets..."
	source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.run_batch \
	    --scenario "$(MULTI_BATCH_ROOT)/scenarios/tuned_scenario.yaml" \
	    --batch-root "$(MULTI_BATCH_ROOT)" \
	    --input-mode raw \
	    --closed-loop-uuids "$(CLOSED_LOOP_UUIDS)" \
	    $(if $(RESUME),--resume,)

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
# 再実行する (Stage Cross Dataset + マルチ DS report.html)。per-dataset の sim/解析はスキップするため、
# 横断分析コードの変更を数秒で結果に反映できる (local_analysis_run の横断版)。
COLLECTION_DIR ?= $(LOCAL_SAMPLE_DIR)/out/batch_latest

local_cross_analysis_run:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -d "$(COLLECTION_DIR)" || \
	    { echo "[ERROR] COLLECTION_DIR が見つかりません: $(COLLECTION_DIR) (先に local_batch_run か collect_datasets.py で収集してください)"; exit 1; }
	@echo "[INFO] COLLECTION_DIR: $(COLLECTION_DIR)" ; \
	    source $(SETUP_BASH) \
	    && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.step_cross_dataset \
	        --collection-dir "$(COLLECTION_DIR)" \
	        $(if $(wildcard $(LOCAL_SCENARIO)),--scenario "$(LOCAL_SCENARIO)",) \
	    && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.step_report_html \
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
	        --scenario "$(LOCAL_SCENARIO)"

# MOB rosbag リンク（file_id）から課題シナリオを一括生成するワークフロー。
# download.py で rosbag + 地図を取得し、課題テンプレートから scenario.yaml を生成する。
# annotation-dataset が存在しない課題（MOB リンクのみ）の場合に使用する。
#
# 使い方:
#   make issue_scenario ROSBAG_ID=<file_id> TEMPLATE=<scenario_template.yaml>
#
# 例（テレポート駅ロータリー左旋回）:
#   make -C <このディレクトリ> issue_scenario \
#       ROSBAG_ID=f8b5e21f-8361-45a6-9079-05fbf40366a5 \
#       TEMPLATE=$(pwd)/sample/scenario_issue_rotary_left.yaml
#
# 完了後:
#   make local_cloud_run WEBAUTO_T4_ROOT=$(pwd)/work/dataset LOCAL_SCENARIO=work/scenario_<uuid>.yaml

ROSBAG_ID ?=
TEMPLATE  ?= $(LOCAL_SAMPLE_DIR)/scenario.yaml
PROVENANCE ?=
WORK_DIR  ?= work

issue_scenario:
	@test -n "$(ROSBAG_ID)" || \
	    { echo "[ERROR] ROSBAG_ID が指定されていません。例: make issue_scenario ROSBAG_ID=<file_id>"; exit 1; }
	@test -f "$(TEMPLATE)" || \
	    { echo "[ERROR] TEMPLATE が見つかりません: $(TEMPLATE)"; exit 1; }
	@echo "[INFO] ROSBAG_ID : $(ROSBAG_ID)"
	@echo "[INFO] TEMPLATE  : $(TEMPLATE)"
	@echo "[INFO] WORK_DIR  : $(WORK_DIR)"
	$(PYTHON) $(MAKEFILE_DIR)/download.py \
	    --rosbag-id $(ROSBAG_ID) \
	    --work-dir "$(WORK_DIR)" \
	    --scenario-template "$(TEMPLATE)" \
	    $(if $(PROVENANCE),--provenance "$(PROVENANCE)",) \
	    $(if $(START),--start "$(START)",) \
	    $(if $(END),--end "$(END)",)
	@echo ""
	@echo "[INFO] 次のステップ:"
	@echo "  make local_cloud_run WEBAUTO_T4_ROOT=\$$(pwd)/$(WORK_DIR)/dataset LOCAL_SCENARIO=$(WORK_DIR)/scenario_<uuid>.yaml"

# ------ open-loop: J6 #15号車の生rosbag 収集 & モデル磨き上げ ------
# 軽量/重量の二経路で open-loop 専用の評価・同定フローを提供する。
# closed-loop (local_cloud_run / local_batch_run) を壊さない振る舞い保存設計。
#
# 経路の使い分け:
#   軽量ループ (主): collect_raw_rosbags → open_loop_tune
#     real.lite さえあれば ROS launch 不要で数百本の横断ロバスト同定ができる。
#     「できるだけ多く」の bag でモデルを磨き上げるメインルート。
#
#   重量ループ (subset 精査): collect_raw_rosbags → open_loop_batch
#     厳選した数本〜数十本のみ full launch (SKIP_SIM=1) + step_cross_dataset + 集約レポート。
#     OL1/OL2 の per-bag 詳細解析・検証ビューアが欲しい時に使う。

# J6 #15 号車の vehicle_id (デフォルト)
VEHICLE_ID ?= e14508f0-5cc8-410c-886b-23aea4a8d5d8
# 収集ルート (collect_raw_rosbags / open_loop_tune / open_loop_batch が共通で使う)
ROOT       ?= $(LOCAL_SAMPLE_DIR)/out/open_loop_$(shell date +%Y%m%d_%H%M%S)
# 収集期間フィルタ (省略時は全期間)
DATE_FROM  ?=
DATE_TO    ?=
# 最大収集本数 (DL 量ガード)
LIMIT      ?= 20

# Phase 0+1: rosbag 収集 + real.lite 化
# (ROS 環境が source 済みである必要があるのは step0_make_lite のみ)
collect_raw_rosbags:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -n "$(VEHICLE_ID)" || \
	    { echo "[ERROR] VEHICLE_ID が未設定です。例: make collect_raw_rosbags VEHICLE_ID=<uuid>"; exit 1; }
	@echo "[INFO] VEHICLE_ID: $(VEHICLE_ID)" ; \
	    echo "[INFO] ROOT: $(ROOT)" ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.collect_raw_rosbags \
	        --vehicle-id "$(VEHICLE_ID)" \
	        --root "$(ROOT)" \
	        --limit "$(LIMIT)" \
	        --jobs "$(N_JOBS)" \
	        $(if $(DATE_FROM),--date-from "$(DATE_FROM)",) \
	        $(if $(DATE_TO),--date-to "$(DATE_TO)",)

# Phase 2a: 軽量ループ (主) — 全 real.lite で横断ロバスト同定 (multi_dataset_tune)
# FINAL params は <ROOT>/tuned_params.yaml に保存される。
# これを scenario.yaml の Conditions.models 該当パラメータへ手動反映し、
# 次の collect_raw_rosbags → open_loop_tune でさらに磨く。
open_loop_tune:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -d "$(ROOT)" || \
	    { echo "[ERROR] ROOT が見つかりません: $(ROOT) (先に collect_raw_rosbags を実行してください)"; exit 1; }
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	@echo "[INFO] ROOT: $(ROOT)" ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.multi_dataset_tune \
	        --collection-dir "$(ROOT)" \
	        --scenario "$(LOCAL_SCENARIO)" \
	        --out "$(ROOT)/tuned_params.yaml"

# Phase 2b: 重量ループ (subset 精査) — curated subset で full launch + 集約レポート
# ROOT/datasets/ 配下の全データセットを SKIP_SIM=1 (open-loop のみ) で実行し、
# aggregate_report.html (クラウド per-dataset の report.html と衝突しない名前) を生成する。
# 全件ではなく厳選した subset を ROOT/datasets/ に置いてから実行することを推奨。
open_loop_batch:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -d "$(ROOT)" || \
	    { echo "[ERROR] ROOT が見つかりません: $(ROOT) (先に collect_raw_rosbags を実行してください)"; exit 1; }
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	@echo "[INFO] ROOT: $(ROOT)" ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.run_batch \
	        --scenario "$(LOCAL_SCENARIO)" \
	        --batch-root "$(ROOT)" \
	        --input-mode raw \
	        --skip-sim \
	        $(if $(RESUME),--resume,)
