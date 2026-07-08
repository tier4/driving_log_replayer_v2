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
PYTHONPATH_PREFIX := $(abspath $(MAKEFILE_DIR)/../..)
MPLCONFIGDIR ?= /tmp/matplotlib-cache
NO_SETUP ?=

ifeq ($(NO_SETUP),1)
RUN_PY := MPLCONFIGDIR="$(MPLCONFIGDIR)" PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON)
else
RUN_PY := source "$(SETUP_BASH)" && MPLCONFIGDIR="$(MPLCONFIGDIR)" PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON)
endif
RUN_PY_NO_SETUP := MPLCONFIGDIR="$(MPLCONFIGDIR)" PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON)

.PHONY: help local_cloud_run local_analysis_run local_batch_run local_cross_analysis_run local_multidataset_run local_multidataset_cloud_run prepare_multidataset issue_scenario collect_raw_rosbags open_loop_tune open_loop_batch physical_validity_report_nodata lite lite_ol lite_cl extract fit_lon fit_steer fit_merge fit_report reidentify apply closed_loop release_model_param reidentify_v2 compare_accel compare_accel_all



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
	@echo "  マルチデータセット同定パイプライン (分割ターゲット; MULTI_BATCH_ROOT を対象):"
	@echo "    local_multidataset_cloud_run  フル: extract→reidentify→apply→closed_loop の薄い連結"
	@echo "    lite                OL向け real.lite+CSV cache と CL向け closed_loop_t4 staging を準備"
	@echo "    lite_ol             OL向け: datasets/<uuid>/real.lite + reidentify_cache.csv を準備"
	@echo "    lite_cl             CL向け: datasets/<uuid>/cl/{t4,scenario.yaml} を準備"
	@echo "    reidentify          再同定のみ (fit_lon→fit_steer→fit_merge→fit_report; extract/sim なし・実測~2.5分)"
	@echo "    extract             Step1: 全 dataset の real.lite 抽出 (RESUME=1 で揃っていればスキップ)"
	@echo "    fit_lon             Step2: 縦方向 直接同定 → phase1_acc.yaml"
	@echo "    fit_steer           Step3: 操舵+k_us 直接同定 (fit_lon 必須) → phase2_steer.yaml"
	@echo "    fit_merge           Step4a: reidentify v2 warm-start Optuna 統合 (fit_steer 必須) → tuned_params.yaml"
	@echo "    fit_report          Step4b: 物理妥当性レポート (fit_merge 必須) → physical_validity_report.html"
	@echo "    apply               Step5: 同定パラメータを scenario に反映 (fit_merge 必須) → tuned_scenario.yaml"
	@echo "    closed_loop         Step6: tuned scenario で closed-loop 実 sim (apply 必須)"
	@echo ""
	@echo "  reidentify v2 (reidentify/ 配下、vehicle_model_fitting 流のシンプルなスクリプト集への再構成版):"
	@echo "    reidentify_v2       extract→fit_lon→fit_steer→fit_merge→report→release を run_reidentify.py で実行"
	@echo "                        (出力は MULTI_BATCH_ROOT/reidentify_v2/ 配下。詳細は reidentify/README.md)"
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
	@if [ "$(NO_SETUP)" != "1" ]; then \
	    test -f $(SETUP_BASH) || { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }; \
	fi
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
	    $(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.run_analysis \
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
# /home/kotaroyoshimoto/data/openloop_j6_16_onwards は唯一の利用 dataset root なので、
# datasets/ は symlink staging ではなく実データディレクトリとして事前構成する。
CLOSED_LOOP_UUIDS  ?= 00d08afd-b605-5de1-9db0-51241fa87b18,01ccf59a-bf38-5d62-8a81-be0fe0e24b95
# closed-loop 実 sim 用の地図ディレクトリ (lanelet2_map.osm を含む)。
# raw モードの datasets/<uuid> は map を持たないため、run_batch が closed-loop UUID の
# t4_dataset_path/map として symlink する (無いと Stage CL1/CL2 がスキップされ runs が空になる)。
# 既定は CLOSED_LOOP_UUIDS の rosbag が記録された environment (ce963c1f) の地図 project 2231
# (MGRS grid 54SUE。実機 pose 範囲が本地図の座標範囲に内包されることを確認済み) の
# 記録日 (2026-06-23) 直前の最新版。
CLOSED_LOOP_MAP    ?= $(HOME)/.webauto/simulation/data/map/x2_dev/2231/2231-20260525005702640410
MULTI_BATCH_ROOT   ?= /home/kotaroyoshimoto/data/openloop_j6_16_onwards
N_TRIALS           ?= 50
N_JOBS             ?= $(or $(REAL_LOG_SIM_COMPARISON_JOBS),32)
MODEL_CASE         ?= current

# パイプライン各段の中間成果物 (分割ターゲット間で受け渡す SSOT パス)。
# fit_lon → fit_steer → fit_merge → fit_report → apply → closed_loop がこれらを介して連結する。
PHASE1_ACC      := $(MULTI_BATCH_ROOT)/phase1_acc.yaml
PHASE2_STEER    := $(MULTI_BATCH_ROOT)/phase2_steer.yaml
TUNED_PARAMS    := $(MULTI_BATCH_ROOT)/tuned_params.yaml
TUNING_REPORT   := $(MULTI_BATCH_ROOT)/tuning_report.html
VALIDITY_REPORT := $(MULTI_BATCH_ROOT)/physical_validity_report.html
RELEASE_NOTE_REPORT := $(MULTI_BATCH_ROOT)/physical_validity_release_note.html
REIDENTIFY_REPORT_PNG := $(MULTI_BATCH_ROOT)/validity_report.png
REIDENTIFY_REPORT_TXT := $(MULTI_BATCH_ROOT)/validity_report.txt
METRICS_CACHE   := $(MULTI_BATCH_ROOT)/metrics_cache.csv
TUNED_SCENARIO  := $(MULTI_BATCH_ROOT)/scenarios/tuned_scenario.yaml
INPUT_PARAM      ?= /home/kotaroyoshimoto/workspace/x2_e2e_44/src/description/vehicle/j6_gen2_description/j6_gen2_description/config/simulator_model.param.yaml

prepare_multidataset:
	@echo "[INFO] Validating real datasets in $(MULTI_BATCH_ROOT)..."
	@test -d "$(MULTI_BATCH_ROOT)/datasets" || \
	    { echo "[ERROR] datasets directory does not exist: $(MULTI_BATCH_ROOT)/datasets"; exit 1; }
	@num_symlinks=$$(find "$(MULTI_BATCH_ROOT)/datasets" -mindepth 1 -maxdepth 1 -type l | wc -l) ; \
	    if [ "$$num_symlinks" -ne 0 ]; then \
	        echo "[ERROR] $(MULTI_BATCH_ROOT)/datasets contains $$num_symlinks top-level symlinks. Convert them to real directories first." ; \
	        exit 1 ; \
	    fi ; \
	    num_datasets=$$(find "$(MULTI_BATCH_ROOT)/datasets" -mindepth 1 -maxdepth 1 -type d | wc -l) ; \
	    if [ "$$num_datasets" -eq 0 ]; then \
	        echo "[ERROR] No real dataset directories found in $(MULTI_BATCH_ROOT)/datasets" ; \
	        exit 1 ; \
	    fi ; \
	    echo "[INFO] Found $$num_datasets real dataset directories."

# ---------------------------------------------------------------------------
# マルチデータセット同定パイプラインの分割ターゲット群。
# local_multidataset_cloud_run はこれらを $(MAKE) で順に呼ぶ薄いオーケストレータ。
# 各サブターゲットは独立に単体実行でき (毎回再実行の .PHONY)、中間成果物 (PHASE1_ACC 等)
# を介して連結する。モデル構造イテレーション中は reidentify (extract/sim なし・実測2.5分) を回す。
#
# 依存グラフ:
#   extract → fit_lon → fit_steer → fit_merge → fit_report → apply → closed_loop
#                                    ( reidentify = fit_lon..fit_report の4段 )
#
# 注意 (実測): 同定・Optuna・レポートは合計 ~2.5分。重いのは extract の初回 (webauto pull +
#   372件 real.lite 抽出) と closed_loop の ros2 launch 実 sim。fit 系の高速化・N_JOBS 増は無意味。
# ---------------------------------------------------------------------------

# Step 1: 全 dataset の real.lite を抽出 (raw → topic 絞り込み mcap)。
# RESUME=1 かつ real.lite が揃っていれば run_batch をスキップ。
lite:
	$(MAKE) lite_ol
	$(MAKE) lite_cl

lite_ol: extract

lite_cl: prepare_multidataset
	@test -n "$(CLOSED_LOOP_UUIDS)" || \
	    { echo "[ERROR] CLOSED_LOOP_UUIDS が未設定です"; exit 1; }
	@if [ -n "$(CLOSED_LOOP_MAP)" ]; then \
	    test -d "$(CLOSED_LOOP_MAP)" || { echo "[ERROR] CLOSED_LOOP_MAP が見つかりません: $(CLOSED_LOOP_MAP)"; exit 1; }; \
	    test -f "$(CLOSED_LOOP_MAP)/lanelet2_map.osm" || { echo "[ERROR] CLOSED_LOOP_MAP/lanelet2_map.osm が見つかりません: $(CLOSED_LOOP_MAP)"; exit 1; }; \
	fi
	@echo "[INFO] lite_cl: Preparing closed-loop staging for UUIDs: $(CLOSED_LOOP_UUIDS)"
	@IFS=',' read -ra uuids <<< "$(CLOSED_LOOP_UUIDS)" ; \
	for raw_uuid in "$${uuids[@]}"; do \
	    uuid="$$(echo "$$raw_uuid" | xargs)" ; \
	    test -n "$$uuid" || continue ; \
	    ds="$(MULTI_BATCH_ROOT)/datasets/$$uuid" ; \
	    cl_dir="$$ds/cl" ; \
	    out="$$cl_dir/t4" ; \
	    fallback="$(MULTI_BATCH_ROOT)/runs/$$uuid/fallback_raw_dataset/input_bag" ; \
	    src="$$ds/input_bag" ; \
	    test -d "$$ds" || { echo "[ERROR] CL dataset が見つかりません: $$ds"; exit 1; } ; \
	    if [ -f "$$fallback/metadata.yaml" ] && grep -q "/vehicle/status/gear_status" "$$fallback/metadata.yaml"; then \
	        src="$$fallback" ; \
	    fi ; \
	    test -d "$$src" || { echo "[ERROR] CL input_bag が見つかりません: $$src"; exit 1; } ; \
	    mkdir -p "$$out" "$$cl_dir" ; \
	    rm -rf "$$out/input_bag" ; \
	    ln -s "$$(realpath "$$src")" "$$out/input_bag" ; \
	    if [ -n "$(CLOSED_LOOP_MAP)" ]; then \
	        rm -rf "$$out/map" ; \
	        ln -s "$$(realpath "$(CLOSED_LOOP_MAP)")" "$$out/map" ; \
	    fi ; \
	    $(RUN_PY_NO_SETUP) -m driving_log_replayer_v2.real_log_sim_comparison.run_batch \
	        --scenario "$(LOCAL_SCENARIO)" \
	        --batch-root "$(MULTI_BATCH_ROOT)" \
	        --write-single-scenario "$$uuid=$$cl_dir/scenario.yaml" \
	        --input-mode raw ; \
	    echo "[INFO] lite_cl: $$uuid -> $$cl_dir (t4=$$out, scenario=$$cl_dir/scenario.yaml)" ; \
	done

extract:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	@extracted=0 ; \
	if [ -n "$(RESUME)" ] && [ -d "$(MULTI_BATCH_ROOT)/datasets" ]; then \
	    num_datasets=$$(find "$(MULTI_BATCH_ROOT)/datasets" -mindepth 1 -maxdepth 1 -type d | wc -l) ; \
	    num_lite=$$(find "$(MULTI_BATCH_ROOT)/datasets" -path "*/real.lite/real.lite_0.mcap" -type f | wc -l) ; \
	    if [ "$$num_datasets" -gt 0 ] && [ "$$num_lite" -ge "$$num_datasets" ]; then \
	        echo "[INFO] extract: RESUME=1 and found $$num_lite/$$num_datasets real.lite datasets. Skipping run_batch." ; \
	        extracted=1 ; \
	    fi ; \
	fi ; \
	if [ "$$extracted" -eq 0 ]; then \
	    echo "[INFO] extract: Running initial open-loop batch run to collect real.lite across datasets..." ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.run_batch \
	        --scenario "$(LOCAL_SCENARIO)" \
	        --batch-root "$(MULTI_BATCH_ROOT)" \
	        --webauto-root "$(WEBAUTO_T4_ROOT)" \
	        --input-mode raw \
	        --skip-sim \
	        --skip-ol \
	        $(if $(RESUME),--resume,) ; \
	fi
	@echo "[INFO] extract: Generating reidentify CSV cache for ROS-free downstream steps..."
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.extract \
	    --collection-dir "$(MULTI_BATCH_ROOT)"

# Step 2: 縦方向モデルの直接同定 → PHASE1_ACC。
fit_lon:
	@if [ "$(NO_SETUP)" != "1" ]; then \
	    test -f $(SETUP_BASH) || { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }; \
	fi
	@echo "[INFO] fit_lon: Phase 1 縦方向 直接同定 → $(PHASE1_ACC)"
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.fit_lon \
	    --collection-dir "$(MULTI_BATCH_ROOT)" \
	    --out "$(PHASE1_ACC)" \
	    --n-jobs $(N_JOBS)

# Step 3: 操舵 + k_us の直接同定 (PHASE1_ACC の縦方向値を継承) → PHASE2_STEER。
fit_steer:
	@if [ "$(NO_SETUP)" != "1" ]; then \
	    test -f $(SETUP_BASH) || { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }; \
	fi
	@test -f "$(PHASE1_ACC)" || \
	    { echo "[ERROR] $(PHASE1_ACC) がありません。先に 'make fit_lon' を実行してください"; exit 1; }
	@echo "[INFO] fit_steer: Phase 2 操舵+k_us 直接同定 (継承: $(PHASE1_ACC)) → $(PHASE2_STEER)"
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.fit_steer \
	    --collection-dir "$(MULTI_BATCH_ROOT)" \
	    --phase1-params "$(PHASE1_ACC)" \
	    --scenario "$(LOCAL_SCENARIO)" \
	    --case "$(MODEL_CASE)" \
	    --out "$(PHASE2_STEER)" \
	    --n-jobs $(N_JOBS)

# Step 4a: warm-start Optuna 統合チューニング → TUNED_PARAMS + TUNING_REPORT。
fit_merge:
	@if [ "$(NO_SETUP)" != "1" ]; then \
	    test -f $(SETUP_BASH) || { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }; \
	fi
	@test -f "$(PHASE2_STEER)" || \
	    { echo "[ERROR] $(PHASE2_STEER) がありません。先に 'make fit_steer' を実行してください"; exit 1; }
	@echo "[INFO] fit_merge: reidentify v2 warm-start Optuna 統合チューニング → $(TUNED_PARAMS)"
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.fit_merge \
	    --collection-dir "$(MULTI_BATCH_ROOT)" \
	    --scenario "$(LOCAL_SCENARIO)" \
	    --case "$(MODEL_CASE)" \
	    --phase-params "$(PHASE2_STEER)" \
	    --n-trials $(N_TRIALS) \
	    --jobs $(N_JOBS) \
	    --out "$(TUNED_PARAMS)"

# Step 4b: 物理妥当性レポート生成 → VALIDITY_REPORT。
fit_report:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -f "$(TUNED_PARAMS)" || \
	    { echo "[ERROR] $(TUNED_PARAMS) がありません。先に 'make fit_merge' を実行してください"; exit 1; }
	@echo "[INFO] fit_report: 物理妥当性レポート生成 → $(VALIDITY_REPORT)"
	@echo "[INFO] fit_report: リリースノート用レポート生成 → $(RELEASE_NOTE_REPORT)"
	source "$(SETUP_BASH)" && MPLCONFIGDIR="$(MPLCONFIGDIR)" PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.physical_validity_report \
	    --params "$(TUNED_PARAMS)" \
	    --scenario "$(LOCAL_SCENARIO)" \
	    --case "$(MODEL_CASE)" \
	    --collection-dir "$(MULTI_BATCH_ROOT)" \
	    --out "$(VALIDITY_REPORT)" \
	    --release-note-out "$(RELEASE_NOTE_REPORT)" \
	    --metrics-cache "$(METRICS_CACHE)" \
	    --closed-loop-uuids "$(CLOSED_LOOP_UUIDS)" \
	    --n-curve-ds 0

# Step 4c: モデルパラメータリリース用 YAML 生成
release_model_param:
	@if [ "$(NO_SETUP)" != "1" ]; then \
	    test -f $(SETUP_BASH) || { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }; \
	fi
	@test -f "$(TUNED_PARAMS)" || \
	    { echo "[ERROR] $(TUNED_PARAMS) がありません。先に 'make fit_merge' を実行してください"; exit 1; }
	@echo "[INFO] release_model_param: モデルパラメータリリース用 YAML 生成 → $(MULTI_BATCH_ROOT)/simulator_model.param.yaml"
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.release_params \
	    --input-param "$(INPUT_PARAM)" \
	    --tuned-params "$(TUNED_PARAMS)" \
	    --out-dir "$(MULTI_BATCH_ROOT)"

# 集約: 再同定 (extract/closed_loop なし)。モデル構造イテレーションの本命パス (実測 ~2.5分)。
# prepare_multidataset を prereq にし、datasets/ 未整備時は physical_tuning 内部の
# "有効データセット 0 件" ではなく親切な検証エラーで即座に止める。
reidentify: prepare_multidataset
	@echo "[INFO] === reidentify: fit_lon → fit_steer → fit_merge → fit_report → release_model_param (extract/sim なし) ==="
	$(MAKE) fit_lon NO_SETUP=$(NO_SETUP)
	$(MAKE) fit_steer NO_SETUP=$(NO_SETUP)
	$(MAKE) fit_merge NO_SETUP=$(NO_SETUP)
	$(MAKE) fit_report NO_SETUP=$(NO_SETUP)
	$(MAKE) release_model_param NO_SETUP=$(NO_SETUP)

# reidentify v2: vehicle_model_fitting 流のシンプルなスクリプト集 (reidentify/) への再構成版。
# extract→fit_lon→fit_steer→fit_merge→report→release を run_reidentify.py が薄く連結する
# (詳細・依存関係・旧パイプラインとの差分は reidentify/README.md 参照)。
# 出力は $(MULTI_BATCH_ROOT)/reidentify_v2/ 配下 (旧パイプラインの成果物と衝突しない)。
reidentify_v2:
	@if [ "$(NO_SETUP)" != "1" ]; then \
	    test -f $(SETUP_BASH) || { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }; \
	fi
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.run_reidentify \
	    --collection-dir "$(MULTI_BATCH_ROOT)" \
	    --scenario "$(LOCAL_SCENARIO)" \
	    --case "$(MODEL_CASE)" \
	    --out-dir "$(MULTI_BATCH_ROOT)" \
	    --input-param "$(INPUT_PARAM)" \
	    --n-trials $(N_TRIALS) \
	    --n-jobs $(N_JOBS)

# Step 5: 同定パラメータを scenario に反映 → TUNED_SCENARIO。
apply:
	@if [ "$(NO_SETUP)" != "1" ]; then \
	    test -f $(SETUP_BASH) || { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }; \
	fi
	@test -f "$(TUNED_PARAMS)" || \
	    { echo "[ERROR] $(TUNED_PARAMS) がありません。先に 'make fit_merge' を実行してください"; exit 1; }
	@echo "[INFO] apply: 同定パラメータを scenario に反映 → $(TUNED_SCENARIO)"
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.apply_tuned_params \
	    --scenario "$(LOCAL_SCENARIO)" \
	    --params "$(TUNED_PARAMS)" \
	    --out "$(TUNED_SCENARIO)"

# Step 6: 同定済みパラメータで closed-loop 実シミュレーション (ros2 launch)。
closed_loop:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -f "$(TUNED_SCENARIO)" || \
	    { echo "[ERROR] $(TUNED_SCENARIO) がありません。先に 'make apply' を実行してください"; exit 1; }
	$(MAKE) lite_cl LOCAL_SCENARIO="$(TUNED_SCENARIO)"
	@echo "[INFO] closed_loop: tuned scenario で closed-loop 実 sim を実行..."
	source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.run_batch \
	    --scenario "$(TUNED_SCENARIO)" \
	    --batch-root "$(MULTI_BATCH_ROOT)" \
	    --webauto-root "$(WEBAUTO_T4_ROOT)" \
	    --input-mode raw \
	    --closed-loop-uuids "$(CLOSED_LOOP_UUIDS)" \
	    --closed-loop-map "$(CLOSED_LOOP_MAP)" \
	    $(if $(RESUME),--resume,)

# オーケストレータ: extract → reidentify → apply → closed_loop を順に実行 (従来のフルパイプライン)。
local_multidataset_cloud_run: prepare_multidataset
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	$(MAKE) lite
	$(MAKE) reidentify
	$(MAKE) apply NO_SETUP=1
	$(MAKE) closed_loop

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
	@echo "[INFO] collect_raw_rosbags: Generating reidentify CSV cache for ROS-free tuning..."
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.extract \
	    --collection-dir "$(ROOT)"

# Phase 2a: 軽量ループ (主) — 全 real.lite で横断ロバスト同定 (multi_dataset_tune)
# FINAL params は <ROOT>/tuned_params.yaml に保存される。
# これを scenario.yaml の Conditions.models 該当パラメータへ手動反映し、
# 次の collect_raw_rosbags → open_loop_tune でさらに磨く。
open_loop_tune:
	@test -d "$(ROOT)" || \
	    { echo "[ERROR] ROOT が見つかりません: $(ROOT) (先に collect_raw_rosbags を実行してください)"; exit 1; }
	@test -f $(LOCAL_SCENARIO) || \
	    { echo "[ERROR] $(LOCAL_SCENARIO) が見つかりません"; exit 1; }
	@echo "[INFO] ROOT: $(ROOT)"
	$(MAKE) reidentify NO_SETUP=1 MULTI_BATCH_ROOT="$(ROOT)" LOCAL_SCENARIO="$(LOCAL_SCENARIO)"

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

# データセットなしで physical_validity_report を生成する。
# tuned_params.yaml (--params) のパラメータ値のみを使い、real.lite データは一切読まずに
# HTML レポートを生成する。出力ファイル名は physical_validity_report_nodata.html として
# データあり版 (physical_validity_report.html) と区別する。
#
# 使い方:
#   make physical_validity_report_nodata MULTI_BATCH_ROOT=<dir> [PARAMS=<yaml>] [NODATA_OUT=<html>]
#
# 変数:
#   MULTI_BATCH_ROOT  collection ディレクトリ兼デフォルト出力先 (既定: $(MULTI_BATCH_ROOT))
#   PARAMS            使用するパラメータ YAML (既定: $(MULTI_BATCH_ROOT)/tuned_params.yaml)
#   NODATA_OUT        出力 HTML パス (既定: $(MULTI_BATCH_ROOT)/physical_validity_report_nodata.html)
PARAMS     ?= $(MULTI_BATCH_ROOT)/tuned_params.yaml
NODATA_OUT ?= $(MULTI_BATCH_ROOT)/physical_validity_report_nodata.html

physical_validity_report_nodata:
	@test -f $(SETUP_BASH) || \
	    { echo "[ERROR] $(SETUP_BASH) が見つかりません。先にワークスペースをビルドしてください"; exit 1; }
	@test -f "$(PARAMS)" || \
	    { echo "[ERROR] パラメータ YAML が見つかりません: $(PARAMS)"; exit 1; }
	@_nodata_col="$(MULTI_BATCH_ROOT)/.empty_collection" ; \
	    mkdir -p "$$_nodata_col" ; \
	    echo "[INFO] PARAMS   : $(PARAMS)" ; \
	    echo "[INFO] OUT      : $(NODATA_OUT)" ; \
	    echo "[INFO] COLL_DIR : $$_nodata_col (空ディレクトリ = データなし)" ; \
	    source $(SETUP_BASH) && $(PYTHON) -m driving_log_replayer_v2.real_log_sim_comparison.physical_validity_report \
	        --params "$(PARAMS)" \
	        --scenario "$(LOCAL_SCENARIO)" \
	        --case "$(MODEL_CASE)" \
	        --collection-dir "$$_nodata_col" \
	        --out "$(NODATA_OUT)"

# 加速度トピック・微分・位置2回微分の比較プロットを生成する。
#
# 使い方:
#   make compare_accel CSV=<reidentify_cache.csvのパス> [OUT=<png出力パス>]
#   または (MULTI_BATCH_ROOT を使用し、特定の UUID を指定):
#   make compare_accel UUID=<uuid> [OUT=<png出力パス>]
#
# 例:
#   make compare_accel CSV=/path/to/reidentify_cache.csv
#   make compare_accel UUID=00d08afd-b605-5de1-9db0-51241fa87b18
compare_accel:
	@if [ -z "$(CSV)" ] && [ -z "$(UUID)" ]; then \
	    echo "[ERROR] CSV または UUID を指定してください。"; \
	    echo "  例1: make compare_accel CSV=/path/to/reidentify_cache.csv"; \
	    echo "  例2: make compare_accel UUID=00d08afd-b605-5de1-9db0-51241fa87b18 [MULTI_BATCH_ROOT=<dir>]"; \
	    exit 1; \
	fi
	@csv_path="$(CSV)" ; \
	if [ -n "$(UUID)" ]; then \
	    batch_root="$(or $(MULTI_BATCH_ROOT),/home/kotaroyoshimoto/data/openloop_j6_16_onwards)" ; \
	    csv_path="$$batch_root/datasets/$(UUID)/reidentify_cache.csv" ; \
	fi ; \
	test -f "$$csv_path" || { echo "[ERROR] CSV ファイルが見つかりません: $$csv_path"; exit 1; } ; \
	out_path="$(OUT)" ; \
	if [ -z "$$out_path" ]; then \
	    out_path="$$(dirname "$$csv_path")/acceleration_comparison.png" ; \
	fi ; \
	echo "[INFO] Input CSV : $$csv_path" ; \
	echo "[INFO] Output PNG: $$out_path" ; \
	$(RUN_PY_NO_SETUP) $(MAKEFILE_DIR)/tools/compare_accel.py --csv "$$csv_path" --out "$$out_path"

# 全データセットに対して比較プロットを一括生成する。
#
# 使い方:
#   make compare_accel_all [MULTI_BATCH_ROOT=<dir>]
#
# 例:
#   make compare_accel_all
compare_accel_all:
	@batch_root="$(or $(MULTI_BATCH_ROOT),/home/kotaroyoshimoto/data/openloop_j6_16_onwards)" ; \
	test -d "$$batch_root/datasets" || { echo "[ERROR] datasets ディレクトリが見つかりません: $$batch_root/datasets"; exit 1; } ; \
	echo "[INFO] Batch plotting in progress..." ; \
	$(RUN_PY_NO_SETUP) $(MAKEFILE_DIR)/tools/compare_accel.py --collection-dir "$$batch_root/datasets"


