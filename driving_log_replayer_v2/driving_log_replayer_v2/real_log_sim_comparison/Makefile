SHELL := /bin/bash
.DEFAULT_GOAL := help

MAKEFILE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
WS_ROOT ?= $(abspath $(MAKEFILE_DIR)/../../../../../..)
SETUP_BASH := $(WS_ROOT)/install/setup.bash
ROS_SETUP ?= /opt/ros/humble/setup.bash

PYTHON ?= python3
PYTHONPATH_PREFIX := $(abspath $(MAKEFILE_DIR)/../..)

MULTI_BATCH_ROOT ?= /home/kotaroyoshimoto/data/openloop_j6_16_onwards
LOCAL_SAMPLE_DIR ?= $(MAKEFILE_DIR)/sample
LOCAL_SCENARIO ?= $(LOCAL_SAMPLE_DIR)/scenario.yaml

ROOT ?= $(MULTI_BATCH_ROOT)
SCENARIO ?= $(LOCAL_SCENARIO)
INPUT_PARAM ?= $(WS_ROOT)/src/description/vehicle/j6_gen2_description/j6_gen2_description/config/simulator_model.param.yaml
N_TRIALS ?= 160
N_JOBS ?= $(or $(REAL_LOG_SIM_COMPARISON_JOBS),32)
FORCE ?= 0
NO_SETUP ?= 0

# ARROW_DEFAULT_MEMORY_POOL=system: pandas が連鎖 import する pyarrow の既定アロケータを
# jemalloc から system に強制する。fit_plateau の無限ハング事例 (worker 32個中1個だけ
# fork 直後から一度もCPUを使わず futex 待ちのまま停止) の調査で、jemalloc の背景スレッド
# が内部ロックを保持した瞬間に multiprocessing.get_context("fork") で fork すると、その
# ロックを解放するスレッドが子に存在せず fork 直後の malloc でデッドロックするという既知
# パターンが最有力の疑いとして浮上したが、再現テスト (1000試行超) では再現せず未確定。
# 副作用のない対策のため疑いの段階で先に入れている
# (reidentify/fit_plateau.py, reidentify/fit_merge.py の COW dataset 共有 fork が対象)。
# 実際にハングを踏んだ場合は lib/_parallel.py の imap_with_watchdog が数分で検出し失敗する。
ifeq ($(NO_SETUP),1)
RUN_PY := ARROW_DEFAULT_MEMORY_POOL=system PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON)
else
RUN_PY := source "$(ROS_SETUP)" && source "$(SETUP_BASH)" && ARROW_DEFAULT_MEMORY_POOL=system PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON)
endif

FORCE_EXTRACT := $(if $(filter 1 true TRUE yes YES,$(FORCE)),--force-extract,)

.PHONY: help reidentify report

help:
	@echo "車両モデル再同定"
	@echo ""
	@echo "使い方:"
	@echo "  make reidentify"
	@echo "  make reidentify ROOT=<collection> SCENARIO=<scenario.yaml> INPUT_PARAM=<simulator_model.param.yaml>"
	@echo ""
	@echo "入力（環境変数または make 変数で上書き可）:"
	@echo "  ROOT=$(ROOT)"
	@echo "  SCENARIO=$(SCENARIO)"
	@echo "  INPUT_PARAM=$(INPUT_PARAM)  # scenario が release を宣言した場合のみ必須"
	@echo ""
	@echo "実行ステージは scenario の Evaluation.Conditions.fit / release で制御:"
	@echo "  fit: {target: <case>, stages: [lon, steer, xy, merge]}  # stages 省略=全実行, []=fit なし"
	@echo "  release: {model: <case>|tuned, version: <N>}  # 省略時は release/YAML 出力なし"
	@echo ""
	@echo "任意変数:"
	@echo "  N_TRIALS=$(N_TRIALS) N_JOBS=$(N_JOBS) FORCE=$(FORCE) NO_SETUP=$(NO_SETUP)"
	@echo "  従来互換: MULTI_BATCH_ROOT→ROOT, LOCAL_SCENARIO/LOCAL_SAMPLE_DIR→SCENARIO, REAL_LOG_SIM_COMPARISON_JOBS→N_JOBS"
	@echo ""
	@echo "成果物: <ROOT>/reidentify/{phase1_acc.yaml,phase2_steer.yaml,phase3_xy.yaml,tuned_params.yaml,metrics.csv,report.html,simulator_model.param.yaml}"
	@echo ""
	@echo "  make report  # 既存成果物から report.html だけを再生成 (fit 再実行なし)"

reidentify:
	@test -n "$(strip $(ROOT))" || { echo "[ERROR] ROOT is required"; exit 2; }
	@test -n "$(strip $(SCENARIO))" || { echo "[ERROR] SCENARIO is required"; exit 2; }
	@test -d "$(ROOT)/datasets" || { echo "[ERROR] datasets directory not found: $(ROOT)/datasets"; exit 2; }
	@test -f "$(SCENARIO)" || { echo "[ERROR] scenario not found: $(SCENARIO)"; exit 2; }
	@# INPUT_PARAM は scenario が release を宣言した場合のみ必須。存在すれば渡し、
	@# release 指定があるのに欠けている場合は pipeline がエラーにする。
	@if [ "$(NO_SETUP)" != "1" ]; then \
		test -f "$(ROS_SETUP)" || { echo "[ERROR] ROS setup not found: $(ROS_SETUP)"; exit 2; }; \
		test -f "$(SETUP_BASH)" || { echo "[ERROR] ROS setup not found: $(SETUP_BASH) (build the workspace or use NO_SETUP=1)"; exit 2; }; \
	fi
	@echo "[INFO] reidentify: extract -> (scenario fit.stages) -> evaluate/merge -> [release] -> report"
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.pipeline \
		--root "$(ROOT)" \
		--scenario "$(SCENARIO)" \
		$(if $(wildcard $(INPUT_PARAM)),--input-param "$(INPUT_PARAM)",) \
		--n-trials "$(N_TRIALS)" \
		--n-jobs "$(N_JOBS)" $(FORCE_EXTRACT)

report:
	@test -n "$(strip $(ROOT))" || { echo "[ERROR] ROOT is required"; exit 2; }
	@test -n "$(strip $(SCENARIO))" || { echo "[ERROR] SCENARIO is required"; exit 2; }
	@test -f "$(SCENARIO)" || { echo "[ERROR] scenario not found: $(SCENARIO)"; exit 2; }
	@test -f "$(ROOT)/reidentify/tuned_params.yaml" || { echo "[ERROR] artifacts not found; run 'make reidentify' first"; exit 2; }
	@# report は ROS 非依存なので既存成果物から fit 再実行なしで作り直せる。
	@echo "[INFO] report-only: 既存成果物から report.html を再生成"
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.pipeline \
		--root "$(ROOT)" \
		--scenario "$(SCENARIO)" \
		--n-jobs "$(N_JOBS)" \
		--report-only
