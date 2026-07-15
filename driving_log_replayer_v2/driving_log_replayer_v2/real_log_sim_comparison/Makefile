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
N_TRIALS ?= 640
N_JOBS ?= $(or $(REAL_LOG_SIM_COMPARISON_JOBS),32)
FORCE ?= 0
NO_SETUP ?= 0

ifeq ($(NO_SETUP),1)
RUN_PY := PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON)
else
RUN_PY := source "$(ROS_SETUP)" && source "$(SETUP_BASH)" && PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON)
endif

FORCE_EXTRACT := $(if $(filter 1 true TRUE yes YES,$(FORCE)),--force-extract,)

.PHONY: help reidentify

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
	@echo "  INPUT_PARAM=$(INPUT_PARAM)"
	@echo ""
	@echo "任意変数:"
	@echo "  N_TRIALS=$(N_TRIALS) N_JOBS=$(N_JOBS) FORCE=$(FORCE) NO_SETUP=$(NO_SETUP)"
	@echo "  従来互換: MULTI_BATCH_ROOT→ROOT, LOCAL_SCENARIO/LOCAL_SAMPLE_DIR→SCENARIO, REAL_LOG_SIM_COMPARISON_JOBS→N_JOBS"
	@echo ""
	@echo "成果物: <ROOT>/reidentify/{phase1_acc.yaml,phase2_steer.yaml,tuned_params.yaml,metrics.csv,report.html,physical_validity.html,simulator_model.param.yaml}"

reidentify:
	@test -n "$(strip $(ROOT))" || { echo "[ERROR] ROOT is required"; exit 2; }
	@test -n "$(strip $(SCENARIO))" || { echo "[ERROR] SCENARIO is required"; exit 2; }
	@test -n "$(strip $(INPUT_PARAM))" || { echo "[ERROR] INPUT_PARAM is required"; exit 2; }
	@test -d "$(ROOT)/datasets" || { echo "[ERROR] datasets directory not found: $(ROOT)/datasets"; exit 2; }
	@test -f "$(SCENARIO)" || { echo "[ERROR] scenario not found: $(SCENARIO)"; exit 2; }
	@test -f "$(INPUT_PARAM)" || { echo "[ERROR] input parameter file not found: $(INPUT_PARAM)"; exit 2; }
	@if [ "$(NO_SETUP)" != "1" ]; then \
		test -f "$(ROS_SETUP)" || { echo "[ERROR] ROS setup not found: $(ROS_SETUP)"; exit 2; }; \
		test -f "$(SETUP_BASH)" || { echo "[ERROR] ROS setup not found: $(SETUP_BASH) (build the workspace or use NO_SETUP=1)"; exit 2; }; \
	fi
	@echo "[INFO] reidentify: extract -> fit_lon -> fit_steer -> fit_merge -> report -> release"
	$(RUN_PY) -m driving_log_replayer_v2.real_log_sim_comparison.reidentify.pipeline \
		--root "$(ROOT)" \
		--scenario "$(SCENARIO)" \
		--input-param "$(INPUT_PARAM)" \
		--n-trials "$(N_TRIALS)" \
		--n-jobs "$(N_JOBS)" $(FORCE_EXTRACT)
