SHELL := /bin/bash

MAKEFILE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
WS_ROOT ?= $(abspath $(MAKEFILE_DIR)/../../../../../../..)
SETUP_BASH ?= $(WS_ROOT)/install/setup.bash

PYTHON ?= python3
PYTHONPATH_PREFIX := $(abspath $(MAKEFILE_DIR)/../../..)
MPLCONFIGDIR ?= /tmp/matplotlib-cache
COLLECTION_DIR ?=
SCENARIO ?=
CASE ?= current
OUT_DIR ?= $(COLLECTION_DIR)/reidentify_v2
N_TRIALS ?= 50
N_JOBS ?= $(or $(REAL_LOG_SIM_COMPARISON_JOBS),$(shell nproc))
INPUT_PARAM ?= /home/kotaroyoshimoto/workspace/x2_e2e_44/src/description/vehicle/j6_gen2_description/j6_gen2_description/config/simulator_model.param.yaml
ONLY ?=
NO_SETUP ?=

MODULE := driving_log_replayer_v2.real_log_sim_comparison.reidentify
RUN := $(MODULE).run_reidentify

ifeq ($(NO_SETUP),1)
RUN_PY := MPLCONFIGDIR="$(MPLCONFIGDIR)" PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON)
else
RUN_PY := source "$(SETUP_BASH)" && MPLCONFIGDIR="$(MPLCONFIGDIR)" PYTHONPATH="$(PYTHONPATH_PREFIX):$${PYTHONPATH:-}" $(PYTHON)
endif

.PHONY: help run extract fit_lon fit_steer fit_merge report release clean-outputs check require-common require-scenario require-setup

help:
	@echo "Usage: make <target> COLLECTION_DIR=<dir> SCENARIO=<scenario.yaml> [CASE=current]"
	@echo ""
	@echo "Targets:"
	@echo "  run          extract -> fit_lon -> fit_steer -> fit_merge -> report -> release"
	@echo "  extract      real.lite -> CSV cache"
	@echo "  fit_lon      CSV cache -> $(OUT_DIR)/phase1_acc.yaml"
	@echo "  fit_steer    phase1_acc.yaml -> $(OUT_DIR)/phase2_steer.yaml"
	@echo "  fit_merge    phase2_steer.yaml -> $(OUT_DIR)/tuned_params.yaml"
	@echo "  report       tuned_params.yaml -> validity_report.png/txt"
	@echo "  release      tuned_params.yaml -> simulator_model.param.yaml"
	@echo "  check        compile and import-check this package"
	@echo "  clean-outputs remove OUT_DIR"
	@echo ""
	@echo "Variables:"
	@echo "  OUT_DIR=$(OUT_DIR)"
	@echo "  N_TRIALS=$(N_TRIALS)"
	@echo "  N_JOBS=$(N_JOBS)"
	@echo "  ONLY=$(ONLY)"
	@echo "  NO_SETUP=1 skips sourcing SETUP_BASH"
	@echo "  MPLCONFIGDIR=$(MPLCONFIGDIR)"

require-common:
	@test -n "$(COLLECTION_DIR)" || { echo "[ERROR] COLLECTION_DIR is required"; exit 1; }

require-scenario:
	@test -n "$(SCENARIO)" || { echo "[ERROR] SCENARIO is required"; exit 1; }
	@test -f "$(SCENARIO)" || { echo "[ERROR] SCENARIO not found: $(SCENARIO)"; exit 1; }

require-setup:
	@if [ "$(NO_SETUP)" != "1" ]; then \
	    test -f "$(SETUP_BASH)" || { echo "[ERROR] SETUP_BASH not found: $(SETUP_BASH)"; exit 1; }; \
	fi

run: require-common require-scenario require-setup
	$(RUN_PY) -m $(RUN) \
	    --collection-dir "$(COLLECTION_DIR)" \
	    --scenario "$(SCENARIO)" \
	    --case "$(CASE)" \
	    --out-dir "$(OUT_DIR)" \
	    --input-param "$(INPUT_PARAM)" \
	    --n-trials $(N_TRIALS) \
	    --n-jobs $(N_JOBS) \
	    $(if $(ONLY),--only "$(ONLY)",)

extract: require-common require-setup
	$(RUN_PY) -m $(MODULE).extract --collection-dir "$(COLLECTION_DIR)"

fit_lon: require-common require-setup
	$(RUN_PY) -m $(MODULE).fit_lon \
	    --collection-dir "$(COLLECTION_DIR)" \
	    --out "$(OUT_DIR)/phase1_acc.yaml" \
	    --n-jobs $(N_JOBS)

fit_steer: require-common require-scenario require-setup
	@test -f "$(OUT_DIR)/phase1_acc.yaml" || { echo "[ERROR] missing $(OUT_DIR)/phase1_acc.yaml"; exit 1; }
	$(RUN_PY) -m $(MODULE).fit_steer \
	    --collection-dir "$(COLLECTION_DIR)" \
	    --phase1-params "$(OUT_DIR)/phase1_acc.yaml" \
	    --scenario "$(SCENARIO)" \
	    --case "$(CASE)" \
	    --out "$(OUT_DIR)/phase2_steer.yaml" \
	    --n-jobs $(N_JOBS)

fit_merge: require-common require-scenario require-setup
	@test -f "$(OUT_DIR)/phase2_steer.yaml" || { echo "[ERROR] missing $(OUT_DIR)/phase2_steer.yaml"; exit 1; }
	$(RUN_PY) -m $(MODULE).fit_merge \
	    --collection-dir "$(COLLECTION_DIR)" \
	    --scenario "$(SCENARIO)" \
	    --case "$(CASE)" \
	    --phase-params "$(OUT_DIR)/phase2_steer.yaml" \
	    --n-trials $(N_TRIALS) \
	    --jobs $(N_JOBS) \
	    --out "$(OUT_DIR)/tuned_params.yaml"

report: require-common require-scenario require-setup
	@test -f "$(OUT_DIR)/tuned_params.yaml" || { echo "[ERROR] missing $(OUT_DIR)/tuned_params.yaml"; exit 1; }
	$(RUN_PY) -m $(MODULE).report \
	    --collection-dir "$(COLLECTION_DIR)" \
	    --tuned-params "$(OUT_DIR)/tuned_params.yaml" \
	    --scenario "$(SCENARIO)" \
	    --case "$(CASE)" \
	    --out-png "$(OUT_DIR)/validity_report.png" \
	    --out-txt "$(OUT_DIR)/validity_report.txt"

release: require-setup
	@test -f "$(OUT_DIR)/tuned_params.yaml" || { echo "[ERROR] missing $(OUT_DIR)/tuned_params.yaml"; exit 1; }
	$(RUN_PY) -m $(MODULE).release_params \
	    --input-param "$(INPUT_PARAM)" \
	    --tuned-params "$(OUT_DIR)/tuned_params.yaml" \
	    --out-dir "$(OUT_DIR)"

clean-outputs:
	@test -n "$(OUT_DIR)" || { echo "[ERROR] OUT_DIR is empty"; exit 1; }
	@test "$(OUT_DIR)" != "/" || { echo "[ERROR] refusing to remove /"; exit 1; }
	rm -rf "$(OUT_DIR)"

check: require-setup
	$(RUN_PY) -m compileall -q "$(MAKEFILE_DIR)"
	$(RUN_PY) -c "import $(MODULE).fit_lon, $(MODULE).fit_steer, $(MODULE).fit_merge, $(MODULE).report"
