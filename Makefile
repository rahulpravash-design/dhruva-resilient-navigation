# DHRUVA task runner. Windows: needs GNU make (winget install ezwinports.make) and Git's sh on PATH.
PYTHON ?= python
ifeq ($(OS),Windows_NT)
PY := .venv/Scripts/python
else
PY := .venv/bin/python
endif
GRADLE ?= gradle

.PHONY: setup test-fast test-golden test-ui test-full golden-update

setup:
	$(PYTHON) -m venv .venv
	$(PY) -m pip install -q --upgrade pip
	$(PY) -m pip install -q -r requirements.txt

test-fast:
	$(PY) -m ruff check engine_py scripts
	$(PY) -m pytest -m "not slow and not golden" -n auto -q
ifneq ($(wildcard web/package.json),)
	cd web && npm run typecheck && npm test
else
	@echo "[skip] web/ not created yet (P8)"
endif
ifneq ($(wildcard engine_kt/build.gradle.kts),)
	$(GRADLE) :engine_kt:test
else
	@echo "[skip] engine_kt/ not created yet (P7)"
endif

test-golden:
ifneq ($(wildcard engine_py/tests/golden/test_*.py),)
	$(PY) -m pytest -m golden -q
else
	@echo "[skip] no golden tests yet (P2)"
endif

test-ui:
	@echo "[skip] test-ui needs web/ (P8) and android/ (P9)"

test-full:
	@echo "[skip] test-full needs the eval harness (P6)"

golden-update:
	@echo "[skip] no golden outputs yet (P2). Commit message must state the reason for any regeneration."
