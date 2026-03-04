.PHONY: check test-unit test-replay test-hw-maybe test-hw record venv

ELEGOO_PORT ?= $(shell ls /dev/cu.usbserial* 2>/dev/null | head -1)
PYTHON := $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)

check: test-unit test-replay test-hw-maybe

test-unit:
	@echo "[T1] unit-tests:"
	@cd firmware/arduino-bare && pio test -e native 2>&1 | tail -3

test-replay:
	@echo "[T2] replay:"
	@if ls test-fixtures/recordings/*.jsonl 1>/dev/null 2>&1; then \
		$(PYTHON) scripts/replay_serial.py test-fixtures/recordings/*.jsonl; \
	else echo "  0 fixtures (record some with: make record)"; fi

test-hw-maybe:
	@if [ -n "$(ELEGOO_PORT)" ] && [ -e "$(ELEGOO_PORT)" ]; then \
		echo "[T3] hardware:"; \
		$(PYTHON) scripts/test_hardware.py --port "$(ELEGOO_PORT)"; \
	else echo "[T3] hardware: not connected (skipped)"; fi

test-hw:
	$(PYTHON) scripts/test_hardware.py --port "$(ELEGOO_PORT)"

record:
	$(PYTHON) scripts/record_serial.py --port "$(ELEGOO_PORT)"
