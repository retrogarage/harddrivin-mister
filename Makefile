.PHONY: build test check prepare

QUARTUS_SH ?= quartus_sh
PYTHON ?= python3

prepare:
	@test -n "$(ROM)" || (echo 'Usage: make prepare ROM=path/to/harddriv.zip'; exit 2)
	$(PYTHON) tools/prepare_nvram.py "$(ROM)"

build:
	@test -s generated/zram_cockpit_200e.hex -a -s generated/zram_cockpit_210e.hex || (echo 'Run make prepare ROM=path/to/harddriv.zip first'; exit 2)
	$(QUARTUS_SH) --flow compile HardDrivin

test:
	$(PYTHON) tools/test.py

check:
	$(PYTHON) tools/check_release.py
