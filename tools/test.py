#!/usr/bin/env python3
"""Run the ROM-free GSP memory and cache regressions with Verilator."""
import argparse
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent.parent
SOURCES = [
    "rtl/tms34010/tms34010_pkg.sv",
    "rtl/tms34010/cdc/tms34010_sync_bit.sv",
    "rtl/tms34010/cdc/tms34010_cdc_mailbox.sv",
    "rtl/harddrivin_word_ram.sv",
    "rtl/harddrivin_dual_port_word_ram.sv",
    "rtl/harddrivin_tms_word_cdc.sv",
    "rtl/harddrivin_gsp_memory.sv",
    "rtl/harddrivin_cockpit_expander.sv",
]
TESTS = ["tb_cache_drain_overlap", "tb_harddrivin_gsp_memory", "tb_harddrivin_cockpit_memory"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jobs", type=int, default=min(4, os.cpu_count() or 1))
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    for top in TESTS:
        output = ROOT / "build" / "tests" / top
        output.mkdir(parents=True, exist_ok=True)
        command = ["verilator", "--binary", "--timing", "--quiet-build", "-j", str(args.jobs),
                   "-Wno-fatal", "--top-module", top, "--Mdir", str(output / "obj"),
                   *SOURCES, "tests/" + top + ".sv"]
        with (output / "build.log").open("w") as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        result = subprocess.run([str(output / "obj" / ("V" + top))], cwd=ROOT,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (output / "result.log").write_text(result.stdout)
        print(result.stdout, end="", flush=True)
        result.check_returncode()


if __name__ == "__main__":
    main()
