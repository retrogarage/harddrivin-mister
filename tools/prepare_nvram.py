#!/usr/bin/env python3
"""Prepare cockpit power-up RAM from a user-supplied harddriv.zip."""
import argparse
from pathlib import Path
import zipfile
import zlib


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rom", type=Path, help="MAME harddriv.zip containing harddriv.200e and harddriv.210e")
    args = parser.parse_args()
    output = Path(__file__).resolve().parent.parent / "generated"
    expected = {"200e": 0xaed020f7, "210e": 0x4a91835b}
    lanes = {}
    with zipfile.ZipFile(args.rom) as archive:
        for lane, crc in expected.items():
            data = archive.read("harddriv." + lane)
            if len(data) != 2048 or zlib.crc32(data) != crc:
                parser.error("Incorrect cockpit power-up RAM: harddriv." + lane)
            lanes[lane] = data
    output.mkdir(exist_ok=True)
    for lane, data in lanes.items():
        (output / ("zram_cockpit_" + lane + ".hex")).write_text("".join(f"{byte:02x}\n" for byte in data))
    print("Prepared both cockpit power-up RAM lanes.")


if __name__ == "__main__":
    main()
