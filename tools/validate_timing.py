#!/usr/bin/env python3
"""Require every timing check at all four Cyclone V operating corners."""
import argparse
import json
import hashlib
import math
from pathlib import Path
import re

CORNERS = {f'{speed} 1100mV {temp}C' for speed in ['Slow', 'Fast'] for temp in ['100', '-40']}
CHECKS = {'setup', 'hold', 'recovery', 'removal', 'minimum pulse width'}

def validate(text):
    if re.search(r'^\s*(?:Error(?: \(\d+\))?:|.*Timing requirements not met)', text, re.M | re.I):
        raise ValueError('timing analyzer reported an error or unmet requirement')
    rows = {}
    current = None
    for line in text.splitlines():
        m = re.search(r'Info: Analyzing (.+) Model\s*$', line)
        if m:
            current = m[1]
            if current not in CORNERS or current in rows:
                raise ValueError(f'unexpected or duplicated operating corner: {current}')
            rows[current] = {}
        m = re.search(r'Worst-case (setup|hold|recovery|removal|minimum pulse width) slack is (\S+)', line)
        if m:
            if current is None or m[1] in rows[current]:
                raise ValueError('unassociated or duplicated timing check')
            value = float(m[2])
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{current} {m[1]} fails: {value}')
            rows[current][m[1]] = value
    if set(rows) != CORNERS or any(set(row) != CHECKS for row in rows.values()):
        raise ValueError('incomplete timing report: require 4 corners, each with 5 checks')
    return rows

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('report', type=Path)
    p.add_argument('--rbf', type=Path)
    p.add_argument('--write-binding', action='store_true')
    a = p.parse_args()
    try:
        rows = validate(a.report.read_text())
        if a.write_binding and not a.rbf:
            raise ValueError('--write-binding requires --rbf')
        if a.rbf:
            rbf = a.rbf.read_bytes()
            if not rbf:
                raise ValueError('empty RBF')
            binding = {'version': 1, 'report_sha256': hashlib.sha256(a.report.read_bytes()).hexdigest(),
                       'rbf_sha256': hashlib.sha256(rbf).hexdigest()}
            seal = a.report.with_suffix(a.report.suffix + '.binding.json')
            if a.write_binding:
                seal.write_text(json.dumps(binding, indent=2) + '\n')
            elif json.loads(seal.read_text()) != binding:
                raise ValueError('report/RBF hash binding mismatch')
        print(json.dumps(rows, indent=2))
    except (OSError, ValueError) as e:
        p.exit(1, f'Timing rejected: {e}\n')
