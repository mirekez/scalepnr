#!/usr/bin/env python3
"""Simulate the actual generated FIFO RTL with independent write/read clocks."""
import argparse
from pathlib import Path
import subprocess

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--iverilog', required=True)
p.add_argument('--vvp', required=True)
p.add_argument('--work', type=Path, required=True)
a = p.parse_args()
root = Path(__file__).resolve().parent
a.work.mkdir(parents=True, exist_ok=True)
binary = a.work / 'fifo.vvp'
subprocess.run([a.iverilog, '-g2012', '-s', 'async_fifo_tb', '-o', str(binary),
                str(root/'1k/2clk/async_fifo.sv'), str(root/'async_fifo_tb.sv')], check=True)
subprocess.run([a.vvp, str(binary)], check=True, timeout=30)
