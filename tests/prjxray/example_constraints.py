#!/usr/bin/env python3
"""Deterministic package pin allocation for boardless full-flow examples."""
import argparse
import csv
import json
import math
import re
from pathlib import Path


def constraints(design, top, clock, period, pins):
    if not math.isfinite(period) or period <= 0:
        raise ValueError("clock period must be positive and finite")
    module = design["modules"][top]
    ports = module["ports"]
    if clock not in ports or ports[clock]["direction"] != "input" or len(ports[clock]["bits"]) != 1:
        raise ValueError("clock must name a scalar input port")
    # Do not let an unsupported hard macro disappear silently in db2fasm.
    supported = re.compile(r"(?:LUT[1-6]|FDRE|FDSE|FDCE|FDPE|IBUF|OBUF|BUFG|VCC|GND)")
    unsupported = sorted({c["type"] for c in module["cells"].values()
                          if not supported.fullmatch(c["type"])})
    if unsupported:
        raise ValueError(f"unsupported mapped primitives: {', '.join(unsupported)}")
    usable = sorted((p for p in pins if p["site"].startswith("IOB_")
                     and "IOB33" in p["tile"] and p["pin_function"].startswith("IO_")),
                    key=lambda p: p["pin"])
    clock_pin = next((p for p in usable if "MRCC" in p["pin_function"]), None)
    if clock_pin is None:
        raise ValueError("package has no MRCC clock input pin")
    names = []
    for name, port in sorted(ports.items()):
        if port["direction"] not in ("input", "output"):
            raise ValueError(f"boardless examples require unidirectional ports: {name}")
        if not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", name):
            raise ValueError(f"unsafe Tcl port name: {name}")
        if name == clock:
            continue
        offset = port.get("offset", 0)
        names.extend([name] if len(port["bits"]) == 1 and not offset else
                     [f"{name}[{i}]" for i in range(offset, offset + len(port["bits"]))])
    remaining = [p for p in usable if p["pin"] != clock_pin["pin"]]
    if len(names) > len(remaining):
        raise ValueError(f"{len(names) + 1} ports exceed {len(usable)} available I/O pins")
    lines = ["# Boardless test pinout: NOT a board programming constraint file.",
             f"create_clock -name {clock} -period {period:g} [get_ports {{{clock}}}]"]
    for name, pin in [(clock, clock_pin), *zip(names, remaining)]:
        if not re.fullmatch(r"[A-Z]+[0-9]+", pin["pin"]):
            raise ValueError("invalid package pin name")
        lines += [f"set_property IOSTANDARD LVCMOS33 [get_ports {{{name}}}]",
                  f"set_property PACKAGE_PIN {pin['pin']} [get_ports {{{name}}}]"]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("design", type=Path)
    parser.add_argument("top")
    parser.add_argument("clock")
    parser.add_argument("period", type=float)
    parser.add_argument("pins", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    with args.pins.open() as stream:
        pins = list(csv.DictReader(stream))
    args.output.write_text(constraints(json.loads(args.design.read_text()), args.top,
                                      args.clock, args.period, pins))


if __name__ == "__main__":
    main()
