#!/usr/bin/env python3
"""Check post-synthesis combinational depth between sequential boundaries."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SEQUENTIAL_PREFIXES = ("FD", "LD", "RAM", "SRL")
TRANSPARENT_TYPES = {
    "IBUF",
    "IBUFG",
    "OBUF",
    "OBUFT",
    "BUFG",
    "BUFGCE",
    "BUFGCTRL",
}


@dataclass(frozen=True)
class DepthResult:
    depth: int
    path: tuple[str, ...]


def is_sequential(cell_type: str) -> bool:
    return cell_type.startswith(SEQUENTIAL_PREFIXES)


def load_top(path: Path, top: str) -> dict[str, Any]:
    design = json.loads(path.read_text())
    modules = design.get("modules", {})
    if top not in modules:
        available = ", ".join(sorted(modules))
        raise ValueError(f"top module {top!r} not found; available: {available}")
    return modules[top]


def longest_chain(module: dict[str, Any]) -> DepthResult:
    cells: dict[str, dict[str, Any]] = module.get("cells", {})
    producers: dict[int, str] = {}
    for name, cell in cells.items():
        directions = cell.get("port_directions", {})
        for port, bits in cell.get("connections", {}).items():
            if directions.get(port) != "output":
                continue
            for bit in bits:
                if isinstance(bit, int):
                    producers[bit] = name

    memo: dict[str, DepthResult] = {}
    active: set[str] = set()

    def visit(name: str) -> DepthResult:
        if name in memo:
            return memo[name]
        if name in active:
            raise ValueError(f"combinational cycle reaches cell {name}")

        cell = cells[name]
        cell_type = str(cell.get("type", ""))
        if is_sequential(cell_type):
            result = DepthResult(0, ())
            memo[name] = result
            return result

        active.add(name)
        best = DepthResult(0, ())
        directions = cell.get("port_directions", {})
        for port, bits in cell.get("connections", {}).items():
            if directions.get(port) != "input":
                continue
            for bit in bits:
                if not isinstance(bit, int) or bit not in producers:
                    continue
                candidate = visit(producers[bit])
                if candidate.depth > best.depth:
                    best = candidate
        active.remove(name)

        contribution = 0 if cell_type in TRANSPARENT_TYPES else 1
        path = best.path
        if contribution:
            path = path + (f"{name}<{cell_type}>",)
        result = DepthResult(best.depth + contribution, path)
        memo[name] = result
        return result

    longest = DepthResult(0, ())
    for name in cells:
        candidate = visit(name)
        if candidate.depth > longest.depth:
            longest = candidate
    return longest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("design", type=Path, help="Yosys JSON netlist")
    parser.add_argument("--top", default="TestPipeline")
    parser.add_argument("--maximum", type=int, required=True)
    parser.add_argument("--result", type=Path)
    args = parser.parse_args()
    if args.maximum <= 0:
        parser.error("--maximum must be positive")

    try:
        result = longest_chain(load_top(args.design, args.top))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: cannot measure combinational chain length: {error}")
        return 2

    if args.result is not None:
        args.result.write_text(f"{result.depth}\n")
    print(f"maximum combinational chain: measured={result.depth} limit={args.maximum}")
    if result.path:
        print("longest path:")
        for index, cell in enumerate(result.path, 1):
            print(f"  {index:3d}: {cell}")
    if result.depth > args.maximum:
        print(
            f"ERROR: generated design violates maximum chain length: "
            f"{result.depth} > {args.maximum}"
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
