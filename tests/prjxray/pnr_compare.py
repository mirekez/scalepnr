#!/usr/bin/env python3
"""Extract benchmark design data and verify completed PnR logs."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path
import re
import sys


@dataclass(frozen=True)
class DesignStats:
    cells: int
    nets: int
    ports: int
    sequential: bool
    cell_types: str


def load_design_stats(path: Path, top: str) -> DesignStats:
    design = json.loads(path.read_text())
    modules = design.get("modules", {})
    if top not in modules:
        available = ", ".join(sorted(modules))
        raise ValueError(f"top module {top!r} not found; available: {available}")
    module = modules[top]
    cells = module.get("cells", {})
    cell_counts = Counter(str(cell.get("type", "?")) for cell in cells.values())
    cell_types = ";".join(
        f"{cell_type}:{count}" for cell_type, count in sorted(cell_counts.items())
    )
    sequential = any(name.startswith(("FD", "LD")) for name in cell_counts)
    return DesignStats(
        cells=len(cells),
        nets=len(module.get("netnames", {})),
        ports=len(module.get("ports", {})),
        sequential=sequential,
        cell_types=cell_types,
    )


def verify_scalepnr(path: Path, expect_clock: bool) -> None:
    text = path.read_text(errors="replace")
    if "routeDesign stage report: reason=complete" not in text:
        raise ValueError("missing completed routeDesign stage report")

    final_tasks: dict[str, int] = {}
    for stage in (
        "Basic routing",
        "Moving sources",
        "Fanouts routing",
        "Moving destinations",
    ):
        reports = [
            line
            for line in text.splitlines()
            if f"routeDesign stage report: stage={stage}," in line
        ]
        if not reports:
            raise ValueError(f"missing {stage} stage report")
        report = reports[-1]
        if stage in ("Moving sources", "Moving destinations") and "timeout=false" not in report:
            raise ValueError(f"{stage} timed out: {report}")
        tasks = re.search(r"tasks=(\d+)->(\d+)", report)
        if tasks is None:
            raise ValueError(f"{stage} report has no task counts: {report}")
        final_tasks[stage] = int(tasks.group(2))

    # Basic may hand trunks to source relocation, but Fanouts cannot begin until
    # that mandatory barrier has completed every physical source trunk.
    if final_tasks["Moving sources"] != 0:
        raise ValueError("Moving sources retained unrouted trunk tasks")
    # Fanout leftovers are the documented input to destination relocation.
    if final_tasks["Moving destinations"] != 0:
        raise ValueError("Moving destinations retained unrouted physical routes")

    if expect_clock:
        clock_reports = [
            line for line in text.splitlines() if "clock routing clocks=" in line
        ]
        if not clock_reports:
            raise ValueError("missing clock-routing completion report")
        if not re.search(r"failed=0(?:\s|$)", clock_reports[-1]):
            raise ValueError(f"clock routing retained failures: {clock_reports[-1]}")


def verify_nextpnr(path: Path) -> None:
    text = path.read_text(errors="replace")
    required = ("Routing complete.", "Program finished normally.")
    for marker in required:
        if marker not in text:
            raise ValueError(f"missing nextpnr completion marker: {marker}")
    if re.search(r"^ERROR:", text, re.MULTILINE):
        raise ValueError("nextpnr log contains an ERROR line")

    route_iterations = re.findall(
        r"iter=\d+\s+wires=\d+\s+overused=(\d+)\s+overuse=(\d+)", text
    )
    if not route_iterations:
        raise ValueError("missing nextpnr router utilization report")
    overused, overuse = route_iterations[-1]
    if int(overused) != 0 or int(overuse) != 0:
        raise ValueError(
            f"nextpnr retained overused routing resources: "
            f"overused={overused}, overuse={overuse}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    stats_parser = subparsers.add_parser("design-stats")
    stats_parser.add_argument("json", type=Path)
    stats_parser.add_argument("--top", required=True)

    scalepnr_parser = subparsers.add_parser("verify-scalepnr")
    scalepnr_parser.add_argument("log", type=Path)
    scalepnr_parser.add_argument("--expect-clock", action="store_true")

    nextpnr_parser = subparsers.add_parser("verify-nextpnr")
    nextpnr_parser.add_argument("log", type=Path)

    args = parser.parse_args()
    try:
        if args.command == "design-stats":
            stats = load_design_stats(args.json, args.top)
            print(
                stats.cells,
                stats.nets,
                stats.ports,
                int(stats.sequential),
                stats.cell_types,
                sep="\t",
            )
        elif args.command == "verify-scalepnr":
            verify_scalepnr(args.log, args.expect_clock)
            print("0")
        elif args.command == "verify-nextpnr":
            verify_nextpnr(args.log)
            print("0")
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
