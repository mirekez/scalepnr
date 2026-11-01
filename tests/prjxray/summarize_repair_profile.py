#!/usr/bin/env python3
"""Summarize opt-in numeric repair counters without double-counting nested time."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass, field
import gzip
import json
import lzma
from pathlib import Path
import shlex
from typing import TextIO


@dataclass
class Group:
    calls: int = 0
    root_calls: int = 0
    results: Counter[str] = field(default_factory=Counter)
    counters: Counter[str] = field(default_factory=Counter)
    phases: Counter[str] = field(default_factory=Counter)
    roots: Counter[tuple[str, str, str, str]] = field(default_factory=Counter)
    revisions: Counter[tuple[str, ...]] = field(default_factory=Counter)
    slowest: list[dict[str, str]] = field(default_factory=list)

    def add(self, record: dict[str, str]) -> None:
        self.calls += 1
        self.results[record["kind"] + ":" + record["result"]] += 1
        metadata = {"id", "parent", "kind", "net", "route", "binding", "route_id",
                    "revision", "end_revision", "result", "wall_s"}
        for key, value in record.items():
            if key.endswith("_s") and key != "wall_s":
                self.phases[key] += float(value)
            elif key not in metadata:
                self.counters[key] += int(value)
        if record["parent"] == "0":
            self.root_calls += 1
            key = tuple(record[k] for k in ("net", "route", "route_id", "kind"))
            self.roots[key] += 1
            self.revisions[(*key, record["revision"])] += 1
            self.slowest.append(record)
            self.slowest.sort(key=lambda r: float(r["wall_s"]), reverse=True)
            del self.slowest[10:]

    def report(self) -> dict:
        return {
            "calls": self.calls, "root_calls": self.root_calls,
            "results": self.results, "counters": self.counters,
            "exclusive_seconds": self.phases,
            "exclusive_total_seconds": sum(self.phases.values()),
            "distinct_root_bindings": len(self.roots),
            "repeated_root_calls": sum(n - 1 for n in self.roots.values()),
            "same_start_revision_repeated_root_calls": sum(n - 1 for n in self.revisions.values()),
            "most_repeated_bindings": [
                {"net": k[0], "route": k[1], "route_id": k[2], "kind": k[3], "calls": n}
                for k, n in self.roots.most_common(10)
            ],
            "slowest_root_calls": self.slowest,
        }


def summarize(log: TextIO) -> dict:
    groups: dict[str, Group] = defaultdict(Group)
    chunk, stage, diagnostics = 0, "initialization", False
    for line in log:
        if "routeDesign stage: Clock routing" in line:
            chunk += 1
            stage = "Clock routing"
            diagnostics = False
        if "routeDesign stagnation detected:" in line or "routeDesign deadline diagnostics:" in line:
            diagnostics = True
        # The console logger can leave its preceding message unterminated.
        # Our records end with a newline, but need not begin at column zero.
        if "REPAIR_TOTAL context=" in line:
            text = line[line.index("REPAIR_TOTAL context="):]
            record = dict(token.split("=", 1) for token in shlex.split(text)[1:])
            context = record["context"]
            if context.startswith("before "):
                stage = context.removeprefix("before ")
                diagnostics = False
        elif "REPAIR_CALL id=" in line:
            text = line[line.index("REPAIR_CALL id="):]
            record = dict(token.split("=", 1) for token in shlex.split(text)[1:])
            suffix = " [diagnostics active at return]" if diagnostics else ""
            groups[f"chunk_{chunk}:{stage}{suffix}"].add(record)
    return {key: group.report() for key, group in groups.items()}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    opener = {".xz": lzma.open, ".gz": gzip.open}.get(args.log.suffix, open)
    with opener(args.log, "rt") as log:
        print(json.dumps(summarize(log), indent=2))


if __name__ == "__main__":
    main()
