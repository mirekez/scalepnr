#!/usr/bin/env python3
"""Extract a numeric-only free-frontier replay from a failed-route trace.

Search indexes are local to each SOURCE_SEARCH. Merge only resolved arcs,
then add the successful diagnostic witness. No device or cell names survive.
"""

import argparse
import gzip
import json
import re
from collections import defaultdict
from pathlib import Path

Node = tuple[int, int, int, int]


def extract(log: Path, output: Path, searches: int, cuts_output: Path | None = None) -> None:
    node_pattern = re.compile(
        r"SOURCE_SEARCH_NODE index=(\d+) parent=-?\d+ tile=(-?\d+),(-?\d+) kind=(\d+) node=(\d+)"
    )
    edge_pattern = re.compile(
        r"SOURCE_SEARCH_EDGE from=(\d+) tile=(-?\d+),(-?\d+) kind=(\d+) node=(\d+)"
    )
    witness_pattern = re.compile(
        r"SOURCE_DIAGNOSTIC_NODE tile=(-?\d+),(-?\d+) kind=(\d+) node=(\d+)"
    )
    cut_pattern = re.compile(
        r"SOURCE_(?:GROUNDING|TAKEOFF)_CUT tile=(-?\d+),(-?\d+) kind=(\d+) node=(\d+)"
    )
    arcs: dict[Node, set[Node]] = defaultdict(set)
    nodes: dict[int, Node] = {}
    pending: dict[int, set[Node]] = defaultdict(set)
    witness: list[Node] = []
    cuts: set[Node] = set()
    search = 0

    def flush() -> None:
        for index, targets in pending.items():
            if index not in nodes:
                raise ValueError(f"missing search node {index} in search {search}")
            arcs[nodes[index]].update(targets)
        nodes.clear()
        pending.clear()

    opener = gzip.open if log.suffix == ".gz" else open
    with opener(log, "rt") as stream:
        for line in stream:
            if "SOURCE_SEARCH begin" in line:
                flush()
                search += 1
            if 0 < search <= searches:
                match = node_pattern.search(line)
                if match:
                    index, *node = map(int, match.groups())
                    nodes[index] = tuple(node)
                match = edge_pattern.search(line)
                if match:
                    index, *node = map(int, match.groups())
                    pending[index].add(tuple(node))
            match = witness_pattern.search(line)
            if match:
                witness.append(tuple(map(int, match.groups())))
            match = cut_pattern.search(line)
            if match:
                cuts.add(tuple(map(int, match.groups())))
    flush()
    if len(witness) < 2:
        raise ValueError("trace has no successful diagnostic witness")
    if cuts and cuts_output is None:
        raise ValueError("witness requires occupied cuts; use --cuts-output and install real owners in the replay")
    for source, target in zip(witness, witness[1:]):
        arcs[source].add(target)
    with output.open("w") as stream:
        for source, targets in sorted(arcs.items()):
            row = [*source, len(targets)]
            for target in sorted(targets):
                row.extend(target)
            stream.write(" ".join(map(str, row)) + "\n")
    if cuts_output is not None:
        cuts_output.write_text(json.dumps({"occupied_nodes": sorted(cuts)}, indent=2) + "\n")
    print(f"groups={len(arcs)} edges={sum(map(len, arcs.values()))} "
          f"source={witness[0]} sink={witness[-1]} witness_nodes={len(witness)} cuts={len(cuts)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--searches", type=int, default=2)
    parser.add_argument("--cuts-output", type=Path,
                        help="explicit occupied-node manifest; replay must reconstruct their route owners")
    args = parser.parse_args()
    if args.searches < 1:
        parser.error("--searches must be positive")
    extract(args.log, args.output, args.searches, args.cuts_output)
