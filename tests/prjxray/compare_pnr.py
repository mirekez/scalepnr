#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable


def parse_export(path: Path) -> tuple[dict[str, tuple[str, str, str, str]], dict[str, set[str]]]:
    section: str | None = None
    placement: dict[str, tuple[str, str, str, str]] = {}
    routing: dict[str, set[str]] = defaultdict(set)

    for line in path.read_text(errors="replace").splitlines():
        if line.startswith("SECTION: PLACEMENT"):
            section = "placement"
            continue
        if line.startswith("SECTION: ROUTING_PIPS"):
            section = "routing"
            continue
        if not line or line.startswith("=") or line in {"cell,ref_name,site,bel,loc", "net,pip"}:
            continue
        if section == "placement":
            parts = line.split(",", 4)
            if len(parts) == 5:
                placement[parts[0]] = tuple(parts[1:])  # type: ignore[assignment]
        elif section == "routing" and "," in line:
            net, pip = line.split(",", 1)
            routing[net].add(pip)

    return placement, routing


def tile_type(pip: str) -> str:
    tile = pip.split("/", 1)[0] if "/" in pip else pip.split(".", 1)[0]
    return tile.split("_X", 1)[0]


def pip_class(pip: str) -> str:
    typ = tile_type(pip)
    if typ.startswith("INT_"):
        return "INT"
    if typ.startswith("CLB"):
        return "CLB_SITE"
    if "IO" in typ or typ.startswith(("LIO", "RIO")):
        return "IO"
    if "CLK" in typ or "HCLK" in typ:
        return "CLOCK"
    return "OTHER"


def counter_text(counter: Counter[str]) -> str:
    return ", ".join(f"{name}={count}" for name, count in counter.most_common()) or "none"


def print_top_nets(title: str, nets: Iterable[tuple[str, set[str]]], limit: int) -> None:
    print(title)
    shown = 0
    for net, pips in nets:
        if shown >= limit:
            break
        print(f"  {len(pips):5d} {net} [{counter_text(Counter(pip_class(pip) for pip in pips))}]")
        shown += 1
    if shown == 0:
        print("  none")


def classify_exports(scalepnr: Path, vivado: Path, top: int) -> None:
    sp, sr = parse_export(scalepnr)
    vp, vr = parse_export(vivado)

    placement_common = set(sp) & set(vp)
    placement_mismatches = [name for name in placement_common if sp[name] != vp[name]]
    print("PLACEMENT")
    print(f"  scalepnr rows: {len(sp)}")
    print(f"  vivado rows:   {len(vp)}")
    print(f"  common rows:   {len(placement_common)}")
    print(f"  only scalepnr: {len(set(sp) - set(vp))}")
    print(f"  only vivado:   {len(set(vp) - set(sp))}")
    print(f"  mismatches:    {len(placement_mismatches)}")
    if placement_mismatches:
        for name in sorted(placement_mismatches)[:top]:
            print(f"    {name}: scalepnr={sp[name]} vivado={vp[name]}")

    scalepnr_pips = {pip for pips in sr.values() for pip in pips}
    vivado_pips = {pip for pips in vr.values() for pip in pips}
    common_pips = scalepnr_pips & vivado_pips
    scalepnr_only = scalepnr_pips - vivado_pips
    vivado_only = vivado_pips - scalepnr_pips
    print("\nROUTING TOTALS")
    print(f"  scalepnr net rows: {sum(len(pips) for pips in sr.values())}")
    print(f"  vivado net rows:   {sum(len(pips) for pips in vr.values())}")
    print(f"  scalepnr nets:     {len(sr)}")
    print(f"  vivado nets:       {len(vr)}")
    print(f"  common nets:       {len(set(sr) & set(vr))}")
    print(f"  only scalepnr net: {len(set(sr) - set(vr))}")
    print(f"  only vivado net:   {len(set(vr) - set(sr))}")
    print(f"  scalepnr pips:     {len(scalepnr_pips)}")
    print(f"  vivado pips:       {len(vivado_pips)}")
    print(f"  common pips:       {len(common_pips)}")
    print(f"  only scalepnr pip: {len(scalepnr_only)} [{counter_text(Counter(pip_class(pip) for pip in scalepnr_only))}]")
    print(f"  only vivado pip:   {len(vivado_only)} [{counter_text(Counter(pip_class(pip) for pip in vivado_only))}]")

    print("\nROUTING BY TILE TYPE")
    for label, pips in (("common", common_pips), ("only scalepnr", scalepnr_only), ("only vivado", vivado_only)):
        print(f"  {label}: {counter_text(Counter(tile_type(pip) for pip in pips))}")

    common_nets = set(sr) & set(vr)
    print("\nCOMMON NETS WITH LARGEST DIFFERENCES")
    for net in sorted(common_nets, key=lambda item: abs(len(vr[item]) - len(sr[item])), reverse=True)[:top]:
        s_only = sr[net] - vr[net]
        v_only = vr[net] - sr[net]
        print(
            f"  {len(sr[net]):5d} vs {len(vr[net]):5d}; common={len(sr[net] & vr[net]):4d}; "
            f"s_only={len(s_only):4d}; v_only={len(v_only):4d}; {net}"
        )
        print(f"      s_only: {counter_text(Counter(pip_class(pip) for pip in s_only))}")
        print(f"      v_only: {counter_text(Counter(pip_class(pip) for pip in v_only))}")

    print_top_nets(
        "\nVIVADO-ONLY NETS",
        sorted(((net, vr[net]) for net in set(vr) - set(sr)), key=lambda item: -len(item[1])),
        top,
    )
    print_top_nets(
        "\nSCALEPNR-ONLY NETS",
        sorted(((net, sr[net]) for net in set(sr) - set(vr)), key=lambda item: -len(item[1])),
        top,
    )


def classify_routing_tcl(path: Path) -> None:
    text = path.read_text(errors="replace")
    groups: list[tuple[str, str, int, int]] = []
    current_name: str | None = None
    current_lines: list[str] = []

    def flush() -> None:
        if current_name is None:
            return
        block = "\n".join(current_lines)
        if "disconnected from selected root" in block:
            status = "skipped_disconnected_tree"
        elif "multiple source roots" in block:
            status = "skipped_multi_root"
        elif "single-node scalepnr route" in block:
            status = "skipped_single_node"
        elif "no route nodes exported" in block:
            status = "skipped_no_nodes"
        elif "scalepnr_set_fixed_route_tree" in block:
            status = "fixed_tree"
        elif "scalepnr_set_fixed_route" in block:
            status = "fixed_simple"
        else:
            status = "unknown"
        groups.append((current_name, status, block.count("# full_node "), block.count("# pip ")))

    for line in text.splitlines():
        if line.startswith("# route "):
            flush()
            current_name = line[len("# route "):]
            current_lines = [line]
        elif current_name is not None:
            current_lines.append(line)
    flush()

    print("\nROUTING TCL EXPORT")
    print(f"  route groups:          {text.count('# route ')}")
    print(f"  simple fixed routes:   {text.count('scalepnr_set_fixed_route $net')}")
    print(f"  fixed route trees:     {text.count('scalepnr_set_fixed_route_tree $net')}")
    print(f"  skipped multi-root:    {text.count('multiple source roots')}")
    print(f"  skipped disconnected:  {text.count('disconnected from selected root')}")
    print(f"  skipped single-node:   {text.count('single-node scalepnr route')}")
    print(f"  skipped no-node:       {text.count('no route nodes exported')}")
    print(f"  annotated full nodes:  {text.count('# full_node ')}")
    print(f"  annotated route pips:  {text.count('# pip ')}")
    print("  group status summary:")
    for status, count in Counter(status for _, status, _, _ in groups).most_common():
        pips = sum(pip_count for _, item_status, _, pip_count in groups if item_status == status)
        nodes = sum(node_count for _, item_status, node_count, _ in groups if item_status == status)
        print(f"    {status}: groups={count} full_nodes={nodes} pips={pips}")
    for status in ("skipped_disconnected_tree", "skipped_multi_root", "skipped_single_node", "skipped_no_nodes"):
        examples = sorted((item for item in groups if item[1] == status), key=lambda item: -item[3])
        if examples:
            print(f"  largest {status} examples:")
            for name, _, node_count, pip_count in examples[:5]:
                print(f"    pips={pip_count:4d} full_nodes={node_count:4d} {name}")


def classify_vivado_log(path: Path) -> None:
    lines = path.read_text(errors="replace").splitlines()
    print("\nVIVADO LOG")
    for label in ("ERROR:", "CRITICAL WARNING", "FIXED_ROUTE failed", "FIXED_ROUTE tree failed", "Number of Failed Nets"):
        hits = [line for line in lines if label in line and not line.startswith("##")]
        print(f"  {label}: {len(hits)}")
        for line in hits[:5]:
            print(f"    {line[:240]}")


def classify_design_db(path: Path) -> None:
    state = json.loads(path.read_text())
    routes: list[tuple[str, int, int, int]] = []
    zero_pip_types: Counter[str] = Counter()
    zero_pip_node_kinds: Counter[str] = Counter()
    empty_routes = 0
    for inst in state.get("insts", []):
        for route in inst.get("routes", []):
            if not route:
                empty_routes += 1
                continue
            name = next((wire.get("net") or wire.get("net_name") for wire in route if wire.get("net") or wire.get("net_name")), "")
            pip_count = sum(len(wire.get("annotation", {}).get("fasm_features", [])) for wire in route)
            node_count = sum(len(wire.get("annotation", {}).get("nodes", [])) for wire in route)
            routes.append((str(name), pip_count, node_count, len(route)))
            if pip_count == 0:
                zero_pip_types["+".join(str(wire.get("type", "?")) for wire in route)] += 1
                kinds: list[str] = []
                for wire in route:
                    for node in wire.get("annotation", {}).get("nodes", []):
                        kind = node.get("kind")
                        if kind:
                            kinds.append(str(kind))
                zero_pip_node_kinds["+".join(kinds) if kinds else "none"] += 1

    print("\nDESIGN DB ROUTE ANNOTATION")
    print(f"  nonempty routes:       {len(routes)}")
    print(f"  empty route vectors:   {empty_routes}")
    print(f"  zero-pip routes:       {sum(1 for _, pips, _, _ in routes if pips == 0)}")
    print(f"  zero-node routes:      {sum(1 for _, _, nodes, _ in routes if nodes == 0)}")
    print(f"  pip count histogram:   {counter_text(Counter(pips for _, pips, _, _ in routes).most_common().__iter__()) if False else counter_text(Counter(str(pips) for _, pips, _, _ in routes))}")
    print(f"  zero-pip wire types:   {counter_text(zero_pip_types)}")
    print("  zero-pip node-kind examples:")
    for kinds, count in zero_pip_node_kinds.most_common(8):
        print(f"    {count:4d} {kinds}")

    print("  largest zero-pip examples:")
    examples = sorted((item for item in routes if item[1] == 0), key=lambda item: -item[3])
    for name, pip_count, node_count, fragments in examples[:10]:
        print(f"    fragments={fragments:3d} nodes={node_count:3d} pips={pip_count:3d} {name}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare scalepnr and Vivado PNR textual exports")
    parser.add_argument("--scalepnr", type=Path, default=Path("vivado_export/scalepnr_place_route_export.txt"))
    parser.add_argument("--vivado", type=Path, default=Path("vivado_export/place_route_export.txt"))
    parser.add_argument("--routing-tcl", type=Path, default=Path("vivado_export/routing.tcl"))
    parser.add_argument("--vivado-log", type=Path, default=Path("vivado_export/create_project.tcl.log"))
    parser.add_argument("--design-db", type=Path, default=Path("design_state.db"))
    parser.add_argument("--top", type=int, default=20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    classify_exports(args.scalepnr, args.vivado, args.top)
    if args.routing_tcl.exists():
        classify_routing_tcl(args.routing_tcl)
    if args.vivado_log.exists():
        classify_vivado_log(args.vivado_log)
    if args.design_db.exists():
        classify_design_db(args.design_db)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
