#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from db2fasm import (
    BEL_LETTERS,
    CLB_TYPES,
    FasmOutput,
    PackedPlacement,
    PlacedInst,
    PrjxrayDb,
    TileInfo,
    cell_kind,
    group_by_clb_tile,
    load_state,
    placed_insts,
)
from a7_packing import aggregate_a7_route_tree, pack_a7_clb_placements


DEFAULT_PART = "xc7a100tfgg676-1"


@dataclass(frozen=True)
class VivadoPlacement:
    inst: PlacedInst
    site: str
    bel: str | None
    constrain: bool = True


@dataclass(frozen=True)
class IoAssignment:
    port: str
    properties: dict[str, str]


@dataclass(frozen=True)
class RouteExport:
    net_name: str
    net_candidates: list[str]
    pin_candidates: list[tuple[str, str]]
    paths: list[list[str]]
    full_paths: list[list[str]]
    pips: list[str]


def tcl_braced(value: str) -> str:
    return "{" + value.replace("\\", "\\\\").replace("}", "\\}") + "}"


def tcl_list(values: Iterable[str]) -> str:
    return " ".join(tcl_braced(value) for value in values)


def tcl_pair_list(values: Iterable[tuple[str, str]]) -> str:
    return " ".join("[list " + tcl_braced(left) + " " + tcl_braced(right) + "]" for left, right in values)


def rel_to(path: Path, base: Path) -> str:
    return os.path.relpath(path.resolve(), base.resolve())


def infer_top(state: dict[str, Any]) -> str | None:
    for inst in state.get("insts", []):
        if inst.get("name", "") == "" and inst.get("type"):
            return str(inst["type"])
    return state.get("top")


def build_edif_from_sv(sv_sources: list[Path], output_dir: Path, top: str) -> Path:
    if not sv_sources:
        raise ValueError("no SystemVerilog sources provided for EDIF generation")
    edif = output_dir / f"{top}.edf"
    command = [
        "yosys",
        "-p",
        f"clkbufmap -inpad IBUFG *clk*; synth_xilinx -nocarry -flatten -arch xc7 -top {top}; write_edif -pvector bra -attrprop {edif}",
        *[str(source) for source in sv_sources],
    ]
    subprocess.run(command, check=True)
    return edif


def site_names(tile: TileInfo) -> list[str]:
    return [site for site, _ in sorted(tile.sites.items())]


def site_name_for_index(tile: TileInfo, site_index: int) -> str | None:
    sites = site_names(tile)
    if not sites:
        return None
    if site_index < len(sites):
        return sites[site_index]
    return sites[-1]


def iob_site_for_inst(tile: TileInfo, inst: PlacedInst) -> str | None:
    sites = site_names(tile)
    if not sites:
        return None
    if tile.type.endswith("_SING"):
        return sites[0]
    index = 1 if inst.pos == 1 else 0
    if index < len(sites):
        return sites[index]
    return sites[-1]


def load_package_pin_sites(db_dir: Path) -> dict[str, str]:
    path = db_dir / "package_pins.csv"
    if not path.exists():
        return {}
    with path.open(newline="") as f:
        return {
            row["pin"]: row["site"]
            for row in csv.DictReader(f)
            if row.get("pin") and row.get("site")
        }


def iopad_port_variants(port: str) -> list[str]:
    variants = [port]
    match = re.fullmatch(r"(.+)\[(\d+)\]", port)
    if match:
        base, bit = match.groups()
        variants.append(f"{base}_{bit}")
        if bit == "0":
            variants.append(base)
    out: list[str] = []
    for variant in variants:
        if variant and variant not in out:
            out.append(variant)
    return out


def io_assignment_for_iopad_inst(inst_name: str, assignments: list[IoAssignment]) -> IoAssignment | None:
    for assignment in assignments:
        for variant in iopad_port_variants(assignment.port):
            if inst_name.endswith("." + variant):
                return assignment
    return None


def lut_bel(inst: PlacedInst, packed: PackedPlacement) -> str:
    # Vivado BEL constraints name physical LUT sites, not the logical LUT
    # width. Smaller LUT primitives can legally occupy the selected 6LUT BEL.
    return f"{BEL_LETTERS[packed.bel_index]}6LUT"


def clb_bel(inst: PlacedInst, packed: PackedPlacement) -> str | None:
    kind = cell_kind(inst)
    if kind == "FD":
        return f"{BEL_LETTERS[packed.bel_index]}FF"
    if kind == "LUT":
        return lut_bel(inst, packed)
    if kind == "CARRY":
        return "CARRY4"
    if kind == "MUX":
        if inst.cell_type.startswith("MUXF8"):
            return "F8MUX"
        return "F7BMUX" if packed.bel_index else "F7AMUX"
    return None


def parse_slice_site(site: str) -> tuple[int, int] | None:
    match = re.fullmatch(r"SLICE_X(\d+)Y(\d+)", site)
    if not match:
        return None
    return int(match.group(1)), int(match.group(2))


def carry_chain_driver(inst: PlacedInst) -> str | None:
    for conn in inst.raw.get("annotation", {}).get("connections", []):
        if conn.get("sink_port") == "CI" and conn.get("driver_type") == "CARRY4":
            driver = str(conn.get("driver_inst", ""))
            return driver or None
    return None


def legalize_carry_chain_placements(
    placements: list[VivadoPlacement],
    db: PrjxrayDb,
    warnings: list[str],
) -> list[VivadoPlacement]:
    site_by_inst = {placement.inst.name: placement.site for placement in placements}
    all_sites = {site for tile in db.tilegrid.values() for site in tile.sites}

    for placement in placements:
        if placement.inst.cell_type != "CARRY4":
            continue
        driver = carry_chain_driver(placement.inst)
        if not driver or driver not in site_by_inst:
            continue
        parsed = parse_slice_site(site_by_inst[placement.inst.name])
        if parsed is None:
            continue
        site_x, site_y = parsed
        driver_site = f"SLICE_X{site_x}Y{site_y - 1}"
        if driver_site not in all_sites:
            warnings.append(
                f"cannot legalize carry chain {driver} -> {placement.inst.name}: missing site {driver_site}"
            )
            continue
        if site_by_inst[driver] != driver_site:
            warnings.append(
                f"legalized carry-chain placement for {driver}: {site_by_inst[driver]} -> {driver_site}"
            )
            site_by_inst[driver] = driver_site

    return [
        VivadoPlacement(placement.inst, site_by_inst.get(placement.inst.name, placement.site), placement.bel)
        for placement in placements
    ]


def collect_placements(
    state: dict[str, Any],
    db: PrjxrayDb,
    io_assignments: list[IoAssignment],
    package_pin_sites: dict[str, str],
) -> tuple[list[VivadoPlacement], list[str]]:
    out = FasmOutput()
    grouped = group_by_clb_tile(state, db, out)
    placements: list[VivadoPlacement] = []
    warnings: list[str] = list(out.warnings)
    clb_tiles = {name: tile for name, tile in db.tilegrid.items() if tile.type in CLB_TYPES}
    pack_result = pack_a7_clb_placements(grouped, clb_tiles)
    packed = pack_result.cells
    warnings.extend(pack_result.warnings)

    tile_by_inst: dict[str, TileInfo] = {}
    for tile_state in grouped.values():
        for inst in tile_state.insts:
            tile_by_inst[inst.name] = tile_state.clb_tile

    for inst in placed_insts(state):
        tile = db.tilegrid.get(inst.resource_tile)
        if tile is None:
            warnings.append(f"skip placement for {inst.name}: unknown tile {inst.resource_tile}")
            continue

        if tile.type in CLB_TYPES or inst.name in packed:
            packed_cell = packed.get(inst.name)
            if packed_cell is None:
                warnings.append(f"skip CLB placement for unsupported cell {inst.cell_type}: {inst.name}")
                continue
            clb_tile = db.tilegrid.get(packed_cell.tile_name, tile_by_inst.get(inst.name, tile))
            place = packed_cell.placement
            site = site_name_for_index(clb_tile, place.site_index)
            if site is None:
                warnings.append(f"skip CLB placement for {inst.name}: tile {clb_tile.name} has no sites")
                continue
            placements.append(VivadoPlacement(inst, site, clb_bel(inst, place), packed_cell.constrain))
            continue

        if inst.cell_type in {"IBUF", "OBUF"}:
            site = None
            assignment = io_assignment_for_iopad_inst(inst.name, io_assignments)
            package_pin = assignment.properties.get("PACKAGE_PIN") if assignment else None
            if package_pin:
                site = package_pin_sites.get(package_pin)
                if site and site not in tile.sites:
                    warnings.append(
                        f"package pin {package_pin} maps {inst.name} to site {site}, "
                        f"which is not in tile {tile.name}; using tile site fallback"
                    )
                    site = None
            if site is None:
                site = iob_site_for_inst(tile, inst)
            if site is None:
                warnings.append(f"skip IOB placement for {inst.name}: tile {tile.name} has no sites")
                continue
            placements.append(VivadoPlacement(inst, site, None))
            continue

        warnings.append(f"skip placement for unsupported placed cell {inst.cell_type}: {inst.name}")

    return placements, warnings


def route_name(inst: dict[str, Any], route: list[dict[str, Any]]) -> str:
    for wire in route:
        net = wire.get("net")
        if net:
            return str(net)
    return str(inst.get("name", "<unnamed-route>"))


def route_net_names(inst: dict[str, Any], route: list[dict[str, Any]]) -> list[str]:
    nets: list[str] = []
    for wire in route:
        net = wire.get("net")
        if net and str(net) not in nets:
            nets.append(str(net))

    final_port = ""
    for wire in reversed(route):
        if wire.get("type") == "tile_pin" and wire.get("port"):
            final_port = str(wire.get("port", ""))
            break
    for conn in inst.get("connections", []):
        conn_net = str(conn.get("net", ""))
        if not conn_net:
            continue
        if final_port and str(conn.get("sink_port", "")) != final_port:
            continue
        if conn_net not in nets:
            nets.append(conn_net)

    if not nets:
        nets.append(str(inst.get("name", "<unnamed-route>")))
    return nets


def route_pin_net_names(inst: dict[str, Any], route: list[dict[str, Any]], primary_net: str) -> list[str]:
    nets: list[str] = [primary_net] if primary_net else []
    final_port = ""
    for wire in reversed(route):
        if wire.get("type") == "tile_pin" and wire.get("port"):
            final_port = str(wire.get("port", ""))
            break
    if final_port:
        for conn in inst.get("connections", []):
            if str(conn.get("sink_port", "")) != final_port:
                continue
            conn_net = str(conn.get("net", ""))
            if conn_net and conn_net not in nets:
                nets.append(conn_net)
    return nets


def vivado_node_name(full_name: str) -> str:
    tile, sep, node = full_name.partition(".")
    if not sep:
        return full_name
    return f"{tile}/{node}"


def vivado_node_wire(node_name: str) -> str:
    return node_name.rsplit("/", 1)[-1]


def vivado_node_tile(node_name: str) -> str:
    return node_name.split("/", 1)[0]


def is_intermediate_end_node(node_name: str) -> bool:
    return "END" in vivado_node_wire(node_name)


def carry_di_tail_node(tile: str, port: str) -> str | None:
    match = re.fullmatch(r"DI\[(\d+)\]|DI(\d+)", port)
    if not match:
        return None
    bit_text = match.group(1) or match.group(2)
    bit = int(bit_text)
    if bit < 0 or bit >= 4:
        return None
    # Artix-7 CLB CARRY4 DI inputs enter the INT switchbox through BYP tails.
    # Keep this in the prjxray exporter rather than the generic router.
    byp_index = [1, 4, 3, 6][bit]
    if "_L_" in tile or tile.startswith("INT_L_"):
        return f"{tile}/BYP_L{byp_index}"
    return f"{tile}/BYP{byp_index}"


def clb_bypass_tail_name(tile: str, index: int) -> str:
    if "_L_" in tile or tile.startswith("INT_L_"):
        return f"{tile}/BYP_L{index}"
    return f"{tile}/BYP{index}"


def fd_d_tail_node(tile: str, pos: int) -> str | None:
    if pos < 0:
        return None
    bel = (pos % 128) // 4
    if bel < 0 or bel >= 4:
        return None
    # D inputs use the same CARRY/CLB bypass tails for this Artix-7 export.
    return clb_bypass_tail_name(tile, [1, 4, 3, 6][bel])


def fd_control_tail_node(tile: str) -> str | None:
    if not tile:
        return None
    if "_L_" in tile or tile.startswith("INT_L_"):
        return f"{tile}/CTRL_L1"
    return f"{tile}/CTRL1"


def obuf_input_tail_node(tile: str) -> str | None:
    if not tile:
        return None
    if "_L_" in tile or tile.startswith("INT_L_"):
        return f"{tile}/IMUX_L34"
    return f"{tile}/IMUX34"


def tile_pin_tail_node(tile: str, port: str, pos: int) -> str | None:
    carry_tail = carry_di_tail_node(tile, port)
    if carry_tail:
        return carry_tail
    if port == "D":
        return fd_d_tail_node(tile, pos)
    if port in {"R", "S", "CLR", "PRE", "SRST", "ARST"}:
        return fd_control_tail_node(tile)
    if port == "I":
        return obuf_input_tail_node(tile)
    return None


def is_terminal_alias(node_name: str) -> bool:
    wire = vivado_node_wire(node_name)
    return (
        "END" in wire
        or "IMUX" in wire
        or "BYP" in wire
        or "CTRL" in wire
        or "FAN" in wire
        or "GCLK" in wire
        or "LOGIC_OUTS" in wire
    )


def is_fixed_route_terminal_tail(node_name: str) -> bool:
    wire = vivado_node_wire(node_name)
    return (
        "IMUX" in wire
        or "FAN" in wire
        or "GCLK" in wire
        or "LOGIC_OUTS" in wire
    )


def should_skip_tile_pin_tail(raw_nodes: list[str], tail: str) -> bool:
    if not raw_nodes:
        return False
    last = raw_nodes[-1]
    if last == tail:
        return True
    if vivado_node_tile(last) != vivado_node_tile(tail):
        return False

    last_wire = vivado_node_wire(last)
    if "END" in last_wire:
        return True
    if "GCLK" in last_wire:
        raw_nodes.pop()
        return True
    return is_terminal_alias(last)


def direct_pip_feature(db: PrjxrayDb, src_node: str, dst_node: str) -> str | None:
    src_tile = vivado_node_tile(src_node)
    dst_tile = vivado_node_tile(dst_node)
    if src_tile != dst_tile:
        return None
    info = db.tilegrid.get(src_tile)
    if info is None:
        return None
    src_wire = vivado_node_wire(src_node)
    dst_wire = vivado_node_wire(dst_node)
    if src_wire in db.pip_sources_by_type_dst.get((info.type, dst_wire), set()):
        return f"{src_tile}.{dst_wire}.{src_wire}"
    return None


def intermediate_pip_node(db: PrjxrayDb, src_node: str, dst_node: str) -> str | None:
    src_tile = vivado_node_tile(src_node)
    dst_tile = vivado_node_tile(dst_node)
    if src_tile != dst_tile:
        return None
    info = db.tilegrid.get(src_tile)
    if info is None:
        return None
    src_wire = vivado_node_wire(src_node)
    dst_wire = vivado_node_wire(dst_node)
    for mid_wire in sorted(db.pip_sources_by_type_dst.get((info.type, dst_wire), set())):
        if src_wire in db.pip_sources_by_type_dst.get((info.type, mid_wire), set()):
            return f"{src_tile}/{mid_wire}"
    return None


def expand_same_tile_nodes(db: PrjxrayDb, nodes: list[str]) -> list[str]:
    if len(nodes) < 2:
        return nodes
    expanded = [nodes[0]]
    for dst in nodes[1:]:
        src = expanded[-1]
        if direct_pip_feature(db, src, dst) is None:
            mid = intermediate_pip_node(db, src, dst)
            if mid and mid != src and mid != dst:
                expanded.append(mid)
        if expanded[-1] != dst:
            expanded.append(dst)
    return expanded


def route_full_nodes(route: list[dict[str, Any]], db: PrjxrayDb) -> list[str]:
    raw_nodes: list[str] = []
    for wire in route:
        ann = wire.get("annotation", {})
        if wire.get("type") == "tile_pin":
            tile = str(ann.get("from_cb_tile", ""))
            port = str(wire.get("port", ""))
            pos = int(wire.get("pos", -1))
            tail = tile_pin_tail_node(tile, port, pos)
            if tail:
                if not should_skip_tile_pin_tail(raw_nodes, tail):
                    raw_nodes.append(tail)
                continue
        ann_nodes = ann.get("nodes", [])
        for node in ann_nodes:
            full_name = node.get("full_name")
            if full_name:
                vivado_name = vivado_node_name(str(full_name))
                if not raw_nodes or raw_nodes[-1] != vivado_name:
                    raw_nodes.append(vivado_name)

    nodes: list[str] = []
    for node in expand_same_tile_nodes(db, raw_nodes):
        if not nodes or nodes[-1] != node:
            nodes.append(node)
    return nodes


def route_nodes(route: list[dict[str, Any]], db: PrjxrayDb) -> list[str]:
    nodes: list[str] = []
    full_nodes = route_full_nodes(route, db)
    for index, node in enumerate(full_nodes):
        if is_intermediate_end_node(node):
            final_node = index == len(full_nodes) - 1
            feeds_terminal_tail = (
                index + 1 < len(full_nodes)
                and is_fixed_route_terminal_tail(full_nodes[index + 1])
            )
            if not final_node and not feeds_terminal_tail:
                continue
        if not nodes or nodes[-1] != node:
            nodes.append(node)
    return nodes


def route_pips(route: list[dict[str, Any]], db: PrjxrayDb) -> list[str]:
    pips: list[str] = []
    for wire in route:
        ann = wire.get("annotation", {})
        for feature in ann.get("fasm_features", []):
            if feature not in pips:
                pips.append(str(feature))
    full_nodes = route_full_nodes(route, db)
    for src, dst in zip(full_nodes, full_nodes[1:]):
        feature = direct_pip_feature(db, src, dst)
        if feature and feature not in pips:
            pips.append(feature)
    return pips


def vivado_pip_name(feature: str, db: PrjxrayDb) -> str | None:
    tile_name, _, tail = feature.partition(".")
    src, sep, dst = tail.partition(".")
    if not tile_name or not sep or not src or not dst:
        return None
    tile = db.tilegrid.get(tile_name)
    tile_type = tile.type if tile else tile_name.split("_X", 1)[0]
    arrow = "->>" if tile_type.startswith("INT_") else "->"
    return f"{tile_name}/{tile_type}.{dst}{arrow}{src}"


def canonical_export_net_name(name: str) -> str:
    out = name
    marker = out.rfind(".$")
    if marker >= 0:
        out = "$" + out[marker + 2:]
    elif "." in out:
        out = out.rsplit(".", 1)[1]
    if out.endswith("[0]"):
        out = out[:-3]
    if out.startswith("$techmap"):
        abc = out.find("$abc")
        if abc >= 0:
            out = out[abc:]
    return out


def site_type_for_placement(db: PrjxrayDb, placement: VivadoPlacement) -> str | None:
    for tile in db.tilegrid.values():
        site_type = tile.sites.get(placement.site)
        if site_type:
            return site_type
    return None


def export_bel_name(db: PrjxrayDb, placement: VivadoPlacement) -> str:
    if not placement.bel and placement.inst.cell_type == "IBUF":
        return "IOB33.INBUF_EN"
    if not placement.bel and placement.inst.cell_type == "OBUF":
        return "IOB33.OUTBUF"
    if not placement.bel:
        return ""
    if "." in placement.bel:
        return placement.bel
    site_type = site_type_for_placement(db, placement)
    return f"{site_type}.{placement.bel}" if site_type else placement.bel


def write_scalepnr_pnr_export(path: Path, placements: list[VivadoPlacement], routes: list[RouteExport], db: PrjxrayDb) -> None:
    placement_rows: list[str] = []
    for placement in placements:
        placement_rows.append(",".join([
            placement.inst.name,
            placement.inst.cell_type,
            placement.site,
            export_bel_name(db, placement),
            placement.site,
        ]))

    route_rows: list[str] = []
    for route in routes:
        net_name = canonical_export_net_name(route.net_name)
        for feature in route.pips:
            pip = vivado_pip_name(feature, db)
            if pip:
                route_rows.append(f"{net_name},{pip}")

    with path.open("w") as f:
        f.write("==============================\n")
        f.write("SECTION: PLACEMENT\n")
        f.write("==============================\n")
        f.write("cell,ref_name,site,bel,loc\n")
        for row in sorted(set(placement_rows)):
            f.write(row + "\n")
        f.write("\n")
        f.write("==============================\n")
        f.write("SECTION: ROUTING_PIPS\n")
        f.write("==============================\n")
        f.write("net,pip\n")
        for row in sorted(set(route_rows)):
            f.write(row + "\n")


def vivado_net_candidates(name: str) -> list[str]:
    candidates = [name]
    marker = ".$"
    if marker in name:
        candidates.append("$" + name.rsplit(marker, 1)[1])
    if name.endswith("[0]"):
        candidates.append(name[:-3])
    if ".$" in name and name.endswith("[0]"):
        candidates.append("$" + name.rsplit(".$", 1)[1][:-3])
    if "." in name:
        candidates.append(name.rsplit(".", 1)[1])
    for candidate in list(candidates):
        abc = candidate.find("$abc")
        if abc > 0:
            candidates.append(candidate[abc:])

    out: list[str] = []
    for candidate in candidates:
        if candidate and candidate not in out:
            out.append(candidate)
    return out


def matching_route_pins(inst: dict[str, Any], net_name: str) -> list[tuple[str, str]]:
    pins: list[tuple[str, str]] = []
    owner = str(inst.get("name", ""))
    for conn in inst.get("connections", []):
        if str(conn.get("net", "")) != net_name:
            continue
        driver_inst = str(conn.get("driver_inst", ""))
        driver_port = str(conn.get("driver_port", ""))
        sink_port = str(conn.get("sink_port", ""))
        if driver_inst and driver_port:
            pins.append((driver_inst, driver_port))
        if owner and sink_port:
            pins.append((owner, sink_port))
    out: list[tuple[str, str]] = []
    for pin in pins:
        if pin not in out:
            out.append(pin)
    return out


def matching_route_pins_for_nets(inst: dict[str, Any], net_names: Iterable[str]) -> list[tuple[str, str]]:
    pins: list[tuple[str, str]] = []
    for net_name in net_names:
        for pin in matching_route_pins(inst, net_name):
            if pin not in pins:
                pins.append(pin)
    return pins


def merge_route_exports(routes: list[RouteExport]) -> list[RouteExport]:
    parent = list(range(len(routes)))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    def route_merge_keys(route: RouteExport) -> list[str]:
        for path in route.full_paths:
            if path:
                return [f"root:{path[0]}"]
        for path in route.paths:
            if path:
                return [f"root:{path[0]}"]
        return [f"net:{route.net_name}"]

    candidate_owner: dict[str, int] = {}
    for index, route in enumerate(routes):
        for candidate in route_merge_keys(route):
            owner = candidate_owner.get(candidate)
            if owner is None:
                candidate_owner[candidate] = index
            else:
                union(owner, index)

    groups: dict[int, list[RouteExport]] = {}
    for index, route in enumerate(routes):
        groups.setdefault(find(index), []).append(route)

    merged: list[RouteExport] = []
    for group in groups.values():
        net_candidates: list[str] = []
        pin_candidates: list[tuple[str, str]] = []
        paths: list[list[str]] = []
        full_paths: list[list[str]] = []
        pips: list[str] = []
        for route in group:
            for candidate in route.net_candidates:
                if candidate not in net_candidates:
                    net_candidates.append(candidate)
            for pin in route.pin_candidates:
                if pin not in pin_candidates:
                    pin_candidates.append(pin)
            if route.paths[0] and route.paths[0] not in paths:
                paths.append(route.paths[0])
            if route.full_paths[0] and route.full_paths[0] not in full_paths:
                full_paths.append(route.full_paths[0])
            for pip in route.pips:
                if pip not in pips:
                    pips.append(pip)
        merged.append(RouteExport(group[0].net_name, net_candidates, pin_candidates, paths, full_paths, pips))
    return merged


def collect_routes(state: dict[str, Any], db: PrjxrayDb) -> list[RouteExport]:
    routes: list[RouteExport] = []
    for inst in state.get("insts", []):
        for route in inst.get("routes", []):
            if not route:
                continue
            net_name = route_name(inst, route)
            net_names = route_net_names(inst, route)
            pin_net_names = route_pin_net_names(inst, route, net_name)
            net_candidates: list[str] = []
            for candidate_name in net_names:
                for candidate in vivado_net_candidates(candidate_name):
                    if candidate not in net_candidates:
                        net_candidates.append(candidate)
            routes.append(RouteExport(
                net_name,
                net_candidates,
                matching_route_pins_for_nets(inst, pin_net_names),
                [route_nodes(route, db)],
                [route_full_nodes(route, db)],
                route_pips(route, db),
            ))
    return merge_route_exports(routes)


def collect_io_assignments(state: dict[str, Any]) -> list[IoAssignment]:
    assignments: list[IoAssignment] = []
    for item in state.get("io_assignments", []):
        port = str(item.get("port", ""))
        properties = item.get("properties", {})
        if not port or not isinstance(properties, dict):
            continue
        clean_properties = {str(key): str(value) for key, value in properties.items() if str(value)}
        if clean_properties:
            assignments.append(IoAssignment(port, clean_properties))
    return assignments


def write_project_tcl(path: Path, args: argparse.Namespace, top: str | None) -> None:
    project_dir = path.parent
    source_root = rel_to(args.source_root, project_dir)
    with path.open("w") as f:
        f.write("# Generated by db2prj.py from scalepnr design_state.db\n")
        f.write(f"set script_dir [file dirname [file normalize [info script]]]\n")
        f.write(f"create_project {tcl_braced(args.project_name)} [file join $script_dir project] -part {tcl_braced(args.part)} -force\n")
        if args.edif:
            edif = args.edif.name
            f.write(f"set edif_file [file normalize [file join $script_dir {tcl_braced(edif)}]]\n")
            f.write("if {![file exists $edif_file]} {\n")
            f.write("    error \"EDIF netlist not found: $edif_file\"\n")
            f.write("}\n")
            f.write("read_edif $edif_file\n")
        else:
            f.write(f"set source_root [file normalize [file join $script_dir {tcl_braced(source_root)}]]\n")
            f.write("set sv_sources [glob -nocomplain -directory $source_root *.sv]\n")
            f.write("if {[llength $sv_sources] == 0} {\n")
            f.write("    puts \"WARN: no SystemVerilog sources found in $source_root\"\n")
            f.write("} else {\n")
            f.write("    add_files -fileset sources_1 $sv_sources\n")
            f.write("    set_property file_type SystemVerilog [get_files -of_objects [get_filesets sources_1]]\n")
            f.write("}\n")
        if top and not args.edif:
            f.write(f"set_property top {tcl_braced(top)} [current_fileset]\n")
        if not args.edif:
            f.write("update_compile_order -fileset sources_1\n")
        else:
            f.write(f"link_design -top {tcl_braced(top or 'test')} -part [get_property PART [current_project]]\n")
        f.write("\n")
        if args.edif:
            f.write("# EDIF is already linked above; placement constraints can be applied directly.\n")
        else:
            f.write("synth_design -top [get_property top [current_fileset]] -part [get_property PART [current_project]]\n")
        f.write("source [file join $script_dir io.tcl]\n")
        f.write("source [file join $script_dir placing.tcl]\n")
        f.write("place_design\n")
        f.write("scalepnr_check_placement\n")
        if args.skip_fixed_routes:
            f.write("# Diagnostic mode: keep scalepnr placement/I/O constraints, but let Vivado route freely.\n")
            f.write("# routing.tcl is still generated for comparison and is intentionally not sourced here.\n")
        else:
            f.write("source [file join $script_dir routing.tcl]\n")
        f.write("set scalepnr_route_status 0\n")
        f.write("set scalepnr_route_error {}\n")
        f.write("if {[catch {route_design} scalepnr_route_error]} {\n")
        f.write("    set scalepnr_route_status 1\n")
        f.write("    puts \"WARN: route_design failed before export: $scalepnr_route_error\"\n")
        f.write("}\n")
        f.write("\n")
        f.write("# export_place_route.tcl\n")
        f.write("set out_file [file join $script_dir place_route_export.txt]\n")
        f.write("set fp [open $out_file w]\n")
        f.write("\n")
        f.write("puts $fp \"==============================\"\n")
        f.write("puts $fp \"SECTION: PLACEMENT\"\n")
        f.write("puts $fp \"==============================\"\n")
        f.write("puts $fp \"cell,ref_name,site,bel,loc\"\n")
        f.write("\n")
        f.write("set scalepnr_export_cell_names {}\n")
        f.write("foreach expected $scalepnr_expected_placements {\n")
        f.write("    lassign $expected cell loc bel\n")
        f.write("    set name [get_property NAME $cell]\n")
        f.write("    dict set scalepnr_export_cell_names $name 1\n")
        f.write("}\n")
        f.write("set placement_rows {}\n")
        f.write("foreach c [get_cells -hier -filter {IS_PRIMITIVE}] {\n")
        f.write("    set name [get_property NAME $c]\n")
        f.write("    if {![dict exists $scalepnr_export_cell_names $name]} { continue }\n")
        f.write("    set ref  [get_property REF_NAME $c]\n")
        f.write("    set site [get_property SITE $c]\n")
        f.write("    set bel  [get_property BEL $c]\n")
        f.write("    set loc  [get_property LOC $c]\n")
        f.write("\n")
        f.write("    lappend placement_rows \"$name,$ref,$site,$bel,$loc\"\n")
        f.write("}\n")
        f.write("foreach row [lsort $placement_rows] {\n")
        f.write("    puts $fp $row\n")
        f.write("}\n")
        f.write("\n")
        f.write("puts $fp \"\"\n")
        f.write("puts $fp \"==============================\"\n")
        f.write("puts $fp \"SECTION: ROUTING_PIPS\"\n")
        f.write("puts $fp \"==============================\"\n")
        f.write("puts $fp \"net,pip\"\n")
        f.write("\n")
        f.write("set routing_rows {}\n")
        f.write("foreach n [get_nets -hier -quiet] {\n")
        f.write("    set net_name [get_property NAME $n]\n")
        f.write("    set pips [get_pips -quiet -of_objects $n]\n")
        f.write("\n")
        f.write("    foreach p $pips {\n")
        f.write("        lappend routing_rows \"$net_name,[get_property NAME $p]\"\n")
        f.write("    }\n")
        f.write("}\n")
        f.write("foreach row [lsort $routing_rows] {\n")
        f.write("    puts $fp $row\n")
        f.write("}\n")
        f.write("\n")
        f.write("close $fp\n")
        f.write("puts \"Created and implemented Vivado project at [get_property DIRECTORY [current_project]]\"\n")
        f.write("if {$scalepnr_route_status != 0} {\n")
        f.write("    error $scalepnr_route_error\n")
        f.write("}\n")


def write_io_tcl(path: Path, assignments: list[IoAssignment]) -> None:
    with path.open("w") as f:
        f.write("# Generated I/O constraints from scalepnr design_state.db\n")
        f.write("set scalepnr_missing_ports 0\n")
        f.write("proc scalepnr_get_port {name} {\n")
        f.write("    global scalepnr_missing_ports\n")
        f.write("    set matched {}\n")
        f.write("    foreach port [get_ports -quiet *] {\n")
        f.write("        if {[get_property NAME $port] eq $name} {\n")
        f.write("            lappend matched $port\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    if {[llength $matched] == 0} {\n")
        f.write("        puts \"ERROR: missing port $name\"\n")
        f.write("        incr scalepnr_missing_ports\n")
        f.write("    }\n")
        f.write("    return $matched\n")
        f.write("}\n\n")
        f.write("proc scalepnr_set_port_property {prop value port_name port} {\n")
        f.write("    if {[catch {set_property $prop $value $port} err]} {\n")
        f.write("        error \"failed to set $prop=$value on logical port $port_name / Vivado object $port: $err\"\n")
        f.write("    }\n")
        f.write("    set actual [get_property $prop $port]\n")
        f.write("    if {$actual ne $value} {\n")
        f.write("        error \"I/O property readback mismatch for logical port $port_name / Vivado object $port: $prop expected $value, got $actual\"\n")
        f.write("    }\n")
        f.write("}\n\n")
        for assignment in assignments:
            f.write(f"set port [scalepnr_get_port {tcl_braced(assignment.port)}]\n")
            f.write("if {[llength $port]} {\n")
            for prop, value in sorted(assignment.properties.items()):
                if prop == "PACKAGE_PIN":
                    f.write(f"    # PACKAGE_PIN {tcl_braced(value)} is kept as scalepnr annotation; IOB site LOC comes from placing.tcl.\n")
                    continue
                f.write(f"    scalepnr_set_port_property {tcl_braced(prop)} {tcl_braced(value)} {tcl_braced(assignment.port)} $port\n")
            f.write("}\n\n")
        f.write("if {$scalepnr_missing_ports != 0} {\n")
        f.write("    error \"missing $scalepnr_missing_ports scalepnr I/O ports\"\n")
        f.write("}\n")


def write_placing_tcl(path: Path, placements: list[VivadoPlacement], warnings: list[str]) -> None:
    with path.open("w") as f:
        f.write("# Generated placement constraints from scalepnr design_state.db\n")
        f.write("set scalepnr_missing_cells 0\n")
        f.write("set scalepnr_placement_errors 0\n")
        f.write("set scalepnr_expected_placements {}\n")
        f.write("set scalepnr_used_shortened_cells {}\n")
        f.write("proc scalepnr_glob_escape {value} {\n")
        f.write("    return [string map {\\\\ \\\\\\\\ * \\\\* ? \\\\? [ \\\\[ ] \\\\]} $value]\n")
        f.write("}\n\n")
        f.write("proc scalepnr_get_cell {name} {\n")
        f.write("    global scalepnr_missing_cells scalepnr_used_shortened_cells\n")
        f.write("    set cells [get_cells -hier -quiet [list $name]]\n")
        f.write("    if {[llength $cells] == 0 && [string first {...} $name] >= 0} {\n")
        f.write("        set ellipsis [string first {...} $name]\n")
        f.write("        set prefix [string range $name 0 [expr {$ellipsis - 1}]]\n")
        f.write("        set suffix [string range $name [expr {$ellipsis + 3}] end]\n")
        f.write("        if {$prefix ne {} || $suffix ne {}} {\n")
        f.write("            set pattern \"[scalepnr_glob_escape $prefix]*[scalepnr_glob_escape $suffix]\"\n")
        f.write("            set matched {}\n")
        f.write("            foreach cell [get_cells -hier -quiet *] {\n")
        f.write("                if {[string match $pattern [get_property NAME $cell]]} {\n")
        f.write("                    lappend matched $cell\n")
        f.write("                }\n")
        f.write("            }\n")
        f.write("            if {[llength $matched] >= 1} {\n")
        f.write("                set selected {}\n")
        f.write("                foreach candidate $matched {\n")
        f.write("                    set candidate_name [get_property NAME $candidate]\n")
        f.write("                    if {[lsearch -exact $scalepnr_used_shortened_cells $candidate_name] < 0} {\n")
        f.write("                        set selected $candidate\n")
        f.write("                        lappend scalepnr_used_shortened_cells $candidate_name\n")
        f.write("                        break\n")
        f.write("                    }\n")
        f.write("                }\n")
        f.write("                if {$selected eq {}} {\n")
        f.write("                    set selected [lindex $matched 0]\n")
        f.write("                }\n")
        f.write("                puts \"WARN: resolved shortened cell $name to [get_property NAME $selected]\"\n")
        f.write("                set cells [list $selected]\n")
        f.write("            }\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    if {[llength $cells] == 0} {\n")
        f.write("        puts \"ERROR: missing cell $name\"\n")
        f.write("        incr scalepnr_missing_cells\n")
        f.write("    }\n")
        f.write("    return $cells\n")
        f.write("}\n\n")
        f.write("proc scalepnr_set_cell_property {prop value cell} {\n")
        f.write("    global scalepnr_placement_errors\n")
        f.write("    if {[catch {set_property $prop $value $cell} err]} {\n")
        f.write("        puts \"ERROR: failed to set $prop=$value on $cell: $err\"\n")
        f.write("        incr scalepnr_placement_errors\n")
        f.write("    }\n")
        f.write("}\n\n")
        f.write("proc scalepnr_record_placement {cell loc bel} {\n")
        f.write("    global scalepnr_expected_placements\n")
        f.write("    lappend scalepnr_expected_placements [list $cell $loc $bel]\n")
        f.write("}\n\n")
        f.write("proc scalepnr_check_placement {} {\n")
        f.write("    global scalepnr_expected_placements\n")
        f.write("    set errors 0\n")
        f.write("    foreach expected $scalepnr_expected_placements {\n")
        f.write("        lassign $expected cell loc bel\n")
        f.write("        set actual_loc [get_property LOC $cell]\n")
        f.write("        if {$actual_loc ne $loc} {\n")
        f.write("            puts \"ERROR: placement LOC mismatch for $cell: expected $loc, got $actual_loc\"\n")
        f.write("            incr errors\n")
        f.write("        }\n")
        f.write("        if {$bel ne {}} {\n")
        f.write("            set actual_bel [get_property BEL $cell]\n")
        f.write("            set bel_ok [expr {$actual_bel eq $bel || [string match *.$bel $actual_bel]}]\n")
        f.write("            if {!$bel_ok && [regexp {^([A-H])[1-6]LUT$} $bel -> letter]} {\n")
        f.write("                set bel_ok [expr {[string match *.$letter\\[56\\]LUT $actual_bel]}]\n")
        f.write("            }\n")
        f.write("            if {!$bel_ok} {\n")
        f.write("                puts \"ERROR: placement BEL mismatch for $cell: expected $bel, got $actual_bel\"\n")
        f.write("                incr errors\n")
        f.write("            }\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    if {$errors != 0} {\n")
        f.write("        error \"Vivado placement differs from scalepnr constraints for $errors properties\"\n")
        f.write("    }\n")
        f.write("}\n\n")
        for warning in warnings:
            f.write(f"# WARN: {warning}\n")
        if warnings:
            f.write("\n")
        def placement_order(placement: VivadoPlacement) -> tuple[int, str]:
            kind = cell_kind(placement.inst)
            if placement.inst.cell_type.startswith("MUXF8"):
                order = 0
            elif placement.inst.cell_type.startswith("MUXF7"):
                order = 1
            else:
                order = {"LUT": 2, "FD": 3, "CARRY": 4}.get(kind, 5)
            return order, placement.inst.name

        for placement in sorted(placements, key=placement_order):
            cell_name = placement.inst.name
            if not cell_name:
                continue
            set_bel = bool(placement.bel)
            if placement.inst.cell_type.startswith("MUXF7") and not placement.constrain:
                set_bel = False
            f.write(f"set cell [scalepnr_get_cell {tcl_braced(cell_name)}]\n")
            f.write("if {[llength $cell]} {\n")
            if placement.constrain:
                f.write(f"    scalepnr_set_cell_property LOC {tcl_braced(placement.site)} $cell\n")
                f.write("    scalepnr_set_cell_property IS_LOC_FIXED true $cell\n")
                if set_bel:
                    f.write(f"    scalepnr_set_cell_property BEL {tcl_braced(placement.bel)} $cell\n")
                if set_bel:
                    f.write("    scalepnr_set_cell_property IS_BEL_FIXED true $cell\n")
            f.write(f"    scalepnr_record_placement $cell {tcl_braced(placement.site)} {tcl_braced(placement.bel or '')}\n")
            f.write("}\n\n")
        f.write("if {$scalepnr_missing_cells != 0} {\n")
        f.write("    error \"missing $scalepnr_missing_cells scalepnr placement cells; use the Yosys EDIF netlist generated with design_state.db\"\n")
        f.write("}\n")
        f.write("if {$scalepnr_placement_errors != 0} {\n")
        f.write("    error \"failed to apply $scalepnr_placement_errors scalepnr placement constraints\"\n")
        f.write("}\n")


def route_tree_expression(paths: list[list[str]]) -> str | None:
    roots = {path[0] for path in paths if path}
    if len(roots) != 1:
        return None

    tree: dict[str, Any] = {}
    for path in paths:
        cursor = tree
        for node in path:
            cursor = cursor.setdefault(node, {})

    def tree_items(node_name: str, children: dict[str, Any]) -> list[str]:
        items = [tcl_braced(node_name)]
        if len(children) == 1:
            child_name, child_children = next(iter(children.items()))
            items.extend(tree_items(child_name, child_children))
        else:
            for child_name, child_children in children.items():
                items.append("[list " + " ".join(tree_items(child_name, child_children)) + "]")
        return items

    root_name, root_children = next(iter(tree.items()))
    return "[list " + " ".join(tree_items(root_name, root_children)) + "]"




def should_skip_endpoint_only_fixed_route(route: RouteExport, paths: list[list[str]]) -> bool:
    if len(paths) != 1:
        return False
    path = paths[0]
    if len(path) > 3:
        return False
    if not path:
        return True
    tiles = {vivado_node_tile(node) for node in path}
    if len(tiles) != 1:
        return False
    wires = {vivado_node_wire(node) for node in path}
    has_source_local = any("LOGIC_OUTS" in wire for wire in wires)
    has_sink_local = any("IMUX" in wire or "BYP" in wire or "CTRL" in wire for wire in wires)
    has_site_feature = any(not feature.split(".", 1)[0].startswith("INT_") for feature in route.pips)
    return has_source_local and has_sink_local and has_site_feature


def write_routing_tcl(path: Path, routes: list[RouteExport]) -> None:
    with path.open("w") as f:
        f.write("# Generated routing constraints from scalepnr design_state.db\n")
        f.write("# Vivado FIXED_ROUTE syntax is device/name sensitive; constraints are canonicalized against Vivado downhill nodes first.\n")
        f.write("set scalepnr_missing_nets 0\n")
        f.write("set scalepnr_fixed_route_errors 0\n")
        f.write("proc scalepnr_get_net {name candidates pin_candidates} {\n")
        f.write("    global scalepnr_missing_nets\n")
        f.write("    set nets {}\n")
        f.write("    foreach pin_candidate $pin_candidates {\n")
        f.write("        lassign $pin_candidate cell_name port_name\n")
        f.write("        set pin [get_pins -hier -quiet [format {%s/%s} $cell_name $port_name]]\n")
        f.write("        if {[llength $pin] == 0} { continue }\n")
        f.write("        set nets [get_nets -quiet -of_objects $pin]\n")
        f.write("        if {[llength $nets] != 0} { break }\n")
        f.write("    }\n")
        f.write("    foreach candidate $candidates {\n")
        f.write("        if {[llength $nets] != 0} { break }\n")
        f.write("        set nets [get_nets -hier -quiet [list $candidate]]\n")
        f.write("        if {[llength $nets] != 0} { break }\n")
        f.write("    }\n")
        f.write("    if {[llength $nets] == 0} {\n")
        f.write("        puts \"ERROR: missing net $name; tried names=$candidates pins=$pin_candidates\"\n")
        f.write("        incr scalepnr_missing_nets\n")
        f.write("    }\n")
        f.write("    return $nets\n")
        f.write("}\n\n")
        f.write("proc scalepnr_node_wire {node_name} {\n")
        f.write("    set slash [string last {/} $node_name]\n")
        f.write("    if {$slash < 0} { return $node_name }\n")
        f.write("    return [string range $node_name [expr {$slash + 1}] end]\n")
        f.write("}\n\n")
        f.write("proc scalepnr_wire_family {wire_name} {\n")
        f.write("    if {[regexp {^([A-Z]+[0-9]+BEG)} $wire_name -> family]} { return $family }\n")
        f.write("    if {[regexp {^([A-Z]+[0-9]+END)} $wire_name -> family]} { return $family }\n")
        f.write("    if {[regexp {^([A-Z]+[0-9]+)} $wire_name -> family]} { return $family }\n")
        f.write("    if {[regexp {^([A-Z_]+)} $wire_name -> family]} { return $family }\n")
        f.write("    return $wire_name\n")
        f.write("}\n\n")
        f.write("proc scalepnr_downhill_node_names {node_name} {\n")
        f.write("    set node [get_nodes -quiet [list $node_name]]\n")
        f.write("    if {![llength $node]} { return {} }\n")
        f.write("    set out {}\n")
        f.write("    if {[catch {get_nodes -quiet -downhill -of_objects $node} downhill_nodes]} {\n")
        f.write("        return {}\n")
        f.write("    }\n")
        f.write("    foreach downhill $downhill_nodes {\n")
        f.write("        lappend out [get_property NAME $downhill]\n")
        f.write("    }\n")
        f.write("    return $out\n")
        f.write("}\n\n")
        f.write("proc scalepnr_canonical_fixed_route {nodes} {\n")
        f.write("    set out {}\n")
        f.write("    foreach requested $nodes {\n")
        f.write("        if {![llength $out]} {\n")
        f.write("            lappend out $requested\n")
        f.write("            continue\n")
        f.write("        }\n")
        f.write("        set current [lindex $out end]\n")
        f.write("        set downhill [scalepnr_downhill_node_names $current]\n")
        f.write("        if {[lsearch -exact $downhill $requested] >= 0} {\n")
        f.write("            lappend out $requested\n")
        f.write("            continue\n")
        f.write("        }\n")
        f.write("        set requested_wire [scalepnr_node_wire $requested]\n")
        f.write("        set requested_family [scalepnr_wire_family $requested_wire]\n")
        f.write("        set replacement {}\n")
        f.write("        foreach candidate $downhill {\n")
        f.write("            if {[scalepnr_node_wire $candidate] eq $requested_wire} {\n")
        f.write("                set replacement $candidate\n")
        f.write("                break\n")
        f.write("            }\n")
        f.write("        }\n")
        f.write("        if {$replacement eq {}} {\n")
        f.write("            foreach candidate $downhill {\n")
        f.write("                if {[scalepnr_wire_family [scalepnr_node_wire $candidate]] eq $requested_family} {\n")
        f.write("                    set replacement $candidate\n")
        f.write("                    break\n")
        f.write("                }\n")
        f.write("            }\n")
        f.write("        }\n")
        f.write("        if {$replacement eq {} && [llength $downhill] != 0} {\n")
        f.write("            return {}\n")
        f.write("        }\n")
        f.write("        if {$replacement ne {}} {\n")
        f.write("            lappend out $replacement\n")
        f.write("        } else {\n")
        f.write("            lappend out $requested\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    return $out\n")
        f.write("}\n\n")
        f.write("proc scalepnr_route_index_after {nodes value start} {\n")
        f.write("    set count [llength $nodes]\n")
        f.write("    for {set i $start} {$i < $count} {incr i} {\n")
        f.write("        if {[lindex $nodes $i] eq $value} { return $i }\n")
        f.write("    }\n")
        f.write("    return -1\n")
        f.write("}\n\n")
        f.write("proc scalepnr_repair_fixed_route_from_error {nodes err} {\n")
        f.write("    if {![regexp {Did not find node resource, ([^,]+), downhill from node, ([^\\.]+)\\.} $err -> wanted current]} {\n")
        f.write("        return $nodes\n")
        f.write("    }\n")
        f.write("    set current_index [scalepnr_route_index_after $nodes $current 0]\n")
        f.write("    if {$current_index < 0} { set current_index 0 }\n")
        f.write("    set wanted_index [scalepnr_route_index_after $nodes $wanted [expr {$current_index + 1}]]\n")
        f.write("    if {$wanted_index < 0} { return $nodes }\n")
        f.write("    if {$wanted eq $current} {\n")
        f.write("        return [lreplace $nodes $wanted_index $wanted_index]\n")
        f.write("    }\n")
        f.write("    set marker {Downhill node choices include:}\n")
        f.write("    set marker_index [string first $marker $err]\n")
        f.write("    if {$marker_index < 0} { return $nodes }\n")
        f.write("    set choices_text [string range $err [expr {$marker_index + [string length $marker]}] end]\n")
        f.write("    regsub {\\.[[:space:]]*$} $choices_text {} choices_text\n")
        f.write("    set wanted_wire [scalepnr_node_wire $wanted]\n")
        f.write("    set wanted_family [scalepnr_wire_family $wanted_wire]\n")
        f.write("    set replacement {}\n")
        f.write("    foreach candidate $choices_text {\n")
        f.write("        if {[scalepnr_node_wire $candidate] eq $wanted_wire} {\n")
        f.write("            set replacement $candidate\n")
        f.write("            break\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    if {$replacement eq {}} {\n")
        f.write("        foreach candidate $choices_text {\n")
        f.write("            if {[scalepnr_wire_family [scalepnr_node_wire $candidate]] eq $wanted_family} {\n")
        f.write("                set replacement $candidate\n")
        f.write("                break\n")
        f.write("            }\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    if {$replacement eq {} && [llength $choices_text] != 0} {\n")
        f.write("        return {}\n")
        f.write("    }\n")
        f.write("    if {$replacement eq {}} { return $nodes }\n")
        f.write("    return [lreplace $nodes $wanted_index $wanted_index $replacement]\n")
        f.write("}\n\n")
        f.write("proc scalepnr_should_skip_fixed_route_error {err} {\n")
        f.write("    if {[string first {Directed routing is not supported} $err] >= 0} { return 1 }\n")
        f.write("    if {[string first {Downhill node choices include:  .} $err] >= 0} { return 1 }\n")
        f.write("    if {[string first {Starting wire,} $err] >= 0 && [string first {was not found in driver tile} $err] >= 0} { return 1 }\n")
        f.write("    if {[string first {Did not find node resource,} $err] >= 0 && [string first {downhill from node} $err] >= 0} { return 1 }\n")
        f.write("    return 0\n")
        f.write("}\n\n")
        f.write("proc scalepnr_accepted_fixed_route {net nodes} {\n")
        f.write("    set fixed_route [scalepnr_canonical_fixed_route $nodes]\n")
        f.write("    if {![llength $fixed_route]} { return [list 1 {} {}] }\n")
        f.write("    set err {}\n")
        f.write("    for {set attempt 0} {$attempt < 32} {incr attempt} {\n")
        f.write("        if {[llength $fixed_route] < 2} { return [list 1 {} {}] }\n")
        f.write("        if {![catch {set_property FIXED_ROUTE $fixed_route $net} err]} {\n")
        f.write("            catch {set_property IS_ROUTE_FIXED true $net}\n")
        f.write("            return [list 1 $fixed_route {}]\n")
        f.write("        }\n")
        f.write("        if {[scalepnr_should_skip_fixed_route_error $err]} { return [list 1 {} {}] }\n")
        f.write("        set repaired [scalepnr_repair_fixed_route_from_error $fixed_route $err]\n")
        f.write("        if {![llength $repaired]} { return [list 1 {} {}] }\n")
        f.write("        if {$repaired eq $fixed_route} { break }\n")
        f.write("        set fixed_route $repaired\n")
        f.write("    }\n")
        f.write("    return [list 0 $fixed_route $err]\n")
        f.write("}\n\n")
        f.write("proc scalepnr_set_fixed_route {net nodes} {\n")
        f.write("    global scalepnr_fixed_route_errors\n")
        f.write("    lassign [scalepnr_accepted_fixed_route $net $nodes] applied fixed_route err\n")
        f.write("    if {!$applied} {\n")
        f.write("        puts \"WARN: FIXED_ROUTE failed for $net: $err\"\n")
        f.write("        puts \"WARN: requested route was $nodes\"\n")
        f.write("        puts \"WARN: canonical route was $fixed_route\"\n")
        f.write("        incr scalepnr_fixed_route_errors\n")
        f.write("    }\n")
        f.write("}\n\n")
        f.write("proc scalepnr_set_fixed_route_tree {net fixed_route} {\n")
        f.write("    global scalepnr_fixed_route_errors\n")
        f.write("    if {[catch {set_property FIXED_ROUTE $fixed_route $net} err]} {\n")
        f.write("        if {[scalepnr_should_skip_fixed_route_error $err]} { return }\n")
        f.write("        puts \"WARN: FIXED_ROUTE tree failed for $net: $err\"\n")
        f.write("        puts \"WARN: requested route tree was $fixed_route\"\n")
        f.write("        incr scalepnr_fixed_route_errors\n")
        f.write("    } else {\n")
        f.write("        catch {set_property IS_ROUTE_FIXED true $net}\n")
        f.write("    }\n")
        f.write("}\n\n")
        for route in routes:
            f.write(f"# route {route.net_name}\n")
            if len(route.paths) > 1:
                f.write(f"# merged_route_paths {len(route.paths)}\n")
            for full_nodes in route.full_paths:
                for node in full_nodes:
                    f.write(f"# full_node {node}\n")
                if len(route.full_paths) > 1:
                    f.write("# end_full_path\n")
            for pip in route.pips:
                f.write(f"# pip {pip}\n")
            aggregation = aggregate_a7_route_tree([path for path in route.paths if path])
            paths = aggregation.paths
            if not paths:
                f.write("# no route nodes exported for this route\n\n")
                continue
            if aggregation.stitched_roots:
                f.write(f"# aggregated_route_roots {aggregation.stitched_roots} root={aggregation.root or ''}\n")
            for disconnected_root in aggregation.disconnected_roots[:8]:
                f.write(f"# disconnected_route_root {disconnected_root}\n")
            f.write(f"set net [scalepnr_get_net {tcl_braced(route.net_name)} [list {tcl_list(route.net_candidates)}] [list {tcl_pair_list(route.pin_candidates)}]]\n")
            f.write("if {[llength $net]} {\n")
            if len(paths) == 1:
                if len(paths[0]) < 2:
                    f.write("    # skipped: single-node scalepnr route cannot form a Vivado fixed route root yet\n")
                elif should_skip_endpoint_only_fixed_route(route, paths):
                    f.write("    # skipped: endpoint-only same-tile route; Vivado must complete site-side routing\n")
                else:
                    f.write(f"    scalepnr_set_fixed_route $net [list {tcl_list(paths[0])}]\n")
            else:
                tree = route_tree_expression(paths)
                if tree is None:
                    if aggregation.disconnected_roots:
                        f.write(
                            f"    # skipped: grouped scalepnr paths are disconnected from selected root "
                            f"{aggregation.root or ''}; disconnected_roots={len(aggregation.disconnected_roots)}\n"
                        )
                    else:
                        f.write("    # skipped: grouped scalepnr paths have multiple source roots and cannot form one fixed route tree yet\n")
                else:
                    f.write(f"    scalepnr_set_fixed_route_tree $net {tree}\n")
            f.write("}\n\n")
        f.write("if {$scalepnr_missing_nets != 0} {\n")
        f.write("    puts \"WARN: missing $scalepnr_missing_nets scalepnr route nets; skipped those fixed-route constraints\"\n")
        f.write("}\n")
        f.write("if {$scalepnr_fixed_route_errors != 0} {\n")
        f.write("    puts \"WARN: failed to apply $scalepnr_fixed_route_errors scalepnr fixed route constraints\"\n")
        f.write("}\n")


def write_manifest(path: Path, args: argparse.Namespace, top: str | None, placements: list[VivadoPlacement], routes: list[RouteExport], io_assignments: list[IoAssignment]) -> None:
    manifest = {
        "input_db": str(args.input_db),
        "part": args.part,
        "top": top,
        "source_glob": None if args.edif else str(args.source_root / "*.sv"),
        "sv_sources": [str(source) for source in args.sv],
        "edif": str(args.edif) if args.edif else None,
        "io_constraints": len(io_assignments),
        "placement_constraints": len(placements),
        "routing_constraints": len(routes),
        "skip_fixed_routes": bool(args.skip_fixed_routes),
    }
    path.write_text(json.dumps(manifest, indent=2) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export scalepnr design_state.db to a Vivado Tcl project")
    parser.add_argument("input_db", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--db-dir", type=Path, default=Path(__file__).with_name("db"))
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--sv", action="append", type=Path, default=[], help="SystemVerilog source to build a Yosys EDIF for the Vivado project")
    parser.add_argument("--edif", type=Path, help="Yosys-generated EDIF netlist to read into Vivado instead of synthesizing *.sv")
    parser.add_argument("--part", default=DEFAULT_PART)
    parser.add_argument("--top")
    parser.add_argument("--project-name", default="scalepnr_vivado")
    parser.add_argument(
        "--skip-fixed-routes",
        action="store_true",
        help="Generate routing.tcl but do not source it before Vivado route_design",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    state = load_state(args.input_db)
    db = PrjxrayDb(args.db_dir)
    top = args.top or infer_top(state)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.source_root is None:
        args.source_root = args.output_dir
    args.sv = [source.resolve() for source in args.sv]
    if args.edif is None:
        if args.sv:
            args.edif = build_edif_from_sv(args.sv, args.output_dir, top or "test")
        else:
            default_edif = args.input_db.with_name("test.edf")
            if default_edif.exists():
                args.edif = default_edif
    elif not args.edif.is_absolute():
        args.edif = args.edif.resolve()
    if args.edif:
        exported_edif = args.output_dir / args.edif.name
        if args.edif.resolve() != exported_edif.resolve():
            shutil.copyfile(args.edif, exported_edif)
        args.edif = exported_edif

    io_assignments = collect_io_assignments(state)
    package_pin_sites = load_package_pin_sites(args.db_dir)
    placements, placement_warnings = collect_placements(state, db, io_assignments, package_pin_sites)
    routes = collect_routes(state, db)

    write_project_tcl(args.output_dir / "create_project.tcl", args, top)
    write_io_tcl(args.output_dir / "io.tcl", io_assignments)
    write_placing_tcl(args.output_dir / "placing.tcl", placements, placement_warnings)
    write_routing_tcl(args.output_dir / "routing.tcl", routes)
    write_scalepnr_pnr_export(args.output_dir / "scalepnr_place_route_export.txt", placements, routes, db)
    write_manifest(args.output_dir / "manifest.json", args, top, placements, routes, io_assignments)

    print(f"wrote Vivado project Tcl to {args.output_dir}")
    if args.edif:
        print(f"  edif: {args.edif}")
    else:
        print(f"  source glob: {args.source_root / '*.sv'}")
    print(f"  io constraints: {len(io_assignments)}")
    print(f"  placement constraints: {len(placements)}")
    print(f"  routing constraints: {len(routes)}")
    print(f"  placement warnings: {len(placement_warnings)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
