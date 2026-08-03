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
    slice_count,
)
from a7_packing import A7PackedCell, aggregate_a7_route_tree, legalize_a7_mux_placements


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
class PassthroughReplacement:
    pin: tuple[str, str]
    output_nodes: list[str]


@dataclass(frozen=True)
class RouteExport:
    net_name: str
    net_candidates: list[str]
    pin_candidates: list[tuple[str, str]]
    source_pin: tuple[str, str] | None
    paths: list[list[str]]
    full_paths: list[list[str]]
    pips: list[str]


_tile_type_cache: dict[str, dict[str, Any]] = {}
_tileconn_cache: dict[Path, list[dict[str, Any]]] = {}
_tile_by_type_coord_cache: dict[int, dict[tuple[str, int, int], str]] = {}
_node_neighbor_cache: dict[tuple[int, str], list[str]] = {}
_pip_neighbors_by_source_cache: dict[int, dict[tuple[str, str], list[str]]] = {}
_tile_json_pip_edges_cache: dict[tuple[int, str], frozenset[tuple[str, str]]] = {}
_tileconn_neighbors_by_source_cache: dict[int, dict[tuple[str, str], list[tuple[str, int, int, str]]]] = {}
_tileconn_alias_nodes_cache: dict[tuple[int, str], frozenset[str]] = {}


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


def lut_bel(inst: PlacedInst, packed: A7PackedCell) -> str:
    return f"{BEL_LETTERS[packed.placement.bel_index]}{packed.lut_bel_size}LUT"


def clb_bel(inst: PlacedInst, packed: A7PackedCell) -> str | None:
    kind = cell_kind(inst)
    placement = packed.placement
    if kind == "FD":
        suffix = "5FF" if placement.bel_index >= 4 else "FF"
        return f"{BEL_LETTERS[placement.bel_index % 4]}{suffix}"
    if kind == "LUT":
        return lut_bel(inst, packed)
    if kind == "CARRY":
        return "CARRY4"
    if kind == "MUX":
        if inst.cell_type.startswith("MUXF8"):
            return "F8MUX"
        return "F7BMUX" if placement.bel_index else "F7AMUX"
    return None


def load_tile_type_spec(db: PrjxrayDb, tile_type: str) -> dict[str, Any] | None:
    if tile_type in _tile_type_cache:
        return _tile_type_cache[tile_type]
    path = db.db_dir / f"tile_type_{tile_type}.json"
    if not path.exists():
        return None
    with path.open() as f:
        data = json.load(f)
    _tile_type_cache[tile_type] = data
    return data


def load_tileconn_spec(db: PrjxrayDb) -> list[dict[str, Any]]:
    path = db.db_dir / "tileconn.json"
    if path not in _tileconn_cache:
        with path.open() as f:
            _tileconn_cache[path] = json.load(f)
    return _tileconn_cache[path]


def clb_route_wire_for_site_pin(
    db: PrjxrayDb,
    resource_tile_name: str,
    route_tile_name: str,
    site_index: int,
    site_pin: str,
) -> str | None:
    resource_tile = db.tilegrid.get(resource_tile_name)
    route_tile = db.tilegrid.get(route_tile_name)
    if resource_tile is None or route_tile is None:
        return None

    tile_spec = load_tile_type_spec(db, resource_tile.type)
    if tile_spec is None:
        return None
    sites = tile_spec.get("sites", [])
    if site_index < 0 or site_index >= len(sites):
        return None
    site_pins = sites[site_index].get("site_pins", {})
    resource_wire = (site_pins.get(site_pin) or {}).get("wire")
    if not resource_wire:
        return None

    clb_wire = None
    for pip_name in tile_spec.get("pips", {}):
        prefix = f"{resource_tile.type}."
        if not pip_name.startswith(prefix) or "->" not in pip_name:
            continue
        src, dst = pip_name[len(prefix):].split("->", 1)
        if dst == resource_wire and "IMUX" in src:
            clb_wire = src
            break
    if clb_wire is None:
        return None

    grid_delta = (
        route_tile.grid_x - resource_tile.grid_x,
        route_tile.grid_y - resource_tile.grid_y,
    )
    for row in load_tileconn_spec(db):
        if row.get("tile_types") != [resource_tile.type, route_tile.type]:
            continue
        if tuple(int(value) for value in row.get("grid_deltas", [])) != grid_delta:
            continue
        for left, right in row.get("wire_pairs", []):
            if left == clb_wire:
                return str(right)
    return None


def packed_lut_sink_tail_node(
    db: PrjxrayDb,
    route_tile_name: str,
    resource_tile_name: str,
    packed: A7PackedCell,
    port: str,
) -> str | None:
    match = re.fullmatch(r"I([0-5])", port)
    if match is None:
        return None
    bel = packed.placement.bel_index % 4
    site_pin = f"{BEL_LETTERS[bel]}{int(match.group(1)) + 1}"
    route_wire = clb_route_wire_for_site_pin(
        db,
        resource_tile_name,
        route_tile_name,
        packed.placement.site_index,
        site_pin,
    )
    if route_wire is None:
        return None
    return f"{route_tile_name}/{route_wire}"


def clb_route_wire_for_site_output(
    db: PrjxrayDb,
    resource_tile_name: str,
    route_tile_name: str,
    site_index: int,
    site_pin: str,
) -> tuple[str, str, str] | None:
    resource_tile = db.tilegrid.get(resource_tile_name)
    route_tile = db.tilegrid.get(route_tile_name)
    if resource_tile is None or route_tile is None:
        return None

    tile_spec = load_tile_type_spec(db, resource_tile.type)
    if tile_spec is None:
        return None
    sites = tile_spec.get("sites", [])
    if site_index < 0 or site_index >= len(sites):
        return None
    site_pins = sites[site_index].get("site_pins", {})
    resource_wire = (site_pins.get(site_pin) or {}).get("wire")
    if not resource_wire:
        return None

    clb_wire = None
    prefix = f"{resource_tile.type}."
    for pip_name in tile_spec.get("pips", {}):
        if not pip_name.startswith(prefix) or "->" not in pip_name:
            continue
        src, dst = pip_name[len(prefix):].split("->", 1)
        if src == resource_wire and "LOGIC_OUTS" in dst:
            clb_wire = dst
            break
    if clb_wire is None:
        return None

    grid_delta = (
        route_tile.grid_x - resource_tile.grid_x,
        route_tile.grid_y - resource_tile.grid_y,
    )
    for row in load_tileconn_spec(db):
        if row.get("tile_types") != [resource_tile.type, route_tile.type]:
            continue
        if tuple(int(value) for value in row.get("grid_deltas", [])) != grid_delta:
            continue
        for left, right in row.get("wire_pairs", []):
            if left == clb_wire:
                return resource_wire, clb_wire, str(right)
    return None


def packed_output_site_pin(inst: dict[str, Any], packed: A7PackedCell) -> str | None:
    kind = cell_kind(PlacedInst(
        name=str(inst.get("name", "")),
        cell_type=str(inst.get("type", "")),
        pos=int(inst.get("pos", -1)),
        cb_tile=str(inst.get("annotation", {}).get("cb_tile", "")),
        resource_tile=str(inst.get("annotation", {}).get("resource_tile", inst.get("resource_tile", ""))),
        grid_coord=tuple(inst.get("annotation", {}).get("grid_coord", [-1, -1])),
        raw=inst,
    ))
    bel = packed.placement.bel_index
    if kind == "LUT":
        # Vivado exposes a fractured LUT's O5 output on the column *Q tile
        # wire even when no flip-flop is packed in that column.
        suffix = "Q" if packed.lut_bel_size == 5 else ""
        return f"{BEL_LETTERS[bel % 4]}{suffix}"
    if kind == "FD":
        # The secondary *5FF BEL leaves the slice through the corresponding
        # *MUX site pin; only the primary FF uses the *Q site pin.
        suffix = "MUX" if bel >= 4 else "Q"
        return f"{BEL_LETTERS[bel % 4]}{suffix}"
    if kind == "MUX":
        if str(inst.get("type", "")).startswith("MUXF8"):
            return "BMUX"
        return "CMUX" if bel else "AMUX"
    return None


def packed_clb_output_nodes(
    inst: dict[str, Any],
    wire: dict[str, Any],
    db: PrjxrayDb,
    packed_cells: dict[str, A7PackedCell],
) -> list[str]:
    packed = packed_cells.get(str(inst.get("name", "")))
    if packed is None:
        return []
    ann = wire.get("annotation", {})
    route_tile_name = str(ann.get("from_cb_tile") or ann.get("to_cb_tile") or "")
    original_resource_tile = str(
        (ann.get("tile_resource") or {}).get("tile")
        or ann.get("resource_tile")
        or inst.get("resource_tile")
        or ""
    )
    if not route_tile_name or not original_resource_tile:
        return []
    if packed.tile_name != original_resource_tile:
        return []

    site_pin = packed_output_site_pin(inst, packed)
    if site_pin is None:
        return []

    resolved = clb_route_wire_for_site_output(
        db,
        packed.tile_name,
        route_tile_name,
        packed.placement.site_index,
        site_pin,
    )
    if resolved is None:
        return []
    resource_wire, clb_wire, route_wire = resolved
    return [f"{packed.tile_name}/{resource_wire}", f"{packed.tile_name}/{clb_wire}", f"{route_tile_name}/{route_wire}"]


def annotated_clb_output_nodes(wire: dict[str, Any], db: PrjxrayDb) -> list[str]:
    ann = wire.get("annotation", {})
    route_tile_name = str(ann.get("from_cb_tile") or ann.get("to_cb_tile") or "")
    tile_resource = ann.get("tile_resource") or {}
    output = tile_resource.get("output") or {}
    resource_full_name = str(output.get("resource_full_name") or "")
    local_wire_full_name = str(output.get("local_wire_full_name") or "")
    if not route_tile_name or not resource_full_name or not local_wire_full_name:
        return []

    resource_tile_name = str(tile_resource.get("tile") or resource_full_name.split(".", 1)[0])
    resource_tile = db.tilegrid.get(resource_tile_name)
    route_tile = db.tilegrid.get(route_tile_name)
    if resource_tile is None or route_tile is None:
        return []

    resource_node = vivado_node_name(resource_full_name)
    local_node = vivado_node_name(local_wire_full_name)
    local_wire = vivado_node_wire(local_node)
    route_wire = ""
    for node in ann.get("nodes", []):
        full_name = str(node.get("full_name") or "")
        if full_name.startswith(route_tile_name + "."):
            route_wire = full_name.split(".", 1)[1]
            break
    if not route_wire:
        return []

    grid_delta = (
        route_tile.grid_x - resource_tile.grid_x,
        route_tile.grid_y - resource_tile.grid_y,
    )
    clb_wire = ""
    for row in load_tileconn_spec(db):
        if row.get("tile_types") != [resource_tile.type, route_tile.type]:
            continue
        if tuple(int(value) for value in row.get("grid_deltas", [])) != grid_delta:
            continue
        for left, right in row.get("wire_pairs", []):
            if right != route_wire:
                continue
            candidate = f"{resource_tile_name}/{left}"
            if (candidate == local_node
                    or direct_pip_feature(db, local_node, candidate) is not None
                    or intermediate_pip_node(db, local_node, candidate)):
                clb_wire = str(left)
                break
        if clb_wire:
            break
    if not clb_wire:
        return []

    out = [resource_node]
    if out[-1] != local_node:
        out.append(local_node)
    clb_node = f"{resource_tile_name}/{clb_wire}"
    if out[-1] != clb_node:
        out.append(clb_node)
    route_node = f"{route_tile_name}/{route_wire}"
    if out[-1] != route_node:
        out.append(route_node)
    return out


def original_clb_bel_index(inst: PlacedInst) -> int:
    local = inst.pos % 128
    kind = cell_kind(inst)
    if kind == "FD":
        bel = local // 4
        if local % 4 == 1:
            bel += 4
        return max(0, min(7, bel))
    if kind == "MUX":
        if inst.cell_type.startswith("MUXF8"):
            return 0
        return 1 if local >= 8 else 0
    if kind == "CARRY":
        return 0
    return max(0, min(3, local // 4))


def original_clb_packed_cells(grouped: dict[str, Any], clb_tiles: dict[str, TileInfo]) -> dict[str, A7PackedCell]:
    packed: dict[str, A7PackedCell] = {}
    for tile_state in grouped.values():
        tile = tile_state.clb_tile
        if tile.name not in clb_tiles:
            continue
        for inst in tile_state.insts:
            kind = cell_kind(inst)
            if kind == "OTHER":
                continue
            site = min(1 if inst.pos >= 128 else 0, slice_count(tile) - 1)
            bel = original_clb_bel_index(inst)
            packed[inst.name] = A7PackedCell(
                tile.name,
                PackedPlacement(site, bel, inst.pos, "scalepnr-routed-pos"),
                True,
            )
    return packed


def annotated_element_packed_cells(grouped: dict[str, Any], clb_tiles: dict[str, TileInfo]) -> dict[str, A7PackedCell]:
    packed: dict[str, A7PackedCell] = {}
    for tile_state in grouped.values():
        tile = tile_state.clb_tile
        if tile.name not in clb_tiles:
            continue
        for inst in tile_state.insts:
            kind = cell_kind(inst)
            if kind == "OTHER":
                continue
            element = inst.raw.get("annotation", {}).get("element", {})
            if not isinstance(element, dict):
                continue
            element_type = str(element.get("type", ""))
            bit = int(element.get("bit", -1))
            site = int(element.get("site_index", 1 if inst.pos >= 128 else 0))
            lane = int(element.get("lane_index", bit % 8 if kind == "FD" else bit % 4))
            site = max(0, min(site, slice_count(tile) - 1))
            if kind == "FD":
                bel = max(0, min(7, lane))
            elif kind == "MUX" and inst.cell_type.startswith("MUXF7"):
                bel = 1 if lane >= 2 else 0
            elif kind in {"MUX", "CARRY"} or element_type in {"MUXF8", "CARRY4"}:
                bel = 0
            else:
                bel = max(0, min(3, lane % 4))
            packed[inst.name] = A7PackedCell(
                tile.name,
                PackedPlacement(site, bel, int(element.get("pos", inst.pos)), "scalepnr-element"),
                True,
            )
    return packed


def annotated_element_packed_cell(inst: dict[str, Any], tile_name: str, fallback: A7PackedCell | None = None) -> A7PackedCell | None:
    element = inst.get("annotation", {}).get("element", {})
    if not isinstance(element, dict):
        return fallback
    kind = str(inst.get("type", ""))
    cell = PlacedInst(
        name=str(inst.get("name", "")),
        cell_type=str(inst.get("type", "")),
        pos=int(inst.get("pos", -1)),
        cb_tile=str(inst.get("annotation", {}).get("cb_tile", "")),
        resource_tile=str(inst.get("annotation", {}).get("resource_tile", inst.get("resource_tile", ""))),
        grid_coord=tuple(inst.get("annotation", {}).get("grid_coord", [-1, -1])),
        raw=inst,
    )
    cell_kind_name = cell_kind(cell)
    if cell_kind_name == "OTHER":
        return fallback

    bit = int(element.get("bit", -1))
    site = int(element.get("site_index", 1 if int(inst.get("pos", -1)) >= 128 else 0))
    lane = int(element.get("lane_index", bit % 8 if cell_kind_name == "FD" else bit % 4))
    if cell_kind_name == "FD":
        bel = max(0, min(7, lane))
    elif cell_kind_name == "MUX" and kind.startswith("MUXF7"):
        bel = 1 if lane >= 2 else 0
    elif cell_kind_name in {"MUX", "CARRY"} or str(element.get("type", "")) in {"MUXF8", "CARRY4"}:
        bel = 0
    else:
        bel = max(0, min(3, lane % 4))
    return A7PackedCell(
        tile_name,
        PackedPlacement(site, bel, int(element.get("pos", inst.get("pos", -1))), "scalepnr-element-endpoint"),
        True,
    )


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
        VivadoPlacement(
            placement.inst,
            site_by_inst.get(placement.inst.name, placement.site),
            placement.bel,
            placement.constrain,
        )
        for placement in placements
    ]


def is_scalepnr_passthrough(inst: PlacedInst) -> bool:
    return bool(inst.raw.get("attrs", {}).get("scalepnr_passthrough"))


def is_scalepnr_constant(inst: PlacedInst) -> bool:
    return str(inst.raw.get("attrs", {}).get("scalepnr_constant", "")) == "1"


def mux_shape_children(state: dict[str, Any]) -> set[str]:
    """Return non-root cells that Vivado places as part of a wide-mux shape."""
    insts = {str(inst.get("name", "")): inst for inst in state.get("insts", [])}
    children: set[str] = set()

    def collect_inputs(inst: dict[str, Any]) -> None:
        for conn in inst.get("connections", []):
            driver = str(conn.get("driver_inst", ""))
            if not driver or driver in children:
                continue
            driver_inst = insts.get(driver)
            if driver_inst is None:
                continue
            driver_type = str(driver_inst.get("type", ""))
            if driver_type.startswith("MUXF7") or driver_type.startswith("LUT"):
                children.add(driver)
                collect_inputs(driver_inst)

    for inst in insts.values():
        if str(inst.get("type", "")).startswith("MUXF8"):
            collect_inputs(inst)
    return children


def collect_placements(
    state: dict[str, Any],
    db: PrjxrayDb,
    io_assignments: list[IoAssignment],
    package_pin_sites: dict[str, str],
) -> tuple[list[VivadoPlacement], list[str], dict[str, A7PackedCell]]:
    out = FasmOutput()
    grouped = group_by_clb_tile(state, db, out)
    placements: list[VivadoPlacement] = []
    warnings: list[str] = list(out.warnings)
    clb_tiles = {name: tile for name, tile in db.tilegrid.items() if tile.type in CLB_TYPES}
    annotated_packed = annotated_element_packed_cells(grouped, clb_tiles)
    pack_result = legalize_a7_mux_placements(grouped, annotated_packed)
    packed = dict(pack_result.cells)
    warnings.extend(pack_result.warnings)
    if annotated_packed:
        warnings.append(f"use {len(annotated_packed)} DB element endpoint annotations as A7 packing hints")
    tile_by_inst: dict[str, TileInfo] = {}
    for tile_state in grouped.values():
        for inst in tile_state.insts:
            tile_by_inst[inst.name] = tile_state.clb_tile

    for inst in placed_insts(state):
        if is_scalepnr_passthrough(inst) or is_scalepnr_constant(inst):
            kind = "constant source" if is_scalepnr_constant(inst) else "passthrough"
            warnings.append(f"skip generated {kind} placement for Vivado EDIF cell: {inst.name}")
            continue

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
            # Every real packed EDIF primitive must keep the site/BEL identity
            # used to reconstruct its route endpoints. Wide-mux children are
            # already legalized by a7_packing and cannot be left to Vivado.
            placements.append(VivadoPlacement(inst, site, clb_bel(inst, packed_cell), packed_cell.constrain))
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

        # Any database-loaded site can place a compatible primitive without
        # adding its architecture identity to the generic scalepnr model.
        site_annotation = inst.raw.get("annotation", {}).get("site", {})
        site_index = int(site_annotation.get("index", inst.pos)) if isinstance(site_annotation, dict) else inst.pos
        site = site_name_for_index(tile, site_index)
        site_type = tile.sites.get(site, "") if site else ""
        annotated_type = str(site_annotation.get("type", "")) if isinstance(site_annotation, dict) else ""
        compatible_type = site_type == inst.cell_type or site_type.startswith(inst.cell_type)
        compatible_type = compatible_type or annotated_type == inst.cell_type or annotated_type.startswith(inst.cell_type)
        if site and compatible_type:
            # A site type that begins with the primitive type exposes a
            # same-named leaf BEL. Keep that endpoint identity in the export.
            bel = f"{inst.cell_type}.{inst.cell_type}" if site_type.startswith(inst.cell_type) else None
            placements.append(VivadoPlacement(inst, site, bel))
            continue

        warnings.append(f"skip placement for unsupported placed cell {inst.cell_type}: {inst.name}")

    return placements, warnings, packed


def route_name(inst: dict[str, Any], route: list[dict[str, Any]]) -> str:
    for wire in route:
        net = wire.get("net")
        if net:
            return str(net)
    return str(inst.get("name", "<unnamed-route>"))


def strip_net_bit(name: str) -> str:
    return re.sub(r"\[\d+\]$", "", name)


def net_relation_keys(name: str) -> set[str]:
    # Keep bus bits distinct. Collapsing foo[3] to foo makes fixed-route
    # constraints for one bit claim pins/routes from sibling bits.
    keys = {name}
    if ".$" in name:
        suffix = "$" + name.rsplit(".$", 1)[1]
        keys.add(suffix)
    for candidate in list(keys):
        abc = candidate.find("$abc")
        if abc > 0:
            keys.add(candidate[abc:])
    return {key for key in keys if len(key) > 2}


def related_route_net(left: str, right: str) -> bool:
    left_keys = net_relation_keys(left)
    right_keys = net_relation_keys(right)
    return bool(left_keys & right_keys)


def route_final_sink_pin(route: list[dict[str, Any]]) -> dict[str, Any] | None:
    for wire in reversed(route):
        if wire.get("type") != "tile_pin":
            continue
        if int(wire.get("pin_dir", 0)) > 0:
            continue
        return wire
    return None


def route_first_source_pin(route: list[dict[str, Any]]) -> dict[str, Any] | None:
    for wire in route:
        if wire.get("type") != "tile_pin":
            continue
        if int(wire.get("pin_dir", 0)) > 0:
            return wire
    return None


def route_final_sink_ports(inst: dict[str, Any], route: list[dict[str, Any]]) -> list[str]:
    wire = route_final_sink_pin(route)
    if wire is None:
        return []
    port = str(wire.get("port", ""))
    if port:
        return [port]

    tile_resource = (wire.get("annotation") or {}).get("tile_resource") or {}
    input_pin = str((tile_resource.get("input") or {}).get("pin") or "")
    lut_match = re.fullmatch(r"[A-D](\d+)", input_pin)
    if lut_match:
        index = int(lut_match.group(1)) - 1
        if 0 <= index <= 5:
            return [f"I{index}"]

    cell_type = str(inst.get("type", ""))
    if re.fullmatch(r"[A-D]X", input_pin):
        if cell_type.startswith("MUXF"):
            return ["S"]
        if cell_type.startswith("FD"):
            return ["D"]
    if cell_type.startswith("LUT") and re.fullmatch(r"[A-D]I", input_pin):
        return ["I0"]
    return []


def route_final_sink_resource_pin(route: list[dict[str, Any]]) -> str:
    wire = route_final_sink_pin(route)
    if wire is None:
        return ""
    tile_resource = (wire.get("annotation") or {}).get("tile_resource") or {}
    return str((tile_resource.get("input") or {}).get("pin") or "")


def route_sink_connection_matches(conn: dict[str, Any], final_ports: list[str], final_resource_pin: str) -> bool:
    if final_ports and str(conn.get("sink_port", "")) in final_ports:
        return True
    if final_resource_pin and str(conn.get("sink_pin", "")) == final_resource_pin:
        return True
    return False


def route_sink_has_constraint(final_ports: list[str], final_resource_pin: str) -> bool:
    return bool(final_ports or final_resource_pin)


def route_add_matching_connection_nets(
    inst: dict[str, Any],
    route: list[dict[str, Any]],
    nets: list[str],
) -> None:
    final_ports = route_final_sink_ports(inst, route)
    final_resource_pin = route_final_sink_resource_pin(route)
    constrained = route_sink_has_constraint(final_ports, final_resource_pin)

    for conn in inst.get("connections", []):
        conn_net = str(conn.get("net", ""))
        if not conn_net:
            continue
        if constrained:
            if not route_sink_connection_matches(conn, final_ports, final_resource_pin):
                continue
        elif not any(related_route_net(route_net, conn_net) for route_net in nets):
            continue
        if conn_net not in nets:
            nets.append(conn_net)


def route_net_names(inst: dict[str, Any], route: list[dict[str, Any]]) -> list[str]:
    nets: list[str] = []
    for wire in route:
        net = wire.get("net")
        if net and str(net) not in nets:
            nets.append(str(net))

    route_add_matching_connection_nets(inst, route, nets)

    if not nets:
        nets.append(str(inst.get("name", "<unnamed-route>")))
    return nets


def route_pin_net_names(inst: dict[str, Any], route: list[dict[str, Any]], primary_net: str) -> list[str]:
    nets: list[str] = [primary_net] if primary_net else []
    route_add_matching_connection_nets(inst, route, nets)
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
    if vivado_node_tile(node_name).startswith("BRKH_INT_"):
        return False
    return "END" in vivado_node_wire(node_name)


def is_fixed_route_interior_alias(node_name: str) -> bool:
    return False


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
    site = pos // 128
    bel = (pos % 128) // 4
    if bel < 0 or bel >= 4:
        return None
    # D inputs use the CLB bypass tails. The two slices are wired to
    # different bypass indices, so include the packed site half.
    site_maps = ([1, 4, 3, 6], [0, 5, 2, 7])
    site_index = 1 if site % 2 else 0
    return clb_bypass_tail_name(tile, site_maps[site_index][bel])


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


def placed_iob_input_tail_node(tile: str, placed_site: str | None) -> str | None:
    if not placed_site:
        return obuf_input_tail_node(tile)
    site_match = re.fullmatch(r"IOB_X\d+Y(\d+)", placed_site)
    tile_match = re.fullmatch(r"(INT_[LR]_X\d+)Y\d+", tile)
    if site_match is None or tile_match is None:
        return obuf_input_tail_node(tile)
    target_tile = f"{tile_match.group(1)}Y{site_match.group(1)}"
    return obuf_input_tail_node(target_tile)


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


def json_direct_pip_feature(db: PrjxrayDb, src_node: str, dst_node: str) -> str | None:
    """Return an edge recorded only in tile JSON, without changing route search."""
    src_tile = vivado_node_tile(src_node)
    if src_tile != vivado_node_tile(dst_node):
        return None
    info = db.tilegrid.get(src_tile)
    if info is None:
        return None
    src_wire = vivado_node_wire(src_node)
    dst_wire = vivado_node_wire(dst_node)
    cache_key = (id(db), info.type)
    if cache_key not in _tile_json_pip_edges_cache:
        tile_spec = load_tile_type_spec(db, info.type)
        _tile_json_pip_edges_cache[cache_key] = frozenset(
            (str(pip.get("src_wire", "")), str(pip.get("dst_wire", "")))
            for pip in (tile_spec or {}).get("pips", {}).values()
        )
    if (src_wire, dst_wire) in _tile_json_pip_edges_cache[cache_key]:
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


def tile_by_type_coord(db: PrjxrayDb) -> dict[tuple[str, int, int], str]:
    key = id(db)
    if key not in _tile_by_type_coord_cache:
        _tile_by_type_coord_cache[key] = {
            (tile.type, tile.grid_x, tile.grid_y): tile.name
            for tile in db.tilegrid.values()
        }
    return _tile_by_type_coord_cache[key]


def pip_neighbors_by_source(db: PrjxrayDb) -> dict[tuple[str, str], list[str]]:
    key = id(db)
    if key not in _pip_neighbors_by_source_cache:
        indexed: dict[tuple[str, str], list[str]] = {}
        for (tile_type, dst_wire), src_wires in db.pip_sources_by_type_dst.items():
            for src_wire in sorted(src_wires):
                indexed.setdefault((tile_type, src_wire), []).append(dst_wire)
        for destinations in indexed.values():
            destinations.sort()
        _pip_neighbors_by_source_cache[key] = indexed
    return _pip_neighbors_by_source_cache[key]


def tileconn_neighbors_by_source(db: PrjxrayDb) -> dict[tuple[str, str], list[tuple[str, int, int, str]]]:
    key = id(db)
    if key not in _tileconn_neighbors_by_source_cache:
        indexed: dict[tuple[str, str], list[tuple[str, int, int, str]]] = {}
        for row in load_tileconn_spec(db):
            tile_types = row.get("tile_types", [])
            deltas = row.get("grid_deltas", [])
            if len(tile_types) != 2 or len(deltas) != 2:
                continue
            for left, right in row.get("wire_pairs", []):
                indexed.setdefault((str(tile_types[0]), str(left)), []).append(
                    (str(tile_types[1]), int(deltas[0]), int(deltas[1]), str(right))
                )
                # tileconn wire pairs are two names for one physical node, not
                # directed PIPs. Either tile can therefore be the lookup side.
                indexed.setdefault((str(tile_types[1]), str(right)), []).append(
                    (str(tile_types[0]), -int(deltas[0]), -int(deltas[1]), str(left))
                )
        _tileconn_neighbors_by_source_cache[key] = indexed
    return _tileconn_neighbors_by_source_cache[key]


def is_direct_tileconn_edge(db: PrjxrayDb, src_node: str, dst_node: str) -> bool:
    src_info = db.tilegrid.get(vivado_node_tile(src_node))
    dst_info = db.tilegrid.get(vivado_node_tile(dst_node))
    if src_info is None or dst_info is None:
        return False
    src_wire = vivado_node_wire(src_node)
    dst_wire = vivado_node_wire(dst_node)
    for target_type, delta_x, delta_y, target_wire in tileconn_neighbors_by_source(db).get(
        (src_info.type, src_wire), []
    ):
        if (
            target_type == dst_info.type
            and target_wire == dst_wire
            and src_info.grid_x + delta_x == dst_info.grid_x
            and src_info.grid_y + delta_y == dst_info.grid_y
        ):
            return True
    return False


def route_node_neighbors(db: PrjxrayDb, node: str) -> list[str]:
    cache_key = (id(db), node)
    if cache_key in _node_neighbor_cache:
        return _node_neighbor_cache[cache_key]

    tile_name = vivado_node_tile(node)
    wire = vivado_node_wire(node)
    info = db.tilegrid.get(tile_name)
    if info is None:
        _node_neighbor_cache[cache_key] = []
        return []

    neighbors = [
        f"{tile_name}/{dst_wire}"
        for dst_wire in pip_neighbors_by_source(db).get((info.type, wire), [])
    ]

    by_type_coord = tile_by_type_coord(db)
    for target_type, delta_x, delta_y, right in tileconn_neighbors_by_source(db).get((info.type, wire), []):
        target_tile = by_type_coord.get((target_type, info.grid_x + delta_x, info.grid_y + delta_y))
        if target_tile is None:
            continue
        neighbors.append(f"{target_tile}/{right}")

    out = sorted(set(neighbors))
    _node_neighbor_cache[cache_key] = out
    return out


def tileconn_alias_nodes(db: PrjxrayDb, node: str) -> frozenset[str]:
    cache_key = (id(db), node)
    cached = _tileconn_alias_nodes_cache.get(cache_key)
    if cached is not None:
        return cached

    aliases = {node}
    queue = [node]
    by_type_coord = tile_by_type_coord(db)
    for current in queue:
        info = db.tilegrid.get(vivado_node_tile(current))
        if info is None:
            continue
        wire = vivado_node_wire(current)
        for target_type, delta_x, delta_y, target_wire in tileconn_neighbors_by_source(db).get(
            (info.type, wire), []
        ):
            target_tile = by_type_coord.get(
                (target_type, info.grid_x + delta_x, info.grid_y + delta_y)
            )
            if target_tile is None:
                continue
            target = f"{target_tile}/{target_wire}"
            if target not in aliases:
                aliases.add(target)
                queue.append(target)

    result = frozenset(aliases)
    for alias in aliases:
        _tileconn_alias_nodes_cache[(id(db), alias)] = result
    return result


def physical_route_pips(db: PrjxrayDb, full_nodes: list[str]) -> list[str]:
    components: list[set[str]] = []
    component_nodes: list[list[str]] = []
    for node in full_nodes:
        aliases = set(tileconn_alias_nodes(db, node))
        if components and components[-1].intersection(aliases):
            components[-1].update(aliases)
            component_nodes[-1].append(node)
        else:
            components.append(aliases)
            component_nodes.append([node])

    pips: list[str] = []
    for index, (sources, destinations) in enumerate(zip(components, components[1:])):
        destinations_by_tile: dict[str, list[str]] = {}
        for destination in destinations:
            destinations_by_tile.setdefault(vivado_node_tile(destination), []).append(destination)
        features = sorted({
            feature
            for source in sources
            for destination in destinations_by_tile.get(vivado_node_tile(source), [])
            if (feature := direct_pip_feature(db, source, destination)) is not None
        })
        if not features:
            features = sorted({
                feature
                for source in component_nodes[index]
                for destination in component_nodes[index + 1]
                if (feature := json_direct_pip_feature(db, source, destination)) is not None
            })
        for feature in features:
            if feature not in pips:
                pips.append(feature)
    return pips


def short_route_expansion(db: PrjxrayDb, src_node: str, dst_node: str, max_depth: int = 10) -> list[str] | None:
    if src_node == dst_node:
        return [src_node]
    queue: list[list[str]] = [[src_node]]
    seen = {src_node}
    for path in queue:
        if len(path) > max_depth:
            continue
        if path[-1] != src_node and is_route_search_terminal(path[-1]):
            continue
        for neighbor in route_node_neighbors(db, path[-1]):
            if neighbor in seen:
                continue
            next_path = [*path, neighbor]
            if neighbor == dst_node:
                return next_path
            seen.add(neighbor)
            queue.append(next_path)
    return None


def is_route_search_terminal(node: str) -> bool:
    wire = vivado_node_wire(node)
    return "IMUX" in wire or "CTRL" in wire or "GCLK" in wire


def short_route_expansion_to_targets(
    db: PrjxrayDb,
    src_node: str,
    dst_nodes: Iterable[str],
    max_depth: int = 24,
    blocked_nodes: set[str] | None = None,
) -> tuple[str, list[str]] | None:
    blocked = blocked_nodes or set()
    targets = set(dst_nodes) - blocked
    if src_node in targets:
        return src_node, [src_node]
    queue: list[list[str]] = [[src_node]]
    seen = {src_node}
    for path in queue:
        if len(path) > max_depth:
            continue
        if path[-1] != src_node and is_route_search_terminal(path[-1]):
            continue
        for neighbor in route_node_neighbors(db, path[-1]):
            if neighbor in seen or (neighbor in blocked and neighbor != src_node):
                continue
            next_path = [*path, neighbor]
            if neighbor in targets:
                return neighbor, next_path
            seen.add(neighbor)
            queue.append(next_path)
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
            elif mid is None:
                path = short_route_expansion(db, src, dst)
                if path is not None:
                    for node in path[1:-1]:
                        if expanded[-1] != node:
                            expanded.append(node)
        if expanded[-1] != dst:
            expanded.append(dst)
    return expanded


def repair_clb_output_hops(
    db: PrjxrayDb,
    nodes: list[str],
    blocked_nodes: set[str] | None = None,
) -> list[str]:
    repaired = list(nodes)
    index = 0
    while index + 2 < len(repaired):
        resource_output = repaired[index]
        route_output = repaired[index + 1]
        if not (
            "LOGIC_OUTS" in vivado_node_wire(resource_output)
            and not vivado_node_tile(resource_output).startswith("INT_")
            and vivado_node_tile(route_output).startswith("INT_")
            and "LOGIC_OUTS" in vivado_node_wire(route_output)
        ):
            index += 1
            continue

        next_node = repaired[index + 2]
        blocked = blocked_nodes or set()
        blocked_indices = [
            candidate_index
            for candidate_index, candidate in enumerate(repaired[index + 2:], start=index + 2)
            if candidate in blocked
        ]
        if (
            direct_pip_feature(db, route_output, next_node) is not None
            or is_direct_tileconn_edge(db, route_output, next_node)
        ) and not blocked_indices:
            index += 2
            continue

        # Packing can replace a generated site-internal source with its real
        # driver. Reconnect that driver's route output to the earliest node of
        # the existing trunk reachable through directed database edges.
        target_start = index + 3
        targets = {
            node: target_index
            for target_index, node in enumerate(repaired[target_start:], start=target_start)
            if node not in blocked
        }
        expansion = short_route_expansion_to_targets(
            db,
            route_output,
            targets,
            max_depth=48,
            blocked_nodes=blocked_nodes,
        )
        if expansion is None:
            index += 2
            continue

        target, bridge = expansion
        target_index = targets[target]
        repaired[index + 1:target_index + 1] = bridge
        index += len(bridge)
    return repaired


def repair_reserved_route_nodes(
    db: PrjxrayDb,
    nodes: list[str],
    reserved_nodes: set[str] | None,
) -> list[str]:
    """Bridge around packed-resource nodes reserved for another physical net."""
    if not reserved_nodes or not any(node in reserved_nodes for node in nodes):
        return nodes

    repaired = list(nodes)
    while True:
        blocked_index = next(
            (index for index, node in enumerate(repaired) if node in reserved_nodes),
            None,
        )
        if blocked_index is None:
            return repaired

        replacement: tuple[int, int, list[str]] | None = None
        left_start = max(0, blocked_index - 12)
        right_end = min(len(repaired), blocked_index + 24)
        for left in range(blocked_index - 1, left_start - 1, -1):
            targets = {
                repaired[right]: right
                for right in range(blocked_index + 1, right_end)
                if repaired[right] not in reserved_nodes
            }
            if not targets:
                continue
            expansion = short_route_expansion_to_targets(
                db,
                repaired[left],
                targets,
                max_depth=48,
                blocked_nodes=reserved_nodes,
            )
            if expansion is None:
                continue
            target, bridge = expansion
            replacement = (left, targets[target], bridge)
            break

        if replacement is None:
            raise ValueError(f"cannot bridge around reserved route node {repaired[blocked_index]}")
        left, right, bridge = replacement
        repaired[left:right + 1] = bridge


def route_tile_pin_node(
    inst: dict[str, Any],
    wire: dict[str, Any],
    db: PrjxrayDb,
    packed_cells: dict[str, A7PackedCell],
    sink_ports: list[str] | None = None,
    placed_site: str | None = None,
) -> str | None:
    ann = wire.get("annotation", {})
    tile = str(ann.get("from_cb_tile", ""))
    port = str(wire.get("port", ""))
    pos = int(wire.get("pos", -1))
    cell_type = str(inst.get("type", ""))
    if int(wire.get("pin_dir", 0)) <= 0 and cell_type.startswith("LUT"):
        fallback = packed_cells.get(str(inst.get("name", "")))
        resource_tile = str(ann.get("resource_tile") or inst.get("resource_tile") or "")
        packed = fallback
        resource_tile = packed.tile_name if packed is not None else resource_tile
        if packed is not None and resource_tile:
            for candidate_port in [port, *(sink_ports or [])]:
                packed_tail = packed_lut_sink_tail_node(db, tile, resource_tile, packed, candidate_port)
                if packed_tail:
                    return packed_tail
    if cell_type == "CARRY4":
        return carry_di_tail_node(tile, port)
    if cell_type.startswith("FD"):
        if port == "D":
            return fd_d_tail_node(tile, pos)
        # Shared control lanes are already identified exactly by the terminal
        # tile-pin annotation; a synthetic lane guess can suppress that pin.
        if port in {"R", "S", "CLR", "PRE", "SRST", "ARST"}:
            return None
    if cell_type == "OBUF" and port == "I":
        return placed_iob_input_tail_node(tile, placed_site)
    return None


def route_iob_resource_endpoint(
    tile_name: str,
    local_wire: str,
    db: PrjxrayDb | None,
    previous_node: str | None = None,
    placed_site: str | None = None,
) -> str | None:
    if db is None or not tile_name.startswith(("LIOB", "RIOB")):
        return None
    match = re.search(r"_X\d+Y\d+$", tile_name)
    if match is None:
        return None
    side = "LIOI" if tile_name.startswith("LIOB") else "RIOI"
    candidates = sorted(name for name in db.tilegrid if name.endswith(match.group(0)) and name.startswith(side))
    if not candidates:
        return None
    wire_match = re.fullmatch(r"IOB_O(\d+)", local_wire)
    if wire_match is None:
        return None
    io_tile = candidates[0]
    io_info = db.tilegrid.get(io_tile)
    resource_info = db.tilegrid.get(tile_name)
    if placed_site and io_info is not None and resource_info is not None:
        sites = site_names(resource_info)
        if placed_site in sites:
            site_index = len(sites) - 1 - sites.index(placed_site)
            endpoint = f"{io_tile}/IOI_OLOGIC{site_index}_D1"
            tile_spec = load_tile_type_spec(db, io_info.type)
            if tile_spec is not None and f"IOI_OLOGIC{site_index}_D1" in tile_spec.get("wires", {}):
                return endpoint
    prev_info = db.tilegrid.get(vivado_node_tile(previous_node or ""))
    prev_wire = vivado_node_wire(previous_node or "")
    imux_match = re.fullmatch(r"IMUX(?:_L)?(\d+)", prev_wire)
    if io_info is not None and prev_info is not None and imux_match is not None:
        imux_index = 0 if io_info.grid_y == prev_info.grid_y else 1
        io_imux = f"IOI_IMUX{imux_match.group(1)}_{imux_index}"
        for dst in sorted(
            key[1]
            for key, sources in db.pip_sources_by_type_dst.items()
            if key[0] == io_info.type and io_imux in sources
        ):
            if re.fullmatch(r"IOI_OLOGIC\d+_D1", dst):
                return f"{io_tile}/{dst}"
    return None


def iob_input_source_nodes(
    wire: dict[str, Any],
    db: PrjxrayDb | None,
    placed_site: str | None = None,
) -> list[str]:
    """Expand an annotated input-buffer output into its physical IOI source chain."""
    if db is None:
        return []
    endpoint = (((wire.get("annotation") or {}).get("tile_resource") or {}).get("output") or {})
    full_name = str(endpoint.get("resource_full_name") or "")
    if not full_name:
        return []
    resource_tile, _, resource_wire = full_name.replace("/", ".").partition(".")
    lane_match = re.fullmatch(r"IOB_IBUF([01])", resource_wire)
    if lane_match is None or not resource_tile.startswith(("LIOB", "RIOB")):
        return []

    coord_match = re.search(r"_X\d+Y\d+$", resource_tile)
    if coord_match is None:
        return []
    side = "LIOI" if resource_tile.startswith("LIOB") else "RIOI"
    io_tiles = sorted(
        name
        for name in db.tilegrid
        if name.startswith(side) and name.endswith(coord_match.group(0))
    )
    if not io_tiles:
        return []

    lane = int(lane_match.group(1))
    resource_info = db.tilegrid.get(resource_tile)
    if placed_site and resource_info is not None:
        sites = site_names(resource_info)
        if placed_site in sites:
            lane = len(sites) - 1 - sites.index(placed_site)
    io_tile = io_tiles[0]
    io_info = db.tilegrid.get(io_tile)
    logic_source = f"IOI_ILOGIC{lane}_O"
    logic_outputs: list[str] = []
    if io_info is not None:
        logic_outputs = sorted(
            dst
            for (tile_type, dst), sources in getattr(db, "pip_sources_by_type_dst", {}).items()
            if tile_type == io_info.type
            and logic_source in sources
            and re.fullmatch(r"IOI_LOGIC_OUTS18_\d+", dst)
        )
    logic_output = logic_outputs[0] if logic_outputs else f"IOI_LOGIC_OUTS18_{1 - lane}"
    return [
        f"{resource_tile}/IOB_IBUF{lane}",
        f"{io_tile}/{side}_IBUF{lane}",
        f"{io_tile}/{side}_I{lane}",
        f"{io_tile}/{side}_ILOGIC{lane}_D",
        f"{io_tile}/IOI_ILOGIC{lane}_O",
        f"{io_tile}/{logic_output}",
    ]


def ioi_output_tail_nodes(endpoint_node: str) -> list[str]:
    match = re.fullmatch(r"(.+)/(IOI_OLOGIC(\d+)_D1)", endpoint_node)
    if match is None:
        return []
    tile, _, index = match.groups()
    side = "RIOI" if vivado_node_tile(endpoint_node).startswith("RIOI") else "LIOI"
    return [
        f"{tile}/{side}_OLOGIC{index}_OQ",
        f"{tile}/{side}_O{index}",
    ]


def route_tile_resource_endpoint(
    wire: dict[str, Any],
    direction: str,
    db: PrjxrayDb | None = None,
    previous_node: str | None = None,
    placed_site: str | None = None,
) -> str | None:
    tile_resource = (wire.get("annotation") or {}).get("tile_resource") or {}
    endpoint = tile_resource.get(direction) or {}
    full_name = str(endpoint.get("resource_full_name") or "")
    if not full_name:
        return None
    tile_name = full_name.split(".", 1)[0]
    if tile_name.startswith(("LIOB", "RIOB")):
        if direction == "output" and not re.fullmatch(r"IOB_O\d+", str(endpoint.get("local_wire") or "")):
            return None
        return route_iob_resource_endpoint(
            tile_name,
            str(endpoint.get("local_wire") or ""),
            db,
            previous_node,
            placed_site,
        )
    return vivado_node_name(full_name)


def route_tile_resource_local(
    wire: dict[str, Any],
    direction: str,
    db: PrjxrayDb | None = None,
) -> str | None:
    tile_resource = (wire.get("annotation") or {}).get("tile_resource") or {}
    endpoint = tile_resource.get(direction) or {}
    full_name = str(endpoint.get("local_wire_full_name") or "")
    if not full_name:
        return None
    tile_name = full_name.split(".", 1)[0]
    if tile_name.startswith(("LIOB", "RIOB")):
        if direction == "output":
            local_wire = str(endpoint.get("local_wire") or "")
            match = re.fullmatch(r"IOI_LOGIC_OUTS(\d+)_\d+", local_wire)
            route_tile_name = str((wire.get("annotation") or {}).get("from_cb_tile") or "")
            coord = re.search(r"_X\d+Y\d+$", route_tile_name)
            if db is not None and match is not None and coord is not None:
                side = "L" if tile_name.startswith("LIOB") else "R"
                interface_name = f"IO_INT_INTERFACE_{side}{coord.group(0)}"
                if interface_name in db.tilegrid:
                    wire_prefix = "INT_INTERFACE_LOGIC_OUTS_L" if side == "L" else "INT_INTERFACE_LOGIC_OUTS"
                    return f"{interface_name}/{wire_prefix}{match.group(1)}"
            return vivado_node_name(full_name)
        return None
    return vivado_node_name(full_name)


def inferred_route_sink_ports(inst: dict[str, Any], route: list[dict[str, Any]]) -> list[str]:
    ports = route_final_sink_ports(inst, route)
    if ports:
        return ports
    route_nets = route_net_names(inst, route)
    out: list[str] = []
    for conn in inst.get("connections", []):
        conn_net = str(conn.get("net", ""))
        if not conn_net:
            continue
        if not any(related_route_net(route_net, conn_net) for route_net in route_nets):
            continue
        port = str(conn.get("sink_port", ""))
        if port and port not in out:
            out.append(port)
    return out


def route_full_nodes(
    route: list[dict[str, Any]],
    db: PrjxrayDb,
    inst: dict[str, Any] | None = None,
    packed_cells: dict[str, A7PackedCell] | None = None,
    source_output_nodes: list[str] | None = None,
    blocked_nodes: set[str] | None = None,
    placed_sites: dict[str, str] | None = None,
) -> list[str]:
    raw_nodes: list[str] = []
    packed_cells = packed_cells or {}
    source_output_nodes = source_output_nodes or []
    placed_sites = placed_sites or {}
    source_output_nodes_used = False
    sink_ports = inferred_route_sink_ports(inst or {}, route)

    def append_annotation_nodes(wire: dict[str, Any]) -> bool:
        kept = False
        forced_first = ""
        if wire.get("type") == "crossbar" and str(wire.get("from_wire", "")):
            from_tile = str((wire.get("annotation") or {}).get("from_cb_tile") or "")
            if from_tile:
                forced_first = f"{from_tile}/{wire.get('from_wire')}"
        for node in wire.get("annotation", {}).get("nodes", []):
            full_name = node.get("full_name")
            vivado_name = vivado_node_name(str(full_name)) if full_name else ""
            if forced_first:
                vivado_name = forced_first
                forced_first = ""
            if vivado_name and (not raw_nodes or raw_nodes[-1] != vivado_name):
                raw_nodes.append(vivado_name)
                kept = True
        return kept

    def tail_is_reachable_from_current(tail: str) -> bool:
        if not raw_nodes:
            return True
        last = raw_nodes[-1]
        if last == tail:
            return True
        if vivado_node_tile(last) != vivado_node_tile(tail):
            return True
        return direct_pip_feature(db, last, tail) is not None or intermediate_pip_node(db, last, tail) is not None

    def append_input_resource_endpoint(wire: dict[str, Any]) -> None:
        endpoint = route_tile_resource_endpoint(
            wire,
            "input",
            db,
            raw_nodes[-1] if raw_nodes else None,
            placed_sites.get(str((inst or {}).get("name", ""))),
        )
        if endpoint and (not raw_nodes or raw_nodes[-1] != endpoint):
            raw_nodes.append(endpoint)
            for tail in ioi_output_tail_nodes(endpoint):
                if raw_nodes[-1] != tail:
                    raw_nodes.append(tail)

    def reconnect_iob_sink_tail(tail: str) -> bool:
        if not raw_nodes:
            return False
        for index in range(len(raw_nodes) - 1, max(-1, len(raw_nodes) - 32), -1):
            expansion = short_route_expansion_to_targets(
                db,
                raw_nodes[index],
                [tail],
                max_depth=24,
                blocked_nodes=blocked_nodes,
            )
            if expansion is None:
                continue
            raw_nodes[index:] = expansion[1]
            return True
        return False

    for wire in route:
        ann = wire.get("annotation", {})
        if wire.get("type") == "tile_pin":
            output_local_added = False
            if int(wire.get("pin_dir", 0)) > 0:
                packed_output = []
                if source_output_nodes and not source_output_nodes_used:
                    packed_output = source_output_nodes
                    source_output_nodes_used = True
                else:
                    packed_output = iob_input_source_nodes(wire, db)
                if not packed_output:
                    packed_output = annotated_clb_output_nodes(wire, db)
                if packed_output:
                    for node in packed_output:
                        if not raw_nodes or raw_nodes[-1] != node:
                            raw_nodes.append(node)
                    output_local_added = True
                else:
                    endpoint = route_tile_resource_endpoint(
                        wire,
                        "output",
                        db,
                        placed_site=placed_sites.get(str((inst or {}).get("name", ""))),
                    )
                    if endpoint and (not raw_nodes or raw_nodes[-1] != endpoint):
                        raw_nodes.append(endpoint)
                    local = route_tile_resource_local(wire, "output", db)
                    if local and (not raw_nodes or raw_nodes[-1] != local):
                        raw_nodes.append(local)
                        output_local_added = True
                if output_local_added:
                    continue
            if int(wire.get("pin_dir", 0)) <= 0:
                append_annotation_nodes(wire)
                placed_site = placed_sites.get(str((inst or {}).get("name", "")))
                packed_tail = route_tile_pin_node(
                    inst or {},
                    wire,
                    db,
                    packed_cells,
                    sink_ports,
                    placed_site,
                )
                if packed_tail:
                    if raw_nodes and raw_nodes[-1] == packed_tail:
                        append_input_resource_endpoint(wire)
                    elif str((inst or {}).get("type", "")) == "OBUF" and reconnect_iob_sink_tail(packed_tail):
                        append_input_resource_endpoint(wire)
                    elif tail_is_reachable_from_current(packed_tail):
                        raw_nodes.append(packed_tail)
                        append_input_resource_endpoint(wire)
                    continue
                if raw_nodes:
                    append_input_resource_endpoint(wire)
                    continue
            tail = route_tile_pin_node(
                inst or {},
                wire,
                db,
                packed_cells,
                sink_ports,
                placed_sites.get(str((inst or {}).get("name", ""))),
            )
            if tail:
                if not should_skip_tile_pin_tail(raw_nodes, tail):
                    raw_nodes.append(tail)
                    if int(wire.get("pin_dir", 0)) <= 0:
                        append_input_resource_endpoint(wire)
                continue
        append_annotation_nodes(wire)

    nodes: list[str] = []
    repaired_nodes = repair_clb_output_hops(
        db,
        expand_same_tile_nodes(db, raw_nodes),
        blocked_nodes,
    )
    repaired_nodes = repair_reserved_route_nodes(db, repaired_nodes, blocked_nodes)
    for node in repaired_nodes:
        if not nodes or nodes[-1] != node:
            nodes.append(node)
    return nodes


def route_nodes(
    route: list[dict[str, Any]],
    db: PrjxrayDb,
    inst: dict[str, Any] | None = None,
    packed_cells: dict[str, A7PackedCell] | None = None,
    source_output_nodes: list[str] | None = None,
    blocked_nodes: set[str] | None = None,
    placed_sites: dict[str, str] | None = None,
) -> list[str]:
    nodes: list[str] = []
    full_nodes = canonical_fixed_route_nodes(
        route_full_nodes(route, db, inst, packed_cells, source_output_nodes, blocked_nodes, placed_sites),
        db,
    )
    for node in full_nodes:
        if not nodes or nodes[-1] != node:
            nodes.append(node)
    return nodes


def fixed_route_direct_alias(db: PrjxrayDb, src_node: str, dst_node: str) -> str | None:
    if vivado_node_tile(src_node) != vivado_node_tile(dst_node):
        return None
    src_wire = vivado_node_wire(src_node)
    dst_wire = vivado_node_wire(dst_node)
    if "LOGIC_OUTS" not in src_wire:
        return None
    match = re.fullmatch(r"(FAN|BYP)(\d+)", dst_wire)
    if match is None:
        return None

    tile_name = vivado_node_tile(dst_node)
    info = db.tilegrid.get(tile_name)
    if info is None:
        return None
    prefix = f"{match.group(1)}_ALT"
    candidates = [
        wire
        for tile_type, wire in db.pip_sources_by_type_dst
        if tile_type == info.type and wire.startswith(prefix)
    ]
    for wire in sorted(candidates):
        candidate = f"{tile_name}/{wire}"
        if direct_pip_feature(db, src_node, candidate) is not None:
            return candidate
    return None


def canonical_fixed_route_nodes(full_nodes: list[str], db: PrjxrayDb) -> list[str]:
    physical_nodes: list[str] = []
    previous_full = ""
    for node in full_nodes:
        # Consecutive tileconn names identify one physical routing node. Keep
        # its first spelling and discard every following alias in the chain.
        if previous_full and is_direct_tileconn_edge(db, previous_full, node):
            previous_full = node
            continue
        physical_nodes.append(node)
        previous_full = node

    nodes: list[str] = []
    for index, node in enumerate(physical_nodes):
        # Vivado FIXED_ROUTE uses canonical route nodes. END wires are aliases
        # of a completed hop and are not accepted as explicit downhill nodes.
        if is_intermediate_end_node(node):
            continue
        if (
            nodes
            and "LOGIC_OUTS" in vivado_node_wire(nodes[-1])
            and not vivado_node_tile(nodes[-1]).startswith("INT_")
            and vivado_node_tile(node).startswith("INT_")
            and "LOGIC_OUTS" in vivado_node_wire(node)
        ):
            continue
        # A tile-connection target can be a second name for the same physical
        # node. Vivado expects the source spelling before the following PIP.
        if (
            nodes
            and index + 1 < len(physical_nodes)
            and is_direct_tileconn_edge(db, nodes[-1], node)
            and direct_pip_feature(db, node, physical_nodes[index + 1]) is not None
        ):
            continue
        if nodes:
            alias = fixed_route_direct_alias(db, nodes[-1], node)
            if alias:
                node = alias
        if nodes and index + 1 < len(physical_nodes) and is_fixed_route_interior_alias(node):
            continue
        if not nodes or nodes[-1] != node:
            nodes.append(node)
    return nodes


def route_pips(
    route: list[dict[str, Any]],
    db: PrjxrayDb,
    inst: dict[str, Any] | None = None,
    packed_cells: dict[str, A7PackedCell] | None = None,
    source_output_nodes: list[str] | None = None,
    full_nodes_override: list[str] | None = None,
) -> list[str]:
    full_nodes = full_nodes_override
    if full_nodes is None:
        full_nodes = route_full_nodes(route, db, inst, packed_cells, source_output_nodes)
    return physical_route_pips(db, full_nodes)


def vivado_pip_name(feature: str, db: PrjxrayDb) -> str | None:
    tile_name, _, tail = feature.partition(".")
    src, sep, dst = tail.partition(".")
    if not tile_name or not sep or not src or not dst:
        return None
    tile = db.tilegrid.get(tile_name)
    tile_type = tile.type if tile else tile_name.split("_X", 1)[0]
    tile_spec = load_tile_type_spec(db, tile_type)
    if tile_spec is not None:
        prefix = f"{tile_type}."
        for pip_name in tile_spec.get("pips", {}):
            if not pip_name.startswith(prefix):
                continue
            body = pip_name[len(prefix):]
            match = re.fullmatch(r"(.+?)(<<->>|->>)(.+)", body)
            if match is not None:
                left, arrow, right = match.groups()
                if (left == dst and right == src) or (
                    arrow == "<<->>" and left == src and right == dst
                ):
                    return f"{tile_name}/{tile_type}.{body}"
            match = re.fullmatch(r"(.+?)->(.+)", body)
            if match is not None and match.groups() == (dst, src):
                return f"{tile_name}/{tile_type}.{body}"

    # Older databases may omit a tile JSON while still providing segbits.
    return f"{tile_name}/{tile_type}.{dst}->{src}"


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
        if not placement.constrain:
            continue
        placement_rows.append(",".join([
            placement.inst.name,
            placement.inst.cell_type,
            placement.site,
            export_bel_name(db, placement),
            placement.site,
        ]))

    route_rows: list[str] = []
    for route in routes:
        net_name = route.net_name
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


def is_passthrough_inst(inst: dict[str, Any] | None) -> bool:
    if inst is None:
        return False
    attrs = inst.get("attrs", {})
    return isinstance(attrs, dict) and bool(attrs.get("scalepnr_passthrough"))


def passthrough_driver_connection(inst: dict[str, Any], prefer_void: bool = True) -> dict[str, Any] | None:
    connections = [
        conn
        for conn in inst.get("connections", [])
        if str(conn.get("driver_inst", "")) and str(conn.get("driver_port", ""))
    ]
    if not connections:
        return None
    if prefer_void:
        for conn in connections:
            if ".void" in str(conn.get("net", "")):
                return conn
    return connections[0]


def resolve_passthrough_pin(
    pin: tuple[str, str],
    inst_by_name: dict[str, dict[str, Any]],
) -> tuple[str, str] | None:
    cell, _port = pin
    current = inst_by_name.get(cell)
    seen: set[str] = set()
    while is_passthrough_inst(current):
        name = str(current.get("name", ""))
        if name in seen:
            return None
        seen.add(name)
        conn = passthrough_driver_connection(current)
        if conn is None:
            return None
        driver_name = str(conn.get("driver_inst", ""))
        driver_port = str(conn.get("driver_port", "O"))
        driver = inst_by_name.get(driver_name)
        if not is_passthrough_inst(driver):
            return (driver_name, driver_port)
        current = driver
    return pin


def replace_passthrough_pin_candidates(
    pins: list[tuple[str, str]],
    inst_by_name: dict[str, dict[str, Any]],
) -> list[tuple[str, str]]:
    out: list[tuple[str, str]] = []
    for pin in pins:
        replacement = resolve_passthrough_pin(pin, inst_by_name)
        if replacement is None:
            replacement = pin
        if replacement not in out:
            out.append(replacement)
    return out


def resolve_passthrough_source_replacement(
    route: list[dict[str, Any]],
    source_pin: tuple[str, str] | None,
    inst_by_name: dict[str, dict[str, Any]],
    db: PrjxrayDb,
    packed_cells: dict[str, A7PackedCell],
) -> PassthroughReplacement | None:
    if source_pin is None:
        return None
    source_inst = inst_by_name.get(source_pin[0])
    if not is_passthrough_inst(source_inst):
        return None
    real_pin = resolve_passthrough_pin(source_pin, inst_by_name)
    if real_pin is None or real_pin == source_pin:
        return None
    real_inst = inst_by_name.get(real_pin[0])
    first_source = route_first_source_pin(route)
    if real_inst is None or first_source is None:
        return PassthroughReplacement(real_pin, [])
    # Generated passthrough cells do not exist in EDIF. Reconstruct the endpoint
    # from the real driver's packed identity before considering old annotations.
    output_nodes = packed_clb_output_nodes(real_inst, first_source, db, packed_cells)
    if not output_nodes:
        output_nodes = annotated_clb_output_nodes(first_source, db)
    return PassthroughReplacement(real_pin, output_nodes)


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
        if route.source_pin is not None:
            return [f"source:{route.source_pin}"]
        net_key = route.net_name
        for path in route.full_paths:
            if path:
                return [f"net:{net_key} root:{path[0]}"]
        for path in route.paths:
            if path:
                return [f"net:{net_key} root:{path[0]}"]
        return [f"net:{net_key}"]

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
        merged.append(RouteExport(group[0].net_name, net_candidates, pin_candidates, group[0].source_pin, paths, full_paths, pips))
    return merged


def packed_site_internal_branch(
    route: list[dict[str, Any]],
    source_pin: tuple[str, str],
    sink_name: str,
    placed_sites: dict[str, str],
    inst_by_name: dict[str, dict[str, Any]] | None = None,
    packed_cells: dict[str, A7PackedCell] | None = None,
) -> bool:
    source_site = placed_sites.get(source_pin[0])
    sink_site = placed_sites.get(sink_name)
    same_site = (
        bool(route)
        and bool(source_site)
        and source_site == sink_site
    )
    if not same_site:
        return False

    inst_by_name = inst_by_name or {}
    packed_cells = packed_cells or {}
    source_inst = inst_by_name.get(source_pin[0], {})
    sink_inst = inst_by_name.get(sink_name, {})
    source_packed = packed_cells.get(source_pin[0])
    sink_packed = packed_cells.get(sink_name)
    if source_packed is None or sink_packed is None:
        return False

    # Only a lane-aligned LUT output has a dedicated path to the matching FF.
    # Other same-site directions still require the site's external route wires.
    return (
        str(source_inst.get("type", "")).startswith("LUT")
        and str(sink_inst.get("type", "")).startswith("FD")
        and source_packed.placement.site_index == sink_packed.placement.site_index
        and source_packed.placement.bel_index % 4 == sink_packed.placement.bel_index % 4
    )


def packed_lut5_static_routes(
    state: dict[str, Any],
    db: PrjxrayDb,
    packed_cells: dict[str, A7PackedCell],
) -> tuple[list[RouteExport], set[str]]:
    """Build and reserve static slice routes required by packed O5 outputs."""
    inst_by_name = {str(inst.get("name", "")): inst for inst in state.get("insts", [])}
    constant = next(
        (inst for inst in placed_insts(state) if is_scalepnr_constant(inst)),
        None,
    )
    source_pin = (constant.name, "O") if constant is not None else None
    routes: list[RouteExport] = []
    reserved: set[str] = set()
    handled_sites: set[tuple[str, str, int]] = set()

    for name, packed in packed_cells.items():
        if packed.lut_bel_size != 5:
            continue
        inst = inst_by_name.get(name)
        if inst is None or not str(inst.get("type", "")).startswith("LUT"):
            continue
        route_tile = str((inst.get("annotation") or {}).get("cb_tile") or "")
        key = (route_tile, packed.tile_name, packed.placement.site_index)
        if not route_tile or key in handled_sites:
            continue
        handled_sites.add(key)

        resource_tile = db.tilegrid.get(packed.tile_name)
        if resource_tile is None:
            raise ValueError(f"packed LUT5 references unknown tile {packed.tile_name}")
        tile_spec = load_tile_type_spec(db, resource_tile.type)
        sites = tile_spec.get("sites", []) if tile_spec is not None else []
        if packed.placement.site_index >= len(sites):
            raise ValueError(f"packed LUT5 site index is outside {packed.tile_name}")
        site_clk = str(
            (sites[packed.placement.site_index].get("site_pins", {}).get("CLK") or {}).get("wire") or ""
        )
        if not site_clk:
            raise ValueError(f"packed LUT5 site in {packed.tile_name} has no CLK endpoint")

        start = f"{route_tile}/VCC_WIRE"
        target = f"{route_tile}/CLK1"
        expansion = short_route_expansion(db, start, target, max_depth=8)
        if expansion is None:
            raise ValueError(f"cannot resolve packed LUT5 static route {start} -> {target}")
        full_nodes = [*expansion, f"{packed.tile_name}/{site_clk}"]
        fixed_nodes = canonical_fixed_route_nodes(full_nodes, db)
        if len(fixed_nodes) < 2:
            raise ValueError(f"packed LUT5 static route is incomplete for {packed.tile_name}")
        reserved.update(fixed_nodes)
        net_name = f"$scalepnr_packed_static${packed.tile_name}.{packed.placement.site_index}"
        routes.append(RouteExport(
            net_name,
            ["VCC_NET"],
            [source_pin] if source_pin is not None else [],
            source_pin,
            [fixed_nodes],
            [full_nodes],
            physical_route_pips(db, full_nodes),
        ))
    return routes, reserved


def collect_routes(
    state: dict[str, Any],
    db: PrjxrayDb,
    packed_cells: dict[str, A7PackedCell] | None = None,
    placed_sites: dict[str, str] | None = None,
    reserved_nodes: set[str] | None = None,
) -> list[RouteExport]:
    routes: list[RouteExport] = []
    packed_cells = packed_cells or {}
    placed_sites = placed_sites or {}
    reserved_nodes = reserved_nodes or set()
    inst_by_name = {str(inst.get("name", "")): inst for inst in state.get("insts", [])}

    # Version 2 stores one authoritative tree per physical driver endpoint.
    # Branch wire lists are retained inside that tree for exact endpoint repair.
    physical_trees = [
        tree for tree in state.get("route_trees", [])
        if isinstance(tree, dict) and isinstance(tree.get("branches"), list)
    ]
    if physical_trees:
        original_owners_by_node: dict[str, set[int]] = {}
        for tree_index, tree in enumerate(physical_trees):
            for branch in tree.get("branches", []):
                route = branch.get("wires", [])
                if not route:
                    continue
                branch_sink = branch.get("sink", {})
                sink_name = str(branch_sink.get("inst", ""))
                owner_name = str(branch.get("owner", ""))
                owner = inst_by_name.get(sink_name) or inst_by_name.get(owner_name)
                if owner is None:
                    continue
                for node in route_nodes(route, db, owner, packed_cells, placed_sites=placed_sites):
                    original_owners_by_node.setdefault(node, set()).add(tree_index)

        generated_owner_by_node: dict[str, int] = {}
        for tree_index, tree in enumerate(physical_trees):
            source = tree.get("source", {})
            source_pin = (str(source.get("inst", "")), str(source.get("port", "")))
            if not source_pin[0] or not source_pin[1]:
                raise ValueError(f"physical route tree {tree.get('id', tree.get('net', ''))!r} has no source endpoint")

            aliases: list[str] = []
            for candidate in [tree.get("net", ""), *tree.get("aliases", [])]:
                name = str(candidate)
                if name and name not in aliases:
                    aliases.append(name)
            net_candidates: list[str] = []
            for alias in aliases:
                for candidate in vivado_net_candidates(alias):
                    if candidate not in net_candidates:
                        net_candidates.append(candidate)

            pin_candidates = replace_passthrough_pin_candidates([source_pin], inst_by_name)
            exported_source = pin_candidates[0] if pin_candidates else source_pin
            paths: list[list[str]] = []
            full_paths: list[list[str]] = []
            pips: list[str] = []
            blocked_nodes = {
                node
                for node, owners in original_owners_by_node.items()
                if any(owner != tree_index for owner in owners)
            }
            blocked_nodes.update(reserved_nodes)
            blocked_nodes.update(
                node
                for node, owner in generated_owner_by_node.items()
                if owner != tree_index
            )
            source_output_nodes: list[str] = []
            source_inst = inst_by_name.get(exported_source[0])
            source_is_passthrough = is_passthrough_inst(inst_by_name.get(source_pin[0]))
            for source_branch in tree.get("branches", []):
                source_route = source_branch.get("wires", [])
                if not source_route:
                    continue
                if source_is_passthrough:
                    replacement = resolve_passthrough_source_replacement(
                        source_route,
                        source_pin,
                        inst_by_name,
                        db,
                        packed_cells,
                    )
                    if replacement is not None:
                        exported_source = replacement.pin
                        source_inst = inst_by_name.get(exported_source[0])
                        source_output_nodes = replacement.output_nodes
                        if replacement.pin not in pin_candidates:
                            pin_candidates.insert(0, replacement.pin)
                        break
                elif source_inst is not None:
                    first_source = route_first_source_pin(source_route)
                    if first_source is not None:
                        source_output_nodes = iob_input_source_nodes(
                            first_source,
                            db,
                            placed_sites.get(exported_source[0]),
                        )
                        if not source_output_nodes:
                            source_output_nodes = packed_clb_output_nodes(
                                source_inst,
                                first_source,
                                db,
                                packed_cells,
                            )
                        if source_output_nodes:
                            break

            for branch in tree.get("branches", []):
                route = branch.get("wires", [])
                if not route:
                    continue
                owner_name = str(branch.get("owner", ""))
                branch_sink = branch.get("sink", {})
                sink_name = str(branch_sink.get("inst", ""))
                sink_port = str(branch_sink.get("port", ""))
                # The route-storage owner may be a generated passthrough cell.
                # Endpoint reconstruction must use the actual branch sink.
                owner = inst_by_name.get(sink_name) or inst_by_name.get(owner_name)
                if owner is None:
                    raise ValueError(
                        f"physical route tree {tree.get('id', tree.get('net', ''))!r} "
                        f"references unknown branch owner {owner_name!r}"
                    )
                if sink_name and sink_port and not is_passthrough_inst(inst_by_name.get(sink_name)):
                    sink_pin = (sink_name, sink_port)
                    if sink_pin not in pin_candidates:
                        pin_candidates.append(sink_pin)

                # A packed same-site endpoint-only branch is implemented by
                # the selected BEL connectivity. Do not invent a crossbar path
                # between its endpoint annotations; actual routed fragments in
                # the same site remain part of the fixed route tree.
                if packed_site_internal_branch(
                    route,
                    exported_source,
                    sink_name,
                    placed_sites,
                    inst_by_name,
                    packed_cells,
                ):
                    continue

                full_nodes = route_full_nodes(
                    route,
                    db,
                    owner,
                    packed_cells,
                    source_output_nodes,
                    blocked_nodes,
                    placed_sites,
                )
                fixed_nodes = canonical_fixed_route_nodes(full_nodes, db)
                if fixed_nodes and fixed_nodes not in paths:
                    paths.append(fixed_nodes)
                if full_nodes and full_nodes not in full_paths:
                    full_paths.append(full_nodes)
                for pip in route_pips(
                    route,
                    db,
                    owner,
                    packed_cells,
                    source_output_nodes,
                    full_nodes,
                ):
                    if pip not in pips:
                        pips.append(pip)

                for alias in (branch.get("logical_net", ""), branch.get("route_name", "")):
                    alias = str(alias)
                    if alias and alias not in aliases:
                        aliases.append(alias)
                        for candidate in vivado_net_candidates(alias):
                            if candidate not in net_candidates:
                                net_candidates.append(candidate)

            if paths:
                for path in paths:
                    for node in path:
                        generated_owner_by_node.setdefault(node, tree_index)
                routes.append(RouteExport(
                    str(tree.get("id") or tree.get("net") or aliases[0]),
                    net_candidates,
                    pin_candidates,
                    exported_source,
                    paths,
                    full_paths,
                    pips,
                ))
        return routes

    # Version 1 fallback: reconstruct trees from independent instance routes.
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
            pin_candidates = matching_route_pins_for_nets(inst, pin_net_names)
            original_source_pin = pin_candidates[0] if pin_candidates else None
            replacement = resolve_passthrough_source_replacement(route, original_source_pin, inst_by_name, db, packed_cells)
            pin_candidates = replace_passthrough_pin_candidates(pin_candidates, inst_by_name)
            if replacement is not None and replacement.pin not in pin_candidates:
                pin_candidates.insert(0, replacement.pin)
            source_pin = pin_candidates[0] if pin_candidates else None
            source_output_nodes = replacement.output_nodes if replacement is not None else None
            fixed_nodes = route_nodes(
                route,
                db,
                inst,
                packed_cells,
                source_output_nodes,
                placed_sites=placed_sites,
            )
            full_nodes = route_full_nodes(
                route,
                db,
                inst,
                packed_cells,
                source_output_nodes,
                placed_sites=placed_sites,
            )
            routes.append(RouteExport(
                net_name,
                net_candidates,
                pin_candidates,
                source_pin,
                [fixed_nodes],
                [full_nodes],
                route_pips(route, db, inst, packed_cells, source_output_nodes),
            ))
    return merge_route_exports(routes)


def filter_site_internal_routes(
    routes: list[RouteExport],
    placements: list[VivadoPlacement],
) -> tuple[list[RouteExport], list[str]]:
    site_by_cell = {
        placement.inst.name: placement.site
        for placement in placements
        if placement.site
    }
    filtered: list[RouteExport] = []
    warnings: list[str] = []
    internal = 0

    for route in routes:
        pin_sites = {
            site_by_cell[cell_name]
            for cell_name, _ in route.pin_candidates
            if cell_name in site_by_cell
        }
        if len(pin_sites) == 1 and len(route.pin_candidates) >= 2:
            internal += 1
            continue
        filtered.append(route)

    if internal:
        warnings.append(f"represent {internal} site-internal nets through packed placement instead of directed routing")
    return filtered, warnings


def filter_conflicting_route_paths(routes: list[RouteExport]) -> tuple[list[RouteExport], list[str]]:
    owner_by_node: dict[str, str] = {}
    filtered: list[RouteExport] = []
    warnings: list[str] = []
    dropped_routes = 0

    for route in routes:
        reserved_nodes: list[str] = []
        for index, path in enumerate(route.paths):
            full_path = route.full_paths[index] if index < len(route.full_paths) else []
            path_nodes = list(path)
            for node in full_path:
                if node not in path_nodes:
                    path_nodes.append(node)
            for node in path_nodes:
                if node not in reserved_nodes:
                    reserved_nodes.append(node)

        conflict = next((node for node in reserved_nodes if node in owner_by_node), None)
        if conflict is not None:
            dropped_routes += 1
            warnings.append(
                f"drop fixed-route constraint for {route.net_name}: node {conflict} already owned by {owner_by_node[conflict]}"
            )
            continue

        for node in reserved_nodes:
            owner_by_node.setdefault(node, route.net_name)
        filtered.append(route)

    if dropped_routes:
        warnings.append(f"dropped {dropped_routes} fixed-route constraints that reused nodes owned by earlier routes")
    return filtered, warnings


def moved_packed_cell_names(state: dict[str, Any], packed_cells: dict[str, A7PackedCell]) -> set[str]:
    moved: set[str] = set()
    for inst in placed_insts(state):
        packed = packed_cells.get(inst.name)
        if packed is None:
            continue
        if inst.resource_tile != packed.tile_name:
            moved.add(inst.name)
    return moved


def filter_moved_endpoint_routes(routes: list[RouteExport], moved_cells: set[str]) -> tuple[list[RouteExport], list[str]]:
    if not moved_cells:
        return routes, []
    filtered: list[RouteExport] = []
    skipped = 0
    for route in routes:
        if any(cell_name in moved_cells for cell_name, _ in route.pin_candidates):
            skipped += 1
            continue
        filtered.append(route)
    warnings: list[str] = []
    if skipped:
        warnings.append(f"skip {skipped} fixed routes whose endpoint cells moved to another tile during A7 packing")
    return filtered, warnings


def packed_mux_tile_names(state: dict[str, Any], packed_cells: dict[str, A7PackedCell]) -> set[str]:
    mux_tiles: set[str] = set()
    for inst in placed_insts(state):
        if not inst.cell_type.startswith("MUXF"):
            continue
        packed = packed_cells.get(inst.name)
        if packed is not None:
            mux_tiles.add(packed.tile_name)
    return mux_tiles


def filter_mux_tile_endpoint_routes(
    routes: list[RouteExport],
    packed_cells: dict[str, A7PackedCell],
    mux_tiles: set[str],
) -> tuple[list[RouteExport], list[str]]:
    return routes, []


def mux_select_route_tiles(state: dict[str, Any], packed_cells: dict[str, A7PackedCell]) -> set[str]:
    protected: set[str] = set()
    for inst in state.get("insts", []):
        name = str(inst.get("name", ""))
        if not str(inst.get("type", "")).startswith("MUXF"):
            continue
        packed = packed_cells.get(name)
        if packed is None:
            continue
        has_external_select = any(
            str(conn.get("sink_port", "")) == "S" and not bool(conn.get("same_tile", False))
            for conn in inst.get("connections", [])
        )
        if not has_external_select:
            continue
        cb_tile = str((inst.get("annotation") or {}).get("cb_tile") or "")
        if cb_tile:
            protected.add(cb_tile)
    return protected


def filter_mux_select_tile_routes(routes: list[RouteExport], protected_tiles: set[str]) -> tuple[list[RouteExport], list[str]]:
    if not protected_tiles:
        return routes, []

    filtered: list[RouteExport] = []
    skipped = 0
    for route in routes:
        protected_local_use = False
        for path in route.paths:
            for node in path:
                if vivado_node_tile(node) not in protected_tiles:
                    continue
                wire = vivado_node_wire(node)
                if wire.startswith(("BYP", "FAN", "IMUX")) or "IMUX" in wire:
                    protected_local_use = True
                    break
            if protected_local_use:
                break
        if protected_local_use:
            skipped += 1
            continue
        filtered.append(route)

    warnings: list[str] = []
    if skipped:
        warnings.append(f"skip {skipped} fixed routes using local muxing in tiles with external mux-select endpoints")
    return filtered, warnings


def filter_passthrough_endpoint_routes(routes: list[RouteExport]) -> tuple[list[RouteExport], list[str]]:
    filtered: list[RouteExport] = []
    skipped = 0
    for route in routes:
        if any(cell_name.startswith("$scalepnr_passthrough$") for cell_name, _ in route.pin_candidates):
            skipped += 1
            continue
        filtered.append(route)
    warnings: list[str] = []
    if skipped:
        warnings.append(f"skip {skipped} fixed routes touching generated passthrough endpoint cells")
    return filtered, warnings


def expected_iob_output_node(db: PrjxrayDb, site: str) -> str | None:
    if re.fullmatch(r"IOB_X\d+Y\d+", site) is None:
        return None
    for tile in db.tilegrid.values():
        if site not in tile.sites or not tile.type.startswith(("LIOB", "RIOB")):
            continue
        match = re.search(r"_X\d+Y\d+$", tile.name)
        if match is None:
            return None
        side = "LIOI" if tile.type.startswith("LIOB") else "RIOI"
        io_tiles = sorted(name for name in db.tilegrid if name.endswith(match.group(0)) and name.startswith(side))
        if not io_tiles:
            return None
        sites = site_names(tile)
        index = 0 if len(sites) <= 1 or site == sites[-1] else 1
        return f"{io_tiles[0]}/{side}_O{index}"
    return None


def filter_iob_endpoint_mismatch_routes(
    routes: list[RouteExport],
    placements: list[VivadoPlacement],
    db: PrjxrayDb,
) -> tuple[list[RouteExport], list[str]]:
    iob_output_by_cell: dict[str, str] = {}
    for placement in placements:
        if placement.inst.cell_type not in {"IBUF", "OBUF"}:
            continue
        expected = expected_iob_output_node(db, placement.site)
        if expected:
            iob_output_by_cell[placement.inst.name] = expected

    filtered: list[RouteExport] = []
    warnings: list[str] = []
    for route in routes:
        sink_cells = [cell for cell, port in route.pin_candidates if port == "I" and cell in iob_output_by_cell]
        if not sink_cells:
            filtered.append(route)
            continue
        expected_nodes = {iob_output_by_cell[cell] for cell in sink_cells}
        actual_nodes = {
            node
            for path in route.full_paths
            for node in path
            if re.search(r"/[LR]IOI_O[01]$", node)
        }
        if actual_nodes and not actual_nodes <= expected_nodes:
            warnings.append(
                f"skip IOB fixed-route constraint for {route.net_name}: routed endpoint "
                f"{sorted(actual_nodes)} does not match placed site endpoint {sorted(expected_nodes)}"
            )
            continue
        filtered.append(route)
    return filtered, warnings


def filter_multi_branch_source_routes(routes: list[RouteExport]) -> tuple[list[RouteExport], list[str]]:
    source_counts: dict[tuple[str, str], int] = {}
    for route in routes:
        if route.source_pin is not None:
            source_counts[route.source_pin] = source_counts.get(route.source_pin, 0) + len(route.paths)

    filtered: list[RouteExport] = []
    skipped = 0
    for route in routes:
        if route.source_pin is not None and source_counts.get(route.source_pin, 0) > 1:
            skipped += 1
            continue
        filtered.append(route)

    warnings: list[str] = []
    if skipped:
        warnings.append(f"skip {skipped} fixed routes from source pins with multiple surviving branches")
    return filtered, warnings


def filter_branching_fixed_route_trees(routes: list[RouteExport]) -> tuple[list[RouteExport], list[str]]:
    filtered: list[RouteExport] = []
    skipped = 0
    for route in routes:
        if len(route.paths) > 1:
            skipped += 1
            continue
        filtered.append(route)
    warnings: list[str] = []
    if skipped:
        warnings.append(f"skip {skipped} branching fixed-route trees until route-tree export is structurally complete")
    return filtered, warnings


def filter_static_multi_sink_single_paths(routes: list[RouteExport]) -> tuple[list[RouteExport], list[str]]:
    filtered: list[RouteExport] = []
    skipped = 0
    for route in routes:
        if len(route.paths) != 1:
            filtered.append(route)
            continue
        sink_pins = [pin for pin in route.pin_candidates if pin != route.source_pin]
        if len(set(sink_pins)) > 1:
            skipped += 1
            continue
        filtered.append(route)

    warnings: list[str] = []
    if skipped:
        warnings.append(f"skip {skipped} single-path fixed routes with multiple static sink pins")
    return filtered, warnings


def fixed_route_paths_invalid_reason(paths: list[list[str]], db: PrjxrayDb) -> str | None:
    child_by_node: dict[str, set[str]] = {}

    def imux_continues_to_fabric(path: list[str], index: int) -> bool:
        for child in path[index + 1:]:
            if vivado_node_tile(child).startswith("INT_"):
                return True
        return False

    for path in paths:
        for index, node in enumerate(path):
            wire = vivado_node_wire(node)
            if index > 0 and index + 1 < len(path) and "IMUX" in wire and imux_continues_to_fabric(path, index):
                return f"interior sink terminal {node}"
        for src, dst in zip(path, path[1:]):
            if vivado_node_tile(src) == vivado_node_tile(dst):
                if direct_pip_feature(db, src, dst) is None and intermediate_pip_node(db, src, dst) is None:
                    return f"invalid same-tile edge {src} -> {dst}"
            child_by_node.setdefault(src, set()).add(dst)

    for node, children in child_by_node.items():
        if "IMUX" in vivado_node_wire(node) and any(vivado_node_tile(child).startswith("INT_") for child in children):
            return f"merged tree continues after sink terminal {node}"
    return None


def filter_invalid_fixed_route_groups(routes: list[RouteExport], db: PrjxrayDb) -> tuple[list[RouteExport], list[str]]:
    filtered: list[RouteExport] = []
    warnings: list[str] = []
    skipped = 0

    for route in routes:
        reason = fixed_route_paths_invalid_reason(route.paths, db)
        if reason is None:
            reason = fixed_route_paths_invalid_reason(route.full_paths, db)
        if reason is None:
            aggregation = aggregate_a7_route_tree([path for path in route.paths if path])
            reason = fixed_route_paths_invalid_reason(aggregation.paths, db)
        if reason is not None:
            skipped += 1
            warnings.append(f"skip fixed-route constraint for {route.net_name}: {reason}")
            continue
        filtered.append(route)

    if skipped:
        warnings.append(f"skip {skipped} fixed-route constraints with invalid route-tree structure")
    return filtered, warnings


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


def collect_lut_pin_locks(state: dict[str, Any]) -> tuple[dict[str, dict[str, str]], list[str]]:
    locks: dict[str, dict[str, str]] = {}
    warnings: list[str] = []
    inst_by_name = {
        str(inst.get("name", "")): inst
        for inst in state.get("insts", [])
        if str(inst.get("name", ""))
    }

    def record(inst_name: str, logical_pin: str, wires: list[dict[str, Any]]) -> None:
        inst = inst_by_name.get(inst_name)
        if inst is None or not str(inst.get("type", "")).startswith("LUT"):
            return
        if not re.fullmatch(r"I[0-5]", logical_pin):
            return
        for wire in reversed(wires):
            if wire.get("type") != "tile_pin" or int(wire.get("pin_dir", 0)) > 0:
                continue
            tile_resource = wire.get("annotation", {}).get("tile_resource", {})
            physical_pin = str((tile_resource.get("input") or {}).get("pin") or "")
            if not re.fullmatch(r"[A-H][1-6]", physical_pin):
                continue
            physical_pin = f"A{physical_pin[1]}"
            pin_locks = locks.setdefault(inst_name, {})
            previous = pin_locks.get(logical_pin)
            if previous is not None and previous != physical_pin:
                warnings.append(
                    f"skip conflicting LOCK_PINS entry for {inst_name}: {logical_pin} maps to both {previous} and {physical_pin}"
                )
                return
            pin_locks[logical_pin] = physical_pin
            return

    # Legacy DBs retained route vectors under each destination instance.
    for inst_name, inst in inst_by_name.items():
        for route in inst.get("routes", []):
            logical_pin = ""
            for wire in reversed(route):
                if wire.get("type") == "tile_pin" and int(wire.get("pin_dir", 0)) <= 0:
                    logical_pin = str(wire.get("port", ""))
                    break
            record(inst_name, logical_pin, route)

    # DB v2 stores authoritative branches under one physical driver tree.
    for tree in state.get("route_trees", []):
        for branch in tree.get("branches", []):
            sink = branch.get("sink", {})
            wires = branch.get("wires", [])
            if isinstance(sink, dict) and isinstance(wires, list):
                record(str(sink.get("inst", "")), str(sink.get("port", "")), wires)

    return locks, warnings


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
        f.write("# Comparison export may use non-CCIO package pins from scalepnr I/O placement.\n")
        f.write("foreach n [get_nets -hier -quiet *] {\n")
        f.write("    set net_name [get_property NAME $n]\n")
        f.write("    if {[string match {*clkbufmap*} $net_name]} {\n")
        f.write("        catch {set_property CLOCK_DEDICATED_ROUTE FALSE $n}\n")
        f.write("    }\n")
        f.write("}\n")
        f.write("place_design\n")
        f.write("scalepnr_check_placement\n")
        if args.skip_fixed_routes:
            f.write("# Diagnostic mode: keep scalepnr placement/I/O constraints, but let Vivado route freely.\n")
            f.write("# routing.tcl is still generated for comparison and is intentionally not sourced here.\n")
        else:
            f.write("source [file join $script_dir routing.tcl]\n")
        f.write("catch {report_route_status -file [file join $script_dir pre_route_status.rpt]}\n")
        f.write("if {[info exists ::env(SCALEPNR_PRE_ROUTE_ONLY)] && $::env(SCALEPNR_PRE_ROUTE_ONLY) ne \"0\"} {\n")
        f.write("    puts \"Stopping before route_design because SCALEPNR_PRE_ROUTE_ONLY is set\"\n")
        f.write("    close_project\n")
        f.write("    exit 0\n")
        f.write("}\n")
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


def write_placing_tcl(
    path: Path,
    placements: list[VivadoPlacement],
    warnings: list[str],
    lut_pin_locks: dict[str, dict[str, str]],
) -> None:
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
            if not placement.constrain:
                continue
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
                pin_locks = lut_pin_locks.get(cell_name)
                if pin_locks:
                    lock_items = [f"{logical}:{physical}" for logical, physical in sorted(pin_locks.items())]
                    f.write(f"    scalepnr_set_cell_property LOCK_PINS {tcl_braced(' '.join(lock_items))} $cell\n")
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
        if not children:
            return items
        if len(children) == 1:
            child_name, child_children = next(iter(children.items()))
            items.extend(tree_items(child_name, child_children))
        else:
            ordered_children = sorted(
                children.items(),
                # A one-node nested Tcl list is indistinguishable from a
                # scalar route node. Keep a terminal leaf on the main path.
                key=lambda item: (not _route_tree_terminal_leaf(*item), -_route_tree_size(item[1])),
            )
            trunk_name, trunk_children = ordered_children[0]
            for child_name, child_children in ordered_children[1:]:
                items.append("[list " + " ".join(tree_items(child_name, child_children)) + "]")
            items.extend(tree_items(trunk_name, trunk_children))
        return items

    root_name, root_children = next(iter(tree.items()))
    return "[list " + " ".join(tree_items(root_name, root_children)) + "]"


def _route_tree_size(children: dict[str, Any]) -> int:
    return 1 + sum(_route_tree_size(child_children) for child_children in children.values())


def _route_tree_terminal_leaf(node_name: str, children: dict[str, Any]) -> bool:
    del node_name
    return not children




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


def write_routing_tcl(path: Path, routes: list[RouteExport], db: PrjxrayDb) -> None:
    with path.open("w") as f:
        f.write("# Generated routing constraints from scalepnr design_state.db\n")
        f.write("# Routes are emitted strictly from scalepnr annotations; failures expose export/model mismatches.\n")
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
        f.write("proc scalepnr_should_skip_fixed_route_error {err} {\n")
        f.write("    return 0\n")
        f.write("}\n\n")
        f.write("proc scalepnr_report_fixed_route_owners {requested_net route_tree} {\n")
        f.write("    foreach item $route_tree {\n")
        f.write("        if {[string first {/} $item] < 0} {\n")
        f.write("            scalepnr_report_fixed_route_owners $requested_net $item\n")
        f.write("            continue\n")
        f.write("        }\n")
        f.write("        set node [get_nodes -quiet $item]\n")
        f.write("        if {[llength $node] == 0} { continue }\n")
        f.write("        foreach owner [get_nets -quiet -of_objects $node] {\n")
        f.write("            if {$owner eq $requested_net} { continue }\n")
        f.write("            if {[get_property -quiet IS_ROUTE_FIXED $owner]} {\n")
        f.write("                puts \"ERROR: fixed route conflict node=$item requested=$requested_net owner=$owner\"\n")
        f.write("            }\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("}\n\n")
        f.write("proc scalepnr_set_fixed_route {net nodes} {\n")
        f.write("    global scalepnr_fixed_route_errors\n")
        f.write("    if {[llength $nodes] < 2} {\n")
        f.write("        puts \"WARN: FIXED_ROUTE failed for $net: fixed route has fewer than two nodes\"\n")
        f.write("        incr scalepnr_fixed_route_errors\n")
        f.write("        return\n")
        f.write("    }\n")
        f.write("    if {[catch {set_property FIXED_ROUTE $nodes $net} err]} {\n")
        f.write("        puts \"WARN: FIXED_ROUTE failed for $net: $err\"\n")
        f.write("        puts \"WARN: requested route was $nodes\"\n")
        f.write("        scalepnr_report_fixed_route_owners $net $nodes\n")
        f.write("        incr scalepnr_fixed_route_errors\n")
        f.write("    }\n")
        f.write("}\n\n")
        f.write("proc scalepnr_set_fixed_route_tree {net fixed_route} {\n")
        f.write("    global scalepnr_fixed_route_errors\n")
        f.write("    if {[catch {set_property ROUTE $fixed_route $net} err]} {\n")
        f.write("        puts \"WARN: FIXED_ROUTE tree failed for $net: $err\"\n")
        f.write("        puts \"WARN: requested route tree was $fixed_route\"\n")
        f.write("        scalepnr_report_fixed_route_owners $net $fixed_route\n")
        f.write("        incr scalepnr_fixed_route_errors\n")
        f.write("    } elseif {[catch {set_property IS_ROUTE_FIXED true $net} err]} {\n")
        f.write("        puts \"WARN: FIXED_ROUTE tree failed for $net: $err\"\n")
        f.write("        puts \"WARN: requested route tree was $fixed_route\"\n")
        f.write("        incr scalepnr_fixed_route_errors\n")
        f.write("    }\n")
        f.write("}\n\n")
        f.write("proc scalepnr_net_sink_count {net} {\n")
        f.write("    set count 0\n")
        f.write("    foreach pin [get_pins -quiet -of_objects $net -filter {DIRECTION == IN}] {\n")
        f.write("        incr count\n")
        f.write("    }\n")
        f.write("    foreach port [get_ports -quiet -of_objects $net -filter {DIRECTION == OUT}] {\n")
        f.write("        incr count\n")
        f.write("    }\n")
        f.write("    return $count\n")
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
                f.write(f"set net [scalepnr_get_net {tcl_braced(route.net_name)} [list {tcl_list(route.net_candidates)}] [list {tcl_pair_list(route.pin_candidates)}]]\n")
                f.write("if {[llength $net]} {\n")
                f.write("    puts \"WARN: no route nodes exported for $net\"\n")
                f.write("}\n\n")
                continue
            if aggregation.stitched_roots:
                f.write(f"# aggregated_route_roots {aggregation.stitched_roots} root={aggregation.root or ''}\n")
            for disconnected_root in aggregation.disconnected_roots[:8]:
                f.write(f"# disconnected_route_root {disconnected_root}\n")
            f.write(f"set net [scalepnr_get_net {tcl_braced(route.net_name)} [list {tcl_list(route.net_candidates)}] [list {tcl_pair_list(route.pin_candidates)}]]\n")
            f.write("if {[llength $net]} {\n")
            if len(paths) == 1:
                if len(paths[0]) < 2:
                    f.write("    puts \"WARN: scalepnr route has fewer than two nodes for $net\"\n")
                else:
                    f.write(f"    scalepnr_set_fixed_route $net [list {tcl_list(paths[0])}]\n")
            else:
                tree = route_tree_expression(paths)
                if tree is None:
                    f.write("    puts \"WARN: scalepnr branch route has disconnected roots for $net\"\n")
                    f.write("    incr scalepnr_fixed_route_errors\n")
                else:
                    f.write(f"    scalepnr_set_fixed_route_tree $net {tree}\n")
            f.write("}\n\n")
        f.write("if {$scalepnr_missing_nets != 0} {\n")
        f.write("    error \"missing $scalepnr_missing_nets scalepnr route nets\"\n")
        f.write("}\n")
        f.write("if {$scalepnr_fixed_route_errors != 0} {\n")
        f.write("    error \"failed to apply $scalepnr_fixed_route_errors scalepnr fixed route constraints\"\n")
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
    lut_pin_locks, lut_pin_lock_warnings = collect_lut_pin_locks(state)
    package_pin_sites = load_package_pin_sites(args.db_dir)
    placements, placement_warnings, packed_cells = collect_placements(state, db, io_assignments, package_pin_sites)
    placement_warnings.extend(lut_pin_lock_warnings)
    placed_sites = {placement.inst.name: placement.site for placement in placements}
    static_routes, static_reserved_nodes = packed_lut5_static_routes(state, db, packed_cells)
    routes = [
        *static_routes,
        *collect_routes(state, db, packed_cells, placed_sites, static_reserved_nodes),
    ]
    placement_warnings.append("route skip filters disabled; exporting every route with nodes for Vivado diagnostics")

    write_project_tcl(args.output_dir / "create_project.tcl", args, top)
    write_io_tcl(args.output_dir / "io.tcl", io_assignments)
    write_placing_tcl(args.output_dir / "placing.tcl", placements, placement_warnings, lut_pin_locks)
    write_routing_tcl(args.output_dir / "routing.tcl", routes, db)
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
