#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable


BEL_LETTERS = "ABCD"
CLB_TYPES = {"CLBLL_L", "CLBLL_R", "CLBLM_L", "CLBLM_R"}
CLB_CELL_TYPES = ("FD", "LUT", "CARRY", "MUX")


@dataclass(frozen=True)
class TileInfo:
    name: str
    type: str
    grid_x: int
    grid_y: int
    sites: dict[str, str]


@dataclass
class PlacedInst:
    name: str
    cell_type: str
    pos: int
    cb_tile: str
    resource_tile: str
    grid_coord: tuple[int, int]
    raw: dict[str, Any]


@dataclass
class TileState:
    clb_tile: TileInfo
    insts: list[PlacedInst] = field(default_factory=list)
    outgoing_by_driver: dict[str, list[dict[str, Any]]] = field(default_factory=dict)


@dataclass(frozen=True)
class PackedPlacement:
    site_index: int
    bel_index: int
    pos: int
    reason: str


@dataclass
class RouteGroup:
    name: str
    features: list[str] = field(default_factory=list)
    fragments: list[str] = field(default_factory=list)


@dataclass
class FasmOutput:
    features: set[str] = field(default_factory=set)
    warnings: list[str] = field(default_factory=list)
    placement_comments: list[str] = field(default_factory=list)

    def add(self, feature: str, db: PrjxrayDb, reason: str) -> str | None:
        if not feature:
            return None
        if db.has_feature(feature):
            self.features.add(feature)
            return feature
        elif db.has_pseudo_feature(feature):
            return None
        else:
            self.warnings.append(f"skip unknown feature for {reason}: {feature}")
            return None

    def warn(self, message: str) -> None:
        self.warnings.append(message)


class PrjxrayDb:
    def __init__(self, db_dir: Path) -> None:
        self.db_dir = db_dir
        self.tilegrid = self._load_tilegrid(db_dir / "tilegrid.json")
        self.by_grid = {(t.grid_x, t.grid_y): t for t in self.tilegrid.values()}
        self.features = self._load_features(db_dir)
        self.pseudo_features = self._load_pseudo_features(db_dir)
        self.pseudo_suffixes = {feature.partition(".")[2] for feature in self.pseudo_features}
        self.pip_sources_by_type_dst = self._build_pip_source_index()

    @staticmethod
    def _load_tilegrid(path: Path) -> dict[str, TileInfo]:
        with path.open() as f:
            raw = json.load(f)
        return {
            name: TileInfo(
                name=name,
                type=value["type"],
                grid_x=int(value["grid_x"]),
                grid_y=int(value["grid_y"]),
                sites=dict(value.get("sites", {})),
            )
            for name, value in raw.items()
        }

    @staticmethod
    def _load_features(db_dir: Path) -> set[str]:
        roots = [db_dir, db_dir.parent / "prjxray-db" / "artix7"]
        features: set[str] = set()
        for root in roots:
            if not root.exists():
                continue
            for path in root.glob("segbits_*.db"):
                if "origin_info" in path.name or ".block_ram." in path.name:
                    continue
                with path.open() as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith("#"):
                            features.add(line.split()[0])
        return features

    @staticmethod
    def _load_pseudo_features(db_dir: Path) -> set[str]:
        roots = [db_dir, db_dir.parent / "prjxray-db" / "artix7"]
        features: set[str] = set()
        for root in roots:
            if not root.exists():
                continue
            for path in root.glob("ppips_*.db"):
                if "origin_info" in path.name:
                    continue
                with path.open() as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith("#"):
                            features.add(line.split()[0])
        return features

    def has_feature(self, full_feature: str) -> bool:
        tile, feature = split_feature(full_feature)
        info = self.tilegrid.get(tile)
        if info is None:
            return False
        return f"{info.type}.{feature}" in self.features

    def has_pseudo_feature(self, full_feature: str) -> bool:
        tile, feature = split_feature(full_feature)
        info = self.tilegrid.get(tile)
        if info is None:
            return False
        return f"{info.type}.{feature}" in self.pseudo_features

    def _build_pip_source_index(self) -> dict[tuple[str, str], set[str]]:
        indexed: dict[tuple[str, str], set[str]] = {}
        for feature in self.features | self.pseudo_features:
            parts = feature.split(".")
            if len(parts) != 3:
                continue
            tile_type, dst, src = parts
            indexed.setdefault((tile_type, dst), set()).add(src)
        return indexed

    def repair_route_feature(self, full_feature: str) -> str | None:
        tile, feature = split_feature(full_feature)
        info = self.tilegrid.get(tile)
        if info is None:
            return None

        parts = feature.split(".")
        if len(parts) != 2:
            return None
        dst, requested_src = parts
        sources = self.pip_sources_by_type_dst.get((info.type, dst))
        if not sources:
            return None

        requested_family = wire_family(requested_src)
        requested_length = wire_length(requested_src)

        def score(src: str) -> tuple[int, str]:
            family = wire_family(src)
            length = wire_length(src)
            if src == requested_src:
                return (0, src)
            if family == requested_family and length == requested_length:
                return (1, src)
            if family == requested_family:
                return (2, src)
            if requested_length is not None and length == requested_length and requested_family and requested_family[-1:] in family:
                return (3, src)
            if requested_length is not None and length == requested_length:
                return (4, src)
            return (5, src)

        repaired_src = min(sources, key=score)
        repaired = f"{tile}.{dst}.{repaired_src}"
        if self.has_feature(repaired) or self.has_pseudo_feature(repaired):
            return repaired
        return None

    def clb_neighbor_for_cb(self, cb_tile_name: str) -> TileInfo | None:
        cb = self.tilegrid.get(cb_tile_name)
        if cb is None:
            return None
        return self.clb_neighbor_for_grid(cb.grid_x, cb.grid_y)

    def clb_neighbor_for_grid(self, grid_x: int, grid_y: int) -> TileInfo | None:
        for dx in (-1, 1):
            tile = self.by_grid.get((grid_x + dx, grid_y))
            if tile and tile.type in CLB_TYPES:
                return tile
        return None

    def tile_at_grid(self, grid: tuple[int, int]) -> TileInfo | None:
        return self.by_grid.get(grid)


def split_feature(full_feature: str) -> tuple[str, str]:
    tile, _, feature = full_feature.partition(".")
    return tile, feature


def wire_family(wire: str) -> str:
    match = re.match(r"([A-Z]+)", wire)
    return match.group(1) if match else ""


def wire_length(wire: str) -> int | None:
    match = re.match(r"[A-Z]+([0-9]+)", wire)
    return int(match.group(1)) if match else None


def load_state(path: Path) -> dict[str, Any]:
    with path.open() as f:
        state = json.load(f)
    if state.get("format") != "scalepnr-design-state":
        raise ValueError(f"{path} is not a scalepnr design state DB")
    return state


def placed_insts(state: dict[str, Any]) -> Iterable[PlacedInst]:
    for inst in state["insts"]:
        ann = inst.get("annotation")
        if not inst.get("placed") or not ann:
            continue
        yield PlacedInst(
            name=inst["name"],
            cell_type=inst.get("type", ""),
            pos=int(inst.get("pos", -1)),
            cb_tile=ann.get("cb_tile", ""),
            resource_tile=ann.get("resource_tile", ""),
            grid_coord=tuple(ann.get("grid_coord", [-1, -1])),
            raw=inst,
        )


def group_by_clb_tile(state: dict[str, Any], db: PrjxrayDb, out: FasmOutput) -> dict[str, TileState]:
    grouped: dict[str, TileState] = {}
    insts = list(placed_insts(state))
    outgoing_by_driver: dict[str, list[dict[str, Any]]] = {}
    for raw_inst in state["insts"]:
        for conn in raw_inst.get("connections", []):
            driver = conn.get("driver_inst")
            if driver:
                outgoing_by_driver.setdefault(str(driver), []).append(conn)

    for inst in insts:
        if inst.cell_type in {"IBUF", "OBUF", "BUFG"}:
            continue
        actual_tile = db.tilegrid.get(inst.resource_tile)
        actual_cb = db.tile_at_grid(inst.grid_coord)
        if actual_tile is not None and actual_tile.type in CLB_TYPES:
            clb_tile = actual_tile
        elif actual_cb is not None and actual_cb.type in CLB_TYPES:
            clb_tile = actual_cb
        else:
            clb_tile = db.clb_neighbor_for_grid(*inst.grid_coord)
        if clb_tile is None:
            where = actual_cb.name if actual_cb else inst.cb_tile
            out.warn(f"skip cell without CLB neighbor: {inst.name} at {where}")
            continue
        tile_state = grouped.setdefault(
            clb_tile.name,
            TileState(clb_tile=clb_tile, outgoing_by_driver=outgoing_by_driver),
        )
        tile_state.insts.append(inst)
    return grouped


def site_for_pos(tile: TileInfo, pos: int) -> tuple[str, int]:
    site_index = 1 if pos >= 128 else 0
    site_types = [site_type for _, site_type in sorted(tile.sites.items())]
    if not site_types:
        site_type = "SLICEL"
    elif site_index < len(site_types):
        site_type = site_types[site_index]
    else:
        site_type = site_types[-1]
    return f"{site_type}_X{site_index}", site_index


def site_for_index(tile: TileInfo, site_index: int) -> str:
    site_types = [site_type for _, site_type in sorted(tile.sites.items())]
    if not site_types:
        site_type = "SLICEL"
    elif site_index < len(site_types):
        site_type = site_types[site_index]
    else:
        site_type = site_types[-1]
    return f"{site_type}_X{site_index}"


def slice_count(tile: TileInfo) -> int:
    return max(1, len(tile.sites))


def bel_for_pos(pos: int) -> str:
    local_pos = pos % 128
    return BEL_LETTERS[(local_pos // 4) % 4]


def bel_for_packed(packed: PackedPlacement) -> str:
    return BEL_LETTERS[packed.bel_index]


def packed_pos(site_index: int, bel_index: int, kind: str) -> int:
    if kind == "CARRY":
        local = 2
    elif kind == "LUT":
        local = bel_index * 4 + 3
    else:
        local = bel_index * 4
    return site_index * 128 + local


def cell_kind(inst: PlacedInst) -> str:
    if inst.cell_type.startswith("FD"):
        return "FD"
    if inst.cell_type.startswith("LUT"):
        return "LUT"
    if inst.cell_type == "CARRY4":
        return "CARRY"
    return "OTHER"


def local_connections(inst: PlacedInst, tile_insts: dict[str, PlacedInst]) -> list[dict[str, Any]]:
    conns: list[dict[str, Any]] = []
    for conn in inst.raw.get("connections", []):
        if conn.get("same_tile") and conn.get("driver_inst") in tile_insts:
            conns.append(conn)
    return conns


def inferred_carry_bit(port: str) -> int | None:
    match = re.search(r"\[(\d+)\]", port)
    if match:
        bit = int(match.group(1))
        return bit if 0 <= bit < 4 else None
    match = re.search(r"([0-3])$", port)
    if match:
        return int(match.group(1))
    return None


def carry_s_nets(inst: PlacedInst) -> dict[int, str]:
    nets: dict[int, str] = {}
    for conn in inst.raw.get("connections", []):
        bit = inferred_carry_bit(str(conn.get("sink_port", "")))
        if bit is None:
            continue
        if not str(conn.get("sink_port", "")).startswith("S"):
            continue
        net = str(conn.get("net", ""))
        if net:
            nets[bit] = net
    return nets


def lut_output_nets(inst: PlacedInst, outgoing_by_driver: dict[str, list[dict[str, Any]]]) -> set[str]:
    nets: set[str] = set()
    for conn in outgoing_by_driver.get(inst.name, []):
        if str(conn.get("driver_port", "")) != "O":
            continue
        net = str(conn.get("net", ""))
        if net:
            nets.add(net)
    return nets


def placement_compatible(inst: PlacedInst, site_index: int, bel_index: int,
                         placed: dict[str, PackedPlacement], tile_insts: dict[str, PlacedInst]) -> bool:
    kind = cell_kind(inst)
    if kind == "CARRY":
        return bel_index == 0

    for conn in local_connections(inst, tile_insts):
        driver_name = conn["driver_inst"]
        driver = tile_insts[driver_name]
        driver_place = placed.get(driver_name)
        if driver_place is None:
            continue
        driver_kind = cell_kind(driver)
        sink_port = str(conn.get("sink_port", ""))
        driver_port = str(conn.get("driver_port", ""))

        if driver_kind == "LUT" and kind == "FD":
            if driver_place.site_index != site_index or driver_place.bel_index != bel_index:
                return False
        if driver_kind == "CARRY" and kind == "FD":
            bit = inferred_carry_bit(driver_port)
            expected_bel = bit if bit is not None else bel_index
            if driver_place.site_index != site_index or bel_index != expected_bel:
                return False
        if driver_kind == "LUT" and kind == "CARRY":
            bit = inferred_carry_bit(sink_port)
            expected_bel = bit if bit is not None else driver_place.bel_index
            if driver_place.site_index != site_index or expected_bel != driver_place.bel_index:
                return False
    return True


def pack_tile(tile_state: TileState) -> dict[str, PackedPlacement]:
    insts = [inst for inst in tile_state.insts if cell_kind(inst) != "OTHER"]
    tile_insts = {inst.name: inst for inst in insts}
    sites = range(slice_count(tile_state.clb_tile))
    occupied: set[tuple[str, int, int]] = set()
    carry_s_net_by_slot: dict[tuple[int, int], str] = {}
    lut_nets_by_slot: dict[tuple[int, int], set[str]] = {}
    placed: dict[str, PackedPlacement] = {}

    def candidates(inst: PlacedInst) -> list[tuple[int, int]]:
        kind = cell_kind(inst)
        original_site = 1 if inst.pos >= 128 else 0
        original_bel = (inst.pos % 128) // 4
        if kind == "CARRY":
            ordered = [(site, 0) for site in sites]
            preferred = (original_site, 0)
        else:
            ordered = [(site, bel) for site in sites for bel in range(4)]
            preferred = (original_site, original_bel)
        if preferred in ordered:
            ordered.remove(preferred)
            ordered.insert(0, preferred)
        return ordered

    def can_place(inst: PlacedInst, site: int, bel: int) -> bool:
        kind = cell_kind(inst)
        key = (kind, site, bel)
        if key in occupied:
            return False
        if kind == "CARRY":
            for bit, net in carry_s_nets(inst).items():
                slot_nets = lut_nets_by_slot.get((site, bit))
                if slot_nets is not None and net not in slot_nets:
                    return False
        if kind == "LUT":
            reserved_net = carry_s_net_by_slot.get((site, bel))
            if reserved_net is not None and reserved_net not in lut_output_nets(inst, tile_state.outgoing_by_driver):
                return False
        return placement_compatible(inst, site, bel, placed, tile_insts)

    def mark_placed(inst: PlacedInst, site: int, bel: int) -> None:
        kind = cell_kind(inst)
        if kind == "CARRY":
            for bit, net in carry_s_nets(inst).items():
                carry_s_net_by_slot[(site, bit)] = net
        if kind == "LUT":
            nets = lut_output_nets(inst, tile_state.outgoing_by_driver)
            if nets:
                lut_nets_by_slot.setdefault((site, bel), set()).update(nets)

    ordered = sorted(insts, key=lambda inst: (
        {"CARRY": 0, "LUT": 1, "FD": 2}.get(cell_kind(inst), 3),
        -len(local_connections(inst, tile_insts)),
        inst.pos,
        inst.name,
    ))

    remaining = ordered[:]
    progress = True
    while remaining and progress:
        progress = False
        next_remaining: list[PlacedInst] = []
        for inst in remaining:
            selected: tuple[int, int] | None = None
            for site, bel in candidates(inst):
                if can_place(inst, site, bel):
                    selected = (site, bel)
                    break
            if selected is None:
                next_remaining.append(inst)
                continue
            site, bel = selected
            kind = cell_kind(inst)
            occupied.add((kind, site, bel))
            mark_placed(inst, site, bel)
            placed[inst.name] = PackedPlacement(site, bel, packed_pos(site, bel, kind), "greedy")
            progress = True
        remaining = next_remaining

    if remaining:
        blocked = ", ".join(f"{inst.cell_type}:{inst.name}" for inst in remaining)
        raise SystemExit(f"cannot pack tile {tile_state.clb_tile.name}; connectivity/occupancy failed for {blocked}")

    for inst in insts:
        packed = placed[inst.name]
        inst.raw.setdefault("annotation", {})["packed_pos"] = packed.pos
        inst.raw["annotation"]["packed_site"] = site_for_index(tile_state.clb_tile, packed.site_index)
        inst.raw["annotation"]["packed_bel"] = BEL_LETTERS[packed.bel_index]

    return placed


def pack_tiles(grouped: dict[str, TileState], out: FasmOutput) -> dict[str, PackedPlacement]:
    packed: dict[str, PackedPlacement] = {}
    for tile_state in grouped.values():
        tile_packed = pack_tile(tile_state)
        for inst in tile_state.insts:
            if inst.name in tile_packed:
                place = tile_packed[inst.name]
                out.placement_comments.append(
                    f"pack {inst.cell_type} {inst.name} tile={tile_state.clb_tile.name} "
                    f"site={site_for_index(tile_state.clb_tile, place.site_index)} bel={BEL_LETTERS[place.bel_index]} pos={place.pos}"
                )
        packed.update(tile_packed)
    return packed


def emit_fdre(inst: PlacedInst, tile: TileInfo, packed: PackedPlacement, out: FasmOutput, db: PrjxrayDb) -> None:
    site = site_for_index(tile, packed.site_index)
    bel = bel_for_packed(packed)
    prefix = f"{tile.name}.{site}"
    reason = f"{inst.cell_type} {inst.name}"
    out.add(f"{prefix}.{bel}FF.ZRST", db, reason)
    out.add(f"{prefix}.{bel}FFMUX.O6", db, reason)
    out.add(f"{prefix}.NOCLKINV", db, reason)
    out.add(f"{prefix}.SRUSEDMUX", db, reason)


def emit_lut(inst: PlacedInst, tile: TileInfo, packed: PackedPlacement, out: FasmOutput, db: PrjxrayDb) -> None:
    site = site_for_index(tile, packed.site_index)
    bel = bel_for_packed(packed)
    params = inst.raw.get("params", {})
    init = params.get("INIT")
    if init is None:
        out.warn(f"skip LUT without INIT parameter: {inst.name}")
        return
    try:
        init_value = int(str(init), 0)
    except ValueError:
        out.warn(f"skip LUT with unsupported INIT value {init!r}: {inst.name}")
        return
    for bit in range(64):
        if (init_value >> bit) & 1:
            out.add(f"{tile.name}.{site}.{bel}LUT.INIT[{bit:02d}]", db, f"LUT {inst.name}")


def emit_carry4(inst: PlacedInst, tile: TileInfo, packed: PackedPlacement, out: FasmOutput, db: PrjxrayDb) -> None:
    site = site_for_index(tile, packed.site_index)
    prefix = f"{tile.name}.{site}"
    reason = f"CARRY4 {inst.name}"

    # Minimal arithmetic carry-chain site configuration. The exact DI/S input
    # mapping still comes from routed resource pins; these bits make the placed
    # CARRY4 site visible and configurable in prjxray FASM.
    for feature in (
        "PRECYINIT.C0",
        "CARRY4.ACY0",
        "CARRY4.BCY0",
        "CARRY4.CCY0",
        "CARRY4.DCY0",
    ):
        out.add(f"{prefix}.{feature}", db, reason)


def iob_y_index(inst: PlacedInst, tile: TileInfo) -> int:
    if tile.type.endswith("_SING"):
        return 0
    return 1 if inst.pos == 1 else 0


def emit_iob(inst: PlacedInst, tile: TileInfo, out: FasmOutput, db: PrjxrayDb) -> None:
    if tile.type.endswith("_SING"):
        out.warn(f"describe {inst.cell_type} on {tile.name} only as comment: {tile.type} has no segbits DB")
        return

    y = iob_y_index(inst, tile)
    prefix = f"{tile.name}.IOB_Y{y}"
    reason = f"{inst.cell_type} {inst.name}"
    if inst.cell_type == "IBUF":
        features = (
            "LVCMOS25_LVCMOS33_LVTTL.IN",
            "LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVDS_25_LVTTL_SSTL135_SSTL15_TMDS_33.IN_ONLY",
            "PULLTYPE.NONE",
            "ZIBUF_LOW_PWR",
        )
    else:
        features = (
            "LVCMOS33_LVTTL.DRIVE.I12_I8",
            "LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL_SSTL135_SSTL15.SLEW.SLOW",
            "PULLTYPE.NONE",
        )
    for feature in features:
        out.add(f"{prefix}.{feature}", db, reason)


def emit_cells(grouped: dict[str, TileState], packed: dict[str, PackedPlacement], out: FasmOutput, db: PrjxrayDb) -> None:
    for tile_state in grouped.values():
        for inst in tile_state.insts:
            placement = packed.get(inst.name)
            if placement is None:
                continue
            if inst.cell_type.startswith("FD"):
                emit_fdre(inst, tile_state.clb_tile, placement, out, db)
            elif inst.cell_type.startswith("LUT"):
                emit_lut(inst, tile_state.clb_tile, placement, out, db)
            elif inst.cell_type == "CARRY4":
                emit_carry4(inst, tile_state.clb_tile, placement, out, db)
            else:
                out.warn(f"skip unsupported placed cell {inst.cell_type}: {inst.name}")


def emit_placement(state: dict[str, Any], out: FasmOutput, db: PrjxrayDb) -> None:
    for inst in placed_insts(state):
        tile = db.tilegrid.get(inst.resource_tile)
        if tile is None:
            out.warn(f"skip placement for {inst.name}: unknown resource tile {inst.resource_tile}")
            continue
        out.placement_comments.append(
            f"place {inst.cell_type} {inst.name} tile={tile.name} tile_type={tile.type} pos={inst.pos}"
        )
        if inst.cell_type in {"IBUF", "OBUF"}:
            emit_iob(inst, tile, out, db)


def route_features(state: dict[str, Any]) -> Iterable[str]:
    for inst in state["insts"]:
        for route in inst.get("routes", []):
            for wire in route:
                ann = wire.get("annotation", {})
                for feature in ann.get("fasm_features", []):
                    yield feature


def route_name(inst: dict[str, Any], route: list[dict[str, Any]]) -> str:
    for wire in route:
        net = wire.get("net")
        if net:
            return str(net)
    return str(inst.get("name", "<unnamed-route>"))


def fasm_comment(text: str) -> str:
    return "# " + text.replace("\n", " ").replace("\r", " ")


def wire_fragment_comment(wire: dict[str, Any]) -> str:
    ann = wire.get("annotation", {})
    names: list[str] = []
    for node in ann.get("nodes", []):
        full_name = node.get("full_name")
        if full_name:
            names.append(str(full_name))
    if names:
        return " -> ".join(names)

    from_coord = wire.get("from", [])
    to_coord = wire.get("to", [])
    kind = str(wire.get("type", "wire"))
    local = wire.get("local", -1)
    jump = wire.get("jump", -1)
    joint = wire.get("joint", -1)
    return f"{kind} from={from_coord} to={to_coord} local={local} jump={jump} joint={joint}"


def accepted_route_feature(feature: str, out: FasmOutput, db: PrjxrayDb) -> str | None:
    tile, _ = split_feature(feature)
    if tile not in db.tilegrid:
        return None
    if db.has_feature(feature):
        return out.add(feature, db, "routing")
    repaired = db.repair_route_feature(feature)
    if repaired:
        return out.add(repaired, db, f"routing repaired from {feature}")
    return out.add(feature, db, "routing")


def emit_routing(state: dict[str, Any], out: FasmOutput, db: PrjxrayDb) -> list[RouteGroup]:
    route_groups: list[RouteGroup] = []
    for inst in state["insts"]:
        for route in inst.get("routes", []):
            group: list[str] = []
            fragments: list[str] = []
            seen_in_route: set[str] = set()
            for wire in route:
                fragments.append(wire_fragment_comment(wire))
                ann = wire.get("annotation", {})
                for feature in ann.get("fasm_features", []):
                    accepted = accepted_route_feature(feature, out, db)
                    if accepted and accepted not in seen_in_route:
                        group.append(accepted)
                        seen_in_route.add(accepted)
            route_groups.append(RouteGroup(route_name(inst, route), group, fragments))
    return route_groups


def write_fasm(path: Path, features: Iterable[str], route_groups: Iterable[RouteGroup], placement_comments: Iterable[str]) -> None:
    route_groups = list(route_groups)
    placement_comments = list(placement_comments)
    route_feature_set = {feature for group in route_groups for feature in group.features}
    cell_features = sorted(set(features) - route_feature_set)
    emitted: set[str] = set()
    with path.open("w") as f:
        if placement_comments:
            f.write(fasm_comment("placement"))
            f.write("\n")
            for comment in placement_comments:
                f.write(fasm_comment(comment))
                f.write("\n")
            f.write("\n")

        if cell_features:
            f.write(fasm_comment("cell and site configuration"))
            f.write("\n")
        for feature in cell_features:
            f.write(feature)
            f.write("\n")
            emitted.add(feature)

        if cell_features and route_feature_set:
            f.write("\n")

        first_route = True
        for group in route_groups:
            if not first_route:
                f.write("\n")
            first_route = False
            f.write(fasm_comment(f"route {group.name}"))
            f.write("\n")
            route_lines = [feature for feature in group.features if feature not in emitted]
            if not route_lines:
                f.write(fasm_comment("no emitted PIPs for this route"))
                f.write("\n")
            for fragment in group.fragments:
                f.write(fasm_comment(f"wire {fragment}"))
                f.write("\n")
            for feature in route_lines:
                f.write(feature)
                f.write("\n")
                emitted.add(feature)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert scalepnr design_state.db to prjxray FASM")
    parser.add_argument("input_db", type=Path)
    parser.add_argument("output_fasm", type=Path)
    parser.add_argument("--db-dir", type=Path, default=Path(__file__).with_name("db"))
    parser.add_argument("--warnings", type=Path, help="Optional file to write conversion warnings")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    state = load_state(args.input_db)
    db = PrjxrayDb(args.db_dir)
    out = FasmOutput()

    grouped = group_by_clb_tile(state, db, out)
    emit_placement(state, out, db)
    packed = pack_tiles(grouped, out)
    emit_cells(grouped, packed, out, db)
    route_groups = emit_routing(state, out, db)
    write_fasm(args.output_fasm, out.features, route_groups, out.placement_comments)

    warnings_text = "\n".join(out.warnings)
    if args.warnings:
        args.warnings.write_text(warnings_text + ("\n" if warnings_text else ""))
    elif warnings_text:
        print(warnings_text, file=sys.stderr)

    print(f"wrote {len(out.features)} FASM features to {args.output_fasm}")
    print(f"{len(out.warnings)} warnings", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
