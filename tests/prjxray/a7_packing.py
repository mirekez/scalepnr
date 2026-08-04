#!/usr/bin/env python3
from __future__ import annotations

"""Artix-7 placement legalization for Vivado comparison projects.

This module is intentionally kept outside scalepnr core.  It translates the
abstract placed CLB state into concrete Xilinx 7-series slice/BEL placements.
"""

from collections import defaultdict, deque
from dataclasses import dataclass, field
import re
from typing import Iterable

from db2fasm import (
    CLB_TYPES,
    PackedPlacement,
    PlacedInst,
    TileInfo,
    TileState,
    carry_s_nets,
    cell_kind,
    inferred_carry_bit,
    lut_output_nets,
    slice_count,
)


@dataclass(frozen=True)
class A7PackedCell:
    tile_name: str
    placement: PackedPlacement
    constrain: bool = True
    lut_bel_size: int = 6


@dataclass(frozen=True)
class A7PackResult:
    cells: dict[str, A7PackedCell]
    warnings: list[str] = field(default_factory=list)


def legalize_a7_mux_placements(
    grouped: dict[str, TileState],
    base: dict[str, A7PackedCell],
) -> A7PackResult:
    """Legalize deterministic wide-mux lanes without moving unrelated cells."""
    placed = dict(base)
    warnings: list[str] = []
    insts = [inst for tile_state in grouped.values() for inst in tile_state.insts]
    inst_by_name = {inst.name: inst for inst in insts}

    def replace(inst: PlacedInst, parent: A7PackedCell, bel: int, reason: str) -> None:
        kind = cell_kind(inst)
        if kind == "LUT":
            local = bel * 4 + 3
        elif kind == "MUX":
            local = 1 if inst.cell_type.startswith("MUXF8") or bel == 0 else 9
        else:
            return
        site = parent.placement.site_index
        placed[inst.name] = A7PackedCell(
            parent.tile_name,
            PackedPlacement(site, bel, site * 128 + local, reason),
            True,
            6,
        )

    # F8 I0 is driven by F7B and I1 by F7A in a 7-series slice.
    for mux8 in (inst for inst in insts if inst.cell_type.startswith("MUXF8")):
        parent = placed.get(mux8.name)
        if parent is None:
            continue
        for port, pair in (("I0", 1), ("I1", 0)):
            driver = _driver_by_port(mux8, inst_by_name).get(port)
            if driver is not None and driver.cell_type.startswith("MUXF7"):
                replace(driver, parent, pair, "a7-mux8-data")

    # F7 I0 is driven by the upper LUT lane and I1 by the lower LUT lane.
    for mux7 in (inst for inst in insts if inst.cell_type.startswith("MUXF7")):
        parent = placed.get(mux7.name)
        if parent is None:
            continue
        pair = parent.placement.bel_index
        for port, bel in (("I0", pair * 2 + 1), ("I1", pair * 2)):
            driver = _driver_by_port(mux7, inst_by_name).get(port)
            if driver is not None and cell_kind(driver) == "LUT":
                replace(driver, parent, bel, "a7-mux7-data")

    # A missing F7 data input is tied to ground through that lane's O6 path.
    # An unrelated LUT5 may still use the paired O5 path without conflicting.
    for mux7 in (inst for inst in insts if inst.cell_type.startswith("MUXF7")):
        parent = placed.get(mux7.name)
        if parent is None:
            continue
        pair = parent.placement.bel_index
        drivers = _driver_by_port(mux7, inst_by_name)
        for port, bel in (("I0", pair * 2 + 1), ("I1", pair * 2)):
            driver = drivers.get(port)
            if driver is not None and cell_kind(driver) == "LUT":
                continue
            occupants = [
                (name, packed)
                for name, packed in placed.items()
                if packed.tile_name == parent.tile_name
                and packed.placement.site_index == parent.placement.site_index
                and packed.placement.bel_index == bel
                and name in inst_by_name
                and cell_kind(inst_by_name[name]) == "LUT"
            ]
            for name, packed in occupants:
                occupant = inst_by_name[name]
                match = re.fullmatch(r"LUT(\d+)", occupant.cell_type)
                width = 1 if occupant.cell_type == "INV" else int(match.group(1)) if match else 6
                if width > 5:
                    warnings.append(
                        f"cannot share grounded F7 O6 lane with {occupant.cell_type}: {name}"
                    )
                    continue
                placed[name] = A7PackedCell(
                    packed.tile_name,
                    packed.placement,
                    packed.constrain,
                    5,
                )

    return A7PackResult(placed, warnings)


@dataclass(frozen=True)
class A7RouteTreeAggregation:
    paths: list[list[str]]
    root: str | None
    connected: bool
    disconnected_roots: list[str] = field(default_factory=list)
    stitched_roots: int = 0


@dataclass(frozen=True)
class MuxShape:
    root: PlacedInst
    mux7_by_pair: dict[int, PlacedInst]
    lut_by_slot: dict[int, PlacedInst]


@dataclass
class A7PackState:
    tile_by_name: dict[str, TileInfo]
    inst_by_name: dict[str, PlacedInst]
    original_tile_by_inst: dict[str, str]
    outgoing_by_driver: dict[str, list[dict]]
    placed: dict[str, A7PackedCell] = field(default_factory=dict)
    lut_used: set[tuple[str, int, int]] = field(default_factory=set)
    fd_used: set[tuple[str, int, int]] = field(default_factory=set)
    carry_used: set[tuple[str, int]] = field(default_factory=set)
    mux7_used: set[tuple[str, int, int]] = field(default_factory=set)
    mux8_used: set[tuple[str, int]] = field(default_factory=set)
    carry_s_net_by_slot: dict[tuple[str, int, int], str] = field(default_factory=dict)
    lut_nets_by_slot: dict[tuple[str, int, int], set[str]] = field(default_factory=dict)

    def snapshot(self) -> tuple:
        return (
            dict(self.placed),
            set(self.lut_used),
            set(self.fd_used),
            set(self.carry_used),
            set(self.mux7_used),
            set(self.mux8_used),
            dict(self.carry_s_net_by_slot),
            {key: set(value) for key, value in self.lut_nets_by_slot.items()},
        )

    def restore(self, snapshot: tuple) -> None:
        (
            self.placed,
            self.lut_used,
            self.fd_used,
            self.carry_used,
            self.mux7_used,
            self.mux8_used,
            self.carry_s_net_by_slot,
            self.lut_nets_by_slot,
        ) = snapshot

    def tile(self, tile_name: str) -> TileInfo:
        return self.tile_by_name[tile_name]

    def original_tile(self, inst: PlacedInst) -> str:
        return self.original_tile_by_inst[inst.name]

    def original_site(self, inst: PlacedInst, tile_name: str) -> int:
        site = 1 if inst.pos >= 128 else 0
        return min(site, slice_count(self.tile(tile_name)) - 1)

    @staticmethod
    def original_bel(inst: PlacedInst) -> int:
        return max(0, min(3, (inst.pos % 128) // 4))

    @staticmethod
    def packed_pos(site: int, bel: int, kind: str, inst: PlacedInst) -> int:
        if kind == "CARRY":
            local = 2
        elif kind == "MUX":
            local = 1 if inst.cell_type.startswith("MUXF8") or bel == 0 else 9
        elif kind == "LUT":
            local = bel * 4 + 3
        elif kind == "FD":
            local = (bel % 4) * 4 + (1 if bel >= 4 else 0)
        else:
            local = bel * 4
        return site * 128 + local

    def is_free(self, inst: PlacedInst, tile_name: str, site: int, bel: int) -> bool:
        current = self.placed.get(inst.name)
        if current is not None:
            return current.tile_name == tile_name and current.placement.site_index == site and current.placement.bel_index == bel
        if site < 0 or site >= slice_count(self.tile(tile_name)):
            return False
        kind = cell_kind(inst)
        if kind == "LUT":
            return 0 <= bel < 4 and (tile_name, site, bel) not in self.lut_used
        if kind == "FD":
            return 0 <= bel < 8 and (tile_name, site, bel) not in self.fd_used
        if kind == "CARRY":
            return bel == 0 and (tile_name, site) not in self.carry_used
        if kind == "MUX":
            if inst.cell_type.startswith("MUXF8"):
                return bel == 0 and (tile_name, site) not in self.mux8_used
            return 0 <= bel < 2 and (tile_name, site, bel) not in self.mux7_used
        return False

    def assign(self, inst: PlacedInst, tile_name: str, site: int, bel: int, reason: str, constrain: bool = True) -> bool:
        if not self.is_free(inst, tile_name, site, bel):
            return False
        if inst.name in self.placed:
            return True

        kind = cell_kind(inst)
        if kind == "LUT":
            self.lut_used.add((tile_name, site, bel))
            nets = lut_output_nets(inst, self.outgoing_by_driver)
            if nets:
                self.lut_nets_by_slot.setdefault((tile_name, site, bel), set()).update(nets)
        elif kind == "FD":
            self.fd_used.add((tile_name, site, bel))
        elif kind == "CARRY":
            self.carry_used.add((tile_name, site))
            for bit, net in carry_s_nets(inst).items():
                self.carry_s_net_by_slot[(tile_name, site, bit)] = net
        elif kind == "MUX":
            if inst.cell_type.startswith("MUXF8"):
                self.mux8_used.add((tile_name, site))
            else:
                self.mux7_used.add((tile_name, site, bel))

        self.placed[inst.name] = A7PackedCell(
            tile_name,
            PackedPlacement(site, bel, self.packed_pos(site, bel, kind, inst), reason),
            constrain,
        )
        return True


def _driver_by_port(inst: PlacedInst, inst_by_name: dict[str, PlacedInst]) -> dict[str, PlacedInst]:
    drivers: dict[str, PlacedInst] = {}
    for conn in inst.raw.get("connections", []):
        port = str(conn.get("sink_port", ""))
        driver_name = str(conn.get("driver_inst", ""))
        driver = inst_by_name.get(driver_name)
        if port and driver is not None:
            drivers[port] = driver
    return drivers


def _ordered_unique(values: Iterable[str]) -> list[str]:
    out: list[str] = []
    for value in values:
        if value and value not in out:
            out.append(value)
    return out


def _compact_route_path(path: Iterable[str]) -> list[str]:
    out: list[str] = []
    for node in path:
        if node and (not out or out[-1] != node):
            out.append(node)
    return out


def _route_wire_name(node_name: str) -> str:
    return node_name.rsplit("/", 1)[-1]


def _route_source_rank(node_name: str) -> int:
    wire = _route_wire_name(node_name)
    if "LOGIC_OUTS" in wire:
        return 0
    if "GCLK" in wire or wire.startswith("CLK"):
        return 1
    if "BYP" in wire or "FAN" in wire:
        return 2
    if "BEG" in wire:
        return 3
    if "OUT" in wire:
        return 4
    if "END" in wire:
        return 7
    if "IMUX" in wire:
        return 8
    return 5


def _shortest_route_prefix(adj: dict[str, list[str]], start: str, goal: str) -> list[str] | None:
    if start == goal:
        return [start]

    queue: deque[str] = deque([start])
    parent: dict[str, str | None] = {start: None}
    while queue:
        node = queue.popleft()
        for nxt in adj.get(node, []):
            if nxt in parent:
                continue
            parent[nxt] = node
            if nxt == goal:
                path: list[str] = [goal]
                cursor = node
                while cursor is not None:
                    path.append(cursor)
                    cursor = parent[cursor]
                path.reverse()
                return path
            queue.append(nxt)
    return None


def _route_reachable_roots(adj: dict[str, list[str]], root: str, roots: list[str]) -> set[str]:
    reachable: set[str] = set()
    queue: deque[str] = deque([root])
    seen: set[str] = {root}
    root_set = set(roots)
    while queue:
        node = queue.popleft()
        if node in root_set:
            reachable.add(node)
        for nxt in adj.get(node, []):
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return reachable


def aggregate_a7_route_tree(paths: list[list[str]]) -> A7RouteTreeAggregation:
    compact_paths = [_compact_route_path(path) for path in paths if path]
    compact_paths = [path for path in compact_paths if path]
    if not compact_paths:
        return A7RouteTreeAggregation([], None, True)

    roots = _ordered_unique(path[0] for path in compact_paths)
    if len(roots) == 1:
        root = roots[0]
        adjacency: dict[str, list[str]] = {}
        terminals = _ordered_unique(path[-1] for path in compact_paths)
        for path in compact_paths:
            for src, dst in zip(path, path[1:]):
                neighbors = adjacency.setdefault(src, [])
                if dst not in neighbors:
                    neighbors.append(dst)

        # A fixed route is a tree: every physical node has one parent. Choose
        # shortest-hop parents, then retain only nodes needed by route sinks.
        parent: dict[str, str | None] = {root: None}
        queue: deque[str] = deque([root])
        while queue:
            src = queue.popleft()
            for dst in adjacency.get(src, []):
                if dst in parent:
                    continue
                parent[dst] = src
                queue.append(dst)

        normalized: list[list[str]] = []
        for terminal in terminals:
            if terminal not in parent:
                continue
            path: list[str] = []
            cursor: str | None = terminal
            while cursor is not None:
                path.append(cursor)
                cursor = parent[cursor]
            path.reverse()
            if path not in normalized:
                normalized.append(path)
        return A7RouteTreeAggregation(normalized, root, True)

    unique = []
    for path in compact_paths:
        if path not in unique:
            unique.append(path)

    root = min(roots, key=lambda candidate: (_route_source_rank(candidate), roots.index(candidate)))
    disconnected_unique = [candidate for candidate in roots if candidate != root]
    return A7RouteTreeAggregation(
        unique,
        root,
        False,
        disconnected_unique,
        0,
    )


def _preferred_mux7_pair(inst: PlacedInst) -> int:
    return 1 if (inst.pos % 128) >= 8 else 0


def _physical_mux7_pair(logical_pair: int) -> int:
    return logical_pair


def _physical_lut_bel(logical_bel: int) -> int:
    return logical_bel


def _detect_mux_shapes(insts: list[PlacedInst], inst_by_name: dict[str, PlacedInst]) -> list[MuxShape]:
    shapes: list[MuxShape] = []
    f7_in_f8: set[str] = set()

    for mux8 in sorted((inst for inst in insts if inst.cell_type.startswith("MUXF8")), key=lambda inst: (inst.pos, inst.name)):
        mux7_by_pair: dict[int, PlacedInst] = {}
        lut_by_slot: dict[int, PlacedInst] = {}
        drivers = _driver_by_port(mux8, inst_by_name)
        for port, pair in (("I0", 0), ("I1", 1)):
            mux7 = drivers.get(port)
            if mux7 is None or not mux7.cell_type.startswith("MUXF7"):
                continue
            mux7_by_pair[pair] = mux7
            f7_in_f8.add(mux7.name)
            # In a 7-series F7 pair, I0 is the upper (B/D) LUT and I1 is the lower (A/C) LUT.
            for lut_port, bel in (("I0", pair * 2 + 1), ("I1", pair * 2)):
                lut = _driver_by_port(mux7, inst_by_name).get(lut_port)
                if lut is not None and cell_kind(lut) == "LUT":
                    lut_by_slot[bel] = lut
        shapes.append(MuxShape(mux8, mux7_by_pair, lut_by_slot))

    for mux7 in sorted(
        (inst for inst in insts if inst.cell_type.startswith("MUXF7") and inst.name not in f7_in_f8),
        key=lambda inst: (inst.pos, inst.name),
    ):
        pair = _preferred_mux7_pair(mux7)
        lut_by_slot: dict[int, PlacedInst] = {}
        for lut_port, bel in (("I0", pair * 2 + 1), ("I1", pair * 2)):
            lut = _driver_by_port(mux7, inst_by_name).get(lut_port)
            if lut is not None and cell_kind(lut) == "LUT":
                lut_by_slot[bel] = lut
        shapes.append(MuxShape(mux7, {pair: mux7}, lut_by_slot))

    return shapes


def _shape_members(shape: MuxShape) -> list[PlacedInst]:
    return _ordered_insts([shape.root, *shape.mux7_by_pair.values(), *shape.lut_by_slot.values()])


def _ordered_insts(insts: Iterable[PlacedInst]) -> list[PlacedInst]:
    out: list[PlacedInst] = []
    seen: set[str] = set()
    for inst in insts:
        if inst.name not in seen:
            seen.add(inst.name)
            out.append(inst)
    return out


def _shape_tile_candidates(state: A7PackState, shape: MuxShape) -> list[str]:
    member_tiles = [state.tile(state.original_tile(inst)) for inst in _shape_members(shape)]
    original_names = {tile.name for tile in member_tiles}

    def score(tile_name: str) -> tuple[int, int, str]:
        tile = state.tile(tile_name)
        distance = min(
            abs(tile.grid_x - original.grid_x) + abs(tile.grid_y - original.grid_y)
            for original in member_tiles
        )
        original_penalty = 0 if tile_name in original_names else 1
        return distance, original_penalty, tile_name

    return sorted(
        (tile_name for tile_name, tile in state.tile_by_name.items() if tile.type in CLB_TYPES),
        key=score,
    )


def _site_type(tile: TileInfo, site_index: int) -> str:
    site_types = [site_type for _, site_type in sorted(tile.sites.items())]
    if not site_types:
        return "SLICEL"
    if site_index < len(site_types):
        return site_types[site_index]
    return site_types[-1]


def _shape_prefers_slicel(shape: MuxShape) -> bool:
    return any(lut.cell_type == "LUT6" for lut in shape.lut_by_slot.values())


def _shape_site_candidates(state: A7PackState, tile_name: str, shape: MuxShape) -> list[int]:
    sites = list(range(slice_count(state.tile(tile_name))))
    preferred: list[int] = []
    for inst in _shape_members(shape):
        site = state.original_site(inst, tile_name)
        if site not in preferred:
            preferred.append(site)
    for site in reversed(preferred):
        if site in sites:
            sites.remove(site)
            sites.insert(0, site)
    if _shape_prefers_slicel(shape):
        original_order = {site: index for index, site in enumerate(sites)}
        sites.sort(key=lambda site: (0 if _site_type(state.tile(tile_name), site) == "SLICEL" else 1, original_order[site]))
    return sites


def _assign_shape(state: A7PackState, shape: MuxShape, tile_name: str, site: int) -> bool:
    if shape.root.cell_type.startswith("MUXF8"):
        if not state.assign(shape.root, tile_name, site, 0, "a7-mux8-shape"):
            return False
    for pair, mux7 in shape.mux7_by_pair.items():
        constrain = shape.root.name == mux7.name
        if not state.assign(mux7, tile_name, site, _physical_mux7_pair(pair), "a7-mux7-shape", constrain=constrain):
            return False
    for bel, lut in shape.lut_by_slot.items():
        if not state.assign(lut, tile_name, site, _physical_lut_bel(bel), "a7-mux-data-lut", constrain=False):
            return False
    if shape.root.cell_type.startswith("MUXF8"):
        for pair in range(2):
            state.mux7_used.add((tile_name, site, _physical_mux7_pair(pair)))
        for bel in range(4):
            state.lut_used.add((tile_name, site, _physical_lut_bel(bel)))
    else:
        for pair in shape.mux7_by_pair:
            state.mux7_used.add((tile_name, site, _physical_mux7_pair(pair)))
            state.lut_used.add((tile_name, site, _physical_lut_bel(pair * 2)))
            state.lut_used.add((tile_name, site, _physical_lut_bel(pair * 2 + 1)))
    return True


def _pack_mux_shapes(state: A7PackState, shapes: list[MuxShape]) -> None:
    # Pack mux trees before scalar cells so tree members can claim a whole
    # legal slice shape and later cells are moved away from occupied slots.
    for shape in sorted(shapes, key=lambda item: (-len(_shape_members(item)), item.root.pos, item.root.name)):
        packed = False
        for tile_name in _shape_tile_candidates(state, shape):
            for site in _shape_site_candidates(state, tile_name, shape):
                snapshot = state.snapshot()
                if _assign_shape(state, shape, tile_name, site):
                    packed = True
                    break
                state.restore(snapshot)
            if packed:
                break
        if not packed:
            members = ", ".join(f"{inst.cell_type}:{inst.name}" for inst in _shape_members(shape))
            raise SystemExit(f"cannot pack A7 mux shape rooted at {shape.root.name}; members: {members}")


def _connected_driver_place(state: A7PackState, inst: PlacedInst) -> tuple[str, int, int] | None:
    for conn in inst.raw.get("connections", []):
        driver = state.placed.get(str(conn.get("driver_inst", "")))
        if driver is None:
            continue
        driver_inst = state.inst_by_name.get(str(conn.get("driver_inst", "")))
        if driver_inst is None:
            continue
        driver_kind = cell_kind(driver_inst)
        sink_port = str(conn.get("sink_port", ""))
        driver_port = str(conn.get("driver_port", ""))
        if driver_kind == "LUT" and cell_kind(inst) == "FD":
            return driver.tile_name, driver.placement.site_index, driver.placement.bel_index
        if driver_kind == "CARRY" and cell_kind(inst) == "FD":
            bit = inferred_carry_bit(driver_port)
            return driver.tile_name, driver.placement.site_index, bit if bit is not None else driver.placement.bel_index
        if driver_kind == "LUT" and cell_kind(inst) == "CARRY":
            bit = inferred_carry_bit(sink_port)
            return driver.tile_name, driver.placement.site_index, bit if bit is not None else driver.placement.bel_index
    return None


def _connected_driver_kind(state: A7PackState, inst: PlacedInst) -> str | None:
    for conn in inst.raw.get("connections", []):
        driver_inst = state.inst_by_name.get(str(conn.get("driver_inst", "")))
        if driver_inst is not None:
            return cell_kind(driver_inst)
    return None


def _candidate_slots(state: A7PackState, inst: PlacedInst) -> list[tuple[str, int, int]]:
    kind = cell_kind(inst)
    connected = _connected_driver_place(state, inst)
    if connected is not None:
        tiles = [connected[0]]
    else:
        original_tile = state.tile(state.original_tile(inst))
        tiles = sorted(
            state.tile_by_name,
            key=lambda tile_name: (
                abs(state.tile(tile_name).grid_x - original_tile.grid_x)
                + abs(state.tile(tile_name).grid_y - original_tile.grid_y),
                tile_name,
            ),
        )

    slots: list[tuple[str, int, int]] = []
    for tile_name in tiles:
        site_count = slice_count(state.tile(tile_name))
        original_site = state.original_site(inst, tile_name)
        if kind == "CARRY":
            tile_slots = [(tile_name, site, 0) for site in range(site_count)]
            preferred = (tile_name, original_site, 0)
        elif kind == "MUX":
            if inst.cell_type.startswith("MUXF8"):
                tile_slots = [(tile_name, site, 0) for site in range(site_count)]
                preferred = (tile_name, original_site, 0)
            else:
                tile_slots = [(tile_name, site, pair) for site in range(site_count) for pair in range(2)]
                preferred = (tile_name, original_site, _preferred_mux7_pair(inst))
        else:
            bel_count = 8 if kind == "FD" else 4
            tile_slots = [(tile_name, site, bel) for site in range(site_count) for bel in range(bel_count)]
            preferred = (tile_name, original_site, state.original_bel(inst))
        if connected is not None:
            preferred = connected
        if preferred in tile_slots:
            tile_slots.remove(preferred)
            tile_slots.insert(0, preferred)
        slots.extend(tile_slots)
    return _ordered_unique_slots(slots)


def _ordered_unique_slots(slots: Iterable[tuple[str, int, int]]) -> list[tuple[str, int, int]]:
    out: list[tuple[str, int, int]] = []
    seen: set[tuple[str, int, int]] = set()
    for slot in slots:
        if slot not in seen:
            seen.add(slot)
            out.append(slot)
    return out


def _is_compatible(state: A7PackState, inst: PlacedInst, tile_name: str, site: int, bel: int) -> bool:
    if not state.is_free(inst, tile_name, site, bel):
        return False
    kind = cell_kind(inst)
    if kind == "CARRY":
        for bit, net in carry_s_nets(inst).items():
            slot_nets = state.lut_nets_by_slot.get((tile_name, site, bit))
            if slot_nets is not None and net not in slot_nets:
                return False
    if kind == "LUT":
        reserved_net = state.carry_s_net_by_slot.get((tile_name, site, bel))
        if reserved_net is not None and reserved_net not in lut_output_nets(inst, state.outgoing_by_driver):
            return False
    connected = _connected_driver_place(state, inst)
    if connected is not None:
        if cell_kind(inst) == "FD":
            if bel >= 4 and _connected_driver_kind(state, inst) in {"LUT", "CARRY"}:
                return False
            if connected[:2] != (tile_name, site) or connected[2] != (bel % 4):
                return False
        elif connected != (tile_name, site, bel):
            return False
    return True


def _pack_remaining_cells(state: A7PackState, insts: list[PlacedInst]) -> None:
    remaining = [
        inst
        for inst in insts
        if cell_kind(inst) != "OTHER" and inst.name not in state.placed
    ]
    remaining.sort(
        key=lambda inst: (
            {"CARRY": 0, "LUT": 1, "MUX": 2, "FD": 3}.get(cell_kind(inst), 4),
            0 if _connected_driver_kind(state, inst) in {"LUT", "CARRY"} else 1,
            -sum(1 for conn in inst.raw.get("connections", []) if str(conn.get("driver_inst", "")) in state.placed),
            inst.pos,
            inst.name,
        )
    )

    progress = True
    while remaining and progress:
        progress = False
        next_remaining: list[PlacedInst] = []
        for inst in remaining:
            selected: tuple[str, int, int] | None = None
            for tile_name, site, bel in _candidate_slots(state, inst):
                if _is_compatible(state, inst, tile_name, site, bel):
                    selected = (tile_name, site, bel)
                    break
            if selected is None:
                next_remaining.append(inst)
                continue
            tile_name, site, bel = selected
            state.assign(inst, tile_name, site, bel, "a7-greedy")
            progress = True
        remaining = next_remaining

    if remaining:
        blocked = ", ".join(f"{inst.cell_type}:{inst.name}" for inst in remaining)
        raise SystemExit(f"cannot pack A7 CLB cells after mux-shape legalization: {blocked}")


def pack_a7_clb_placements(grouped: dict[str, TileState], tile_by_name: dict[str, TileInfo] | None = None) -> A7PackResult:
    # Public entry point used by db2prj.py: detect mux shapes, reserve exact
    # BELs for those shapes, then greedily pack the remaining compatible cells.
    if tile_by_name is None:
        tile_by_name = {name: tile_state.clb_tile for name, tile_state in grouped.items()}
    insts = [
        inst
        for tile_state in grouped.values()
        for inst in tile_state.insts
        if cell_kind(inst) != "OTHER"
    ]
    inst_by_name = {inst.name: inst for inst in insts}
    original_tile_by_inst = {
        inst.name: tile_name
        for tile_name, tile_state in grouped.items()
        for inst in tile_state.insts
        if cell_kind(inst) != "OTHER"
    }
    outgoing_by_driver = next(iter(grouped.values())).outgoing_by_driver if grouped else {}
    state = A7PackState(tile_by_name, inst_by_name, original_tile_by_inst, outgoing_by_driver)

    shapes = _detect_mux_shapes(insts, inst_by_name)
    _pack_mux_shapes(state, shapes)
    _pack_remaining_cells(state, insts)
    return A7PackResult(dict(state.placed), [])
