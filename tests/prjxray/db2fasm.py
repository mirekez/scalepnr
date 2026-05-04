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
    grid_coord: tuple[int, int]
    raw: dict[str, Any]


@dataclass
class TileState:
    clb_tile: TileInfo
    insts: list[PlacedInst] = field(default_factory=list)


@dataclass
class FasmOutput:
    features: set[str] = field(default_factory=set)
    warnings: list[str] = field(default_factory=list)

    def add(self, feature: str, db: PrjxrayDb, reason: str) -> None:
        if not feature:
            return
        if db.has_feature(feature):
            self.features.add(feature)
        elif db.has_pseudo_feature(feature):
            return
        else:
            self.warnings.append(f"skip unknown feature for {reason}: {feature}")

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
            return feature in self.pseudo_suffixes
        return f"{info.type}.{feature}" in self.pseudo_features

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
            grid_coord=tuple(ann.get("grid_coord", [-1, -1])),
            raw=inst,
        )


def group_by_clb_tile(state: dict[str, Any], db: PrjxrayDb, out: FasmOutput) -> dict[str, TileState]:
    grouped: dict[str, TileState] = {}
    for inst in placed_insts(state):
        actual_cb = db.tile_at_grid(inst.grid_coord)
        clb_tile = db.clb_neighbor_for_grid(*inst.grid_coord)
        if clb_tile is None:
            where = actual_cb.name if actual_cb else inst.cb_tile
            out.warn(f"skip cell without CLB neighbor: {inst.name} at {where}")
            continue
        grouped.setdefault(clb_tile.name, TileState(clb_tile=clb_tile)).insts.append(inst)
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


def bel_for_pos(pos: int) -> str:
    local_pos = pos % 128
    return BEL_LETTERS[(local_pos // 4) % 4]


def emit_fdre(inst: PlacedInst, tile: TileInfo, out: FasmOutput, db: PrjxrayDb) -> None:
    site, _ = site_for_pos(tile, inst.pos)
    bel = bel_for_pos(inst.pos)
    prefix = f"{tile.name}.{site}"
    reason = f"{inst.cell_type} {inst.name}"
    out.add(f"{prefix}.{bel}FF.ZRST", db, reason)
    out.add(f"{prefix}.{bel}FFMUX.O6", db, reason)
    out.add(f"{prefix}.NOCLKINV", db, reason)
    out.add(f"{prefix}.SRUSEDMUX", db, reason)


def emit_lut(inst: PlacedInst, tile: TileInfo, out: FasmOutput, db: PrjxrayDb) -> None:
    site, _ = site_for_pos(tile, inst.pos)
    bel = bel_for_pos(inst.pos)
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


def emit_cells(grouped: dict[str, TileState], out: FasmOutput, db: PrjxrayDb) -> None:
    for tile_state in grouped.values():
        for inst in tile_state.insts:
            if inst.cell_type.startswith("FD"):
                emit_fdre(inst, tile_state.clb_tile, out, db)
            elif inst.cell_type.startswith("LUT"):
                emit_lut(inst, tile_state.clb_tile, out, db)
            elif inst.cell_type == "CARRY4":
                out.warn(f"skip CARRY4 config for now: {inst.name}")
            else:
                out.warn(f"skip unsupported placed cell {inst.cell_type}: {inst.name}")


def route_features(state: dict[str, Any]) -> Iterable[str]:
    for inst in state["insts"]:
        for route in inst.get("routes", []):
            for wire in route:
                ann = wire.get("annotation", {})
                for feature in ann.get("fasm_features", []):
                    yield feature


def emit_routing(state: dict[str, Any], out: FasmOutput, db: PrjxrayDb) -> None:
    for feature in route_features(state):
        out.add(feature, db, "routing")


def write_fasm(path: Path, features: Iterable[str]) -> None:
    with path.open("w") as f:
        for feature in sorted(set(features)):
            f.write(feature)
            f.write("\n")


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
    emit_cells(grouped, out, db)
    emit_routing(state, out, db)
    write_fasm(args.output_fasm, out.features)

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
