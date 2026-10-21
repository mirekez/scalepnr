#!/usr/bin/env python3
"""Regression for a reachable buffer pseudo-PIP silently lost during export.

The real routing witness uses a site input -> output pseudo-connection. It
needs the buffer's enable and non-inverted enable settings, despite appearing
in the always-connected PIP table. All fixture tile and wire names are abstract;
the site primitive and configuration fields retain their backend meanings.
"""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from db2fasm import FasmOutput, PrjxrayDb, emit_placement, emit_routing, write_fasm


class BufferPseudoConnectionTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        (self.root / "tilegrid.json").write_text(json.dumps({
            "tile_17": {"type": "REGION", "grid_x": 3, "grid_y": 7},
            "tile_29": {"type": "REGION", "grid_x": 9, "grid_y": 7},
        }))
        (self.root / "segbits_region.db").write_text(
            "REGION.BUFHCE.BUFHCE_X0Y1.IN_USE 00_01\n"
            "REGION.BUFHCE.BUFHCE_X0Y1.ZINV_CE 00_02\n"
            "REGION.node_74.node_61 00_03\n")
        (self.root / "ppips_region.db").write_text(
            "REGION.node_61.node_42 always\n"
            "REGION.node_87.node_74 always\n")
        (self.root / "tile_type_REGION.json").write_text(json.dumps({
            "sites": [{
                "name": "X0Y1", "prefix": "BUFHCE", "type": "BUFHCE",
                "site_pins": {"I": {"wire": "node_42"},
                              "O": {"wire": "node_61"}},
            }],
        }))

    @staticmethod
    def state(features):
        return {"route_trees": [{"net": "signal_23", "branches": [{
            "wires": [{"annotation": {"fasm_features": features}}],
        }]}]}

    def test_routed_pseudo_buffer_is_enabled_in_written_output(self):
        db = PrjxrayDb(self.root)
        out = FasmOutput()
        groups = emit_routing(self.state([
            "tile_17.node_61.node_42", "tile_17.node_61.node_42",
            "tile_17.node_74.node_61", "tile_17.node_87.node_74",
        ]), out, db)
        expected = {
            "tile_17.BUFHCE.BUFHCE_X0Y1.IN_USE",
            "tile_17.BUFHCE.BUFHCE_X0Y1.ZINV_CE",
            "tile_17.node_74.node_61",
        }
        self.assertEqual(out.features, expected)
        self.assertEqual(out.warnings, [])
        output = self.root / "design.fasm"
        write_fasm(output, out.features, groups, [])
        emitted = [line.strip() for line in output.read_text().splitlines()
                   if line.strip() and not line.startswith("#")]
        self.assertCountEqual(emitted, expected)

    def test_fixed_wire_does_not_enable_unrelated_buffer(self):
        out = FasmOutput()
        emit_routing(self.state(["tile_29.node_87.node_74"]),
                     out, PrjxrayDb(self.root))
        self.assertEqual(out.features, set())
        self.assertEqual(out.warnings, [])

    def test_missing_enable_feature_is_reported(self):
        db = PrjxrayDb(self.root)
        db.features.remove("REGION.BUFHCE.BUFHCE_X0Y1.ZINV_CE")
        out = FasmOutput()
        emit_routing(self.state(["tile_29.node_61.node_42"]), out, db)
        self.assertTrue(any("ZINV_CE" in message for message in out.warnings))
        self.assertFalse(any(feature.startswith("tile_17.")
                             for feature in out.features))

    def add_global_buffer(self):
        # A placed global buffer has two separately routed nets (I and O).
        # Neither net necessarily traverses its internal input -> output PIP.
        metadata = json.loads((self.root / "tile_type_REGION.json").read_text())
        metadata["sites"].append({
            "name": "X0Y4", "prefix": "BUFGCTRL", "type": "BUFGCTRL",
            "site_pins": {"I0": {"wire": "node_110"},
                          "I1": {"wire": "node_111"},
                          "O": {"wire": "node_135"}},
        })
        (self.root / "tile_type_REGION.json").write_text(json.dumps(metadata))
        with (self.root / "segbits_region.db").open("a") as stream:
            for name in ("IN_USE", "IS_IGNORE0_INVERTED", "IS_IGNORE1_INVERTED",
                         "ZINV_CE0", "ZINV_CE1", "ZINV_S0", "ZINV_S1"):
                stream.write(f"REGION.BUFGCTRL.BUFGCTRL_X0Y4.{name} 01_01\n")
        with (self.root / "ppips_region.db").open("a") as stream:
            stream.write("REGION.node_135.node_110 always\n"
                         "REGION.node_135.node_111 always\n")

    @staticmethod
    def global_features(input_index):
        return {f"tile_29.BUFGCTRL.BUFGCTRL_X0Y4.{name}" for name in (
            "IN_USE", f"IS_IGNORE{1-input_index}_INVERTED",
            f"ZINV_CE{input_index}", f"ZINV_S{input_index}")}

    def test_global_buffer_pseudo_inputs_select_correct_controls(self):
        self.add_global_buffer()
        db = PrjxrayDb(self.root)
        for index in (0, 1):
            with self.subTest(input=index):
                out = FasmOutput()
                emit_routing(self.state([f"tile_29.node_135.node_{110+index}"]), out, db)
                self.assertEqual(out.features, self.global_features(index))
                self.assertEqual(out.warnings, [])

    def test_placed_global_buffer_is_enabled_without_internal_route_edge(self):
        self.add_global_buffer()
        state = {"insts": [{
            "name": "buffer_8", "type": "BUFG", "placed": True, "pos": 1,
            "annotation": {"resource_tile": "tile_29", "grid_coord": [9, 7],
                           "site": {"name": "X0Y4", "type": "BUFGCTRL"}},
        }]}
        out = FasmOutput()
        emit_placement(state, out, PrjxrayDb(self.root))
        self.assertEqual(out.features, self.global_features(0))
        self.assertEqual(out.warnings, [])
        # Legacy states can omit the site annotation. The placement index is
        # into all sites, including the unrelated regional buffer at index 0.
        del state["insts"][0]["annotation"]["site"]
        out = FasmOutput()
        emit_placement(state, out, PrjxrayDb(self.root))
        self.assertEqual(out.features, self.global_features(0))
        self.assertEqual(out.warnings, [])

    def test_bad_global_buffer_site_is_reported(self):
        self.add_global_buffer()
        state = {"insts": [{
            "name": "buffer_8", "type": "BUFG", "placed": True, "pos": 99,
            "annotation": {"resource_tile": "tile_29", "grid_coord": [9, 7],
                           "site": {"name": "X9Y9", "type": "BUFGCTRL"}},
        }]}
        out = FasmOutput()
        emit_placement(state, out, PrjxrayDb(self.root))
        self.assertEqual(out.features, set())
        self.assertTrue(any("buffer_8" in warning for warning in out.warnings))


if __name__ == "__main__":
    unittest.main()
