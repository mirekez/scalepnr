#!/usr/bin/env python3
import re
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from a7_packing import A7PackedCell, aggregate_a7_route_tree, legalize_a7_mux_placements
from db2fasm import PackedPlacement, PlacedInst, TileInfo, TileState
from db2prj import (
    canonical_fixed_route_nodes,
    clb_bel,
    collect_lut_pin_locks,
    export_ref_name,
    iob_input_source_nodes,
    packed_output_site_pin,
    packed_clb_output_nodes,
    packed_site_internal_branch,
    physical_route_pips,
    placed_iob_input_tail_node,
    remove_closed_source_walk,
    repair_clb_output_hops,
    repair_reserved_route_nodes,
    route_full_nodes,
    route_iob_resource_endpoint,
    route_tile_resource_endpoint,
    route_tile_resource_local,
    route_tile_pin_node,
    route_tree_expression,
    short_route_expansion,
    tileconn_neighbors_by_source,
)


class RouteTreeExpressionTest(unittest.TestCase):
    def test_linked_inverter_reference_matches_vivado_lut_primitive(self) -> None:
        self.assertEqual(export_ref_name("INV"), "LUT1")
        self.assertEqual(export_ref_name("LUT4"), "LUT4")

    def test_implemented_route_removes_only_closed_source_walk(self) -> None:
        self.assertEqual(
            remove_closed_source_walk(["SOURCE", "A", "B", "SOURCE", "TAIL"]),
            ["SOURCE", "TAIL"],
        )
        self.assertEqual(
            remove_closed_source_walk(["SOURCE", "A", "B", "TAIL"]),
            ["SOURCE", "A", "B", "TAIL"],
        )

    def test_short_route_crosses_terminal_alias_with_real_pip_exit(self) -> None:
        aliases = frozenset({"ABC_X0Y0/IMUX0", "EDGE_X1Y0/IMUX_ALIAS0"})
        neighbors = {
            "ABC_X0Y0/IMUX0": ["EDGE_X1Y0/IMUX_ALIAS0"],
            "EDGE_X1Y0/IMUX_ALIAS0": ["ABC_X0Y0/IMUX0", "EDGE_X1Y0/SINK0"],
        }

        with (
            patch("db2prj.tileconn_alias_nodes", return_value=aliases),
            patch(
                "db2prj.route_node_neighbors",
                side_effect=lambda _db, node: neighbors.get(node, []),
            ),
            patch(
                "db2prj.direct_pip_feature",
                side_effect=lambda _db, source, destination: (
                    "EDGE.SINK0.IMUX_ALIAS0"
                    if (source, destination) == ("EDGE_X1Y0/IMUX_ALIAS0", "EDGE_X1Y0/SINK0")
                    else None
                ),
            ),
        ):
            path = short_route_expansion(
                SimpleNamespace(tilegrid={}),
                "ABC_X0Y0/IMUX0",
                "EDGE_X1Y0/SINK0",
            )

        self.assertEqual(path, [
            "ABC_X0Y0/IMUX0",
            "EDGE_X1Y0/IMUX_ALIAS0",
            "EDGE_X1Y0/SINK0",
        ])

    def test_iob_source_uses_annotated_lane_not_site_order(self) -> None:
        db = SimpleNamespace(
            tilegrid={
                "LIOB_X0Y0": TileInfo(
                    "LIOB_X0Y0", "LIOB", 0, 0,
                    {"IOB_X0Y0": "IOB", "IOB_X0Y1": "IOB"},
                ),
                "LIOI_X0Y0": TileInfo("LIOI_X0Y0", "LIOI", 0, 0, {}),
            },
            pip_sources_by_type_dst={},
        )
        wire = {
            "annotation": {
                "tile_resource": {
                    "output": {
                        "resource_full_name": "LIOB_X0Y0.IOB_IBUF1",
                    },
                },
            },
        }

        with patch(
            "db2prj.tileconn_alias_nodes",
            return_value=frozenset({"LIOI_X0Y0/IOI_LOGIC_OUTS18_0"}),
        ), patch("db2prj.route_node_neighbors", return_value=[]):
            nodes = iob_input_source_nodes(wire, db, "IOB_X0Y1")

        self.assertEqual(nodes[0], "LIOB_X0Y0/IOB_IBUF1")
        self.assertEqual(nodes[1], "LIOI_X0Y0/LIOI_IBUF1")

    def test_iob_source_follows_interface_tile_output(self) -> None:
        db = SimpleNamespace(
            tilegrid={
                "LIOB_X0Y0": TileInfo("LIOB_X0Y0", "LIOB", 0, 0, {}),
                "LIOI_X0Y0": TileInfo("LIOI_X0Y0", "LIOI", 0, 0, {}),
            },
            pip_sources_by_type_dst={},
        )
        wire = {
            "annotation": {
                "tile_resource": {
                    "output": {
                        "resource_full_name": "LIOB_X0Y0.IOB_IBUF1",
                    },
                },
            },
        }
        aliases = frozenset({
            "LIOI_X0Y0/IOI_LOGIC_OUTS18_0",
            "EDGE_X0Y1/EDGE_LOGIC_OUTS_B18",
        })
        neighbors = {
            "EDGE_X0Y1/EDGE_LOGIC_OUTS_B18": ["EDGE_X0Y1/EDGE_LOGIC_OUTS18"],
        }
        with (
            patch("db2prj.tileconn_alias_nodes", return_value=aliases),
            patch(
                "db2prj.route_node_neighbors",
                side_effect=lambda _db, node: neighbors.get(node, []),
            ),
            patch("db2prj.direct_pip_feature", return_value="EDGE.OUTS.B18"),
        ):
            nodes = iob_input_source_nodes(wire, db)

        self.assertEqual(nodes[-1], "EDGE_X0Y1/EDGE_LOGIC_OUTS18")

    def test_iob_source_chain_keeps_annotated_route_local(self) -> None:
        route = [{
            "type": "tile_pin",
            "pin_dir": 1,
            "annotation": {
                "from_cb_tile": "ABC_X2Y0",
                "nodes": [{"full_name": "ABC_X2Y0.ROUTE_LOCAL"}],
                "tile_resource": {
                    "output": {"resource_full_name": "IOB_X0Y0.IOB_IBUF1"},
                },
            },
        }]
        with (
            patch(
                "db2prj.iob_input_source_nodes",
                return_value=["IOB_X0Y0/IOB_IBUF1", "IOI_X1Y0/LOGIC_OUT"],
            ),
            patch("db2prj.expand_same_tile_nodes", side_effect=lambda _db, nodes: nodes),
            patch("db2prj.repair_clb_output_hops", side_effect=lambda _db, nodes, _blocked: nodes),
        ):
            nodes = route_full_nodes(route, SimpleNamespace())

        self.assertEqual(nodes, [
            "IOB_X0Y0/IOB_IBUF1",
            "IOI_X1Y0/LOGIC_OUT",
            "ABC_X2Y0/ROUTE_LOCAL",
        ])

    def test_reserved_packed_resource_is_replaced_by_directed_bridge(self) -> None:
        neighbors = {
            "ABC/SRC": ["ABC/ALT0"],
            "ABC/ALT0": ["ABC/ALT1"],
            "ABC/ALT1": ["ABC/DST"],
            "ABC/DST": [],
        }
        with patch("db2prj.route_node_neighbors", side_effect=lambda _db, node: neighbors.get(node, [])):
            repaired = repair_reserved_route_nodes(
                SimpleNamespace(),
                ["ABC/SRC", "ABC/RESERVED", "ABC/DST"],
                {"ABC/RESERVED"},
            )

        self.assertEqual(repaired, ["ABC/SRC", "ABC/ALT0", "ABC/ALT1", "ABC/DST"])

    def test_physical_route_pips_resolve_tileconn_aliases(self) -> None:
        components = {
            "ABC/SRC": frozenset({"ABC/SRC", "DEF/SRC_ALIAS"}),
            "DEF/DST": frozenset({"DEF/DST"}),
        }

        with patch(
            "db2prj.tileconn_alias_nodes",
            side_effect=lambda _db, node: components[node],
        ), patch(
            "db2prj.direct_pip_feature",
            side_effect=lambda _db, source, destination: (
                "DEF.DST.SRC_ALIAS"
                if (source, destination) == ("DEF/SRC_ALIAS", "DEF/DST")
                else None
            ),
        ):
            self.assertEqual(
                physical_route_pips(SimpleNamespace(), ["ABC/SRC", "DEF/DST"]),
                ["DEF.DST.SRC_ALIAS"],
            )

    def test_physical_route_pips_prefers_explicit_node_over_alias(self) -> None:
        components = {
            "ABC/EXPLICIT_END": frozenset({"ABC/EXPLICIT_END", "ABC/ALIAS_END"}),
            "ABC/SINK": frozenset({"ABC/SINK"}),
        }

        with patch(
            "db2prj.tileconn_alias_nodes",
            side_effect=lambda _db, node: components[node],
        ), patch(
            "db2prj.direct_pip_feature",
            side_effect=lambda _db, source, destination: (
                f"ABC.SINK.{source.rsplit('/', 1)[1]}"
                if destination == "ABC/SINK" else None
            ),
        ):
            self.assertEqual(
                physical_route_pips(
                    SimpleNamespace(), ["ABC/EXPLICIT_END", "ABC/SINK"]
                ),
                ["ABC.SINK.EXPLICIT_END"],
            )

    def test_same_site_endpoint_only_branch_is_left_to_packing(self) -> None:
        sites = {"driver": "SITE_X1Y2", "sink": "SITE_X1Y2"}
        endpoint_route = [{"type": "tile_pin"}, {"type": "tile_pin"}]
        insts = {"driver": {"type": "LUT6"}, "sink": {"type": "FDRE"}}
        packed = {
            "driver": A7PackedCell(
                "ABC_X1Y2", PackedPlacement(0, 2, 0, "test")
            ),
            "sink": A7PackedCell(
                "ABC_X1Y2", PackedPlacement(0, 2, 0, "test")
            ),
        }

        self.assertTrue(
            packed_site_internal_branch(
                endpoint_route, ("driver", "O"), "sink", "D", sites, insts, packed
            )
        )
        self.assertTrue(
            packed_site_internal_branch(
                [endpoint_route[0], {"type": "crossbar"}, endpoint_route[1]],
                ("driver", "O"),
                "sink",
                "D",
                sites,
                insts,
                packed,
            )
        )
        self.assertFalse(
            packed_site_internal_branch(
                endpoint_route,
                ("driver", "O"),
                "sink",
                "D",
                {**sites, "sink": "SITE_X2Y2"},
                insts,
                packed,
            )
        )
        self.assertFalse(
            packed_site_internal_branch(
                endpoint_route,
                ("driver", "O"),
                "sink",
                "D",
                sites,
                {"driver": {"type": "MUXF7"}, "sink": {"type": "LUT6"}},
                packed,
            )
        )
        self.assertFalse(
            packed_site_internal_branch(
                endpoint_route,
                ("driver", "O"),
                "sink",
                "D",
                sites,
                insts,
                {
                    **packed,
                    "sink": A7PackedCell(
                        "ABC_X1Y2", PackedPlacement(0, 3, 0, "test")
                    ),
                },
            )
        )

        # FF control pins always use routed site-control resources, even when
        # the logical LUT and FF happen to occupy the same packed lane.
        self.assertFalse(
            packed_site_internal_branch(
                endpoint_route,
                ("driver", "O"),
                "sink",
                "CE",
                sites,
                insts,
                packed,
            )
        )

    def test_route_tree_reconvergence_uses_one_shortest_parent(self) -> None:
        aggregation = aggregate_a7_route_tree([
            ["ROOT", "DETOUR0", "DETOUR1", "JOIN", "SINK0"],
            ["ROOT", "JOIN", "SINK1"],
        ])

        self.assertTrue(aggregation.connected)
        self.assertEqual(
            aggregation.paths,
            [
                ["ROOT", "JOIN", "SINK0"],
                ["ROOT", "JOIN", "SINK1"],
            ],
        )
        expression = route_tree_expression(aggregation.paths)
        self.assertEqual(expression.count("{JOIN}"), 1)

    def test_non_lut_sink_keeps_annotated_resource_endpoint(self) -> None:
        route = [{
            "type": "tile_pin",
            "pin_dir": 0,
            "port": "S",
            "annotation": {
                "from_cb_tile": "ABC_X0Y0",
                "nodes": [{"full_name": "ABC_X0Y0.LOCAL_INPUT"}],
                "tile_resource": {
                    "input": {"resource_full_name": "ABC_X0Y0.SELECT_PIN"},
                },
            },
        }]

        with patch("db2prj.direct_pip_feature", return_value="ABC.SELECT_PIN.LOCAL_INPUT"):
            nodes = route_full_nodes(route, SimpleNamespace(), {"type": "MUXF7"})

        self.assertEqual(nodes, ["ABC_X0Y0/LOCAL_INPUT", "ABC_X0Y0/SELECT_PIN"])

    def test_fd_control_does_not_replace_annotated_control_lane(self) -> None:
        tail = route_tile_pin_node(
            {"type": "FDRE"},
            {
                "port": "R",
                "pin_dir": 0,
                "annotation": {"from_cb_tile": "ABC_X0Y0"},
            },
            SimpleNamespace(),
            {},
        )

        self.assertIsNone(tail)

    def test_lut_pin_locks_are_loaded_from_authoritative_route_trees(self) -> None:
        state = {
            "insts": [{"name": "logic_cell", "type": "LUT4"}],
            "route_trees": [{
                "branches": [{
                    "sink": {"inst": "logic_cell", "port": "I2"},
                    "wires": [{
                        "type": "tile_pin",
                        "pin_dir": 0,
                        "annotation": {
                            "tile_resource": {"input": {"pin": "D5"}},
                        },
                    }],
                }],
            }],
        }

        locks, warnings = collect_lut_pin_locks(state)

        self.assertEqual(locks, {"logic_cell": {"I2": "A5"}})
        self.assertEqual(warnings, [])

    def test_grounded_mux_input_preserves_independent_lut5_in_paired_bel(self) -> None:
        tile = TileInfo("ABC_X0Y0", "ABC", 0, 0, {})
        lut6 = PlacedInst("driver", "LUT6", 15, "", tile.name, (0, 0), {})
        lut5 = PlacedInst("independent", "LUT5", 11, "", tile.name, (0, 0), {})
        mux = PlacedInst(
            "mux",
            "MUXF7",
            9,
            "",
            tile.name,
            (0, 0),
            {"connections": [{"sink_port": "I0", "driver_inst": lut6.name}]},
        )
        base = {
            lut6.name: A7PackedCell(tile.name, PackedPlacement(0, 3, 15, "test")),
            lut5.name: A7PackedCell(tile.name, PackedPlacement(0, 2, 11, "test")),
            mux.name: A7PackedCell(tile.name, PackedPlacement(0, 1, 9, "test")),
        }

        result = legalize_a7_mux_placements(
            {tile.name: TileState(tile, [lut6, lut5, mux])},
            base,
        )

        self.assertEqual(result.cells[lut6.name].lut_bel_size, 6)
        self.assertEqual(result.cells[lut5.name].lut_bel_size, 5)
        self.assertEqual(clb_bel(lut5, result.cells[lut5.name]), "C5LUT")
        self.assertEqual(
            packed_output_site_pin(
                {"name": lut5.name, "type": lut5.cell_type},
                result.cells[lut5.name],
            ),
            "CQ",
        )

    def test_lut_outputs_use_the_vivado_physical_site_pin(self) -> None:
        tile = "ABC_X0Y0"
        for bel in range(4):
            lut6 = A7PackedCell(tile, PackedPlacement(0, bel, bel, "test"), lut_bel_size=6)
            lut5 = A7PackedCell(tile, PackedPlacement(0, bel, bel, "test"), lut_bel_size=5)
            letter = "ABCD"[bel]

            self.assertEqual(packed_output_site_pin({"type": "LUT6"}, lut6), letter)
            self.assertEqual(packed_output_site_pin({"type": "LUT5"}, lut5), f"{letter}Q")

    def test_mux_output_site_pins_follow_physical_wide_mux_levels(self) -> None:
        tile = "ABC_X0Y0"
        mux7a = A7PackedCell(tile, PackedPlacement(0, 0, 1, "test"))
        mux7b = A7PackedCell(tile, PackedPlacement(0, 1, 9, "test"))
        mux8 = A7PackedCell(tile, PackedPlacement(0, 0, 1, "test"))

        self.assertEqual(packed_output_site_pin({"type": "MUXF7"}, mux7a), "AMUX")
        self.assertEqual(packed_output_site_pin({"type": "MUXF7"}, mux7b), "CMUX")
        self.assertEqual(packed_output_site_pin({"type": "MUXF8"}, mux8), "BMUX")

    def test_generated_mux_output_uses_direct_site_path_from_real_lut(self) -> None:
        packed = A7PackedCell("ABC_X0Y0", PackedPlacement(0, 0, 0, "test"))
        wire = {"annotation": {
            "from_cb_tile": "ROUTE_X0Y0",
            "resource_tile": "ABC_X0Y0",
        }}

        with (
            patch(
                "db2prj.clb_route_wire_for_site_output",
                return_value=("ABC_A", "ABC_LOGIC_OUTS0", "LOGIC_OUTS0"),
            ),
            patch(
                "db2prj.annotated_clb_output_nodes",
                return_value=[
                    "ABC_X0Y0/ABC_AMUX",
                    "ABC_X0Y0/ABC_LOGIC_OUTS8",
                    "ROUTE_X0Y0/LOGIC_OUTS8",
                ],
            ),
            patch(
                "db2prj.direct_pip_feature",
                return_value="ABC.ABC_AMUX.ABC_A",
            ),
        ):
            nodes = packed_clb_output_nodes(
                {"name": "driver", "type": "LUT6", "resource_tile": "ABC_X0Y0"},
                wire,
                SimpleNamespace(),
                {"driver": packed},
            )

        # The generated MUX is reached through the site's direct LUT output;
        # no route back through an input pin may be synthesized.
        self.assertEqual(nodes, [
            "ABC_X0Y0/ABC_A",
            "ABC_X0Y0/ABC_AMUX",
            "ABC_X0Y0/ABC_LOGIC_OUTS8",
            "ROUTE_X0Y0/LOGIC_OUTS8",
        ])

    def test_ff_output_site_pin_follows_primary_or_secondary_bel(self) -> None:
        tile = "ABC_X0Y0"
        primary = A7PackedCell(tile, PackedPlacement(0, 3, 3, "test"))
        secondary = A7PackedCell(tile, PackedPlacement(0, 7, 7, "test"))

        self.assertEqual(packed_output_site_pin({"type": "FDRE"}, primary), "DQ")
        self.assertEqual(packed_output_site_pin({"type": "FDRE"}, secondary), "DMUX")

    def test_iob_endpoint_uses_final_package_placed_site_lane(self) -> None:
        db = SimpleNamespace(tilegrid={
            "LIOB33_X0Y7": TileInfo(
                "LIOB33_X0Y7",
                "LIOB33",
                0,
                7,
                {"IOB_X0Y7": "S", "IOB_X0Y8": "M"},
            ),
            "LIOI3_X0Y7": TileInfo("LIOI3_X0Y7", "LIOI3", 0, 7, {}),
        })

        with patch(
            "db2prj.load_tile_type_spec",
            return_value={"wires": {"IOI_OLOGIC0_D1": {}, "IOI_OLOGIC1_D1": {}}},
        ):
            endpoint = route_iob_resource_endpoint(
                "LIOB33_X0Y7",
                "IOB_O1",
                db,
                placed_site="IOB_X0Y8",
            )

        self.assertEqual(endpoint, "LIOI3_X0Y7/IOI_OLOGIC1_D1")
        self.assertEqual(
            placed_iob_input_tail_node("INT_L_X0Y7", "IOB_X0Y8"),
            "INT_L_X0Y7/IMUX_L34",
        )

    def test_ibuf_source_keeps_resource_and_local_endpoint_nodes(self) -> None:
        wire = {
            "annotation": {
                "tile_resource": {
                    "output": {
                        "resource_full_name": "RIOB_X9Y7.INPUT_BUFFER0",
                        "local_wire_full_name": "RIOB_X9Y7.LOCAL_OUTPUT0",
                        "local_wire": "LOCAL_OUTPUT0",
                    },
                },
            },
        }

        self.assertEqual(
            route_tile_resource_endpoint(wire, "output"),
            None,
        )
        self.assertEqual(
            route_tile_resource_local(wire, "output"),
            "RIOB_X9Y7/LOCAL_OUTPUT0",
        )

        canonical_wire = {
            "annotation": {
                "from_cb_tile": "INT_R_X9Y8",
                "tile_resource": {
                    "output": {
                        "resource_full_name": "RIOB_X9Y7/INPUT_BUFFER0",
                        "local_wire_full_name": "RIOB_X9Y7.IOI_LOGIC_OUTS18_0",
                        "local_wire": "IOI_LOGIC_OUTS18_0",
                    },
                },
            },
        }
        db = SimpleNamespace(tilegrid={
            "IO_INT_INTERFACE_R_X9Y8": TileInfo(
                "IO_INT_INTERFACE_R_X9Y8",
                "IO_INT_INTERFACE_R",
                0,
                0,
                {},
            ),
        })
        self.assertEqual(
            route_tile_resource_local(canonical_wire, "output", db),
            "IO_INT_INTERFACE_R_X9Y8/INT_INTERFACE_LOGIC_OUTS18",
        )

        left_wire = {
            "annotation": {
                "from_cb_tile": "INT_L_X0Y8",
                "tile_resource": {
                    "output": {
                        "local_wire_full_name": "LIOB_X0Y7.IOI_LOGIC_OUTS18_1",
                        "local_wire": "IOI_LOGIC_OUTS18_1",
                    },
                },
            },
        }
        left_db = SimpleNamespace(tilegrid={
            "IO_INT_INTERFACE_L_X0Y8": TileInfo(
                "IO_INT_INTERFACE_L_X0Y8",
                "IO_INT_INTERFACE_L",
                0,
                0,
                {},
            ),
        })
        self.assertEqual(
            route_tile_resource_local(left_wire, "output", left_db),
            "IO_INT_INTERFACE_L_X0Y8/INT_INTERFACE_LOGIC_OUTS_L18",
        )

    def test_ibuf_source_expands_both_io_sides_and_lanes(self) -> None:
        db = SimpleNamespace(tilegrid={
            "LIOB33_X0Y7": TileInfo(
                "LIOB33_X0Y7", "LIOB33", 0, 7,
                {"IOB_X0Y7": "S", "IOB_X0Y8": "M"},
            ),
            "LIOI3_X0Y7": TileInfo("LIOI3_X0Y7", "LIOI3", 0, 7, {}),
            "RIOB33_X9Y7": TileInfo(
                "RIOB33_X9Y7", "RIOB33", 9, 7,
                {"IOB_X1Y7": "S", "IOB_X1Y8": "M"},
            ),
            "RIOI3_X9Y7": TileInfo("RIOI3_X9Y7", "RIOI3", 9, 7, {}),
        }, pip_sources_by_type_dst={
            ("LIOI3", "IOI_LOGIC_OUTS18_1"): {"IOI_ILOGIC0_O"},
            ("LIOI3", "IOI_LOGIC_OUTS18_0"): {"IOI_ILOGIC1_O"},
            ("RIOI3", "IOI_LOGIC_OUTS18_1"): {"IOI_ILOGIC0_O"},
            ("RIOI3", "IOI_LOGIC_OUTS18_0"): {"IOI_ILOGIC1_O"},
        })

        def wire(resource: str) -> dict[str, object]:
            return {
                "annotation": {
                    "tile_resource": {
                        "output": {"resource_full_name": resource},
                    },
                },
            }

        with (
            patch(
                "db2prj.tileconn_alias_nodes",
                side_effect=lambda _db, node: frozenset({node}),
            ),
            patch("db2prj.route_node_neighbors", return_value=[]),
        ):
            self.assertEqual(
                iob_input_source_nodes(
                    wire("LIOB33_X0Y7.IOB_IBUF0"), db, "IOB_X0Y8"
                ),
                [
                    "LIOB33_X0Y7/IOB_IBUF0",
                    "LIOI3_X0Y7/LIOI_IBUF0",
                    "LIOI3_X0Y7/LIOI_I0",
                    "LIOI3_X0Y7/LIOI_ILOGIC0_D",
                    "LIOI3_X0Y7/IOI_ILOGIC0_O",
                    "LIOI3_X0Y7/IOI_LOGIC_OUTS18_1",
                ],
            )
            self.assertEqual(
                iob_input_source_nodes(
                    wire("RIOB33_X9Y7/IOB_IBUF1"), db, "IOB_X1Y7"
                ),
                [
                    "RIOB33_X9Y7/IOB_IBUF1",
                    "RIOI3_X9Y7/RIOI_IBUF1",
                    "RIOI3_X9Y7/RIOI_I1",
                    "RIOI3_X9Y7/RIOI_ILOGIC1_D",
                    "RIOI3_X9Y7/IOI_ILOGIC1_O",
                    "RIOI3_X9Y7/IOI_LOGIC_OUTS18_0",
                ],
            )

    def test_terminal_leaf_remains_on_main_path(self) -> None:
        expression = route_tree_expression(
            [
                ["ABC_ROOT", "ABC_LEAF"],
                ["ABC_ROOT", "ABC_BRANCH0", "ABC_BRANCH1"],
            ]
        )

        self.assertEqual(
            expression,
            "[list {ABC_ROOT} [list {ABC_BRANCH0} {ABC_BRANCH1}] {ABC_LEAF}]",
        )
        self.assertIsNone(re.search(r"\[list \{[^{}]+\}\]", expression or ""))

    def test_deep_route_tree_does_not_depend_on_python_recursion(self) -> None:
        trunk = [f"ABC_NODE_{index}" for index in range(2500)]
        expression = route_tree_expression([
            trunk + ["ABC_SINK_0"],
            trunk[:1250] + ["ABC_BRANCH", "ABC_SINK_1"],
        ])

        self.assertIsNotNone(expression)
        self.assertIn("{ABC_NODE_2499}", expression or "")
        self.assertIn("{ABC_BRANCH}", expression or "")
        self.assertEqual((expression or "").count("{ABC_NODE_0}"), 1)

    def test_tile_connection_alias_before_pip_is_canonicalized(self) -> None:
        db = SimpleNamespace(
            tilegrid={
                "ABC_X0Y0": TileInfo("ABC_X0Y0", "ABC", 0, 0, {}),
                "DEF_X0Y1": TileInfo("DEF_X0Y1", "DEF", 0, 1, {}),
            },
            pip_sources_by_type_dst={("DEF", "SINK"): {"ALIAS"}},
        )
        neighbors = {("ABC", "SOURCE"): [("DEF", 0, 1, "ALIAS")]}

        with patch("db2prj.tileconn_neighbors_by_source", return_value=neighbors):
            nodes = canonical_fixed_route_nodes(
                ["ABC_X0Y0/SOURCE", "DEF_X0Y1/ALIAS", "DEF_X0Y1/SINK"],
                db,
            )

        self.assertEqual(nodes, ["ABC_X0Y0/SOURCE", "DEF_X0Y1/SINK"])

    def test_packed_output_reconnects_to_existing_trunk(self) -> None:
        nodes = [
            "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
            "INT_X0Y0/LOGIC_OUTS0",
            "INT_X0Y0/STALE_LOGIC_OUTS1",
            "INT_X0Y0/STALE_EXIT",
            "INT_X0Y1/STALE0",
            "INT_X0Y2/STALE1",
            "INT_X0Y3/STALE2",
            "INT_X0Y4/STALE3",
            "INT_X0Y5/STALE4",
            "INT_X0Y6/STALE5",
            "INT_X0Y7/STALE6",
            "INT_X0Y8/STALE7",
            "INT_X1Y0/TRUNK",
            "INT_X2Y0/SINK",
        ]
        neighbors = {
            "INT_X0Y0/LOGIC_OUTS0": ["INT_X0Y0/BRIDGE"],
            "INT_X0Y0/BRIDGE": ["INT_X1Y0/TRUNK"],
        }

        with (
            patch("db2prj.direct_pip_feature", return_value=None),
            patch("db2prj.is_direct_tileconn_edge", return_value=False),
            patch("db2prj.route_node_neighbors", side_effect=lambda _db, node: neighbors.get(node, [])),
        ):
            repaired = repair_clb_output_hops(SimpleNamespace(), nodes)

        self.assertEqual(
            repaired,
            [
                "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
                "INT_X0Y0/LOGIC_OUTS0",
                "INT_X0Y0/BRIDGE",
                "INT_X1Y0/TRUNK",
                "INT_X2Y0/SINK",
            ],
        )

    def test_tile_connection_index_contains_reverse_orientation(self) -> None:
        row = {
            "tile_types": ["ABC", "XYZ"],
            "grid_deltas": [3, -2],
            "wire_pairs": [["ABC_WIRE", "XYZ_WIRE"]],
        }
        db = SimpleNamespace()

        with (
            patch("db2prj.load_tileconn_spec", return_value=[row]),
            patch("db2prj._tileconn_neighbors_by_source_cache", {}),
        ):
            indexed = tileconn_neighbors_by_source(db)

        self.assertEqual(indexed[("ABC", "ABC_WIRE")], [("XYZ", 3, -2, "XYZ_WIRE")])
        self.assertEqual(indexed[("XYZ", "XYZ_WIRE")], [("ABC", -3, 2, "ABC_WIRE")])

    def test_packed_output_bridge_does_not_cross_input_terminal(self) -> None:
        nodes = [
            "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
            "INT_X0Y0/LOGIC_OUTS0",
            "INT_X0Y0/STALE_LOGIC_OUTS1",
            "INT_X0Y0/STALE_EXIT",
            "INT_X1Y0/TRUNK",
        ]
        neighbors = {
            "INT_X0Y0/LOGIC_OUTS0": ["INT_X0Y0/IMUX0"],
            "INT_X0Y0/IMUX0": ["INT_X1Y0/TRUNK"],
        }

        with (
            patch("db2prj.direct_pip_feature", return_value=None),
            patch("db2prj.is_direct_tileconn_edge", return_value=False),
            patch("db2prj.route_node_neighbors", side_effect=lambda _db, node: neighbors.get(node, [])),
        ):
            repaired = repair_clb_output_hops(SimpleNamespace(), nodes)

        self.assertEqual(repaired, nodes)

    def test_packed_output_bridge_avoids_other_route_nodes(self) -> None:
        nodes = [
            "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
            "INT_X0Y0/LOGIC_OUTS0",
            "INT_X0Y0/STALE_LOGIC_OUTS1",
            "INT_X1Y0/TRUNK0",
            "INT_X2Y0/TRUNK1",
        ]
        neighbors = {
            "INT_X0Y0/LOGIC_OUTS0": ["INT_X0Y0/BUSY", "INT_X0Y0/FREE"],
            "INT_X0Y0/BUSY": ["INT_X1Y0/TRUNK0"],
            "INT_X0Y0/FREE": ["INT_X2Y0/TRUNK1"],
        }

        with (
            patch("db2prj.direct_pip_feature", return_value=None),
            patch("db2prj.is_direct_tileconn_edge", return_value=False),
            patch("db2prj.route_node_neighbors", side_effect=lambda _db, node: neighbors.get(node, [])),
        ):
            repaired = repair_clb_output_hops(
                SimpleNamespace(),
                nodes,
                {"INT_X0Y0/BUSY", "INT_X1Y0/TRUNK0"},
            )

        self.assertEqual(
            repaired,
            [
                "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
                "INT_X0Y0/LOGIC_OUTS0",
                "INT_X0Y0/FREE",
                "INT_X2Y0/TRUNK1",
            ],
        )

    def test_legal_packed_output_suffix_is_rebuilt_when_it_uses_busy_node(self) -> None:
        nodes = [
            "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
            "INT_X0Y0/LOGIC_OUTS0",
            "INT_X0Y0/BUSY",
            "INT_X0Y0/TARGET",
        ]
        neighbors = {
            "INT_X0Y0/LOGIC_OUTS0": ["INT_X0Y0/FREE"],
            "INT_X0Y0/FREE": ["INT_X0Y0/TARGET"],
        }

        with (
            patch(
                "db2prj.direct_pip_feature",
                side_effect=lambda _db, src, dst: "direct" if dst == "INT_X0Y0/BUSY" else None,
            ),
            patch("db2prj.is_direct_tileconn_edge", return_value=False),
            patch("db2prj.route_node_neighbors", side_effect=lambda _db, node: neighbors.get(node, [])),
        ):
            repaired = repair_clb_output_hops(
                SimpleNamespace(),
                nodes,
                {"INT_X0Y0/BUSY"},
            )

        self.assertEqual(
            repaired,
            [
                "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
                "INT_X0Y0/LOGIC_OUTS0",
                "INT_X0Y0/FREE",
                "INT_X0Y0/TARGET",
            ],
        )

    def test_packed_output_bridge_can_reconnect_before_later_blocked_node(self) -> None:
        nodes = [
            "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
            "INT_X0Y0/LOGIC_OUTS0",
            "INT_X0Y0/STALE_LOGIC_OUTS1",
            "INT_X1Y0/EARLY_TRUNK",
            "INT_X2Y0/BUSY_LATE",
            "INT_X3Y0/SINK",
        ]
        neighbors = {
            "INT_X0Y0/LOGIC_OUTS0": ["INT_X0Y0/BRIDGE"],
            "INT_X0Y0/BRIDGE": ["INT_X1Y0/EARLY_TRUNK"],
        }

        with (
            patch("db2prj.direct_pip_feature", return_value=None),
            patch("db2prj.is_direct_tileconn_edge", return_value=False),
            patch("db2prj.route_node_neighbors", side_effect=lambda _db, node: neighbors.get(node, [])),
        ):
            repaired = repair_clb_output_hops(
                SimpleNamespace(),
                nodes,
                {"INT_X2Y0/BUSY_LATE"},
            )

        self.assertEqual(
            repaired,
            [
                "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
                "INT_X0Y0/LOGIC_OUTS0",
                "INT_X0Y0/BRIDGE",
                "INT_X1Y0/EARLY_TRUNK",
                "INT_X2Y0/BUSY_LATE",
                "INT_X3Y0/SINK",
            ],
        )

    def test_packed_output_bridge_can_reconnect_after_long_conflict_free_detour(self) -> None:
        nodes = [
            "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
            "INT_X0Y0/LOGIC_OUTS0",
            "INT_X0Y0/STALE_LOGIC_OUTS1",
            "INT_X9Y9/TRUNK",
            "INT_X9Y9/SINK",
        ]
        bridge = [f"INT_X0Y0/STEP{index}" for index in range(27)]
        neighbors = {
            "INT_X0Y0/LOGIC_OUTS0": [bridge[0]],
            **{left: [right] for left, right in zip(bridge, bridge[1:])},
            bridge[-1]: ["INT_X9Y9/TRUNK"],
        }

        with (
            patch("db2prj.direct_pip_feature", return_value=None),
            patch("db2prj.is_direct_tileconn_edge", return_value=False),
            patch("db2prj.route_node_neighbors", side_effect=lambda _db, node: neighbors.get(node, [])),
        ):
            repaired = repair_clb_output_hops(SimpleNamespace(), nodes)

        self.assertEqual(
            repaired,
            [
                "RESOURCE_X0Y0/RESOURCE_LOGIC_OUTS0",
                "INT_X0Y0/LOGIC_OUTS0",
                *bridge,
                "INT_X9Y9/TRUNK",
                "INT_X9Y9/SINK",
            ],
        )

    def test_tile_connection_alias_chain_collapses_to_one_node(self) -> None:
        db = SimpleNamespace(
            tilegrid={
                "ABC_X0Y0": TileInfo("ABC_X0Y0", "ABC", 0, 0, {}),
                "XYZ_X1Y0": TileInfo("XYZ_X1Y0", "XYZ", 1, 0, {}),
                "QRS_X2Y0": TileInfo("QRS_X2Y0", "QRS", 2, 0, {}),
            },
            pip_sources_by_type_dst={},
        )
        neighbors = {
            ("ABC", "WIRE0"): [("XYZ", 1, 0, "WIRE1")],
            ("XYZ", "WIRE1"): [("ABC", -1, 0, "WIRE0"), ("QRS", 1, 0, "WIRE2")],
            ("QRS", "WIRE2"): [("XYZ", -1, 0, "WIRE1")],
        }

        with patch("db2prj.tileconn_neighbors_by_source", return_value=neighbors):
            nodes = canonical_fixed_route_nodes(
                ["ABC_X0Y0/WIRE0", "XYZ_X1Y0/WIRE1", "QRS_X2Y0/WIRE2"],
                db,
            )

        self.assertEqual(nodes, ["ABC_X0Y0/WIRE0"])

    def test_break_tile_end_node_is_retained(self) -> None:
        db = SimpleNamespace(
            tilegrid={
                "INT_X0Y0": TileInfo("INT_X0Y0", "INT", 0, 0, {}),
                "BRKH_INT_X0Y1": TileInfo("BRKH_INT_X0Y1", "BRKH", 0, 1, {}),
            },
            pip_sources_by_type_dst={},
        )

        with patch("db2prj.tileconn_neighbors_by_source", return_value={}):
            nodes = canonical_fixed_route_nodes(
                ["INT_X0Y0/SOURCE", "BRKH_INT_X0Y1/ABC_END0", "INT_X0Y0/SINK"],
                db,
            )

        self.assertEqual(
            nodes,
            ["INT_X0Y0/SOURCE", "BRKH_INT_X0Y1/ABC_END0", "INT_X0Y0/SINK"],
        )


if __name__ == "__main__":
    unittest.main()
