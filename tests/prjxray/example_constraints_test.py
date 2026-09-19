import copy
import unittest

from example_constraints import constraints


class ConstraintsTest(unittest.TestCase):
    def setUp(self):
        self.design = {"modules": {"top": {"ports": {
            "clk": {"direction": "input", "bits": [2]},
            "data": {"direction": "output", "bits": [3, 4], "offset": 2},
        }, "cells": {"lut": {"type": "LUT2"}}}}}
        self.pins = [dict(pin=pin, site=f"IOB_X0Y{i}", tile="LIOB33_X0Y0",
                          pin_function=func) for i, (pin, func) in enumerate([
                              ("A1", "IO_DATA"), ("B1", "IO_MRCC"), ("C1", "IO_DATA")])]

    def generate(self, period=10):
        return constraints(self.design, "top", "clk", period, self.pins)

    def test_unique_pins_clock_and_bus_indices(self):
        result = self.generate()
        self.assertIn("PACKAGE_PIN B1 [get_ports {clk}]", result)
        self.assertIn("PACKAGE_PIN A1 [get_ports {data[2]}]", result)
        self.assertIn("PACKAGE_PIN C1 [get_ports {data[3]}]", result)
        self.assertEqual(result.count("PACKAGE_PIN"), 3)

    def test_deterministic_without_mutation(self):
        original = copy.deepcopy(self.design)
        expected = self.generate()
        self.pins.reverse()
        self.assertEqual(expected, self.generate())
        self.assertEqual(original, self.design)

    def test_capacity_exhaustion(self):
        self.pins.pop()
        with self.assertRaisesRegex(ValueError, "exceed"):
            self.generate()

    def test_unsupported_macro(self):
        self.design["modules"]["top"]["cells"]["lut"]["type"] = "EHXPLLL"
        with self.assertRaisesRegex(ValueError, "unsupported"):
            self.generate()

    def test_missing_clock_pin(self):
        self.pins[1]["pin_function"] = "IO_DATA"
        with self.assertRaisesRegex(ValueError, "MRCC"):
            self.generate()

    def test_missing_clock_port(self):
        del self.design["modules"]["top"]["ports"]["clk"]
        with self.assertRaisesRegex(ValueError, "scalar input"):
            self.generate()

    def test_invalid_period(self):
        for period in (0, -1, float("nan"), float("inf")):
            with self.subTest(period=period), self.assertRaisesRegex(ValueError, "period"):
                self.generate(period)

    def test_reject_bidirectional_io(self):
        self.design["modules"]["top"]["ports"]["data"]["direction"] = "inout"
        with self.assertRaisesRegex(ValueError, "unidirectional"):
            self.generate()


if __name__ == "__main__":
    unittest.main()
