"""Exercise the real runner's timeout policy without synthesis or routing."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


class ExampleTimeoutTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.scripts = self.root / "tests/prjxray"
        self.scripts.mkdir(parents=True)
        shutil.copyfile(Path(__file__).with_name("build_example.sh"),
                        self.scripts / "build_example.sh")
        for name in ("db/tilegrid.json", "db/tileconn.json", "db/package_pins.csv",
                     "prjxray-db/artix7/xc7a100tfgg676-1/part.yaml",
                     "prjxray-db/artix7/xc7a100tfgg676-1/package_pins.csv",
                     "prjxray-db/artix7/xc7a100t/tilegrid.json",
                     "prjxray/utils/fasm2frames.py"):
            path = self.scripts / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("fixture\n")
        example = self.scripts / "1"
        example.mkdir()
        (example / "example.conf").write_text("top=probe\nclock_port=clk\nclock_period=10\n")
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.trace = self.root / "commands.jsonl"
        self.tool("timeout", """import json, os, sys
with open(os.environ['TIMEOUT_TRACE'], 'a') as out:
    out.write(json.dumps(sys.argv[1:]) + '\\n')
os.execv(sys.argv[3], sys.argv[3:])
""")
        self.tool("yosys", "print('bufnorm [options]')\n")
        self.tool("python", "pass\n")
        self.tool("scalepnr", "import os, sys\nprint('route_budget=' + os.environ['SCALEPNR_ROUTE_STAGE_TIMEOUT'])\nsys.exit(37)\n")
        converter = self.scripts / "prjxray/build/tools/xc7frames2bit"
        converter.parent.mkdir(parents=True)
        converter.write_text("#!/bin/sh\nexit 0\n")
        converter.chmod(0o755)

    def tool(self, name, body):
        path = self.bin / name
        path.write_text(f"#!{sys.executable}\n{body}")
        path.chmod(0o755)

    def run_example(self, **overrides):
        env = {k: v for k, v in os.environ.items() if not k.startswith("SCALEPNR_")}
        env.update(PATH=str(self.bin) + os.pathsep + env["PATH"],
                   YOSYS=str(self.bin / "yosys"), PYTHON=str(self.bin / "python"),
                   SCALEPNR=str(self.bin / "scalepnr"), TIMEOUT_TRACE=str(self.trace))
        env.update(overrides)
        result = subprocess.run(["bash", str(self.scripts / "build_example.sh"), "1"],
                                env=env, capture_output=True, text=True, timeout=10)
        commands = [json.loads(s) for s in self.trace.read_text().splitlines()] if self.trace.exists() else []
        return result, commands

    def test_independent_default_routing_budget_and_failure_propagation(self):
        result, commands = self.run_example()
        self.assertEqual(result.returncode, 37, result.stderr)
        self.assertEqual([c[1] for c in commands], ["600s"] * 4 + ["0s"])
        self.assertIn("route_budget=600", result.stderr)
        self.assertFalse(list(self.scripts.glob("1/build/run.*/design.bit")))

    def test_explicit_whole_pnr_cap_is_separate(self):
        result, commands = self.run_example(SCALEPNR_EXAMPLE_TIMEOUT="12",
                                           SCALEPNR_EXAMPLE_PNR_TIMEOUT="2400",
                                           SCALEPNR_ROUTE_STAGE_TIMEOUT="17")
        self.assertEqual(result.returncode, 37, result.stderr)
        self.assertEqual([c[1] for c in commands], ["12s"] * 4 + ["2400s"])
        self.assertIn("route_budget=17", result.stderr)

    def test_invalid_whole_pnr_timeout_fails_before_execution(self):
        result, commands = self.run_example(SCALEPNR_EXAMPLE_PNR_TIMEOUT="-1")
        self.assertEqual(result.returncode, 2)
        self.assertIn("SCALEPNR_EXAMPLE_PNR_TIMEOUT", result.stderr)
        self.assertEqual(commands, [])


if __name__ == "__main__":
    unittest.main()
