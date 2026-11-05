import contextlib
import io
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

from cleanup_logs import cleanup, truncate_log


class LogRetentionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="scalepnr-log-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "repo"
        self.root.mkdir()
        self.tmp = Path(self.temp.name) / "tmp"
        self.tmp.mkdir()
        subprocess.run(["git", "init", "-q", str(self.root)], check=True)
        self.now = time.time()

    def make_file(self, path, age=3 * 86400):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("test data\n")
        os.utime(path, (self.now - age, self.now - age))
        return path

    def test_expiration_across_folders_preserves_live_and_nonlog_files(self):
        expired = [self.make_file(self.root / "build/old.log"),
                   self.make_file(self.root / "tests/runs/old.log.xz"),
                   self.make_file(self.root / "other/routing_failure_tiles.txt"),
                   self.make_file(self.tmp / "scalepnr-case/old.log.gz")]
        preserved = [self.make_file(self.root / "recent.log", age=60),
                     self.make_file(self.root / "boundary.log", age=2 * 86400),
                     self.make_file(self.root / "recent.log.xz", age=60),
                     self.make_file(self.root / "design.db"),
                     self.make_file(self.root / "source.cpp"),
                     self.make_file(self.root / "tracked.log"),
                     self.make_file(self.tmp / "unrelated/old.log"),
                     self.make_file(self.root / "tool/.git/config"),
                     self.make_file(self.root / "tool/fixture.log")]
        subprocess.run(["git", "-C", str(self.root), "add", "tracked.log"], check=True)
        active = self.make_file(self.root / "active.log")
        outside = self.make_file(Path(self.temp.name) / "outside.log")
        (self.root / "link.log").symlink_to(outside)
        (self.root / "linked_dir").symlink_to(self.tmp, target_is_directory=True)
        with active.open() as opened, contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(opened.closed)
            self.assertEqual(cleanup(self.root, self.tmp, now=self.now)[0], 4)
            self.assertTrue(all(p.exists() for p in expired))  # Dry run is nondestructive.
            self.assertEqual(cleanup(self.root, self.tmp, True, self.now)[0], 4)
            self.assertTrue(active.exists())  # Open logs survive even with an old mtime.
        self.assertTrue(all(not p.exists() for p in expired))
        self.assertTrue(all(p.exists() for p in preserved))
        self.assertTrue(outside.exists())  # Never follow symlinks outside the cleanup scope.

    def test_runner_writes_plain_stdout_and_preserves_failure_status(self):
        scripts = self.root / "tests/prjxray"
        scripts.mkdir(parents=True)
        for name in (".run_50k_test.sh", "cleanup_logs.py"):
            shutil.copyfile(Path(__file__).parent / name, scripts / name)
        fake = self.root / "fake-router"
        fake.write_text(
            "#!/bin/sh\n"
            "test -f /proc/$$/fd/1 || exit 98\n"
            "test -f /proc/$$/fd/2 || exit 99\n"
            "printf 'native stdout\\n'\nprintf 'native stderr\\n' >&2\nexit 7\n")
        fake.chmod(0o755)
        output = self.root / "tests/prjxray/runs/smoke"
        env = dict(os.environ, SCALEPNR_BINARY=str(fake), TMPDIR=str(self.tmp),
                   SCALEPNR_DIAGNOSTIC_RUN_DIR=str(output), SCALEPNR_PROCESS_TIMEOUT="5",
                   SCALEPNR_MIN_AVAILABLE_KB="1", SCALEPNR_MIN_DISK_KB="1",
                   SCALEPNR_MAX_LOG_BYTES="268435456")
        for key in ("SCALEPNR_CPU_PROFILE", "SCALEPNR_HEAP_SAMPLE", "SCALEPNR_MEMORY_GDB"):
            env.pop(key, None)
        result = subprocess.run(["bash", str(scripts / ".run_50k_test.sh")],
                                env=env, capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 7, result.stdout + result.stderr)
        log = (output / "run.log").read_text()
        self.assertIn("native stdout\nnative stderr\n", log)
        self.assertIn("exit=7", log)
        self.assertEqual([p.name for p in output.iterdir()], ["run.log"])
        # A repeated directory must fail instead of truncating the previous evidence.
        result = subprocess.run(["bash", str(scripts / ".run_50k_test.sh")],
                                env=env, capture_output=True, timeout=15)
        self.assertEqual(result.returncode, 2)
        self.assertEqual((output / "run.log").read_text(), log)

        # Default directories include a numeric process ID, not a literal dollar sign.
        env.pop("SCALEPNR_DIAGNOSTIC_RUN_DIR")
        result = subprocess.run(["bash", str(scripts / ".run_50k_test.sh")],
                                env=env, capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 7, result.stdout + result.stderr)
        runs = list(output.parent.glob("50k_*"))
        self.assertEqual(len(runs), 1)
        self.assertRegex(runs[0].name, r"^50k_\d{8}_\d{6}_\d+$")
        self.assertIn("native stdout\nnative stderr\n", (runs[0] / "run.log").read_text())

        # Finalization caps the completed log while retaining start/end and exit status.
        fake.write_text("#!/bin/sh\nprintf 'NATIVE_HEAD_123\\n'\nhead -c 65536 /dev/zero\n"
                        "printf '\\nTAIL\\n'\nexit 7\n")
        env["SCALEPNR_DIAGNOSTIC_RUN_DIR"] = str(output.parent / "large")
        env["SCALEPNR_MAX_LOG_BYTES"] = "4096"
        result = subprocess.run(["bash", str(scripts / ".run_50k_test.sh")],
                                env=env, capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 7, result.stdout + result.stderr)
        capped = (output.parent / "large/run.log").read_bytes()
        self.assertEqual(len(capped), 4096)
        self.assertIn(b"NATIVE_HEAD_123\n", capped)
        self.assertIn(b"LOG_TRUNCATED", capped)
        self.assertIn(b"TAIL\n", capped)
        self.assertIn(b"exit=7", capped)

        # An unavailable disk budget stops the process group, not just the wrapper.
        fake.write_text("#!/bin/sh\nexec sleep 30\n")
        env["SCALEPNR_DIAGNOSTIC_RUN_DIR"] = str(output.parent / "disk_guard")
        env["SCALEPNR_MIN_DISK_KB"] = "1000000000000"
        result = subprocess.run(["bash", str(scripts / ".run_50k_test.sh")],
                                env=env, capture_output=True, text=True, timeout=15)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("DISK_GUARD", (output.parent / "disk_guard/run.log").read_text())

    def test_truncation_preserves_head_tail_and_refuses_active_logs(self):
        path = self.root / "completed.log"
        original = bytes(range(256)) * 20000
        path.write_bytes(original)
        with path.open():
            with self.assertRaisesRegex(ValueError, "still open"):
                truncate_log(path, 4096)
        self.assertEqual(path.stat().st_size, len(original))
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(truncate_log(path, 4096), len(original) - 4096)
        retained = path.read_bytes()
        self.assertEqual(len(retained), 4096)
        self.assertEqual(retained[:1900], original[:1900])
        self.assertEqual(retained[-1900:], original[-1900:])
        self.assertEqual(truncate_log(path, 4096), 0)
        # Tiny trims exercise overlapping source/destination copy ranges.
        path.write_bytes(original)
        with contextlib.redirect_stdout(io.StringIO()):
            truncate_log(path, len(original) - 1)
        self.assertEqual(path.read_bytes()[-2000000:], original[-2000000:])
        link = self.root / "link.log"
        link.symlink_to(path)
        with self.assertRaises(OSError):
            truncate_log(link, 4096)

    @unittest.skipUnless(Path("/usr/lib/x86_64-linux-gnu/libprofiler.so.0").exists(),
                         "optional CPU profiler not installed")
    def test_profiler_survives_line_buffer_preload(self):
        scripts = self.root / "tests/prjxray"
        scripts.mkdir(parents=True)
        for name in (".run_50k_test.sh", "cleanup_logs.py"):
            shutil.copyfile(Path(__file__).parent / name, scripts / name)
        fake = self.root / "fake-router"
        fake.write_text(f"#!{sys.executable}\n"
                        "from pathlib import Path\n"
                        "maps=Path('/proc/self/maps').read_text()\n"
                        "assert 'libprofiler.so' in maps, 'profiler preload lost'\n"
                        "assert 'libstdbuf.so' in maps, 'line buffering preload lost'\n"
                        "print('PROFILE_PRELOAD_OK')\n")
        fake.chmod(0o755)
        output = self.root / "tests/prjxray/runs/profile"
        env = dict(os.environ, SCALEPNR_BINARY=str(fake), TMPDIR=str(self.tmp),
                   SCALEPNR_DIAGNOSTIC_RUN_DIR=str(output), SCALEPNR_PROCESS_TIMEOUT="5",
                   SCALEPNR_MIN_AVAILABLE_KB="1", SCALEPNR_MIN_DISK_KB="1",
                   SCALEPNR_CPU_PROFILE=str(output / "cpu.prof"))
        for key in ("SCALEPNR_HEAP_SAMPLE", "SCALEPNR_MEMORY_GDB"):
            env.pop(key, None)
        result = subprocess.run(["bash", str(scripts / ".run_50k_test.sh")],
                                env=env, capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr +
                         (output / "run.log").read_text())
        self.assertIn("PROFILE_PRELOAD_OK", (output / "run.log").read_text())


if __name__ == "__main__":
    unittest.main()
