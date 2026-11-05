#!/usr/bin/env python3
"""Expire generated logs after 48 hours, without archiving them."""

import argparse
import fnmatch
import os
from pathlib import Path
import stat
import subprocess
import time


LOG_PATTERNS = (
    "*.log", "*.log.*", "*.jou", "*.stdout", "*.stderr", "*.trace",
    "routing_failure*.txt", "routing_forward_search*.txt",
    "basic_timeout_state.txt", "*_attempts.txt", "cpu.prof*", "perf.data*",
)


def truncate_log(path, max_bytes=256 * 1024 * 1024):
    """Keep a completed log's head and tail in place, without an archive."""
    marker = b"\nLOG_TRUNCATED: middle removed after execution; head and tail retained\n"
    if max_bytes <= len(marker):
        raise ValueError("log limit must leave room for the truncation marker")
    opened = open_files()
    fd = os.open(path, os.O_RDWR | os.O_NOFOLLOW)
    with os.fdopen(fd, "r+b", buffering=0) as log:
        info = os.fstat(log.fileno())
        if not stat.S_ISREG(info.st_mode):
            raise ValueError("only regular log files may be truncated")
        if (info.st_dev, info.st_ino) in opened:
            raise ValueError("refusing to truncate a log still open by another process")
        if info.st_size <= max_bytes:
            return 0
        head = (max_bytes - len(marker)) // 2
        tail = max_bytes - len(marker) - head
        source, destination = info.st_size - tail, head + len(marker)
        # Copy toward the beginning in bounded chunks, preserving unread bytes.
        while tail:
            log.seek(source)
            data = log.read(min(tail, 1024 * 1024))
            if not data:
                raise OSError("log changed while truncating")
            log.seek(destination)
            log.write(data)
            source += len(data)
            destination += len(data)
            tail -= len(data)
        log.seek(head)
        log.write(marker)
        log.truncate(max_bytes)
    freed = info.st_size - max_bytes
    print(f"LOG_TRUNCATED bytes_freed={freed} retained_bytes={max_bytes} path={path}")
    return freed


def open_files():
    """Protect logs still open by a process owned by this user."""
    result = set()
    for process in Path("/proc").glob("[0-9]*"):
        try:
            if process.stat().st_uid != os.getuid():
                continue
            for descriptor in (process / "fd").iterdir():
                try:
                    info = descriptor.stat()
                    result.add((info.st_dev, info.st_ino))
                except OSError:
                    pass
        except OSError:
            continue
    return result


def cleanup(root, tmp_root, apply=False, now=None, extra_roots=(), verbose=True):
    root = root.resolve()
    tracked = subprocess.run(
        ["git", "-C", str(root), "ls-files", "-z"],
        check=True, capture_output=True,
    ).stdout.split(b"\0")
    protected = {root / os.fsdecode(name) for name in tracked if name}
    cutoff = (time.time() if now is None else now) - 2 * 24 * 60 * 60
    opened = open_files()
    roots = [root, *extra_roots]
    roots.extend(tmp_root.glob("scalepnr-*"))
    visited = set()
    count = size = 0
    for base in roots:
        if base.is_symlink():
            continue
        if base.is_dir():
            entries = []
            for directory, dirs, files in os.walk(base, followlinks=False):
                # Nested repositories belong to their own tools, not this cleanup.
                if Path(directory) != root and ".git" in dirs + files:
                    dirs[:] = []
                    continue
                dirs[:] = [d for d in dirs if d != ".git"
                           and not (Path(directory) / d).is_symlink()]
                entries.extend(Path(directory) / name for name in files)
        else:
            entries = [base]
        for path in entries:
            path = path.absolute()
            if path in visited or path in protected:
                continue
            visited.add(path)
            if not any(fnmatch.fnmatchcase(path.name, p) for p in LOG_PATTERNS):
                continue
            try:
                info = path.lstat()
                if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                        or info.st_mtime >= cutoff
                        or (info.st_dev, info.st_ino) in opened):
                    continue
                if apply:
                    path.unlink()
                count += 1
                size += info.st_size
                if verbose:
                    print(f"{'DELETE' if apply else 'EXPIRED'} {info.st_size} {path}")
            except FileNotFoundError:
                continue
    print(f"LOG_CLEANUP files={count} bytes={size} applied={int(apply)} retention_hours=48")
    return count, size


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--tmp-root", type=Path, default=Path("/tmp"))
    parser.add_argument("--extra-root", type=Path, action="append", default=[])
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--truncate-log", type=Path, action="append", default=[])
    parser.add_argument("--max-log-bytes", type=int, default=256 * 1024 * 1024)
    args = parser.parse_args()
    if args.apply:
        for path in args.truncate_log:
            truncate_log(path, args.max_log_bytes)
    cleanup(args.root, args.tmp_root, args.apply, extra_roots=args.extra_root,
            verbose=not args.apply or args.verbose)
