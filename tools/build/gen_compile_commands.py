#!/usr/bin/env python3
"""Generate compile_commands.json for clangd.

    python tools/build/gen_compile_commands.py --raylib-cflags "$(pkg-config --cflags raylib)"

Bear cannot intercept the MSYS2 build from a Windows shell, and a wrong compilation database
is worse than none: clangd then reports errors that the real build does not have. So the
database is generated from the same source manifest the build scripts link, which lives in the
Makefile and is read back out through `make print-source-groups`.

This used to keep its own hand-maintained copy of the source lists. By the time it was
replaced that copy had drifted: it was missing twelve translation units that the real build
compiles, and `-Ithird_party`, so clangd was quietly wrong about exactly the files nobody had
opened recently. Reading the manifest is the whole point of this rewrite.

Each translation unit gets the flags of the configuration it is actually built in — the game
module sees CIRCUIT_DEV_TOOLS, the platform layer does not, the tests see CIRCUIT_HEADLESS.
"""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# tools/build/gen_compile_commands.py -> tools/build -> tools -> repository root.
# This script alone resolves the root this way: it shells out to make and writes the root
# compilation database, so it cannot use the caller's working directory.
REPO_ROOT = Path(__file__).resolve().parents[2]

BASE_FLAGS = [
    "-std=c11", "-Isrc", "-Itests", "-Ithird_party", "-Ithird_party/raygui",
    "-Wall", "-Wextra", "-Wshadow", "-Wstrict-prototypes", "-Wmissing-prototypes",
    "-Wpointer-arith",
]

CONFIGURATIONS: Dict[str, List[str]] = {
    "module": ["-DCIRCUIT_HOT_RELOAD", "-DCIRCUIT_GAME_MODULE", "-DCIRCUIT_DEV_TOOLS", "-O0", "-g"],
    "platform": ["-DCIRCUIT_HOT_RELOAD", "-O0", "-g"],
    "tests": ["-DCIRCUIT_HEADLESS", "-O2", "-DNDEBUG"],
    "fuzz": ["-DCIRCUIT_HEADLESS", "-O1", "-g"],
}

# First match wins, so the order encodes which configuration is the truthful one for a file
# that several groups link. src/platform/timestep.c is platform code that the tests happen to link;
# SHARED_SRCS is module code that the harness happens to link.
GROUP_CONFIGURATION: List[Tuple[str, List[str]]] = [
    ("module", ["GAME_SRCS", "DEV_UI_SRCS", "SHARED_SRCS"]),
    ("platform", ["PLATFORM_SRCS", "HOTRELOAD_SRC", "HOTRELOAD_HARNESS_SRCS"]),
    ("tests", ["TEST_RUNNER_SRCS", "TEST_SRCS"]),
]


def read_source_groups() -> Dict[str, List[str]]:
    """Return the Makefile source manifest as {group name: [paths]}."""
    completed = subprocess.run(
        ["make", "--no-print-directory", "-s", "print-source-groups"],
        cwd=str(REPO_ROOT), capture_output=True, text=True,
    )
    if completed.returncode != 0:
        sys.stderr.write(completed.stderr)
        raise SystemExit("gen_compile_commands: could not read the source manifest from make")

    groups: Dict[str, List[str]] = {}
    for line in completed.stdout.splitlines():
        line = line.strip()
        if not line or "=" not in line:
            continue
        name, _, value = line.partition("=")
        # `print-source-groups` emits NAME='a b c'. shlex removes the quoting but returns the
        # whole quoted run as ONE token, so split each token on whitespace afterwards.
        groups[name.strip()] = [
            path for token in shlex.split(value) for path in token.split()
        ]
    if not groups:
        raise SystemExit("gen_compile_commands: the source manifest came back empty")
    return groups


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--raylib-cflags", default="",
                        help="output of `pkg-config --cflags raylib`")
    parser.add_argument("--compiler", default="gcc")
    parser.add_argument("--output", default="compile_commands.json",
                        help="relative paths resolve against the repository root")
    args = parser.parse_args(argv)

    groups = read_source_groups()
    raylib_flags = shlex.split(args.raylib_cflags.replace("\\", "/"))

    assigned: List[Tuple[str, str]] = []
    seen = set()
    for configuration, group_names in GROUP_CONFIGURATION:
        for group_name in group_names:
            for path in groups.get(group_name, []):
                if path in seen:
                    continue
                seen.add(path)
                assigned.append((path, configuration))

    for path in sorted(p.as_posix() for p in (REPO_ROOT / "fuzz").glob("*.c")):
        relative = path[len(REPO_ROOT.as_posix()) + 1:]
        if relative not in seen:
            seen.add(relative)
            assigned.append((relative, "fuzz"))

    entries = []
    missing = []
    for path, configuration in assigned:
        if not (REPO_ROOT / path).exists():
            missing.append(path)
            continue
        arguments = [args.compiler] + BASE_FLAGS + raylib_flags
        arguments += CONFIGURATIONS[configuration]
        arguments += ["-c", path]
        entries.append({
            "directory": REPO_ROOT.as_posix(),
            "file": path,
            "arguments": arguments,
        })

    output = Path(args.output)
    if not output.is_absolute():
        output = REPO_ROOT / output
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        json.dump(entries, handle, indent=2)
        handle.write("\n")

    print("wrote %s (%d translation units)" % (output.as_posix(), len(entries)))
    if missing:
        # A manifest entry with no file on disk is a real error in the manifest, not a
        # cosmetic one: the build would fail on it too.
        sys.stderr.write("gen_compile_commands: manifest names %d missing file(s): %s\n"
                         % (len(missing), " ".join(missing)))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
