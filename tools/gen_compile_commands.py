#!/usr/bin/env python3
"""Generate compile_commands.json for clangd.

    python tools/gen_compile_commands.py --raylib-cflags "$(pkg-config --cflags raylib)"

Bear cannot intercept the MSYS2 build from a Windows shell, and a wrong compilation database
is worse than none: clangd then reports errors that the real build does not have. So the
database is generated from the same source lists and flags the build scripts use, which is
the only place they are defined.

Each translation unit gets the flags of the configuration it is actually built in — the game
module sees DRIFTY_DEV_TOOLS, the platform layer does not, the tests see DRIFTY_HEADLESS.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import sys
from typing import Dict, List, Optional

SHARED = ["src/input.c", "src/math_utils.c", "src/dev_scenario.c", "src/profile.c"]
DEV = ["src/dev_params.c", "src/dev_replay.c", "src/dev_state.c", "src/failure_bundle.c"]
GAME = [
    "src/game.c", "src/vehicle.c", "src/physics.c", "src/tire.c", "src/drivetrain.c",
    "src/render.c", "src/replay.c", "src/telemetry.c",
] + DEV
PLATFORM = ["src/main.c", "src/timestep.c", "src/hotreload_windows.c"]
MODULE_UI = ["src/dev_lab.c"]
TESTS = ["tests/physics_tests.c", "tests/hotreload_harness.c"]
FUZZ = ["fuzz/fuzz_profile.c", "fuzz/fuzz_replay.c", "fuzz/fuzz_tire.c"]

BASE_FLAGS = [
    "-std=c11", "-Isrc", "-Ithird_party/raygui",
    "-Wall", "-Wextra", "-Wshadow", "-Wstrict-prototypes", "-Wmissing-prototypes",
    "-Wpointer-arith",
]

CONFIGURATIONS: Dict[str, List[str]] = {
    "module": ["-DDRIFTY_HOT_RELOAD", "-DDRIFTY_GAME_MODULE", "-DDRIFTY_DEV_TOOLS", "-O0", "-g"],
    "platform": ["-DDRIFTY_HOT_RELOAD", "-O0", "-g"],
    "tests": ["-DDRIFTY_HEADLESS", "-O2", "-DNDEBUG"],
}

FILE_CONFIGURATION = (
    [(path, "module") for path in GAME + MODULE_UI + SHARED]
    + [(path, "platform") for path in PLATFORM]
    + [(path, "tests") for path in TESTS + FUZZ]
)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--raylib-cflags", default="",
                        help="output of `pkg-config --cflags raylib`")
    parser.add_argument("--compiler", default="gcc")
    parser.add_argument("--output", default="compile_commands.json")
    args = parser.parse_args(argv)

    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    raylib_flags = shlex.split(args.raylib_cflags.replace("\\", "/"))

    entries = []
    seen = set()
    for path, configuration in FILE_CONFIGURATION:
        if path in seen:
            continue
        seen.add(path)
        if not os.path.exists(os.path.join(root, path)):
            continue
        flags = [args.compiler] + BASE_FLAGS + raylib_flags + CONFIGURATIONS[configuration]
        flags += ["-c", path]
        entries.append({
            "directory": root.replace("\\", "/"),
            "file": path,
            "arguments": flags,
        })

    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(entries, handle, indent=2)
        handle.write("\n")

    print("wrote %s (%d translation units)" % (args.output, len(entries)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
