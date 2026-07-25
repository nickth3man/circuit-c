#!/bin/sh
#
# build.sh — canonical Drifty build, executed inside MSYS2 UCRT64.
#
# Rebuilds the game module every time, and rebuilds drifty.exe only when it is not already
# running. It always terminates in well under a second and returns the compiler's exit
# status. It never launches or supervises drifty.exe: the developer starts that once and
# leaves it open. Use --smoke-test for a bounded visual run that exits on its own.
#
# A failed compile leaves the previous, working module untouched — the link output goes to a
# temporary name and is only renamed into place on success — so a compile error can never
# close the running game.
#
#   ./build.sh              build the module (and the exe if it is not running)
#   ./build.sh --release    single executable; raylib linked statically (no libraylib.dll)
#   ./build.sh --tests      headless test executable
#   ./build.sh --hotreload-harness  windowless hot-reload validation executable
#   ./build.sh --smoke-test build (if needed) and run drifty.exe --smoke-test
#   ./build.sh --clean      remove generated artifacts
#
# From cmd.exe / PowerShell prefer build.bat, which enters the UCRT64 environment for you.
#
set -u

cd "$(dirname "$0")" || exit 1
ROOT="$(pwd)"

# ---------------------------------------------------------------------------- environment --

if [ "${MSYSTEM:-}" != "UCRT64" ] || [ "${MINGW_PREFIX:-}" != "/ucrt64" ]; then
    echo "build.sh: must run under MSYS2 UCRT64 (MSYSTEM=UCRT64, MINGW_PREFIX=/ucrt64)." >&2
    echo "  From Windows:  build.bat" >&2
    echo "  Or open the   MSYS2 UCRT64 shell and re-run ./build.sh" >&2
    exit 127
fi

CC="${CC:-gcc}"
if ! command -v "$CC" >/dev/null 2>&1; then
    echo "build.sh: '$CC' not found on PATH. Run scripts/setup_windows.ps1." >&2
    exit 127
fi

CC_PATH="$(command -v "$CC")"
case "$CC_PATH" in
    */ucrt64/bin/gcc|*/ucrt64/bin/gcc.exe) ;;
    *)
        echo "build.sh: refusing non-UCRT64 compiler '$CC_PATH'." >&2
        echo "  Use build.bat or the MSYS2 UCRT64 shell. Chocolatey/MinGW PATH entries are unsupported." >&2
        exit 127
        ;;
esac

if ! command -v pkg-config >/dev/null 2>&1; then
    echo "build.sh: pkg-config not found. Run scripts/setup_windows.ps1." >&2
    exit 127
fi

PKG_PATH="$(command -v pkg-config)"
case "$PKG_PATH" in
    */ucrt64/bin/pkg-config|*/ucrt64/bin/pkg-config.exe|*/ucrt64/bin/pkgconf|*/ucrt64/bin/pkgconf.exe) ;;
    *)
        echo "build.sh: refusing non-UCRT64 pkg-config '$PKG_PATH'." >&2
        exit 127
        ;;
esac

if ! pkg-config --exists raylib; then
    echo "build.sh: pkg-config cannot find raylib. Run scripts/setup_windows.ps1." >&2
    exit 127
fi

# ------------------------------------------------------------------------------- raylib --
#
# Development / hot reload: shared libraylib.dll (MSYS2 name). Both drifty.exe and
# build/game.dll must import the same DLL so raylib global state is not duplicated.
#
# Release: static libraylib.a. The MSYS2 static archive still references shared GLFW
# (__imp_glfw*), so glfw3.dll remains a runtime dependency of the release executable.
# That is a package limitation, not a project fallback to vendored raylib.

RAYLIB_CFLAGS="$(pkg-config --cflags raylib)"

# Prefer the import library explicitly so -lraylib cannot quietly pick libraylib.a.
RAYLIB_SHARED_LIBS="${MINGW_PREFIX}/lib/libraylib.dll.a -lopengl32 -lgdi32 -lwinmm"
RAYLIB_STATIC_LIBS="-l:libraylib.a -lglfw3 -lopengl32 -lgdi32 -lwinmm"

RAYLIB_SHARED_DLL="${MINGW_PREFIX}/bin/libraylib.dll"
GLFW_SHARED_DLL="${MINGW_PREFIX}/bin/glfw3.dll"

# -------------------------------------------------------------------------------- flags --

CSTD="-std=c11"
INCLUDES="-Isrc"
WARNINGS="-Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith"
DEBUG_FLAGS="-O0 -g"
RELEASE_FLAGS="-O2 -DNDEBUG"

SHARED_SRCS="src/input.c src/math_utils.c"
GAME_SRCS="src/game.c src/vehicle.c src/physics.c src/render.c src/replay.c src/telemetry.c"
PLATFORM_SRCS="src/main.c src/timestep.c"
HOTRELOAD_SRC="src/hotreload_windows.c"
TEST_SRCS="tests/physics_tests.c src/timestep.c $GAME_SRCS $SHARED_SRCS"
HOTRELOAD_HARNESS_SRCS="tests/hotreload_harness.c src/timestep.c $HOTRELOAD_SRC $SHARED_SRCS"

MODULE="build/game.dll"
EXE="drifty.exe"
EXE_RELEASE="drifty_release.exe"
EXE_TESTS="drifty_tests.exe"
EXE_HOTRELOAD="drifty_hotreload_harness.exe"

mkdir -p build telemetry

copy_dev_runtime() {
    # Development binaries import libraylib.dll, which itself imports glfw3.dll.
    if [ -f "$RAYLIB_SHARED_DLL" ]; then
        cp -f "$RAYLIB_SHARED_DLL" "$ROOT/libraylib.dll"
    else
        echo "build.sh: missing $RAYLIB_SHARED_DLL" >&2
        return 1
    fi
    if [ -f "$GLFW_SHARED_DLL" ]; then
        cp -f "$GLFW_SHARED_DLL" "$ROOT/glfw3.dll"
    else
        echo "build.sh: missing $GLFW_SHARED_DLL" >&2
        return 1
    fi
}

copy_release_runtime() {
    # Release embeds raylib statically but still needs glfw3.dll with the MSYS2 package.
    if [ -f "$GLFW_SHARED_DLL" ]; then
        cp -f "$GLFW_SHARED_DLL" "$ROOT/glfw3.dll"
    else
        echo "build.sh: missing $GLFW_SHARED_DLL" >&2
        return 1
    fi
}

# ------------------------------------------------------------------------------ actions --

MODE="dev"
if [ $# -gt 0 ]; then
    case "$1" in
        --release) MODE="release" ;;
        --tests)   MODE="tests" ;;
        --hotreload-harness) MODE="hotreload-harness" ;;
        --smoke-test) MODE="smoke-test" ;;
        --clean)   MODE="clean" ;;
        --help|-h)
            sed -n '2,22p' "$0"
            exit 0
            ;;
        *)
            echo "build.sh: unrecognised argument '$1' (try --help)" >&2
            exit 2
            ;;
    esac
fi

if [ "$MODE" = "clean" ]; then
    rm -rf build
    rm -f "$EXE" "$EXE_RELEASE" "$EXE_TESTS" "$EXE_HOTRELOAD"
    rm -f libraylib.dll raylib.dll glfw3.dll
    rm -f telemetry/phase1_smoke.png
    echo "cleaned."
    exit 0
fi

if [ "$MODE" = "tests" ]; then
    # shellcheck disable=SC2086
    $CC $CSTD $INCLUDES $WARNINGS $RELEASE_FLAGS -DDRIFTY_HEADLESS \
        $TEST_SRCS -o "$EXE_TESTS" $RAYLIB_CFLAGS -lm
    status=$?
    [ $status -eq 0 ] && echo "Built $EXE_TESTS."
    exit $status
fi

if [ "$MODE" = "hotreload-harness" ]; then
    copy_dev_runtime || exit 1

    # The harness loads build/game.dll; build it the same safe way as the dev path.
    # shellcheck disable=SC2086
    $CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -shared \
        -DDRIFTY_HOT_RELOAD -DDRIFTY_GAME_MODULE \
        $GAME_SRCS $SHARED_SRCS -o build/game_tmp.tmp $RAYLIB_CFLAGS $RAYLIB_SHARED_LIBS
    status=$?
    if [ $status -ne 0 ]; then
        rm -f build/game_tmp.tmp
        echo "build.sh: game module failed to compile; harness not built." >&2
        exit $status
    fi
    mv -f build/game_tmp.tmp "$MODULE" || exit 1

    # shellcheck disable=SC2086
    $CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -DDRIFTY_HOT_RELOAD \
        $HOTRELOAD_HARNESS_SRCS -o "$EXE_HOTRELOAD" $RAYLIB_CFLAGS $RAYLIB_SHARED_LIBS
    status=$?
    [ $status -eq 0 ] && echo "Built $MODULE and $EXE_HOTRELOAD."
    exit $status
fi

if [ "$MODE" = "release" ]; then
    copy_release_runtime || exit 1
    # shellcheck disable=SC2086
    $CC $CSTD $INCLUDES $WARNINGS $RELEASE_FLAGS \
        $PLATFORM_SRCS $GAME_SRCS $SHARED_SRCS -o "$EXE_RELEASE" \
        $RAYLIB_CFLAGS $RAYLIB_STATIC_LIBS
    status=$?
    [ $status -eq 0 ] && echo "Built $EXE_RELEASE (no hot reload, no game module; raylib static)."
    exit $status
fi

build_dev() {
    copy_dev_runtime || return 1

    # Always rebuild the game module. Link to a temporary name first: the linker leaves a
    # zero-length file in place while it works, and a running game polling for changes would
    # load that. The rename is atomic, so the running game either sees the old module or a
    # complete new one. If the compile fails, the rename never happens and the previous module
    # survives.
    # shellcheck disable=SC2086
    $CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -shared \
        -DDRIFTY_HOT_RELOAD -DDRIFTY_GAME_MODULE \
        $GAME_SRCS $SHARED_SRCS -o build/game_tmp.tmp $RAYLIB_CFLAGS $RAYLIB_SHARED_LIBS
    status=$?
    if [ $status -ne 0 ]; then
        rm -f build/game_tmp.tmp
        echo "build.sh: game module failed to compile; $MODULE left untouched." >&2
        return $status
    fi
    mv -f build/game_tmp.tmp "$MODULE" || return 1

    # If the game is already running, that is all there is to do: it will pick the module up.
    if command -v tasklist >/dev/null 2>&1; then
        if tasklist 2>/dev/null | grep -qi "drifty\.exe"; then
            echo "Hot reloading $MODULE..."
            return 0
        fi
    fi

    # shellcheck disable=SC2086
    $CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -DDRIFTY_HOT_RELOAD \
        $PLATFORM_SRCS $SHARED_SRCS $HOTRELOAD_SRC -o "$EXE" \
        $RAYLIB_CFLAGS $RAYLIB_SHARED_LIBS
    status=$?
    if [ $status -ne 0 ]; then
        echo "build.sh: platform layer failed to compile." >&2
        return $status
    fi

    echo "Built $MODULE and $EXE - run $EXE and leave it running."
    return 0
}

if [ "$MODE" = "smoke-test" ]; then
    build_dev || exit $?
    echo "Running bounded visual smoke test..."
    ./"$EXE" --smoke-test
    exit $?
fi

build_dev
exit $?
