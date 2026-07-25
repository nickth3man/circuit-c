#!/bin/sh
#
# build.sh — the hot-reload development build.
#
# Rebuilds the game module every time, and rebuilds drifty.exe only when it is not already
# running. It always terminates in well under a second and returns the compiler's exit
# status. It never launches or supervises drifty.exe: the developer starts that once and
# leaves it open.
#
# A failed compile leaves the previous, working module untouched — the link output goes to a
# temporary name and is only renamed into place on success — so a compile error can never
# close the running game.
#
#   ./build.sh              build the module (and the exe if it is not running)
#   ./build.sh --release    single executable, no hot reload
#   ./build.sh --tests      headless test executable
#   ./build.sh --clean      remove generated artifacts
#
set -u

cd "$(dirname "$0")" || exit 1

# ---------------------------------------------------------------------------- toolchain --

if ! command -v "${CC:-gcc}" >/dev/null 2>&1; then
    for candidate in \
        "/c/ProgramData/chocolatey/lib/mingw/tools/install/mingw64/bin" \
        "/mingw64/bin" \
        "/ucrt64/bin" \
        "/c/msys64/ucrt64/bin" \
        "/c/msys64/mingw64/bin" \
        "/c/mingw64/bin"
    do
        if [ -x "$candidate/gcc.exe" ] || [ -x "$candidate/gcc" ]; then
            PATH="$candidate:$PATH"
            export PATH
            break
        fi
    done
fi

CC="${CC:-gcc}"
if ! command -v "$CC" >/dev/null 2>&1; then
    echo "build.sh: no C compiler found (tried '$CC'). See README.md for prerequisites." >&2
    exit 127
fi

# ------------------------------------------------------------------------------- raylib --

RAYLIB_DIR="${RAYLIB_DIR:-vendor/raylib}"

if pkg-config --exists raylib 2>/dev/null; then
    RAYLIB_CFLAGS="$(pkg-config --cflags raylib)"
    RAYLIB_LIBS="$(pkg-config --libs raylib)"
    RAYLIB_RUNTIME=""
else
    RAYLIB_CFLAGS="-I$RAYLIB_DIR/include"
    case "$(uname -s 2>/dev/null || echo Windows)" in
        MINGW*|MSYS*|CYGWIN*|Windows*)
            RAYLIB_LIBS="-L$RAYLIB_DIR/lib -lraylib -lopengl32 -lgdi32 -lwinmm"
            RAYLIB_RUNTIME="$RAYLIB_DIR/bin/raylib.dll"
            ;;
        Darwin)
            RAYLIB_LIBS="-L$RAYLIB_DIR/lib -lraylib -framework OpenGL -framework Cocoa -framework IOKit"
            RAYLIB_RUNTIME=""
            ;;
        *)
            RAYLIB_LIBS="-L$RAYLIB_DIR/lib -lraylib -lm -lpthread -ldl"
            RAYLIB_RUNTIME=""
            ;;
    esac
fi

# -------------------------------------------------------------------------------- flags --

CSTD="-std=c11"
INCLUDES="-Isrc"
WARNINGS="-Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith"
DEBUG_FLAGS="-O0 -g"
RELEASE_FLAGS="-O2 -DNDEBUG"

# Stateless helpers needed by both layers; no mutable global state, so a copy in each
# binary is safe.
SHARED_SRCS="src/input.c src/math_utils.c"
GAME_SRCS="src/game.c src/replay.c src/telemetry.c"
PLATFORM_SRCS="src/main.c src/timestep.c"
TEST_SRCS="tests/physics_tests.c src/timestep.c $GAME_SRCS $SHARED_SRCS"

case "$(uname -s 2>/dev/null || echo Windows)" in
    MINGW*|MSYS*|CYGWIN*|Windows*)
        MODULE="build/game.dll"
        HOTRELOAD_SRC="src/hotreload_windows.c"
        EXE="drifty.exe"
        EXE_RELEASE="drifty_release.exe"
        EXE_TESTS="drifty_tests.exe"
        ;;
    Darwin)
        MODULE="build/libgame.dylib"
        HOTRELOAD_SRC="src/hotreload_posix.c"
        EXE="drifty"
        EXE_RELEASE="drifty_release"
        EXE_TESTS="drifty_tests"
        ;;
    *)
        MODULE="build/libgame.so"
        HOTRELOAD_SRC="src/hotreload_posix.c"
        EXE="drifty"
        EXE_RELEASE="drifty_release"
        EXE_TESTS="drifty_tests"
        ;;
esac

mkdir -p build telemetry

# ------------------------------------------------------------------------------ actions --

MODE="dev"
if [ $# -gt 0 ]; then
    case "$1" in
        --release) MODE="release" ;;
        --tests)   MODE="tests" ;;
        --clean)   MODE="clean" ;;
        --help|-h)
            sed -n '2,20p' "$0"
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
    rm -f "$EXE" "$EXE_RELEASE" "$EXE_TESTS" raylib.dll
    echo "cleaned."
    exit 0
fi

# Keep the raylib runtime next to the executables when using the vendored fallback.
if [ -n "$RAYLIB_RUNTIME" ] && [ -f "$RAYLIB_RUNTIME" ]; then
    cp -f "$RAYLIB_RUNTIME" ./ 2>/dev/null || true
fi

if [ "$MODE" = "tests" ]; then
    # shellcheck disable=SC2086
    $CC $CSTD $INCLUDES $WARNINGS $RELEASE_FLAGS -DDRIFTY_HEADLESS \
        $TEST_SRCS -o "$EXE_TESTS" $RAYLIB_CFLAGS -lm
    status=$?
    [ $status -eq 0 ] && echo "Built $EXE_TESTS."
    exit $status
fi

if [ "$MODE" = "release" ]; then
    # shellcheck disable=SC2086
    $CC $CSTD $INCLUDES $WARNINGS $RELEASE_FLAGS \
        $PLATFORM_SRCS $GAME_SRCS $SHARED_SRCS -o "$EXE_RELEASE" $RAYLIB_CFLAGS $RAYLIB_LIBS
    status=$?
    [ $status -eq 0 ] && echo "Built $EXE_RELEASE (no hot reload, no game module)."
    exit $status
fi

# --- development build ---------------------------------------------------------------- #

# Always rebuild the game module. Link to a temporary name first: the linker leaves a
# zero-length file in place while it works, and a running game polling for changes would
# load that. The rename is atomic, so the running game either sees the old module or a
# complete new one. If the compile fails, the rename never happens and the previous module
# survives.
# shellcheck disable=SC2086
$CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -shared -fPIC \
    -DDRIFTY_HOT_RELOAD -DDRIFTY_GAME_MODULE \
    $GAME_SRCS $SHARED_SRCS -o build/game_tmp.tmp $RAYLIB_CFLAGS $RAYLIB_LIBS
status=$?
if [ $status -ne 0 ]; then
    rm -f build/game_tmp.tmp
    echo "build.sh: game module failed to compile; $MODULE left untouched." >&2
    exit $status
fi
mv -f build/game_tmp.tmp "$MODULE" || exit 1

# If the game is already running, that is all there is to do: it will pick the module up.
if command -v tasklist >/dev/null 2>&1; then
    if tasklist 2>/dev/null | grep -qi "drifty\.exe"; then
        echo "Hot reloading $MODULE..."
        exit 0
    fi
elif command -v pgrep >/dev/null 2>&1; then
    if pgrep -x drifty >/dev/null 2>&1; then
        echo "Hot reloading $MODULE..."
        exit 0
    fi
fi

# shellcheck disable=SC2086
$CC $CSTD $INCLUDES $WARNINGS $DEBUG_FLAGS -DDRIFTY_HOT_RELOAD \
    $PLATFORM_SRCS $SHARED_SRCS $HOTRELOAD_SRC -o "$EXE" $RAYLIB_CFLAGS $RAYLIB_LIBS
status=$?
if [ $status -ne 0 ]; then
    echo "build.sh: platform layer failed to compile." >&2
    exit $status
fi

echo "Built $MODULE and $EXE - run $EXE and leave it running."
exit 0
