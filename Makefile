# Drifty — build system
#
#   make debug       hot-reload development build: build/game.dll + drifty.exe
#   make release     single executable, DRIFTY_HOT_RELOAD undefined, no game module
#   make tests       headless test executable, no window and no raylib linkage
#   make run-tests   build and run the headless tests from the repository root
#   make clean       remove every generated artifact
#   make info        print the resolved toolchain and raylib linkage
#
# Every target terminates. Nothing here launches or supervises a long-lived process.
#
# The recipes use POSIX shell utilities (mkdir -p, mv, cp, rm), so on Windows run make from
# a shell that provides them (MSYS2, Git Bash). From a bare cmd.exe use build.bat instead,
# which needs neither make nor a POSIX shell.
#
# raylib linkage is resolved in this order:
#   1. pkg-config raylib, when pkg-config knows about it (MSYS2, Linux, Homebrew).
#   2. $(RAYLIB_DIR), default vendor/raylib — the documented Windows fallback for a
#      toolchain without pkg-config. See README.md for how to populate it.
#
# The hot-reload configuration requires a SHARED raylib: raylib keeps its state in global
# variables, and a statically linked copy inside game.dll is destroyed on every reload.

# ---------------------------------------------------------------------------- toolchain --

ifeq ($(OS),Windows_NT)
    HOST_OS      := windows
    EXE_SUFFIX   := .exe
    MODULE_NAME  := build/game.dll
    HOTRELOAD_SRC := src/hotreload_windows.c
else
    HOST_OS      := $(shell uname -s)
    EXE_SUFFIX   :=
    ifeq ($(HOST_OS),Darwin)
        MODULE_NAME := build/libgame.dylib
    else
        MODULE_NAME := build/libgame.so
    endif
    HOTRELOAD_SRC := src/hotreload_posix.c
endif

# Locate a compiler. On Windows a MinGW-w64 toolchain is frequently installed without being
# on PATH, so probe the usual locations before giving up.
ifeq ($(origin CC),default)
    CC := gcc
    ifeq ($(HOST_OS),windows)
        ifeq ($(shell command -v gcc 2>/dev/null),)
            MINGW_BIN := $(firstword $(wildcard \
                C:/ProgramData/chocolatey/lib/mingw/tools/install/mingw64/bin \
                C:/msys64/ucrt64/bin \
                C:/msys64/mingw64/bin \
                C:/mingw64/bin))
            ifneq ($(MINGW_BIN),)
                CC := $(MINGW_BIN)/gcc
            endif
        endif
    endif
endif

# ------------------------------------------------------------------------------- raylib --

# Plain conditional rather than ?= so that very old GNU Make releases behave too.
ifeq ($(RAYLIB_DIR),)
    RAYLIB_DIR := vendor/raylib
endif

PKGCONFIG_RAYLIB := $(shell pkg-config --exists raylib 2>/dev/null && echo yes)

ifeq ($(PKGCONFIG_RAYLIB),yes)
    RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib)
    RAYLIB_LIBS   := $(shell pkg-config --libs raylib)
    RAYLIB_SOURCE := pkg-config
    RAYLIB_RUNTIME :=
else
    RAYLIB_CFLAGS := -I$(RAYLIB_DIR)/include
    ifeq ($(HOST_OS),windows)
        RAYLIB_LIBS    := -L$(RAYLIB_DIR)/lib -lraylib -lopengl32 -lgdi32 -lwinmm
        RAYLIB_RUNTIME := $(RAYLIB_DIR)/bin/raylib.dll
    else
        RAYLIB_LIBS    := -L$(RAYLIB_DIR)/lib -lraylib -lm -lpthread -ldl
        RAYLIB_RUNTIME :=
    endif
    RAYLIB_SOURCE := $(RAYLIB_DIR)
endif

# -------------------------------------------------------------------------------- flags --

CSTD     := -std=c11
INCLUDES := -Isrc
WARNINGS := -Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith
DEBUG_FLAGS   := -O0 -g
RELEASE_FLAGS := -O2 -DNDEBUG

# ------------------------------------------------------------------------------ sources --

# Stateless utility translation units needed by BOTH layers: the platform loop samples
# input, and both layers use the math helpers. They hold no mutable global state, so a copy
# living in each binary is safe — there is no state for the two copies to disagree about.
SHARED_SRCS := \
    src/input.c \
    src/math_utils.c

# Hot-reloadable game module.
GAME_SRCS := \
    src/game.c \
    src/replay.c \
    src/telemetry.c

# Platform layer. Never part of the reloadable module.
PLATFORM_SRCS := \
    src/main.c \
    src/timestep.c

# Headless harness: game logic plus the accumulator, with no rendering and no raylib link.
TEST_SRCS := \
    tests/physics_tests.c \
    src/timestep.c \
    $(GAME_SRCS) \
    $(SHARED_SRCS)

EXE_DEBUG   := drifty$(EXE_SUFFIX)
EXE_RELEASE := drifty_release$(EXE_SUFFIX)
EXE_TESTS   := drifty_tests$(EXE_SUFFIX)

# ------------------------------------------------------------------------------ targets --

.PHONY: all debug release tests run-tests clean clean-telemetry info help module platform runtime dirs

all: debug

help:
	@echo "targets: debug release tests run-tests clean info"

info:
	@echo "host os      : $(HOST_OS)"
	@echo "compiler     : $(CC)"
	@echo "raylib from  : $(RAYLIB_SOURCE)"
	@echo "raylib cflags: $(RAYLIB_CFLAGS)"
	@echo "raylib libs  : $(RAYLIB_LIBS)"
	@echo "game module  : $(MODULE_NAME)"
	@echo "hotreload src: $(HOTRELOAD_SRC)"

dirs:
	@mkdir -p build telemetry

# Copy the raylib runtime next to the executables when using the vendored fallback.
runtime: dirs
ifneq ($(RAYLIB_RUNTIME),)
	@cp -f $(RAYLIB_RUNTIME) ./
endif

# --- development / hot reload ---

debug: module platform

# Link to a temporary name and rename, so a game that is polling for changes can never
# observe the zero-length file the linker creates before it fills it. A failed compile
# leaves the previous module in place because the rename never happens.
module: dirs
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(DEBUG_FLAGS) -shared -fPIC \
	    -DDRIFTY_HOT_RELOAD -DDRIFTY_GAME_MODULE \
	    $(GAME_SRCS) $(SHARED_SRCS) -o build/game_tmp.tmp $(RAYLIB_CFLAGS) $(RAYLIB_LIBS)
	@mv -f build/game_tmp.tmp $(MODULE_NAME)

platform: runtime
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(DEBUG_FLAGS) -DDRIFTY_HOT_RELOAD \
	    $(PLATFORM_SRCS) $(SHARED_SRCS) $(HOTRELOAD_SRC) -o $(EXE_DEBUG) $(RAYLIB_CFLAGS) $(RAYLIB_LIBS)

# --- release: one executable, no module, no hot reload, no loader compiled in ---

release: runtime
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(RELEASE_FLAGS) \
	    $(PLATFORM_SRCS) $(GAME_SRCS) $(SHARED_SRCS) -o $(EXE_RELEASE) $(RAYLIB_CFLAGS) $(RAYLIB_LIBS)

# --- headless tests: raylib.h for types only, no raylib library on the link line ---

tests: dirs
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(RELEASE_FLAGS) -DDRIFTY_HEADLESS \
	    $(TEST_SRCS) -o $(EXE_TESTS) $(RAYLIB_CFLAGS) -lm

run-tests: tests
	./$(EXE_TESTS)

# --- housekeeping ---

clean:
	rm -rf build
	rm -f $(EXE_DEBUG) $(EXE_RELEASE) $(EXE_TESTS) raylib.dll
	rm -f *.o src/*.o tests/*.o *.pdb

clean-telemetry:
	rm -f telemetry/*.csv
