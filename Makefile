# Drifty — build system (Windows / MSYS2 UCRT64 only)
#
#   make debug              hot-reload development build: build/game.dll + drifty.exe
#   make release            single executable; raylib linked statically; no game module
#   make tests              headless test executable, no window and no raylib linkage
#   make hotreload-harness  windowless hot-reload validation executable
#   make run-tests          build and run the headless tests from the repository root
#   make smoke-test         build then run drifty.exe --smoke-test (bounded, exits alone)
#   make clean              remove every generated artifact
#   make info               print the resolved compiler and raylib linkage
#
# Every target terminates. Nothing here launches a persistent game process except
# smoke-test, which is bounded and returns.
#
# Run make from an MSYS2 UCRT64 shell, or use build.bat from cmd.exe. From a bare
# cmd.exe without the UCRT64 environment, prefer build.bat.
#
# Development links the MSYS2 shared libraylib.dll (import library libraylib.dll.a).
# Release links libraylib.a statically; the MSYS2 archive still needs glfw3.dll at runtime.

# ---------------------------------------------------------------------------- toolchain --

ifndef MSYSTEM
    $(error Drifty builds only under MSYS2 UCRT64. Use build.bat or an MSYS2 UCRT64 shell.)
endif
ifneq ($(MSYSTEM),UCRT64)
    $(error Drifty builds only under MSYS2 UCRT64 (MSYSTEM=$(MSYSTEM)). Use build.bat.)
endif
ifneq ($(MINGW_PREFIX),/ucrt64)
    $(error MINGW_PREFIX must be /ucrt64 (got $(MINGW_PREFIX)).)
endif

CC := gcc
CC_PATH := $(shell command -v $(CC) 2>/dev/null)
ifeq ($(findstring /ucrt64/bin/,$(CC_PATH)),)
    $(error refusing non-UCRT64 compiler '$(CC_PATH)'. Use build.bat or the UCRT64 shell.)
endif

PKGCONFIG := $(shell command -v pkg-config 2>/dev/null)
ifeq ($(PKGCONFIG),)
    $(error pkg-config not found. Run scripts/setup_windows.ps1.)
endif
ifeq ($(shell pkg-config --exists raylib 2>/dev/null && echo yes),)
    $(error pkg-config cannot find raylib. Run scripts/setup_windows.ps1.)
endif

# ------------------------------------------------------------------------------- raylib --

RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib)
RAYLIB_SHARED_LIBS := $(MINGW_PREFIX)/lib/libraylib.dll.a -lopengl32 -lgdi32 -lwinmm
RAYLIB_STATIC_LIBS := -l:libraylib.a -lglfw3 -lopengl32 -lgdi32 -lwinmm
RAYLIB_SHARED_DLL := $(MINGW_PREFIX)/bin/libraylib.dll
GLFW_SHARED_DLL := $(MINGW_PREFIX)/bin/glfw3.dll

# -------------------------------------------------------------------------------- flags --

CSTD     := -std=c11
INCLUDES := -Isrc
WARNINGS := -Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith
DEBUG_FLAGS   := -O0 -g
RELEASE_FLAGS := -O2 -DNDEBUG

# ------------------------------------------------------------------------------ sources --

SHARED_SRCS := \
    src/input.c \
    src/math_utils.c

GAME_SRCS := \
    src/game.c \
    src/vehicle.c \
    src/physics.c \
    src/render.c \
    src/replay.c \
    src/telemetry.c

PLATFORM_SRCS := \
    src/main.c \
    src/timestep.c

HOTRELOAD_SRC := src/hotreload_windows.c

TEST_SRCS := \
    tests/physics_tests.c \
    src/timestep.c \
    $(GAME_SRCS) \
    $(SHARED_SRCS)

HOTRELOAD_HARNESS_SRCS := \
    tests/hotreload_harness.c \
    src/timestep.c \
    $(HOTRELOAD_SRC) \
    $(SHARED_SRCS)

MODULE_NAME := build/game.dll
EXE_DEBUG   := drifty.exe
EXE_RELEASE := drifty_release.exe
EXE_TESTS   := drifty_tests.exe
EXE_HOTRELOAD := drifty_hotreload_harness.exe

# ------------------------------------------------------------------------------ targets --

.PHONY: all debug release tests hotreload-harness run-tests smoke-test clean clean-telemetry info help module platform runtime-dev runtime-release dirs

all: debug

help:
	@echo "targets: debug release tests hotreload-harness run-tests smoke-test clean info"

info:
	@echo "MSYSTEM      : $(MSYSTEM)"
	@echo "MINGW_PREFIX : $(MINGW_PREFIX)"
	@echo "compiler     : $(CC_PATH)"
	@echo "raylib cflags: $(RAYLIB_CFLAGS)"
	@echo "raylib shared: $(RAYLIB_SHARED_LIBS)"
	@echo "raylib static: $(RAYLIB_STATIC_LIBS)"
	@echo "game module  : $(MODULE_NAME)"
	@echo "hotreload src: $(HOTRELOAD_SRC)"

dirs:
	@mkdir -p build telemetry

runtime-dev: dirs
	@cp -f $(RAYLIB_SHARED_DLL) ./libraylib.dll
	@cp -f $(GLFW_SHARED_DLL) ./glfw3.dll

runtime-release: dirs
	@cp -f $(GLFW_SHARED_DLL) ./glfw3.dll

# --- development / hot reload ---

debug: module platform

module: dirs
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(DEBUG_FLAGS) -shared \
	    -DDRIFTY_HOT_RELOAD -DDRIFTY_GAME_MODULE \
	    $(GAME_SRCS) $(SHARED_SRCS) -o build/game_tmp.tmp $(RAYLIB_CFLAGS) $(RAYLIB_SHARED_LIBS)
	@mv -f build/game_tmp.tmp $(MODULE_NAME)

platform: runtime-dev
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(DEBUG_FLAGS) -DDRIFTY_HOT_RELOAD \
	    $(PLATFORM_SRCS) $(SHARED_SRCS) $(HOTRELOAD_SRC) -o $(EXE_DEBUG) \
	    $(RAYLIB_CFLAGS) $(RAYLIB_SHARED_LIBS)

# --- release: one executable, no module, static raylib, glfw3.dll still required ---

release: runtime-release
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(RELEASE_FLAGS) \
	    $(PLATFORM_SRCS) $(GAME_SRCS) $(SHARED_SRCS) -o $(EXE_RELEASE) \
	    $(RAYLIB_CFLAGS) $(RAYLIB_STATIC_LIBS)

# --- headless tests: raylib.h for types only, no raylib library on the link line ---

tests: dirs
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(RELEASE_FLAGS) -DDRIFTY_HEADLESS \
	    $(TEST_SRCS) -o $(EXE_TESTS) $(RAYLIB_CFLAGS) -lm

run-tests: tests
	./$(EXE_TESTS)

hotreload-harness: runtime-dev module
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(DEBUG_FLAGS) -DDRIFTY_HOT_RELOAD \
	    $(HOTRELOAD_HARNESS_SRCS) -o $(EXE_HOTRELOAD) \
	    $(RAYLIB_CFLAGS) $(RAYLIB_SHARED_LIBS)

smoke-test: debug
	./$(EXE_DEBUG) --smoke-test

# --- housekeeping ---

clean:
	rm -rf build
	rm -f $(EXE_DEBUG) $(EXE_RELEASE) $(EXE_TESTS) $(EXE_HOTRELOAD)
	rm -f libraylib.dll raylib.dll glfw3.dll
	rm -f telemetry/phase1_smoke.png
	rm -f *.o src/*.o tests/*.o *.pdb

clean-telemetry:
	rm -f telemetry/*.csv telemetry/*.png
