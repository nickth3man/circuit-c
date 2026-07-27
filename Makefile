# Drifty — one command per operation.
#
# The rule this file follows: the human and the coding agent run the SAME commands. If a
# check exists in CI it exists here, spelled the same way, and `make ci` is the local
# equivalent of the required GitHub checks.
#
#   make dev              hot-reload development build: build/game.dll + drifty.exe
#   make run              build, then LAUNCH the game (a human does this, never an agent)
#   make test             fast unit and infrastructure scenarios
#   make test-physics     every physics and maneuver scenario, with telemetry
#   make scenario NAME=skidpad     one scenario
#   make report NAME=skidpad       one scenario, then a self-contained HTML report
#   make regression       compare telemetry/ against tests/baselines/ with tolerances
#   make baselines        re-record tests/baselines/ from the current build (explain it!)
#   make verify-fast      format check + tests
#   make verify           static analysis + tests + physics + regression
#   make sanitize         ASan + UBSan build and tests (clang)
#   make coverage         gcov/gcovr text, HTML, and Cobertura output
#   make screenshots      capture the deterministic visual scenes
#   make visual-test      compare captured scenes against tests/visual/baseline
#   make profile          build with the Tracy hooks enabled (DRIFTY_TRACY)
#   make benchmark        fixed-update throughput
#   make release          release build
#   make ci               everything the required CI checks run
#   make params-doc       regenerate docs/PARAMETERS.md from the registry
#   make compile-commands write compile_commands.json for clangd
#   make format           apply .clang-format        make format-check  check only
#   make lint             cppcheck                   make analyze       clang --analyze
#   make fuzz             build and briefly run the libFuzzer targets (clang)
#   make clean            remove every generated artifact
#   make info             print the resolved toolchain and linkage
#   make help             this list
#
# Every target terminates on its own except `run`, which is the developer's to start.
#
# On Windows the canonical build lives in build.sh; the targets below call it rather than
# duplicating the hot-reload-safe link sequence. On Linux and macOS only the headless
# targets work (tests, sanitizers, coverage, fuzzing) — that is what CI needs there, and
# the game itself remains Windows-only.

# ------------------------------------------------------------------------------- host --

ifeq ($(MSYSTEM),UCRT64)
    DRIFTY_HOST := ucrt64
else
    UNAME_S := $(shell uname -s 2>/dev/null)
    ifneq (,$(filter Linux Darwin,$(UNAME_S)))
        DRIFTY_HOST := posix
    else
        DRIFTY_HOST := unsupported
    endif
endif

ifeq ($(DRIFTY_HOST),unsupported)
    $(error Run make from an MSYS2 UCRT64 shell (or use build.bat / mk.bat), or from Linux/macOS for the headless targets.)
endif

# --------------------------------------------------------------------------- toolchain --

ifeq ($(DRIFTY_HOST),ucrt64)

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

RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib)
EXE_SUFFIX    := .exe
# On Windows `python3` is usually the Microsoft Store stub, which does nothing useful.
PYTHON ?= $(shell command -v python 2>/dev/null || command -v python3 2>/dev/null)

else   # posix: headless only

CC ?= cc
# raylib is needed for its HEADER only — the headless build calls no raylib function and
# links no raylib library. Point RAYLIB_INCLUDE_DIR at any raylib source or install tree.
RAYLIB_INCLUDE_DIR ?= $(shell pkg-config --variable=includedir raylib 2>/dev/null)
ifeq ($(strip $(RAYLIB_INCLUDE_DIR)),)
    RAYLIB_INCLUDE_DIR := third_party/raylib-src/src
endif
RAYLIB_CFLAGS := -I$(RAYLIB_INCLUDE_DIR)
EXE_SUFFIX    :=
PYTHON ?= $(shell command -v python3 2>/dev/null || command -v python 2>/dev/null)

endif

# Optional tools. Missing ones make their target explain how to install rather than fail
# with a shell error; CI installs all of them, so nothing is skipped where it matters.
CLANG        := $(shell command -v clang 2>/dev/null)
CLANG_FORMAT := $(shell command -v clang-format 2>/dev/null)
CPPCHECK     := $(shell command -v cppcheck 2>/dev/null)
GCOVR        := $(shell command -v gcovr 2>/dev/null)
MAGICK       := $(shell command -v magick 2>/dev/null)

# ------------------------------------------------------------------------------- flags --

CSTD     := -std=c11
INCLUDES := -Isrc -Ithird_party/raygui
WARNINGS := -Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith
DEBUG_FLAGS   := -O0 -g
RELEASE_FLAGS := -O2 -DNDEBUG

BUILD_COMMIT := $(shell git rev-parse --short=12 HEAD 2>/dev/null || echo unknown)
BUILD_BRANCH := $(shell git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)
BUILD_DIRTY  := $(shell git diff --quiet HEAD 2>/dev/null && echo clean || echo dirty)
BUILD_DEFINES = -DDRIFTY_BUILD_COMMIT=\"$(BUILD_COMMIT)\" \
                -DDRIFTY_BUILD_BRANCH=\"$(BUILD_BRANCH)\" \
                -DDRIFTY_BUILD_DIRTY=\"$(BUILD_DIRTY)\"

# ----------------------------------------------------------------------------- sources --

SHARED_SRCS := src/input.c src/math_utils.c src/dev_scenario.c src/profile.c
DEV_SRCS    := src/dev_params.c src/dev_replay.c src/dev_state.c src/failure_bundle.c
GAME_SRCS   := src/game.c src/vehicle.c src/physics.c src/tire.c src/drivetrain.c \
               src/render.c src/replay.c src/telemetry.c $(DEV_SRCS)
PLATFORM_SRCS := src/main.c src/timestep.c
TEST_SRCS   := tests/physics_tests.c src/timestep.c $(GAME_SRCS) $(SHARED_SRCS)
ALL_C_SRCS  := $(wildcard src/*.c) $(wildcard tests/*.c) $(wildcard fuzz/*.c)
ALL_H_SRCS  := $(wildcard src/*.h)

# Static analysis covers the headless-safe set: the platform layer is Windows-only and
# dev_lab.c is mostly a 6000-line vendored header, which is not our code to fix.
ANALYZE_SRCS := $(filter-out src/main.c src/hotreload_windows.c src/dev_lab.c,\
                             $(wildcard src/*.c)) tests/physics_tests.c

EXE_TESTS := drifty_tests$(EXE_SUFFIX)
EXE_DEBUG := drifty$(EXE_SUFFIX)
EXE_RELEASE := drifty_release$(EXE_SUFFIX)

ARTIFACTS := artifacts
TELEMETRY := telemetry
BASELINES := tests/baselines
SCENES    := debug_overlay tire_curves drift_hud physics_lab \
             accel_load brake_load skidpad_p3 lift_off transition_p3 catchable

# The scenarios that write telemetry and are compared against a baseline.
REGRESSION_SCENARIOS := skidpad step-steer transition lift-off \
                        accel-load brake-load coast-down catchable-drift

.PHONY: all help info dev run release tests test test-physics scenario report regression \
        baselines verify-fast verify sanitize coverage screenshots visual-test profile \
        benchmark ci params-doc compile-commands format format-check lint analyze fuzz \
        clean clean-telemetry dirs windows-only

all: dev

help:
	@sed -n '6,39p' Makefile

info:
	@echo "host        : $(DRIFTY_HOST)"
	@echo "compiler    : $(CC)"
	@echo "raylib cflags: $(RAYLIB_CFLAGS)"
	@echo "python      : $(PYTHON)"
	@echo "commit      : $(BUILD_COMMIT) ($(BUILD_BRANCH), $(BUILD_DIRTY))"
	@echo "clang       : $(if $(CLANG),$(CLANG),not installed)"
	@echo "clang-format: $(if $(CLANG_FORMAT),$(CLANG_FORMAT),not installed)"
	@echo "cppcheck    : $(if $(CPPCHECK),$(CPPCHECK),not installed)"
	@echo "gcovr       : $(if $(GCOVR),$(GCOVR),not installed)"
	@echo "magick      : $(if $(MAGICK),$(MAGICK),not installed)"

dirs:
	@mkdir -p build $(TELEMETRY) $(ARTIFACTS)

windows-only:
ifneq ($(DRIFTY_HOST),ucrt64)
	@echo "This target builds the game itself and needs MSYS2 UCRT64 on Windows." >&2
	@exit 1
endif

# ------------------------------------------------------------------ builds (delegated) --
#
# build.sh owns the link sequence that keeps hot reload safe (temporary output name plus an
# atomic rename). Duplicating it here would eventually mean two subtly different builds.

dev: windows-only
	./build.sh

run: dev
	@echo "Launching drifty.exe. This does not return until you close the window."
	@echo "Coding agents must never run this target — rebuild with 'make dev' instead."
	./$(EXE_DEBUG)

release: windows-only
	./build.sh --release

ifeq ($(DRIFTY_HOST),ucrt64)
tests:
	./build.sh --tests
else
tests: dirs
	$(CC) $(CSTD) $(INCLUDES) $(WARNINGS) $(RELEASE_FLAGS) -DDRIFTY_HEADLESS \
	    $(BUILD_DEFINES) -DDRIFTY_BUILD_MODE=\"tests\" -DDRIFTY_BUILD_FLAGS=\"-O2,-DNDEBUG\" \
	    $(TEST_SRCS) -o $(EXE_TESTS) $(RAYLIB_CFLAGS) -lm
	@echo "Built $(EXE_TESTS)."
endif

# ------------------------------------------------------------------------------- tests --

# Fast feedback: infrastructure and pure-function scenarios, no long maneuvers.
test: tests
	./$(EXE_TESTS) --scenario math
	./$(EXE_TESTS) --scenario units
	./$(EXE_TESTS) --scenario timestep
	./$(EXE_TESTS) --scenario oneshot
	./$(EXE_TESTS) --scenario replay
	./$(EXE_TESTS) --scenario devreplay
	./$(EXE_TESTS) --scenario params
	./$(EXE_TESTS) --scenario telemetry
	./$(EXE_TESTS) --scenario renderscale

test-physics: tests
	./$(EXE_TESTS)

scenario: tests
	@test -n "$(NAME)" || (echo "usage: make scenario NAME=skidpad" >&2; exit 2)
	./$(EXE_TESTS) --scenario $(NAME)

benchmark: tests
	./$(EXE_TESTS) --benchmark 240000

params-doc: tests
	./$(EXE_TESTS) --dump-params docs/PARAMETERS.md

# ------------------------------------------------------------------- telemetry tooling --

report: tests
	@test -n "$(NAME)" || (echo "usage: make report NAME=skidpad" >&2; exit 2)
	./$(EXE_TESTS) --scenario $(NAME)
	$(PYTHON) tools/make_report.py $(TELEMETRY)/scenario_$(NAME).csv \
	    $(if $(wildcard $(BASELINES)/scenario_$(NAME).csv),--baseline $(BASELINES)/scenario_$(NAME).csv,) \
	    --title "Drifty — $(NAME)" --out $(ARTIFACTS)/report_$(NAME).html
	@echo "open $(ARTIFACTS)/report_$(NAME).html"

regression: test-physics
	$(PYTHON) tools/compare_telemetry.py --baseline-dir $(BASELINES) --current-dir $(TELEMETRY) \
	    --markdown $(ARTIFACTS)/regression.md

# Re-recording a baseline is never a way to make a failing test green. Say why in the PR.
baselines: test-physics
	@echo "Re-recording baselines from the CURRENT build."
	@echo "A PR that changes these must explain, in words, why the new numbers are correct."
	cp -f $(TELEMETRY)/scenario_*.csv $(BASELINES)/
	cp -f $(TELEMETRY)/phase2_*.csv $(BASELINES)/ 2>/dev/null || true
	@ls -la $(BASELINES)

# ------------------------------------------------------------------- quality and gates --

format:
ifeq ($(CLANG_FORMAT),)
	@echo "clang-format not installed. pacman -S mingw-w64-ucrt-x86_64-clang-tools-extra" >&2
	@exit 1
else
	$(CLANG_FORMAT) -i $(ALL_C_SRCS) $(ALL_H_SRCS)
	@echo "formatted $(words $(ALL_C_SRCS) $(ALL_H_SRCS)) files"
endif

format-check:
ifeq ($(CLANG_FORMAT),)
	@echo "SKIP format-check: clang-format not installed."
else
	$(CLANG_FORMAT) --dry-run --Werror $(ALL_C_SRCS) $(ALL_H_SRCS)
	@echo "format ok"
endif

lint:
ifeq ($(CPPCHECK),)
	@echo "SKIP lint: cppcheck not installed (pacman -S mingw-w64-ucrt-x86_64-cppcheck)."
else
	# unusedFunction is left to the nightly `--enable=all` pass: a library of registry and
	# inspector helpers legitimately has entry points that any single analysed set does not
	# call, and a gate that cries wolf gets ignored. Anything specific can be silenced with an
	# inline suppression comment.
	$(CPPCHECK) --quiet --error-exitcode=1 --std=c11 --language=c \
	    --enable=warning,style,performance,portability \
	    --inline-suppr --suppress=missingIncludeSystem \
	    --suppress='*:third_party/*' \
	    -I src -I third_party/raygui $(RAYLIB_CFLAGS) \
	    src tests
endif

analyze:
ifeq ($(CLANG),)
	@echo "SKIP analyze: clang not installed (pacman -S mingw-w64-ucrt-x86_64-clang)."
else
	@for f in $(ANALYZE_SRCS); do \
	    echo "  analyze $$f"; \
	    $(CLANG) --analyze -Xanalyzer -analyzer-output=text $(CSTD) $(INCLUDES) \
	        $(RAYLIB_CFLAGS) -DDRIFTY_HEADLESS $$f -o /dev/null || exit 1; \
	done
	@echo "clang --analyze clean"
endif

verify-fast: format-check test
	@echo "verify-fast: ok"

verify: format-check lint analyze test-physics regression
	@echo "verify: ok"

# ------------------------------------------------------------------------- sanitizers --

sanitize:
ifeq ($(CLANG),)
	@echo "SKIP sanitize: clang not installed. The Linux CI job runs ASan and UBSan on" >&2
	@echo "every pull request; mingw-w64 GCC ships no sanitizer runtime." >&2
else
	$(CLANG) $(CSTD) $(INCLUDES) -O1 -g -fsanitize=address,undefined \
	    -fno-omit-frame-pointer -fno-sanitize-recover=all -DDRIFTY_HEADLESS \
	    $(BUILD_DEFINES) -DDRIFTY_BUILD_MODE=\"sanitize\" \
	    -DDRIFTY_BUILD_FLAGS=\"-O1,-g,-fsanitize=address+undefined\" \
	    $(TEST_SRCS) -o drifty_tests_asan$(EXE_SUFFIX) $(RAYLIB_CFLAGS) -lm
	./drifty_tests_asan$(EXE_SUFFIX)
endif

# -------------------------------------------------------------------------- coverage --

coverage:
	$(CC) $(CSTD) $(INCLUDES) -O0 -g --coverage -DDRIFTY_HEADLESS \
	    $(BUILD_DEFINES) -DDRIFTY_BUILD_MODE=\"coverage\" -DDRIFTY_BUILD_FLAGS=\"-O0,--coverage\" \
	    $(TEST_SRCS) -o drifty_tests_cov$(EXE_SUFFIX) $(RAYLIB_CFLAGS) -lm
	./drifty_tests_cov$(EXE_SUFFIX)
ifeq ($(GCOVR),)
	@echo "SKIP gcovr report: gcovr not installed (pip install gcovr). Raw .gcda files kept."
else
	@mkdir -p coverage
	$(GCOVR) --root . --filter 'src/.*' --exclude 'src/dev_lab.c' \
	    --txt --html-details coverage/index.html --cobertura coverage/cobertura.xml \
	    --print-summary
	@echo "coverage/index.html written"
endif

# ------------------------------------------------------------------- visual regression --

screenshots: windows-only dev
	@mkdir -p $(ARTIFACTS)/screenshots
	@for scene in $(SCENES); do \
	    ./$(EXE_DEBUG) --capture-scene $$scene --width 1280 --height 720 --seed 12345 \
	        --output $(ARTIFACTS)/screenshots/$$scene.png || exit 1; \
	done

visual-test: screenshots
ifeq ($(MAGICK),)
	@echo "SKIP visual-test: ImageMagick not installed (winget install ImageMagick.ImageMagick)."
else
	@mkdir -p $(ARTIFACTS)/visual-diff
	@status=0; \
	for scene in $(SCENES); do \
	    base=tests/visual/baseline/$$scene.png; \
	    current=$(ARTIFACTS)/screenshots/$$scene.png; \
	    if [ ! -f $$base ]; then \
	        echo "  no baseline for $$scene (cp $$current $$base to accept it)"; \
	        continue; \
	    fi; \
	    rmse=$$("$(MAGICK)" compare -metric RMSE $$base $$current \
	        $(ARTIFACTS)/visual-diff/$$scene.png 2>&1 | sed 's/.*(\(.*\))/\1/'); \
	    echo "  $$scene RMSE $$rmse"; \
	    awk -v v="$$rmse" 'BEGIN { exit (v+0 > 0.02) ? 1 : 0 }' || \
	        { echo "  ! $$scene differs beyond tolerance"; status=1; }; \
	done; \
	exit $$status
endif

# ----------------------------------------------------------------------------- fuzzing --

fuzz:
ifeq ($(CLANG),)
	@echo "SKIP fuzz: clang with libFuzzer not installed. The scheduled CI job runs these." >&2
else
	@mkdir -p $(ARTIFACTS)/fuzz
	@for target in fuzz/fuzz_*.c; do \
	    name=$$(basename $$target .c); \
	    echo "  building $$name"; \
	    $(CLANG) $(CSTD) $(INCLUDES) -O1 -g -DDRIFTY_HEADLESS \
	        -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all \
	        $$target src/dev_params.c src/dev_replay.c src/vehicle.c src/tire.c \
	        src/replay.c src/math_utils.c src/input.c \
	        -o $(ARTIFACTS)/fuzz/$$name $(RAYLIB_CFLAGS) -lm || exit 1; \
	    $(ARTIFACTS)/fuzz/$$name -max_total_time=$(FUZZ_SECONDS) \
	        -artifact_prefix=$(ARTIFACTS)/fuzz/ || exit 1; \
	done
endif
FUZZ_SECONDS ?= 20

# ---------------------------------------------------------------------------- profiling --

# Tracy is opt-in and vendored by the developer: drop the distribution in third_party/tracy
# and this target compiles its client in. Without it the built-in zone timers are used, which
# need no dependency at all and print a summary at exit.
profile: windows-only
	@if [ -f third_party/tracy/public/TracyClient.cpp ]; then \
	    echo "Building with Tracy (third_party/tracy found)."; \
	    DRIFTY_EXTRA_DEFINES=-DDRIFTY_TRACY ./build.sh; \
	else \
	    echo "third_party/tracy not present — building with the built-in zone timers."; \
	    echo "See docs/DEVTOOLS.md for how to add Tracy."; \
	    DRIFTY_EXTRA_DEFINES=-DDRIFTY_PROFILE ./build.sh; \
	fi

# ---------------------------------------------------------------------------------- CI --

ci: format-check lint analyze test-physics regression sanitize coverage
	@echo ""
	@echo "==============================================="
	@echo "ci: every required check passed locally."
	@echo "Windows-only checks (screenshots, visual-test) are not part of the required set."

# ---------------------------------------------------------------------- editor support --

compile-commands:
	$(PYTHON) tools/gen_compile_commands.py --output compile_commands.json \
	    --raylib-cflags "$(RAYLIB_CFLAGS)"

# -------------------------------------------------------------------------- housekeeping --

clean:
	rm -rf build coverage $(ARTIFACTS)/fuzz $(ARTIFACTS)/plots $(ARTIFACTS)/screenshots
	rm -f $(EXE_DEBUG) $(EXE_RELEASE) $(EXE_TESTS) drifty_hotreload_harness$(EXE_SUFFIX)
	rm -f drifty_tests_asan$(EXE_SUFFIX) drifty_tests_cov$(EXE_SUFFIX)
	rm -f libraylib.dll raylib.dll glfw3.dll
	rm -f *.gcda *.gcno *.gcov
	rm -f $(TELEMETRY)/phase1_smoke.png $(TELEMETRY)/phase2_smoke.png $(TELEMETRY)/phase3_smoke.png
	rm -f *.o src/*.o tests/*.o *.pdb

clean-telemetry:
	rm -f $(TELEMETRY)/*.csv $(TELEMETRY)/*.png
