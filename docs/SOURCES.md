# Technical References — Drifty

This document is the reference index for [SPEC.md](SPEC.md). Every physics equation,
integration rule, raylib API claim, and build instruction in the specification traces
back to one of the sources listed here.

Sources are grouped by subject. Each entry gives the short identifier used elsewhere in
the project (`S1`–`S20`), the title, a description of what the source establishes, and
the URL.

---

## Vehicle Dynamics and Tire Models

### S1 — MPCC nonlinear bicycle model implementation

Front/rear slip angles, Magic Formula tire forces, and planar body equations of motion.

<https://github.com/alexliniger/MPCC/blob/master/C%2B%2B/Model/model.cpp>

### S2 — MPCC project documentation

Dynamic bicycle model, tire constraints, and the mixed kinematic/dynamic low-speed model.

<https://github.com/alexliniger/MPCC>

### S3 — Marco Monster, *Car Physics for Games*

SI units, static and dynamic axle loads, longitudinal load transfer, wheel angular
speed, traction force, and slip ratio.

<https://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html>

---

## Tire Saturation and Drifting Research

### S4 — Autonomous drifting research

Nonlinear and coupled (combined-slip) tire forces near the friction limit.

<https://arxiv.org/abs/2407.13760>

### S5 — RAGE vehicle-dynamics paper

Dynamic vertical loads, combined slip, and drivetrain torque limitations.

<https://arxiv.org/abs/2604.02892>

---

## Simulation Timestep and Integration

### S6 — Glenn Fiedler, *Fix Your Timestep*

Fixed timestep, the accumulator pattern, and spiral-of-death guidance.

<https://www.gafferongames.com/post/fix_your_timestep/>

### S7 — Glenn Fiedler, *Integration Basics*

Semi-implicit (symplectic) Euler integration.

<https://gafferongames.com/post/integration_basics/>

---

## raylib Documentation

### S8 — Official raylib 6.0 header

`Camera2D` zoom semantics and the file-memory APIs (`LoadFileText` / `UnloadFileText` /
`SaveFileText` / `FileExists`).

<https://raw.githubusercontent.com/raysan5/raylib/6.0/src/raylib.h>

### S9 — Official raylib `Camera2D` zoom example

Demonstrates that increasing `Camera2D.zoom` magnifies the world.

<https://github.com/raysan5/raylib/blob/master/examples/core/core_2d_camera_mouse_zoom.c>

### S10 — Official raylib build configuration

Shows that module inclusion (`SUPPORT_MODULE_*`) is resolved when raylib itself is
compiled, not when a consumer application is compiled.

<https://github.com/raysan5/raylib/blob/master/src/CMakeLists.txt>

### S14 — Official raylib changelog

Version history and API changes between releases.

<https://github.com/raysan5/raylib/blob/master/CHANGELOG>

---

## Reference-Game Developer Material

These sources describe iterative handling refinement, extensive parameter tuning, and
player testing. They do not publicly document tire, drivetrain, or body equations, and
are therefore used only as *handling and feel* references.

### S11 — Art of Rally developer interview

Describes a shared foundation with Absolute Drift, refined to be more predictable.

<https://www.teamvvv.com/interviews/art-of-rally-gameplay-and-developer-interview/>

### S12 — Art of Rally developer interview on handling iteration and testing

<https://gamingbolt.com/art-of-rally-interview-art-style-development-and-more>

---

## Hot Reload and Development Workflow

### S15 — musializer hot-reload implementation

A C + raylib hot-reload platform layer. Establishes the X-macro entry-point list, the
`pre_reload` / `post_reload` state handoff, and parallel Windows and POSIX loaders.

<https://github.com/tsoding/musializer/blob/master/src/hotreload_windows.c>
<https://github.com/tsoding/musializer/blob/master/src/plug.h>

### S16 — panim plugin architecture

The same pattern generalized to multiple swappable plugin modules, also C + raylib.

<https://github.com/tsoding/panim>

### S17 — Odin + Raylib hot-reload game template

Source of the build-script design: rebuild the game module always, rebuild the executable
only when it is not already running, and exit immediately either way. Also documents the
temp-file rename that avoids loading a partially written module, and per-build PDB
numbering for debugging on Windows.

<https://github.com/karl-zylinski/odin-raylib-hot-reload-game-template>

### S18 — Karl Zylinski, *Hot Reload Gameplay Code*

Limitations and mitigations: stale procedure pointers, struct-layout invalidation, and the
requirement that raylib be linked as a shared library so that reloading the game module
does not destroy raylib's global state.

<https://zylinski.se/posts/hot-reload-gameplay-code/>

### S19 — Handmade-Hero-style game code loading

Historical reference for loading a game module behind a function table (POSIX systems used
`dlopen` / `dlsym`; Drifty implements the same pattern with `LoadLibrary` /
`GetProcAddress` on Windows only). Origin of the platform-layer-owns-the-memory approach.

<https://github.com/TheoBendixson/3D-Game-Engine-Template/blob/master/code/mac_platform/mac_game_code.h>

### S20 — Linking raylib with hot reloading

Discussion of the shared-versus-static raylib linking requirement for hot-reload builds.

<https://stackoverflow.com/questions/78604384/how-to-link-raylib-with-hot-reloading-in-c-windows-linux-platforms>

---

## Build and Package Documentation

### S13 — Official MSYS2 raylib package information

Package contents and version for `mingw-w64-ucrt-x86_64-raylib`.

<https://packages.msys2.org/packages/mingw-w64-ucrt-x86_64-raylib>
