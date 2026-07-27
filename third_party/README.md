# third_party

Vendored, unmodified third-party sources. Nothing here is edited by this project; update by
replacing a file wholesale and recording the provenance below.

## raygui/raygui.h — raygui 5.0-dev (`RAYGUI_VERSION_MAJOR 4`, `MINOR 5`, `PATCH 0`)

- Upstream: <https://github.com/raysan5/raygui>
- Obtained from: `.slim/clonedeps/repos/raysan5__raylib/examples/core/raygui.h`, the copy
  shipped inside the raylib 6.0 source tree this project builds against. No download was
  needed and no byte was changed.
- Licence: zlib/libpng, Copyright (c) 2014-2026 Ramon Santamaria (@raysan5). The full
  licence text is in the header's own banner comment.

raygui is a **development-only** dependency. It is compiled into `build/game.dll` only when
`DRIFTY_DEV_TOOLS` is defined (the default for `build.sh` / `build.bat` development builds)
and `DRIFTY_HEADLESS` is not. `src/dev_lab.c` is the single translation unit that defines
`RAYGUI_IMPLEMENTATION`. Release builds (`--release`) and the headless test executable link
none of it, which is why `drifty_release.exe` and `drifty_tests.exe` are unaffected.
