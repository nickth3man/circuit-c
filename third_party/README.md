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

## stb/stb_image_write.h — stb_image_write v1.16

- Upstream: <https://github.com/nothings/stb>
- Obtained from: `.slim/clonedeps/repos/raysan5__raylib/src/external/stb_image_write.h`, the
  copy shipped inside the raylib 6.0 source tree this project builds against. No download was
  needed and no byte was changed (sha256
  `17f339e7582ff4d4d56e40b0e87d77eda8d8f8e62806278ddf8fcf0b8b3d9a84`).
- Licence: dual public domain / MIT, Sean Barrett. The full text is in the header's own banner.

stb_image_write is a **test-only** dependency, used to write the vehicle contact sheet from
`build/tests/drifty_tests.exe` without a GPU or a window. `tests/car_sheet.c` is the single translation
unit that defines `STB_IMAGE_WRITE_IMPLEMENTATION`. Neither `build/dev/game.dll`, `build/dev/drifty.exe`,
nor `build/release/drifty_release.exe` links it.

raygui is a **development-only** dependency. It is compiled into `build/dev/game.dll` only when
`DRIFTY_DEV_TOOLS` is defined (the default for `build.sh` / `build.bat` development builds)
and `DRIFTY_HEADLESS` is not. `src/dev/dev_lab.c` is the single translation unit that defines
`RAYGUI_IMPLEMENTATION`. Release builds (`--release`) and the headless test executable link
none of it, which is why `build/release/drifty_release.exe` and `build/tests/drifty_tests.exe` are unaffected.
