#!/usr/bin/env python3
"""Release bundle packaging (issue #46).

Assembles a self-contained, CWD-independent release bundle under build/package/:

    <bundle>/circuit_release.exe
    <bundle>/glfw3.dll                 (release runtime dependency, from build/release/)
    <bundle>/data/                     (selectable content: tracks, vehicles, classes)
    <bundle>/THIRD_PARTY_NOTICES.txt   (license provenance)
    <bundle>/LICENSE                   (project license, when present)
    <bundle>/MANIFEST.json             (sha256 per artifact + build provenance)
    <bundle>/LAUNCH.md                 (how to run from any directory)

The bundle is runnable from any working directory: content discovery resolves against the
executable's own directory (src/core/content_paths.c). Pure stdlib.
"""

import hashlib
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RELEASE_DIR = ROOT / "build" / "release"
BUNDLE_ROOT = ROOT / "build" / "package"
BUNDLE_NAME = "circuit-release"

NOTICES_SOURCE = ROOT / "third_party" / "README.md"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    if not (RELEASE_DIR / "circuit_release.exe").exists():
        print(
            f"make_bundle: {RELEASE_DIR / 'circuit_release.exe'} missing — run "
            "`make release` first",
            file=sys.stderr,
        )
        return 2

    bundle = BUNDLE_ROOT / BUNDLE_NAME
    if bundle.exists():
        shutil.rmtree(bundle)
    (bundle / "data").mkdir(parents=True, exist_ok=True)

    copies = [
        (RELEASE_DIR / "circuit_release.exe", bundle / "circuit_release.exe"),
        (RELEASE_DIR / "glfw3.dll", bundle / "glfw3.dll"),
    ]
    missing = [src for src, _ in copies if not src.exists()]
    if missing:
        print(f"make_bundle: missing runtime artifacts: {missing}", file=sys.stderr)
        return 2

    manifest = {"schema": "circuit/release-bundle", "version": 1, "files": {}}

    for src, dst in copies:
        shutil.copy2(src, dst)
        manifest["files"][dst.name] = {"sha256": sha256(dst), "size": dst.stat().st_size}

    # Content: the full data/ tree.
    data_src = ROOT / "data"
    for src in sorted(data_src.rglob("*")):
        if src.is_file():
            rel = src.relative_to(ROOT)
            dst = bundle / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            manifest["files"][rel.as_posix()] = {"sha256": sha256(dst), "size": dst.stat().st_size}

    # Notices and license.
    if NOTICES_SOURCE.exists():
        shutil.copy2(NOTICES_SOURCE, bundle / "THIRD_PARTY_NOTICES.txt")
        manifest["files"]["THIRD_PARTY_NOTICES.txt"] = {
            "sha256": sha256(bundle / "THIRD_PARTY_NOTICES.txt"),
            "size": (bundle / "THIRD_PARTY_NOTICES.txt").stat().st_size,
        }
    license_src = ROOT / "LICENSE"
    if license_src.exists():
        shutil.copy2(license_src, bundle / "LICENSE")

    manifest["provenance"] = {
        "built_from": str(ROOT),
        "content": "data/ (tracks, vehicles, classes)",
        "runtime_deps": ["glfw3.dll (GLFW, Zlib license — see THIRD_PARTY_NOTICES.txt)"],
    }
    with (bundle / "MANIFEST.json").open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")

    with (bundle / "LAUNCH.md").open("w", encoding="utf-8") as f:
        f.write(
            "# Launch\n\n"
            "Run `circuit_release.exe` from any directory. Content (data/) resolves relative\n"
            "to the executable, not the working directory (issue #46).\n\n"
            "Interactive: double-click or `./circuit_release.exe`.\n"
            "Bounded smoke: `./circuit_release.exe --smoke-test` (writes artifacts/ beside the\n"
            "working directory and exits).\n\n"
            "Requires glfw3.dll next to the executable (included).\n"
        )

    print(f"bundle: {bundle}")
    print(f"  files: {sum(1 for v in manifest['files'].values() if v.get('size', 0) > 0)}")
    print(f"  bytes: {sum(v['size'] for v in manifest['files'].values())}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
