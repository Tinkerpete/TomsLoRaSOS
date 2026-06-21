#!/usr/bin/env python3
"""
Copy the official LilyGoLib Factory HAL files required by the outdoor prototype
into this Arduino sketch.

The script searches common Arduino library folders and accepts an explicit
LilyGoLib path when needed.
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
DEST = SCRIPT_DIR / "src" / "vendor_factory"

LIBRARY_DIR_CANDIDATES = [
    Path.home() / "Documents" / "Arduino" / "libraries",
    Path.home() / "OneDrive" / "Documents" / "Arduino" / "libraries",
    Path.home() / "Arduino" / "libraries",
]

FILES = [
    "hal_interface.cpp",
    "hal_interface.h",
    "event_define.h",
    "app_nfc.cpp",
    "app_nfc.h",
    "hw_sx1262.cpp",
    "hw_lr1121.cpp",
    "hw_sx1280.cpp",
    "hw_cc1101.cpp",
    "hw_nrf2401.cpp",
]

DIRS = [
    "audio",
]


def factory_dir(lilygolib: Path) -> Path:
    return lilygolib / "examples" / "factory"


def is_valid_root(path: Path) -> bool:
    return (factory_dir(path) / "hal_interface.h").exists()


def expand_candidate(candidate: Path) -> list[Path]:
    result: list[Path] = []
    if candidate.exists():
        result.append(candidate)
        if candidate.is_dir():
            for child in sorted(candidate.iterdir()):
                if child.is_dir() and "lilygolib" in child.name.lower():
                    result.append(child)
    return result


def locate(explicit: str | None) -> Path:
    candidates: list[Path] = []

    if explicit:
        candidates.append(Path(explicit).expanduser())

    env = os.environ.get("LILYGOLIB_PATH")
    if env:
        candidates.append(Path(env).expanduser())

    for library_dir in LIBRARY_DIR_CANDIDATES:
        candidates.extend(expand_candidate(library_dir))

    # Also try direct conventional paths.
    candidates.extend([
        Path.home() / "Documents" / "Arduino" / "libraries" / "LilyGoLib",
        Path.home() / "OneDrive" / "Documents" / "Arduino" / "libraries" / "LilyGoLib",
        Path.home() / "Arduino" / "libraries" / "LilyGoLib",
    ])

    seen: set[Path] = set()
    for candidate in candidates:
        try:
            normalized = candidate.resolve()
        except OSError:
            normalized = candidate
        if normalized in seen:
            continue
        seen.add(normalized)
        if is_valid_root(candidate):
            return candidate.resolve()

    searched = "\n".join(f"  - {c}" for c in sorted(seen, key=str))
    raise SystemExit(
        "LilyGoLib wurde nicht gefunden.\n\n"
        "Suche in der Arduino IDE zunächst das offizielle Factory-Beispiel und "
        "ermittle damit den Library-Ordner. Starte danach:\n\n"
        '  py prepare_vendor_files.py --lilygolib "C:\\\\PFAD\\\\ZU\\\\LilyGoLib"\n\n'
        f"Durchsuchte Pfade:\n{searched}"
    )


def copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    print(f"kopiert: {src.name}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lilygolib", help="Pfad zur installierten LilyGoLib-Library")
    args = parser.parse_args()

    library = locate(args.lilygolib)
    source = factory_dir(library)

    print(f"Verwende LilyGoLib: {library}")
    print(f"Factory-HAL:        {source}\n")

    if DEST.exists():
        shutil.rmtree(DEST)
    DEST.mkdir(parents=True)

    missing: list[str] = []
    for rel in FILES:
        src = source / rel
        if src.exists():
            copy_file(src, DEST / rel)
        else:
            missing.append(rel)

    for rel in DIRS:
        src = source / rel
        if src.exists():
            shutil.copytree(src, DEST / rel)
            print(f"kopiert: {rel}/")
        else:
            missing.append(rel + "/")

    marker = DEST / "VENDOR_SOURCE.txt"
    marker.write_text(
        "Official Factory HAL copied from:\n"
        f"{source}\n",
        encoding="utf-8",
    )

    if not (DEST / "hal_interface.h").exists():
        raise SystemExit("\nFEHLER: hal_interface.h konnte nicht kopiert werden.")

    if missing:
        print("\nHinweis: In dieser LilyGoLib-Version fehlten optionale Dateien:")
        for rel in missing:
            print(f"  - {rel}")

    print("\nFertig.")
    print(f"Vorhanden: {DEST / 'hal_interface.h'}")
    print("Öffne nun TomsLoRaSOS.ino in der Arduino IDE.")


if __name__ == "__main__":
    main()
