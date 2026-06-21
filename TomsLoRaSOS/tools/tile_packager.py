#!/usr/bin/env python3
"""
Download an explicitly selected small tile region from a tile server that
permits offline downloads.

IMPORTANT:
Do not point this tool at https://tile.openstreetmap.org/.
The public OSM standard tile service forbids bulk/offline prefetching.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from urllib.parse import urlparse

import requests

TILE_SIZE = 256
OSM_STANDARD_HOST = "tile.openstreetmap.org"


def latlon_to_tile(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    lat = max(-85.05112878, min(85.05112878, lat))
    lat_rad = math.radians(lat)
    n = 2 ** zoom
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return x, y


def meters_per_tile(lat: float, zoom: int) -> float:
    meters_per_pixel = 156543.03392 * math.cos(math.radians(lat)) / (2 ** zoom)
    return meters_per_pixel * TILE_SIZE


def tile_range(lat: float, lon: float, radius_km: float, zoom: int):
    cx, cy = latlon_to_tile(lat, lon, zoom)
    radius_tiles = radius_km * 1000.0 / meters_per_tile(lat, zoom)
    min_x = math.floor(cx - radius_tiles)
    max_x = math.floor(cx + radius_tiles)
    min_y = math.floor(cy - radius_tiles)
    max_y = math.floor(cy + radius_tiles)
    n = 2 ** zoom

    for x in range(min_x, max_x + 1):
        for y in range(max(0, min_y), min(n - 1, max_y) + 1):
            yield x % n, y


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tile-url", required=True, help="Template containing {z}, {x}, {y}")
    parser.add_argument("--lat", required=True, type=float)
    parser.add_argument("--lon", required=True, type=float)
    parser.add_argument("--radius-km", required=True, type=float)
    parser.add_argument("--zooms", required=True, nargs="+", type=int)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--delay", default=0.12, type=float, help="Delay between requests")
    parser.add_argument("--user-agent", default="LilyGoPagerOfflinePackager/0.1")
    args = parser.parse_args()

    host = (urlparse(args.tile_url).hostname or "").lower()
    if host == OSM_STANDARD_HOST or host.endswith("." + OSM_STANDARD_HOST):
        raise SystemExit(
            "Abbruch: Der öffentliche OSM-Standardserver darf nicht für "
            "Offline-Prefetching verwendet werden. Nutze einen eigenen oder "
            "ausdrücklich dafür freigegebenen Tile-Server."
        )

    if not all(token in args.tile_url for token in ("{z}", "{x}", "{y}")):
        raise SystemExit("--tile-url muss {z}, {x} und {y} enthalten.")

    session = requests.Session()
    session.headers["User-Agent"] = args.user_agent

    jobs: list[tuple[int, int, int]] = []
    for z in args.zooms:
        jobs.extend((z, x, y) for x, y in tile_range(args.lat, args.lon, args.radius_km, z))

    print(f"{len(jobs)} Kacheln vorgesehen.")
    args.out.mkdir(parents=True, exist_ok=True)

    for index, (z, x, y) in enumerate(jobs, start=1):
        dst = args.out / str(z) / str(x) / f"{y}.png"
        if dst.exists():
            print(f"[{index}/{len(jobs)}] vorhanden: {z}/{x}/{y}")
            continue

        dst.parent.mkdir(parents=True, exist_ok=True)
        url = args.tile_url.format(z=z, x=x, y=y)
        print(f"[{index}/{len(jobs)}] lade: {z}/{x}/{y}")
        response = session.get(url, timeout=20)
        response.raise_for_status()

        content_type = response.headers.get("Content-Type", "")
        if "image" not in content_type.lower():
            raise RuntimeError(f"Keine Bildantwort für {url}: {content_type}")

        tmp = dst.with_suffix(".tmp")
        tmp.write_bytes(response.content)
        tmp.replace(dst)
        time.sleep(max(0.0, args.delay))

    print("Fertig. Kopiere den erzeugten Ordnerinhalt nach /maps auf die SD-Karte.")


if __name__ == "__main__":
    main()
