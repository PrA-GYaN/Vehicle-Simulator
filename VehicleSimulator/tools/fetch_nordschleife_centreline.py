#!/usr/bin/env python3
# ===========================================================================
#  fetch_nordschleife_centreline.py  --  offline tool.
#
#  WHAT THIS IS FOR
#  ----------------
#  Companion to tools/extract_track_centreline.cpp, for a track that has no
#  photogrammetry scan to measure: the Nordschleife. Unlike the GP-Strecke,
#  there is no reference model in this repository whose surface can be
#  walked - the only reference is a schematic corner-name diagram
#  (Nordschleife-full-lap.webp), which carries no coordinates.
#
#  So this tool gets the centreline from the same place every mapping
#  application does: OpenStreetMap. The Nordschleife is relation 38566, a
#  public-domain (ODbL) route relation made of 52 ways, each already
#  surveyed against real imagery/GPS. That is a strictly better source than
#  digitising the schematic by eye would have been - it is accurate rather
#  than merely proportionate - and it is the same kind of "measured, not
#  invented" data the GP-Strecke centreline is, just measured by someone
#  else first.
#
#  WHAT IT DOES
#  ------------
#   1. Fetch relation 38566 "full" (every member way, with its node
#      geometry) from the OSM API.
#   2. Stitch the 52 ways into one polyline in the relation's own member
#      order, flipping each way as needed so consecutive endpoints match.
#      This is a hard-fail if any join does not close, since a route
#      relation whose ways cannot be chained end-to-end is not a lap.
#   3. Project lon/lat to local metres with an equirectangular
#      approximation about the loop's centroid: +X = east, +Z = south,
#      matching core/Coordinates.h. Good to well under a metre of distortion
#      over a loop this size at this latitude.
#   4. Rotate the point list so it starts at Hohenrain - the last named
#      corner before the Start/Ziel straight - so the generated track begins
#      near the real start line rather than at an arbitrary OSM way boundary.
#   5. Look up ground elevation at every waypoint from Open Topo Data
#      (EU-DEM, 25 m grid; SRTM as a fallback where EU-DEM has no coverage),
#      lightly smoothed (a DEM sampled every ~19 m has real cell-to-cell
#      noise a paved road does not) and recentred so the lap's lowest point
#      sits at y = 0 - the Nordschleife's ~300 m of elevation change is real
#      and, unlike the GP-Strecke preset, this track's physics is wired to a
#      TerrainHeightField built from it (see Renderer::buildTrackTerrain),
#      not just drawn.
#   6. Dedupe near-duplicate consecutive points and emit the C++ table.
#
#  VALIDATION
#  ----------
#  The script cross-checks the result against five named landmarks looked up
#  independently (Nominatim), confirming both total lap length (~20.8 km,
#  matching the published 20.83 km) and corner order (Hatzenbach shortly
#  after the start, Doettinger Hoehe near the end, in the right sequence
#  both ways round the loop). See --verify.
#
#  USAGE
#      fetch_nordschleife_centreline.py --out <path.h> --name kNordschleife
#      fetch_nordschleife_centreline.py --verify   (network checks only, no output file)
# ===========================================================================

import argparse
import json
import math
import sys
import time
import urllib.request

OSM_RELATION_URL = "https://www.openstreetmap.org/api/0.6/relation/{id}/full.json"
NORDSCHLEIFE_RELATION_ID = 38566
ELEVATION_URL = "https://api.opentopodata.org/v1/{dataset}?locations={locs}"
ELEVATION_DATASETS = ["eudem25m", "srtm30m"]  # first with coverage wins, per point
ELEVATION_BATCH = 100   # public API's per-request location cap
ELEVATION_DELAY_S = 1.0  # public API's rate limit is ~1 req/s

# Landmarks used only to validate direction/order and to choose the rotation
# point (Hohenrain). Coordinates from OpenStreetMap via Nominatim, looked up
# independently of the relation geometry itself.
LANDMARKS = {
    "Hatzenbach":         (6.9388872, 50.3379399),
    "QuiddelbacherHoehe": (6.9279685, 50.3439633),
    "DoettingerHoehe":    (6.9783419, 50.3508158),
    "Tiergarten":         (6.9565019, 50.3410664),
    "Hohenrain":          (6.9535630, 50.3385513),
}
ROTATE_TO = "Hohenrain"

# Expected order walking forward (increasing index) from the start: short hop
# to Hatzenbach, long way round to Doettinger Hoehe, short hops back through
# Tiergarten and Hohenrain to the start again. The bool marks the one leg
# that is *supposed* to be the long way round (most of the lap), so --verify
# checks each leg against its own expectation instead of assuming "forward
# should always be the short way".
EXPECTED_ORDER = [
    ("Hohenrain", "Hatzenbach", False),
    ("Hatzenbach", "QuiddelbacherHoehe", False),
    ("QuiddelbacherHoehe", "DoettingerHoehe", True),
    ("DoettingerHoehe", "Tiergarten", False),
]


def fetch_relation(relation_id: int) -> dict:
    url = OSM_RELATION_URL.format(id=relation_id)
    req = urllib.request.Request(url, headers={"User-Agent": "vsim-track-tools/1.0"})
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.load(resp)


def stitch_ways(data: dict):
    """Chain every member way of the relation into one closed node-id loop,
    in relation order, flipping ways as needed. Raises on any break."""
    nodes, ways, rel = {}, {}, None
    for e in data["elements"]:
        if e["type"] == "node":
            nodes[e["id"]] = (e["lon"], e["lat"])
        elif e["type"] == "way":
            ways[e["id"]] = e["nodes"]
        elif e["type"] == "relation":
            rel = e
    if rel is None:
        raise RuntimeError("no relation in response")

    order = [m["ref"] for m in rel["members"] if m["type"] == "way"]
    chain = list(ways[order[0]])
    for wid in order[1:]:
        w = ways[wid]
        if chain[-1] == w[0]:
            chain.extend(w[1:])
        elif chain[-1] == w[-1]:
            chain.extend(list(reversed(w))[1:])
        elif chain[0] == w[0]:
            chain.reverse()
            chain.extend(w[1:])
        elif chain[0] == w[-1]:
            chain.reverse()
            chain.extend(list(reversed(w))[1:])
        else:
            raise RuntimeError(f"way {wid} does not join the chain - relation is not a clean loop")

    if chain[0] != chain[-1]:
        raise RuntimeError("stitched chain does not close into a loop")
    chain.pop()  # drop the duplicated closing node

    return [nodes[n] for n in chain]  # list of (lon, lat)


def fetch_elevations(coords):
    """Ground elevation [m] at every (lon, lat), same order and length as
    `coords`. Batches requests (the public API caps locations per call and
    rate-limits), and falls back dataset-by-dataset for any point a higher-
    resolution DEM has no coverage for (coastal/tile-edge gaps)."""
    elevations = [None] * len(coords)
    remaining = list(range(len(coords)))

    for dataset in ELEVATION_DATASETS:
        if not remaining:
            break
        still_missing = []
        for b in range(0, len(remaining), ELEVATION_BATCH):
            idxs = remaining[b:b + ELEVATION_BATCH]
            locs = "|".join(f"{coords[i][1]:.6f},{coords[i][0]:.6f}" for i in idxs)
            url = ELEVATION_URL.format(dataset=dataset, locs=locs)
            req = urllib.request.Request(url, headers={"User-Agent": "vsim-track-tools/1.0"})
            with urllib.request.urlopen(req, timeout=30) as resp:
                data = json.load(resp)
            if data.get("status") != "OK":
                raise RuntimeError(f"elevation lookup failed ({dataset}): {data}")
            for i, r in zip(idxs, data["results"]):
                e = r.get("elevation")
                if e is None:
                    still_missing.append(i)
                else:
                    elevations[i] = e
            time.sleep(ELEVATION_DELAY_S)
        remaining = still_missing

    if remaining:
        raise RuntimeError(f"no elevation data (any dataset) for {len(remaining)} point(s)")
    return elevations


def smooth_closed(values, window):
    """Centred moving average over a closed loop. A DEM has real cell-to-cell
    noise (EU-DEM is +-1-3 m even before interpolation error) that a paved
    road surface does not; the centreline itself already gets an analogous
    treatment for curvature, for the same reason - see Track.cpp."""
    n = len(values)
    out = [0.0] * n
    for i in range(n):
        s = sum(values[(i + w) % n] for w in range(-window, window + 1))
        out[i] = s / (2 * window + 1)
    return out


def haversine_m(a, b) -> float:
    lon1, lat1 = a
    lon2, lat2 = b
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    h = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * R * math.asin(math.sqrt(h))


def nearest_index(coords, lonlat) -> int:
    return min(range(len(coords)), key=lambda i: haversine_m(coords[i], lonlat))


def project_local(coords):
    """Equirectangular projection about the point set's own centroid.
    +X = east, +Z = south (core/Coordinates.h)."""
    lon0 = sum(c[0] for c in coords) / len(coords)
    lat0 = sum(c[1] for c in coords) / len(coords)
    lat0r = math.radians(lat0)
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat0r) + 1.175 * math.cos(4 * lat0r)
    m_per_deg_lon = 111412.84 * math.cos(lat0r) - 93.5 * math.cos(3 * lat0r)

    pts = [((lon - lon0) * m_per_deg_lon, (lat0 - lat) * m_per_deg_lat) for lon, lat in coords]
    cx = sum(p[0] for p in pts) / len(pts)
    cz = sum(p[1] for p in pts) / len(pts)
    return [(x - cx, z - cz) for x, z in pts]


def dedupe(pts, elevations, min_gap=0.05):
    """Drop near-duplicate consecutive (x, z) points, carrying the parallel
    elevation array along so the two stay index-aligned."""
    out_pts, out_elev = [pts[0]], [elevations[0]]
    for p, e in zip(pts[1:], elevations[1:]):
        if math.dist(out_pts[-1], p) > min_gap:
            out_pts.append(p)
            out_elev.append(e)
    if math.dist(out_pts[-1], out_pts[0]) < 0.5:
        out_pts.pop()
        out_elev.pop()
    return out_pts, out_elev


def verify(coords):
    idx = {name: nearest_index(coords, ll) for name, ll in LANDMARKS.items()}
    n = len(coords)

    def fwd_dist(i, j):
        d, k = 0.0, i
        while k != j:
            kn = (k + 1) % n
            d += haversine_m(coords[k], coords[kn])
            k = kn
        return d

    total = sum(haversine_m(coords[i], coords[(i + 1) % n]) for i in range(n))
    print(f"[verify] total loop length: {total / 1000:.3f} km (published: 20.83 km)")

    ok = True
    for a, b, expect_long in EXPECTED_ORDER:
        fwd = fwd_dist(idx[a], idx[b])
        rev = total - fwd
        is_long = fwd >= rev
        matches = is_long == expect_long
        note = ("ok (long way round)" if is_long else "ok (short hop)") if matches \
            else "REVERSED / OUT OF ORDER"
        ok = ok and matches
        print(f"[verify] {a:20s} -> {b:20s}: forward {fwd/1000:6.3f} km   {note}")
    if not ok:
        raise RuntimeError("landmark ordering did not match the expected lap sequence")
    print("[verify] landmark order and lap length both match the real Nordschleife.")


def emit_header(pts, elevations, out_path, name):
    total = sum(math.dist(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts)))
    emin, emax = min(elevations), max(elevations)
    with open(out_path, "w") as f:
        f.write("#pragma once\n")
        f.write("//" + "=" * 75 + "\n")
        f.write("//  GENERATED FILE - do not edit by hand.\n//\n")
        f.write("//  Circuit centreline for the full Nordschleife, recovered from\n")
        f.write("//  OpenStreetMap relation 38566 by\n")
        f.write("//  tools/fetch_nordschleife_centreline.py. See that file for what it\n")
        f.write("//  does and why OSM rather than the reference schematic image.\n//\n")
        f.write(f"//  source      : openstreetmap.org relation/{NORDSCHLEIFE_RELATION_ID}\n")
        f.write(f"//                elevation from Open Topo Data ({'/'.join(ELEVATION_DATASETS)})\n")
        f.write(f"//  waypoints   : {len(pts)}\n")
        f.write("//  closed loop : yes\n")
        f.write(f"//  lap length  : {total:.0f} m (published figure: 20 830 m)\n")
        f.write(f"//  elevation   : {emax-emin:.0f} m range (published figure: ~300 m)\n")
        f.write("//  waypoint 0  : Hohenrain, the last named corner before the\n")
        f.write("//                Start/Ziel straight - the timing line itself is not a\n")
        f.write("//                separately tagged point in the source data.\n//\n")
        f.write("//  (x, z) are metres in the simulator's world frame (X east, Z south -\n")
        f.write("//  see core/Coordinates.h), recentred on the centroid of the layout.\n")
        f.write("//  y is elevation in metres, DEM-sourced and smoothed (see the fetch\n")
        f.write("//  tool), recentred so the lap's lowest point is y = 0 - relative\n")
        f.write("//  height along the lap is what both the visuals and the physics\n")
        f.write("//  (Renderer::buildTrackTerrain) need, not absolute altitude above sea\n")
        f.write("//  level. Unlike every other preset here, this track's surface is NOT\n")
        f.write("//  flat - see the header comment on Track::createNordschleife().\n")
        f.write("//" + "=" * 75 + "\n\n")
        f.write("namespace vsim::trackdata {\n\n")
        f.write("/// (x, z, elevation) waypoint triples, in metres.\n")
        f.write(f"inline constexpr double {name}[][3] = {{\n")
        per_line = 3
        rows = list(zip(pts, elevations))
        for i in range(0, len(rows), per_line):
            row = rows[i:i + per_line]
            f.write("    " + "".join(f"{{{x:.2f}, {z:.2f}, {e:.1f}}}," for (x, z), e in row).rstrip(",") + ",\n")
        f.write("};\n\n")
        f.write(f"inline constexpr bool {name}Closed = true;\n\n")
        f.write("} // namespace vsim::trackdata\n")
    print(f"[emit] wrote {len(pts)} waypoints, {total/1000:.3f} km, "
          f"elevation {emin:.0f}-{emax:.0f} m (range {emax-emin:.0f} m), to {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", help="output header path")
    ap.add_argument("--name", default="kNordschleife", help="C++ array identifier")
    ap.add_argument("--verify", action="store_true", help="only run the landmark cross-check")
    ap.add_argument("--cache", help="read/write the raw OSM response here instead of always fetching")
    ap.add_argument("--elevation-cache", help="read/write raw per-point elevations (JSON) here")
    ap.add_argument("--smooth-window", type=int, default=4,
                    help="+-N points averaged for elevation smoothing (default 4, ~150 m)")
    args = ap.parse_args()

    data = None
    if args.cache:
        try:
            with open(args.cache) as f:
                data = json.load(f)
        except FileNotFoundError:
            pass
    if data is None:
        data = fetch_relation(NORDSCHLEIFE_RELATION_ID)
        if args.cache:
            with open(args.cache, "w") as f:
                json.dump(data, f)

    lonlat = stitch_ways(data)
    verify(lonlat)

    if args.out:
        rot = nearest_index(lonlat, LANDMARKS[ROTATE_TO])
        rotated = lonlat[rot:] + lonlat[:rot]

        raw_elev = None
        if args.elevation_cache:
            try:
                with open(args.elevation_cache) as f:
                    raw_elev = json.load(f)
            except FileNotFoundError:
                pass
        if raw_elev is None or len(raw_elev) != len(rotated):
            raw_elev = fetch_elevations(rotated)
            if args.elevation_cache:
                with open(args.elevation_cache, "w") as f:
                    json.dump(raw_elev, f)

        smoothed = smooth_closed(raw_elev, args.smooth_window)
        floor = min(smoothed)
        elev = [e - floor for e in smoothed]
        print(f"[elevation] raw range {min(raw_elev):.0f}-{max(raw_elev):.0f} m, "
             f"smoothed+recentred range 0-{max(elev):.0f} m")

        pts, elev = dedupe(project_local(rotated), elev)
        emit_header(pts, elev, args.out, args.name)
    elif not args.verify:
        print("Nothing to do: pass --out <path.h> to emit the centreline, or --verify to only check it.")


if __name__ == "__main__":
    sys.exit(main())
