#!/usr/bin/env python3
"""Bake the venue's surface textures from ambientCG material sets.

NOT part of the simulator and not on any startup path: this is run by hand and
its OUTPUT (assets/textures/venue/) is what ships. It exists so the textures
can be rebuilt, changed, or re-sourced at a different resolution.

    tools/bake_venue_textures.py [--source DIR] [--size 1024]

SOURCES. Every input is a CC0 (public domain) material from ambientCG.com,
downloaded as its 1K-JPG archive into DIR/<AssetId>.zip (fetched if missing).
The list is MATERIALS below, and is repeated with links in
assets/textures/venue/SOURCES.md.

WHAT IS BAKED. The venue shader reads two texture arrays, one layer per
material, so each material becomes exactly two files:

  NN_name_albedo.jpg   colour, with ambient occlusion multiplied in, then
                       NORMALISED so each channel averages mid-grey. The
                       shader multiplies it by twice the vertex colour, which
                       means the photograph supplies the texture and the
                       variation, and the venue's own palette still decides
                       what colour a surface is. Without this a grey asphalt
                       photograph could not be both the racing surface and
                       the lighter runoff.
  NN_name_detail.jpg   R, G = tangent-space normal x, y (OpenGL convention);
                       B = roughness. For the leaf card B is OPACITY instead:
                       a leaf has no use for a roughness map and JPEG has no
                       alpha channel to put a cut-out in.

The leaf card is composed here rather than downloaded: ambientCG's leaf sets
are atlases of single leaves, and a tree crown is drawn with cards of many.

Needs ImageMagick 7 (`magick`) and bsdtar or unzip.
"""

import argparse
import os
import random
import subprocess
import sys
import urllib.request

# (layer, name, ambientCG asset id)
MATERIALS = [
    (0, "asphalt",    "Asphalt031"),
    (1, "grass",      "Grass004"),
    (2, "gravel",     "Gravel023"),
    (3, "concrete",   "Concrete031"),
    (4, "plaster",    "PaintedPlaster017"),
    (5, "galvanised", "Metal055A"),
    (6, "rubber",     "Rubber004"),
    (7, "roofing",    "CorrugatedSteel007A"),
    (8, "bark",       "Bark012"),
]
LEAF_LAYER = (9, "leaves", "LeafSet024")


def run(*args):
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def fetch(asset, source):
    folder = os.path.join(source, asset)
    if not os.path.isdir(folder) or not any(f.endswith("_Color.jpg") for f in os.listdir(folder)):
        archive = os.path.join(source, asset + ".zip")
        if not os.path.exists(archive):
            url = f"https://ambientcg.com/get?file={asset}_1K-JPG.zip"
            print("  downloading", url)
            urllib.request.urlretrieve(url, archive)
        os.makedirs(folder, exist_ok=True)
        run("bsdtar", "-xf", archive, "-C", folder, "--include=*.jpg")
    return folder


def find(folder, suffix):
    for f in os.listdir(folder):
        if f.endswith(suffix):
            return os.path.join(folder, f)
    return None


def normalise(image, out, size):
    """Scale each channel so it averages 0.5, then write `out`."""
    means = run("magick", image, "-format", "%[fx:mean.r] %[fx:mean.g] %[fx:mean.b]", "info:").split()
    k = [0.5 / max(float(m), 1e-3) for m in means]
    run("magick", image, "-resize", f"{size}x{size}!",
        "-channel", "R", "-evaluate", "multiply", str(k[0]),
        "-channel", "G", "-evaluate", "multiply", str(k[1]),
        "-channel", "B", "-evaluate", "multiply", str(k[2]),
        "+channel", "-quality", "91", "-sampling-factor", "4:4:4", out)


def bake_material(layer, name, asset, source, dest, size, tmp):
    folder = fetch(asset, source)
    colour = find(folder, "_Color.jpg")
    normal = find(folder, "_NormalGL.jpg")
    rough = find(folder, "_Roughness.jpg")
    ao = find(folder, "_AmbientOcclusion.jpg")

    lit = os.path.join(tmp, "lit.png")
    if ao:
        run("magick", colour, ao, "-compose", "Multiply", "-composite", lit)
    else:
        run("magick", colour, lit)
    normalise(lit, os.path.join(dest, f"{layer:02d}_{name}_albedo.jpg"), size)

    # `-set colorspace sRGB` on every input: these are data, not pictures,
    # and must be packed bit for bit with no colour management applied.
    run("magick",
        "(", normal, "-set", "colorspace", "sRGB", "-channel", "R", "-separate", ")",
        "(", normal, "-set", "colorspace", "sRGB", "-channel", "G", "-separate", ")",
        "(", rough, "-set", "colorspace", "sRGB", "-colorspace", "Gray", "-set", "colorspace", "sRGB", ")",
        "-resize", f"{size}x{size}!", "-set", "colorspace", "sRGB", "-combine",
        "-set", "colorspace", "sRGB", "-quality", "93", "-sampling-factor", "4:4:4",
        os.path.join(dest, f"{layer:02d}_{name}_detail.jpg"))
    print(f"  {layer:02d} {name:<11} <- {asset}")


def bake_leaves(layer, name, asset, source, dest, size, tmp):
    folder = fetch(asset, source)
    colour = find(folder, "_Color.jpg")
    opacity = find(folder, "_Opacity.jpg")

    # Cut the atlas into its nine leaves, each with its own alpha.
    atlas = os.path.join(tmp, "leaves.png")
    run("magick", colour, "(", opacity, "-colorspace", "Gray", ")",
        "-alpha", "off", "-compose", "CopyOpacity", "-composite", atlas)
    run("magick", atlas, "-crop", "3x3@", "+repage", os.path.join(tmp, "leaf_%d.png"))

    # Scatter them into a roughly round spray: dense in the middle, ragged
    # at the edge, every leaf at its own angle and size. Fixed seed, so the
    # card is the same every time it is baked.
    rng = random.Random(24)
    cmd = ["magick", "-size", f"{size}x{size}", "xc:none"]
    for _ in range(620):
        r = abs(rng.gauss(0.0, 0.25)) * size
        if r > 0.47 * size:
            continue
        a = rng.uniform(0.0, 6.2832)
        import math
        x = 0.5 * size + r * math.cos(a)
        y = 0.5 * size + r * math.sin(a)
        s = int(size * rng.uniform(0.055, 0.105))
        shade = rng.uniform(62, 118)
        cmd += ["(", os.path.join(tmp, f"leaf_{rng.randrange(9)}.png"),
                "-resize", f"{s}x{s}", "-background", "none", "-rotate", f"{rng.uniform(0, 360):.0f}",
                "-modulate", f"{shade:.0f},100,{rng.uniform(96, 106):.0f}", ")",
                "-gravity", "NorthWest", "-geometry", f"+{int(x - s / 2)}+{int(y - s / 2)}",
                "-compose", "Over", "-composite"]
    spray = os.path.join(tmp, "spray.png")
    run(*cmd, spray)

    # Colour: the spray over its own average green, so the texels outside
    # the cut-out do not bleed a dark fringe into the leaves when filtered.
    # That average is taken over the leaves only (colour over black, divided
    # by coverage), so normalising the card normalises the leaves.
    flat = os.path.join(tmp, "spray_flat.png")
    stats = run("magick", spray, "-background", "black", "-alpha", "remove", "-alpha", "off",
                "-format", "%[fx:mean.r] %[fx:mean.g] %[fx:mean.b]", "info:").split()
    cover = float(run("magick", spray, "-alpha", "extract", "-format", "%[fx:mean]", "info:"))
    r, g, b = (int(255 * float(v) / max(cover, 1e-3)) for v in stats)
    run("magick", spray, "-background", f"rgb({r},{g},{b})", "-alpha", "remove", "-alpha", "off", flat)
    normalise(flat, os.path.join(dest, f"{layer:02d}_{name}_albedo.jpg"), size)

    run("magick",
        "(", "-size", f"{size}x{size}", "xc:gray(50%)", ")",
        "(", "-size", f"{size}x{size}", "xc:gray(50%)", ")",
        "(", spray, "-alpha", "extract", ")",
        "-set", "colorspace", "sRGB", "-combine", "-set", "colorspace", "sRGB",
        "-quality", "93", "-sampling-factor", "4:4:4",
        os.path.join(dest, f"{layer:02d}_{name}_detail.jpg"))
    print(f"  {layer:02d} {name:<11} <- {asset} (composed)")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--source", default=os.path.join(here, "..", "build", "venue_texture_sources"))
    ap.add_argument("--dest", default=os.path.join(here, "..", "assets", "textures", "venue"))
    ap.add_argument("--size", type=int, default=1024)
    args = ap.parse_args()

    os.makedirs(args.source, exist_ok=True)
    os.makedirs(args.dest, exist_ok=True)
    tmp = os.path.join(args.source, "_tmp")
    os.makedirs(tmp, exist_ok=True)

    print("baking venue textures ->", os.path.normpath(args.dest))
    for layer, name, asset in MATERIALS:
        bake_material(layer, name, asset, args.source, args.dest, args.size, tmp)
    bake_leaves(*LEAF_LAYER, args.source, args.dest, args.size, tmp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
