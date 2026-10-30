# Venue surface textures — sources and licence

Every texture in this directory is derived from a material published by
[ambientCG](https://ambientcg.com) under the
[Creative Commons CC0 1.0 Universal](https://creativecommons.org/publicdomain/zero/1.0/)
public-domain dedication. No attribution is required; it is recorded here so
the files can be traced and rebuilt.

| Layer | File prefix | ambientCG asset | Used for |
|---|---|---|---|
| 0 | `00_asphalt` | [Asphalt031](https://ambientcg.com/a/Asphalt031) | racing surface, pit lane, runoff, road paint grain |
| 1 | `01_grass` | [Grass004](https://ambientcg.com/a/Grass004) | verges and terrain |
| 2 | `02_gravel` | [Gravel023](https://ambientcg.com/a/Gravel023) | gravel traps |
| 3 | `03_concrete` | [Concrete031](https://ambientcg.com/a/Concrete031) | walls, kerbs, stands |
| 4 | `04_plaster` | [PaintedPlaster017](https://ambientcg.com/a/PaintedPlaster017) | painted cladding and panels |
| 5 | `05_galvanised` | [Metal055A](https://ambientcg.com/a/Metal055A) | armco, fence posts, steelwork |
| 6 | `06_rubber` | [Rubber004](https://ambientcg.com/a/Rubber004) | tyre walls |
| 7 | `07_roofing` | [CorrugatedSteel007A](https://ambientcg.com/a/CorrugatedSteel007A) | grandstand and pit roofs |
| 8 | `08_bark` | [Bark012](https://ambientcg.com/a/Bark012) | tree trunks |
| 9 | `09_leaves` | [LeafSet024](https://ambientcg.com/a/LeafSet024) | tree crowns (composed into a leaf-spray card) |

Each material is two 1024×1024 JPEGs, produced from the 1K-JPG download by
`tools/bake_venue_textures.py`:

- `*_albedo.jpg` — colour with ambient occlusion multiplied in, normalised so
  each channel averages mid-grey (the venue's own palette supplies the colour).
- `*_detail.jpg` — normal map x, y in red and green (OpenGL convention),
  roughness in blue. For `09_leaves`, blue is opacity instead.

To rebuild or swap a material, edit the list in that script and run it; it
downloads what it needs. The simulator never accesses the network.
