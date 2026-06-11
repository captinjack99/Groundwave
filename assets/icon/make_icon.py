#!/usr/bin/env python3
"""Generate groundwave.ico + PNG rasters from the carrier-comb mark.

The geometry tables below mirror assets/icon/groundwave.svg (full variant,
used at 48 px and above) and groundwave_small.svg (six fatter bars for
16/24/32 px). If you edit the SVGs, update the tables here and re-run:

    python make_icon.py

Requires Pillow. Everything is drawn on a 120x120 logical grid at 4x
supersampling, then Lanczos-downscaled, so edges stay crisp at every
size. The .ico carries per-size artwork (not one master downscaled).
"""
from PIL import Image, ImageDraw

TILE_RX = 26
BG      = (10, 10, 13, 255)      # style.hpp BG_BASE  #0A0A0D
SIGNAL  = (0, 180, 255, 255)     # style.hpp C_SIGNAL #00B4FF
PILOT   = (210, 130, 63, 255)    # brand copper        #D2823F
BASE    = (38, 38, 46, 255)      # baseline            #26262E

# (x, y, w, h, rx, color) on the 120x120 grid — mirrors groundwave.svg
FULL = [
    (18, 72, 6, 16, 3, SIGNAL),
    (28, 58, 6, 30, 3, SIGNAL),
    (38, 44, 6, 44, 3, SIGNAL),
    (48, 34, 6, 54, 3, SIGNAL),
    (66, 34, 6, 54, 3, SIGNAL),
    (76, 44, 6, 44, 3, PILOT),
    (86, 58, 6, 30, 3, SIGNAL),
    (96, 72, 6, 16, 3, SIGNAL),
    (18, 92, 84, 2, 1, BASE),
]

# mirrors groundwave_small.svg (no baseline)
SMALL = [
    (16, 64, 12, 24, 5, SIGNAL),
    (30, 46, 12, 42, 5, SIGNAL),
    (44, 30, 12, 58, 5, SIGNAL),
    (64, 30, 12, 58, 5, SIGNAL),
    (78, 46, 12, 42, 5, PILOT),
    (92, 64, 12, 24, 5, SIGNAL),
]

SS = 4  # supersampling factor


def render(size: int, bars) -> Image.Image:
    s = size * SS / 120.0
    canvas = size * SS
    img = Image.new("RGBA", (canvas, canvas), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([0, 0, canvas - 1, canvas - 1],
                        radius=TILE_RX * s, fill=BG)
    for (x, y, w, h, rx, color) in bars:
        d.rounded_rectangle([x * s, y * s, (x + w) * s - 1, (y + h) * s - 1],
                            radius=rx * s, fill=color)
    return img.resize((size, size), Image.LANCZOS)


def main() -> None:
    full_sizes  = [256, 128, 64, 48]
    small_sizes = [32, 24, 16]

    images = {sz: render(sz, FULL) for sz in full_sizes}
    images.update({sz: render(sz, SMALL) for sz in small_sizes})

    images[256].save("groundwave_256.png")
    images[32].save("groundwave_32.png")

    ordered = [images[sz] for sz in (256, 128, 64, 48, 32, 24, 16)]
    ordered[0].save(
        "groundwave.ico", format="ICO",
        append_images=ordered[1:],
        sizes=[(sz, sz) for sz in (256, 128, 64, 48, 32, 24, 16)],
    )

    with Image.open("groundwave.ico") as ico:
        print("groundwave.ico entries:", ico.info.get("sizes"))
    print("wrote groundwave.ico, groundwave_256.png, groundwave_32.png")


if __name__ == "__main__":
    main()
