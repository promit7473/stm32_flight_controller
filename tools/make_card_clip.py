#!/usr/bin/env python3
"""Cut a 16:9 clip from portrait flight footage, panning to follow the drone.

The phone footage is portrait (1080x1920) but a web card is 16:9, so a fixed
crop shows the take-off and then a patch of empty sky as the aircraft climbs
out of the window. This tracks the drone through every frame and pans a
full-width 16:9 window to follow it, which keeps it framed from the grass to
altitude and reads as a camera move rather than a crop.

    python tools/make_card_clip.py --src media/test_fly.mp4 --out clip.mp4

Only the vertical axis is panned: the window is already the full width of the
source, so horizontal tracking would do nothing.

Why the detector is built the way it is
---------------------------------------
Two simpler detectors were tried first and both failed, in ways that are worth
knowing before someone "simplifies" this:

1. A plain darkness map lost the lock the instant the drone left the ground.
   The tree line is also dark and vastly larger, so it won every frame.
2. A single-scale difference-of-Gaussians failed for a subtler reason. The
   drone's apparent size varies enormously: it fills much of the frame just
   after take-off and shrinks to a speck at altitude. Tuned for the speck, it
   scored distant people and trees above the nearby aircraft.

Hence the scale pyramid below, which takes the strongest response over five
sizes and holds the lock throughout.

Verify before trusting it. Both failures produced plausible-looking numbers
while sitting on a tree; only drawing the detection onto sample frames showed
it. `--check` writes a contact sheet for exactly that.

Needs numpy, scipy and imageio (with imageio-ffmpeg). The raw footage is not
committed, so point --src at your own copy.
"""

import argparse
import os
import sys

import numpy as np

try:
    import imageio.v2 as iio
    from scipy.ndimage import gaussian_filter, gaussian_filter1d
except ImportError as exc:                       # pragma: no cover
    sys.exit("needs numpy, scipy and imageio: %s" % exc)

DOWNSAMPLE = 2                                   # detect at half resolution
SCALES = [2.0, 3.5, 6.0, 10.0, 16.0]             # half-res sigmas
SURROUND = 3.0                                   # surround / centre ratio
SEARCH = 150                                     # search radius, half-res px
SEED_BAND = (600, 860)                           # where the drone starts


def luma(frame):
    return (0.299 * frame[:, :, 0] + 0.587 * frame[:, :, 1]
            + 0.114 * frame[:, :, 2])


def blobness(gray):
    """Strongest small-dark-object response across a range of sizes.

    A single scale responds to one object size. The drone spans many, so take
    the maximum over a pyramid.
    """
    best = None
    for sigma in SCALES:
        response = (gaussian_filter(gray, sigma * SURROUND)
                    - gaussian_filter(gray, sigma))
        best = response if best is None else np.maximum(best, response)
    return best


def track(src):
    """Return (x, y, confidence) per frame, in source pixels."""
    reader = iio.get_reader(src)
    positions, confidence = [], []
    previous, velocity = None, np.zeros(2)

    for frame in reader:
        gray = luma(frame[::DOWNSAMPLE, ::DOWNSAMPLE].astype(np.float32))
        response = blobness(gray)
        height, width = response.shape

        if previous is None:
            band = response[SEED_BAND[0]:SEED_BAND[1], :]
            iy, ix = np.unravel_index(np.argmax(band), band.shape)
            iy += SEED_BAND[0]
        else:
            # Search where it is heading, not where it was: the climb is fast.
            px, py = np.round(np.array(previous, float) + velocity).astype(int)
            y0, y1 = max(0, py - SEARCH), min(height, py + SEARCH)
            x0, x1 = max(0, px - SEARCH), min(width, px + SEARCH)
            if y1 - y0 < 5 or x1 - x0 < 5:
                y0, y1, x0, x1 = 0, height, 0, width
            window = response[y0:y1, x0:x1]
            iy, ix = np.unravel_index(np.argmax(window), window.shape)
            iy, ix = iy + y0, ix + x0

        current = np.array([ix, iy], float)
        if previous is not None:
            velocity = 0.7 * velocity + 0.3 * (current - np.array(previous, float))
            velocity = np.clip(velocity, -60, 60)
        previous = (int(ix), int(iy))
        positions.append([ix * DOWNSAMPLE, iy * DOWNSAMPLE])
        confidence.append(float(response[iy, ix]))

    reader.close()
    p = np.array(positions, float)
    return p[:, 0], p[:, 1], np.array(confidence)


def pan_path(ys, source_height, crop_height, lead=0.58):
    """Smooth window-top path. Keeps the drone above centre so a climbing
    subject has room to rise into, which is how an operator frames one."""
    # Twice smoothed on purpose: a camera move should be slower than the
    # subject, and residual detector jitter reads as a shake.
    ys = gaussian_filter1d(ys, sigma=22, mode="nearest")
    top = np.clip(ys - crop_height * lead, 0, source_height - crop_height)
    return gaussian_filter1d(top, sigma=10, mode="nearest")


def contact_sheet(src, xs, ys, top, crop_height, out):
    from PIL import Image, ImageDraw
    picks = [0, 90, 130, 200, 300, 420]
    reader, tiles = iio.get_reader(src), []
    for i, frame in enumerate(reader):
        if i in picks:
            im = Image.fromarray(frame)
            d = ImageDraw.Draw(im)
            d.ellipse([xs[i] - 55, ys[i] - 55, xs[i] + 55, ys[i] + 55],
                      outline=(255, 40, 40), width=9)
            d.rectangle([0, top[i], im.width, top[i] + crop_height],
                        outline=(60, 200, 255), width=7)
            d.text((30, 30), "f%d" % i, fill=(255, 255, 0))
            im.thumbnail((220, 220))
            tiles.append(im)
        if i > max(picks):
            break
    reader.close()
    w, h = tiles[0].size
    sheet = Image.new("RGB", (w * len(tiles), h), (15, 15, 18))
    for i, im in enumerate(tiles):
        sheet.paste(im, (i * w, 0))
    sheet.save(out)
    print("wrote %s   red = detection, blue = crop window" % out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="portrait source video")
    ap.add_argument("--out", default="card_clip.mp4")
    ap.add_argument("--start", type=int, default=24, help="first frame")
    ap.add_argument("--end", type=int, default=300, help="last frame")
    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--check", metavar="PNG",
                    help="write a contact sheet and stop, to verify the track")
    args = ap.parse_args()

    if not os.path.exists(args.src):
        sys.exit("no such file: %s" % args.src)

    probe = iio.get_reader(args.src)
    src_w, src_h = probe.get_meta_data()["size"]
    probe.close()
    crop_h = int(round(src_w * 9 / 16))
    out_h = int(round(args.width * 9 / 16))

    print("source %dx%d, 16:9 window %dx%d" % (src_w, src_h, src_w, crop_h))
    xs, ys, conf = track(args.src)
    weak = int((conf < np.median(conf) * 0.35).sum())
    print("tracked %d frames, median confidence %.1f, weak frames %d"
          % (len(xs), np.median(conf), weak))
    if weak:
        print("  warning: %d weak frames, check the track before trusting it"
              % weak)

    top = pan_path(ys, src_h, crop_h)
    print("pan %.0f -> %.0f, max %.1f px/frame"
          % (top[args.start], top[args.end - 1],
             np.abs(np.diff(top[args.start:args.end])).max()))

    if args.check:
        contact_sheet(args.src, xs, ys, top, crop_h, args.check)
        return 0

    reader = iio.get_reader(args.src)
    writer = iio.get_writer(args.out, fps=args.fps, codec="libx264",
                            quality=8, macro_block_size=1)
    kept = 0
    for i, frame in enumerate(reader):
        if i < args.start:
            continue
        if i >= args.end:
            break
        y0 = int(round(top[i]))
        tile = frame[y0:y0 + crop_h, :, :]
        yi = (np.arange(out_h) * tile.shape[0] / out_h).astype(int)
        xi = (np.arange(args.width) * tile.shape[1] / args.width).astype(int)
        writer.append_data(tile[yi][:, xi])
        kept += 1
    reader.close()
    writer.close()
    print("wrote %s  (%d frames, %.1f MB)"
          % (args.out, kept, os.path.getsize(args.out) / 1048576))
    print("encode for the web with, for example:")
    print("  ffmpeg -i %s -c:v libx264 -crf 32 -preset slow "
          "-pix_fmt yuv420p -movflags +faststart -an card.mp4" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
