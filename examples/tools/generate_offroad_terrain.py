#!/usr/bin/env python3
"""Generate the terrain of the rayrai off-road buggy example.

Usage: generate_offroad_terrain.py OUTPUT   (rsc/offroad_buggy/terrain)

The ground is a 256 m x 256 m height map with 0.25 m samples:

* Rolling hills: domain-warped gradient-noise fBm with a ridged component,
  shaped by hydraulic erosion (rain droplets that pick up and deposit
  sediment, simulated in parallel) and by thermal erosion (slopes above the
  angle of repose slump), on a 1 m grid. Erosion carves the gullies, the
  dendritic drainage and the smooth sediment fans that make terrain look
  natural.
* A 768 m race circuit, 9 m wide, winds through the hills: a smoothed
  centripetal Catmull-Rom spline through the corners in CONTROL_POINTS. Its
  profile is the terrain smoothed along the loop (with gentler climbs), level
  along the start/finish straight; the outside of each corner rises into a
  curved berm with a lip on the tight ones, two ruts follow the racing line,
  and it has a tabletop jump, whoops, a washboard and a rock garden.
* Level pads beside the straight for the pits and the spectators, cut into
  the hills with grassy banks.
* Small-scale roughness everywhere (a few centimetres), so that the
  suspension always works.

It writes:
  height.png    16-bit heights (row 0 is the lowest y, column 0 the lowest x)
  surface.png   8-bit RGBA layers: bare ground (else grass), rock, loose dirt
                (else the packed racing surface) and gravel
  wear.png      8-bit wear of the racing line, which darkens the track
  terrain.txt   size, height range, the start pose, the straight, the pads,
                the track's features and its centre line every 2 m with its
                height and curvature

Deterministic for a fixed seed. Requires NumPy, SciPy and Pillow.
"""
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree

SEED = 11
EXTENT = 256.0          # metres along x and y
SAMPLES = 1025          # 0.25 m spacing
COARSE = 257            # 1 m spacing for the macro shape and erosion
TRACK_HALF_WIDTH = 4.5
SHOULDER = 5.0          # blend from the track to the natural ground
TRACK_STEP = 0.5        # centre-line sampling [m]


# ---------------------------------------------------------------------------
# Gradient noise.

def perlin(x, y, rng_seed):
    """2D gradient noise in [-1, 1] at arrays of coordinates (period 256)."""
    rng = np.random.default_rng(rng_seed)
    perm = rng.permutation(256)
    perm = np.concatenate([perm, perm])
    angles = rng.uniform(0, 2 * np.pi, 256)
    grads = np.stack([np.cos(angles), np.sin(angles)], 1)
    xi, yi = np.floor(x).astype(int), np.floor(y).astype(int)
    xf, yf = x - xi, y - yi
    xi, yi = xi & 255, yi & 255
    fade = lambda t: t * t * t * (t * (t * 6 - 15) + 10)
    u, v = fade(xf), fade(yf)

    def corner(dx, dy):
        g = grads[perm[perm[xi + dx] + yi + dy]]
        return g[..., 0] * (xf - dx) + g[..., 1] * (yf - dy)

    n0 = corner(0, 0) * (1 - u) + corner(1, 0) * u
    n1 = corner(0, 1) * (1 - u) + corner(1, 1) * u
    return (n0 * (1 - v) + n1 * v) * 1.41


def fbm(x, y, seed, octaves, lacunarity=2.0, gain=0.5, ridged=False):
    total, amplitude, norm = 0.0, 1.0, 0.0
    for octave in range(octaves):
        n = perlin(x, y, seed + octave)
        if ridged:
            n = 1.0 - np.abs(n)
            n = n * n
        total = total + amplitude * n
        norm += amplitude
        x, y = x * lacunarity + 17.3, y * lacunarity - 9.1
        amplitude *= gain
    return total / norm


# ---------------------------------------------------------------------------
# Erosion.

def bilinear(h, px, py):
    ix, iy = np.floor(px).astype(int), np.floor(py).astype(int)
    fx, fy = px - ix, py - iy
    h00, h10, h01, h11 = h[iy, ix], h[iy, ix + 1], h[iy + 1, ix], h[iy + 1, ix + 1]
    height = h00 * (1 - fx) * (1 - fy) + h10 * fx * (1 - fy) + h01 * (1 - fx) * fy + h11 * fx * fy
    gx = (h10 - h00) * (1 - fy) + (h11 - h01) * fy
    gy = (h01 - h00) * (1 - fx) + (h11 - h10) * fx
    return height, gx, gy, ix, iy, fx, fy


def hydraulic_erosion(h, rng, batches=60, droplets=12000, steps=90):
    """Rain droplets run downhill with inertia, erode where they can carry more sediment and
    deposit where they slow down (after Hans Theobald Beyer's particle erosion), simulated
    in parallel batches. h is in metres on a 1 m grid."""
    n = h.shape[0]
    inertia, capacity_factor, min_capacity = 0.05, 6.0, 0.01
    erode_rate, deposit_rate, evaporation, gravity = 0.3, 0.3, 0.02, 4.0
    brush = [(dx, dy, w) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
             for w in [max(0.0, 1.6 - math.hypot(dx, dy))]]
    total = sum(w for _, _, w in brush)
    for _ in range(batches):
        px = rng.uniform(2, n - 3, droplets)
        py = rng.uniform(2, n - 3, droplets)
        dx, dy = np.zeros(droplets), np.zeros(droplets)
        speed, water, sediment = np.ones(droplets), np.ones(droplets), np.zeros(droplets)
        alive = np.ones(droplets, bool)
        for _ in range(steps):
            height, gx, gy, ix, iy, fx, fy = bilinear(h, px, py)
            dx = dx * inertia - gx * (1 - inertia)
            dy = dy * inertia - gy * (1 - inertia)
            length = np.hypot(dx, dy)
            alive &= length > 1e-9
            dx, dy = dx / np.maximum(length, 1e-9), dy / np.maximum(length, 1e-9)
            nx, ny = px + dx, py + dy
            alive &= (nx > 2) & (nx < n - 3) & (ny > 2) & (ny < n - 3)
            nx, ny = np.where(alive, nx, px), np.where(alive, ny, py)
            new_height = bilinear(h, nx, ny)[0]
            dh = np.where(alive, new_height - height, 0.0)
            capacity = np.maximum(-dh * speed * water * capacity_factor, min_capacity)
            deposit = np.where((sediment > capacity) | (dh > 0),
                               np.where(dh > 0, np.minimum(dh, sediment), (sediment - capacity) * deposit_rate), 0.0)
            erode = np.where((sediment <= capacity) & (dh <= 0),
                             np.minimum((capacity - sediment) * erode_rate, -dh), 0.0)
            deposit, erode = deposit * alive, erode * alive
            # Droplets that share a cell share its erosion, as if they had passed one after the
            # other; otherwise converging droplets dig pits that deepen without bound.
            crowd = np.zeros_like(h)
            np.add.at(crowd, (iy, ix), alive.astype(float))
            erode /= np.maximum(crowd[iy, ix], 1.0)
            sediment += erode - deposit
            for ox, oy, w in ((0, 0, (1 - fx) * (1 - fy)), (1, 0, fx * (1 - fy)), (0, 1, (1 - fx) * fy),
                              (1, 1, fx * fy)):
                np.add.at(h, (iy + oy, ix + ox), deposit * w)
            for ox, oy, w in brush:
                np.add.at(h, (iy + oy, ix + ox), -erode * w / total)
            speed = np.sqrt(np.maximum(speed ** 2 + dh * gravity, 0.0))
            water *= 1 - evaporation
            px, py = nx, ny
    return h


def thermal_erosion(h, cell, talus=0.75, iterations=40, rate=0.25):
    """Material slides from a cell to lower neighbours where the slope exceeds talus (rise/run)."""
    limit = talus * cell
    for _ in range(iterations):
        change = np.zeros_like(h)
        for axis, shift in ((0, 1), (0, -1), (1, 1), (1, -1)):
            neighbour = np.roll(h, shift, axis)
            excess = np.maximum(h - neighbour - limit, 0.0) * rate * 0.5
            # No exchange across the border that np.roll wraps around.
            edge = [slice(None), slice(None)]
            edge[axis] = 0 if shift == 1 else -1
            excess[tuple(edge)] = 0.0
            change -= excess
            change += np.roll(excess, -shift, axis)
        h = h + change
    return h


# ---------------------------------------------------------------------------
# Track.

# Corners of the circuit, anticlockwise from the start of the start/finish straight. A centripetal
# Catmull-Rom spline through them, smoothed, gives a 768 m loop whose tightest hairpins have a
# radius of about 15 m.
CONTROL_POINTS = [(-40, -86), (40, -84), (76, -76), (95, -50), (97, -12), (92, 25), (99, 55), (88, 82), (60, 88),
                  (42, 68), (22, 46), (-4, 58), (-28, 82), (-62, 84), (-90, 66), (-98, 30), (-86, 0), (-62, -10),
                  (-56, -36), (-72, -54), (-100, -60), (-104, -84), (-80, -94)]
STRAIGHT = (0.0, 100.0)   # stations of the flat start/finish straight
START_LINE = 30.0
# Level pads that reach up to the straight, for the pits (south) and the spectators (north):
# (x0, y0, x1, y1) [m].
PIT_PAD = (-36.0, -116.0, 44.0, -94.0)
SPECTATOR_PAD = (-24.0, -79.0, 36.0, -62.0)


def catmull_rom(points, samples=60, alpha=0.5):
    points = np.asarray(points, float)
    out = []
    n = len(points)
    for i in range(n):
        p0, p1, p2, p3 = points[i - 1], points[i], points[(i + 1) % n], points[(i + 2) % n]
        t1 = np.linalg.norm(p1 - p0) ** alpha
        t2 = t1 + np.linalg.norm(p2 - p1) ** alpha
        t3 = t2 + np.linalg.norm(p3 - p2) ** alpha
        for t in np.linspace(t1, t2, samples, endpoint=False):
            a1 = (t1 - t) / t1 * p0 + t / t1 * p1
            a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            b1 = (t2 - t) / t2 * a1 + t / t2 * a2
            b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
            out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    return np.array(out)


def periodic_smooth(values, window):
    kernel = np.hanning(window)
    kernel /= kernel.sum()
    padded = np.concatenate([values[-window:], values, values[:window]])
    return np.convolve(padded, kernel, mode='same')[window:-window]


def track_centre_line():
    """The circuit sampled every TRACK_STEP metres: points, length and signed curvature (+ left)."""
    curve = catmull_rom(CONTROL_POINTS)
    closed = np.vstack([curve, curve[:1]])
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    stations = np.arange(0, s[-1], TRACK_STEP)
    x, y = np.interp(stations, s, closed[:, 0]), np.interp(stations, s, closed[:, 1])
    for _ in range(2):
        x, y = periodic_smooth(x, 41), periodic_smooth(y, 41)
    points = np.stack([x, y], 1)
    seg = np.linalg.norm(np.roll(points, -1, 0) - points, axis=1)
    length = float(seg.sum())
    # Resample to exact spacing after the smoothing.
    closed = np.vstack([points, points[:1]])
    s = np.concatenate([[0], np.cumsum(seg)])
    stations = np.arange(0, length, TRACK_STEP)
    points = np.stack([np.interp(stations, s, closed[:, 0]), np.interp(stations, s, closed[:, 1])], 1)
    d1 = (np.roll(points, -1, 0) - np.roll(points, 1, 0)) / (2 * TRACK_STEP)
    d2 = (np.roll(points, -1, 0) - 2 * points + np.roll(points, 1, 0)) / TRACK_STEP ** 2
    curvature = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(np.linalg.norm(d1, axis=1) ** 3, 1e-9)
    return points, length, periodic_smooth(periodic_smooth(curvature, 41), 41)


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


# Track features: name, start station [m], length [m].
FEATURES = [('tabletop jump', 168.0, 26.0), ('whoops', 391.0, 30.0), ('washboard', 450.0, 50.0),
            ('rock garden', 598.0, 32.0)]


def feature_profile(s):
    """Height added to the track surface at stations s."""
    z = np.zeros_like(s)
    for name, start, length in FEATURES:
        u = s - start
        inside = (u >= 0) & (u <= length)
        if name == 'whoops':
            # Regular moguls 0.32 m high every 4.5 m, faded in and out.
            envelope = np.sin(np.pi * np.clip(u / length, 0, 1)) ** 0.4
            z += inside * 0.32 * envelope * np.sin(np.pi * u / 4.5) ** 2
        elif name == 'tabletop jump':
            # 9 m take-off ramp up to 1.3 m, 6 m table, 11 m landing.
            up, table, down = 9.0, 6.0, 11.0
            ramp = np.where(u < up, smoothstep(0, up, u),
                            np.where(u < up + table, 1.0, 1.0 - smoothstep(up + table, up + table + down, u)))
            z += inside * 1.3 * ramp
        elif name == 'washboard':
            # Corrugations 6 cm deep every 0.9 m.
            envelope = smoothstep(0, 6, u) * (1 - smoothstep(length - 6, length, u))
            z += inside * 0.06 * envelope * np.sin(2 * np.pi * u / 0.9)
    return z


def pad_weight(FX, FY, pad, margin=6.0):
    """1 inside the rectangle, falling to 0 over margin metres outside it."""
    x0, y0, x1, y1 = pad
    dx = np.maximum(np.maximum(x0 - FX, FX - x1), 0)
    dy = np.maximum(np.maximum(y0 - FY, FY - y1), 0)
    return 1.0 - smoothstep(0, margin, np.hypot(dx, dy))


def generate(output):
    """Write height.png, surface.png, wear.png and terrain.txt into the folder output."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(SEED)

    # Macro shape on the coarse grid.
    c = np.linspace(-EXTENT / 2, EXTENT / 2, COARSE)
    X, Y = np.meshgrid(c, c)  # rows: y, columns: x
    wx = 22.0 * fbm(X / 130 + 3.1, Y / 130 - 1.7, SEED + 100, 4)
    wy = 22.0 * fbm(X / 130 - 7.4, Y / 130 + 5.2, SEED + 200, 4)
    hills = 15.0 * fbm((X + wx) / 120, (Y + wy) / 120, SEED, 6, gain=0.5)
    ridges = 8.0 * fbm((X - wy) / 70, (Y + wx) / 70, SEED + 300, 5, gain=0.5, ridged=True)
    # Hills rise towards the edges, so that the circuit lies in a broad basin.
    bowl = 0.0010 * (X ** 2 + 1.2 * Y ** 2)
    h = hills + ridges + np.minimum(bowl, 20.0)
    h = hydraulic_erosion(h, rng)
    h = thermal_erosion(h, 1.0)
    h = ndimage.gaussian_filter(h, 0.6)

    # Upsample, then add metre-scale bumps and centimetre-scale roughness.
    f = np.linspace(-EXTENT / 2, EXTENT / 2, SAMPLES)
    FX, FY = np.meshgrid(f, f)
    height = ndimage.map_coordinates(h, [(FY + EXTENT / 2), (FX + EXTENT / 2)], order=3, mode='nearest')
    bumps = 0.22 * fbm(FX / 7.0, FY / 7.0, SEED + 400, 3)
    rough = 0.035 * fbm(FX / 1.1, FY / 1.1, SEED + 500, 3)
    ground = height + bumps + rough

    # Track: smoothed profile of the natural ground along the loop, flat along the straight.
    centre, length, curvature = track_centre_line()
    stations = np.arange(len(centre)) * TRACK_STEP
    to_grid = lambda v: (v + EXTENT / 2) * (SAMPLES - 1) / EXTENT
    natural = ndimage.map_coordinates(ground, [to_grid(centre[:, 1]), to_grid(centre[:, 0])], order=1)
    profile = periodic_smooth(periodic_smooth(natural, 120), 60)
    profile = profile.mean() + 0.6 * (profile - profile.mean())  # gentler climbs than the hills
    on_straight = (stations >= STRAIGHT[0]) & (stations <= STRAIGHT[1])
    straight_z = float(profile[on_straight].mean())
    flat = smoothstep(STRAIGHT[0] - 40, STRAIGHT[0], stations) * (1 - smoothstep(STRAIGHT[1], STRAIGHT[1] + 40, stations))
    flat = np.maximum(flat, smoothstep(length - 40 + STRAIGHT[0], length, stations))
    profile = profile * (1 - flat) + straight_z * flat
    surface_z = profile + feature_profile(stations)

    tree = cKDTree(centre)
    points = np.stack([FX.ravel(), FY.ravel()], 1)
    reach = TRACK_HALF_WIDTH + SHOULDER + 1.0
    distance, nearest = tree.query(points, distance_upper_bound=reach)
    on = np.isfinite(distance)
    distance = np.where(on, distance, 1e9).reshape(FX.shape)
    nearest = np.where(on, nearest, 0).reshape(FX.shape)
    tangent = np.roll(centre, -1, 0) - np.roll(centre, 1, 0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    normal = np.stack([-tangent[:, 1], tangent[:, 0]], 1)  # points left
    dx, dy = FX - centre[nearest, 0], FY - centre[nearest, 1]
    lateral = dx * normal[nearest, 0] + dy * normal[nearest, 1]
    # Continuous station of each sample, so that the profile does not step every TRACK_STEP.
    station = nearest * TRACK_STEP + dx * tangent[nearest, 0] + dy * tangent[nearest, 1]
    periodic = lambda values: np.interp(station, np.append(stations, length), np.append(values, values[0]),
                                        period=length)
    k = periodic(curvature)
    # Banked berms: the outside half of a corner is built up into a curved wall, with a lip along
    # the outside edge of the tight ones. The racing line on the inside stays at track level.
    outside = -np.sign(k) * lateral  # positive on the outside of the turn
    across = np.clip((outside + TRACK_HALF_WIDTH) / (2 * TRACK_HALF_WIDTH), 0, 1.2)
    bank = np.clip(6.0 * np.abs(k), 0, 0.26)
    bank = np.sqrt(bank ** 2 + 0.03 ** 2) - 0.03  # no crease where an S-bend changes direction
    lip = smoothstep(1 / 40, 1 / 25, np.abs(k)) * 0.4 * np.exp(-((outside - (TRACK_HALF_WIDTH - 0.2)) / 0.9) ** 2)
    berm = bank * 2 * TRACK_HALF_WIDTH * across ** 2 + lip
    # Two worn ruts on the racing line, which runs towards the inside of each corner.
    line = np.clip(18.0 * k, -1.8, 1.8)
    ruts = -0.05 * (np.exp(-((lateral - line - 0.95) / 0.3) ** 2) + np.exp(-((lateral - line + 0.95) / 0.3) ** 2))
    track_rough = 0.012 * fbm(FX / 0.6, FY / 0.6, SEED + 600, 2)
    track = periodic(surface_z) + berm + ruts + track_rough
    blend = 1.0 - smoothstep(TRACK_HALF_WIDTH + 0.5, TRACK_HALF_WIDTH + SHOULDER, distance)
    terrain = ground * (1 - blend) + track * blend

    # Pit and spectator pads, level with the straight (the spectators a little higher).
    # Wide margins: the pads are cut into the hills, and the cuts become grassy banks.
    pit = pad_weight(FX, FY, PIT_PAD, 20.0)
    stand = pad_weight(FX, FY, SPECTATOR_PAD, 20.0)
    terrain = terrain * (1 - pit) + (straight_z + 0.05 + rough * 0.3) * pit
    terrain = terrain * (1 - stand) + (straight_z + 0.15 + rough * 0.3) * stand

    # Surface layers: bare ground (vs grass), rock on steep slopes, gravel on the pads, loose dirt
    # on the track edges, berms and shoulders (vs the packed racing surface).
    gy, gx = np.gradient(terrain, EXTENT / (SAMPLES - 1))
    slope = np.hypot(gx, gy)
    breakup = fbm(FX / 2.5, FY / 2.5, SEED + 700, 3)
    track_dirt = 1.0 - smoothstep(TRACK_HALF_WIDTH + 0.2 + 0.8 * breakup, TRACK_HALF_WIDTH + 2.0 + 1.2 * breakup, distance)
    steep = smoothstep(0.62, 0.9, slope + 0.15 * breakup) * smoothstep(reach, reach + 4, distance)
    patches = 0.85 * smoothstep(0.45, 0.65, fbm(FX / 18, FY / 18, SEED + 800, 4) + 0.2 * breakup)
    pads = smoothstep(0.35, 0.65, pit + 0.15 * breakup)
    trampled = stand * smoothstep(0.5, 0.75, fbm(FX / 3, FY / 3, SEED + 900, 3) + 0.3)
    bare = np.maximum.reduce([track_dirt, steep, patches, pads, trampled])
    # The cuts behind the pads are loose earth, not rock.
    rock = steep * (1 - track_dirt) * (1 - smoothstep(0.0, 0.15, np.maximum(pit, stand)))
    gravel = pads
    loose = np.clip(np.maximum(smoothstep(3.0, 4.3, np.abs(lateral) + 0.4 * breakup) * track_dirt,
                               lip / 0.3), 0, 1)
    loose = np.where(distance < reach, loose, 1.0)  # bare patches away from the track are loose soil
    # Wear: the racing line and its ruts are darker and smoother.
    wear = np.exp(-((lateral - line) / 1.6) ** 2) * (distance < TRACK_HALF_WIDTH + 1)
    wear = np.clip(wear + 0.6 * (np.exp(-((lateral - line - 0.95) / 0.35) ** 2) +
                                 np.exp(-((lateral - line + 0.95) / 0.35) ** 2)) * (distance < TRACK_HALF_WIDTH), 0, 1)

    low, high = float(terrain.min()), float(terrain.max())
    quantized = np.round((terrain - low) / (high - low) * 65535).astype(np.uint16)
    Image.fromarray(quantized).save(output / 'height.png')
    layers = np.stack([bare, rock, loose, gravel], -1)
    Image.fromarray(np.round(np.clip(layers, 0, 1) * 255).astype(np.uint8), 'RGBA').save(output / 'surface.png')
    Image.fromarray(np.round(np.clip(wear, 0, 1) * 255).astype(np.uint8)).save(output / 'wear.png')
    # Start just behind the start line, facing along the straight.
    start = int((START_LINE - 18.0) / TRACK_STEP)
    heading = math.atan2(*(centre[start + 1] - centre[start - 1])[::-1])
    every = int(2.0 / TRACK_STEP)
    lines = ['# rayrai_offroad_buggy terrain, written by generate_offroad_terrain.py',
             f'extent {EXTENT}', f'samples {SAMPLES}', f'height {low:.4f} {high:.4f}',
             f'track {TRACK_HALF_WIDTH} {length:.2f}',
             f'start {centre[start, 0]:.3f} {centre[start, 1]:.3f} {heading:.4f}',
             f'start_line {START_LINE}', f'straight {STRAIGHT[0]} {STRAIGHT[1]} {straight_z:.3f}',
             f'pit {" ".join(str(v) for v in PIT_PAD)}', f'spectators {" ".join(str(v) for v in SPECTATOR_PAD)}']
    lines += [f'feature {start_:.1f} {length_:.1f} {name}' for name, start_, length_ in FEATURES]
    lines += [f'centre {x:.3f} {y:.3f} {z:.3f} {k:.5f}'
              for (x, y), z, k in zip(centre[::every], surface_z[::every], curvature[::every])]
    (output / 'terrain.txt').write_text('\n'.join(lines) + '\n')
    print(f'terrain {low:.2f} .. {high:.2f} m, track {length:.0f} m, max slope '
          f'{math.degrees(math.atan(float(slope.max()))):.0f} deg, bare {float(bare.mean()) * 100:.0f} %')


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: generate_offroad_terrain.py OUTPUT')
    generate(sys.argv[1])


if __name__ == '__main__':
    main()
