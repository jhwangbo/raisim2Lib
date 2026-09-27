"""Generate up to three woody tree capsules and convex rock proxies from glTF.

Requires NumPy and SciPy at asset-preparation time; runtime uses only .rasset and OBJ.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

TREES = ("pine_sapling_small", "fir_sapling", "tree_small_02")
ROCKS = tuple(f"rock_moss_{i}" for i in range(6))


def accessor(folder, scene, index):
    item = scene["accessors"][index]
    view = scene["bufferViews"][item["bufferView"]]
    data = (folder / scene["buffers"][view["buffer"]]["uri"]).read_bytes()
    fmt, size = {5126: ("f", 4), 5125: ("I", 4), 5123: ("H", 2)}[item["componentType"]]
    width = {"SCALAR": 1, "VEC3": 3}[item["type"]]
    start = view.get("byteOffset", 0) + item.get("byteOffset", 0)
    stride = view.get("byteStride", size * width)
    return np.array([struct.unpack_from("<" + fmt * width, data, start + i * stride)
                     for i in range(item["count"])])


def mesh_parts(folder, woody_only=False):
    scene = json.loads((folder / "model.gltf").read_text())
    node = scene["nodes"][0]
    scale = np.array(node.get("scale", [1, 1, 1]))
    translation = np.array(node.get("translation", [0, 0, 0]))
    for part in scene["meshes"][0]["primitives"]:
        name = scene["materials"][part.get("material", 0)].get("name", "")
        if woody_only and not any(token in name.lower()
                                  for token in ("trunk", "bark", "branches")):
            continue
        raw = accessor(folder, scene, part["attributes"]["POSITION"])
        # The prepared asset's node rotates Y-up glTF into Z-up Raisim/Rayrai.
        vertices = np.column_stack((raw[:, 0], -raw[:, 2], raw[:, 1])) * scale + translation
        faces = accessor(folder, scene, part["indices"]).astype(np.int64).reshape(-1, 3)
        yield name, vertices, faces


def cross_sections(vertices, faces, height):
    triangles = vertices[faces]
    adjacency = {}
    for tri in triangles:
        pair = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            za, zb = tri[a, 2], tri[b, 2]
            if (za <= height < zb) or (zb <= height < za):
                pair.append(tuple(np.round(tri[a, :2] + (height - za) /
                                           (zb - za) * (tri[b, :2] - tri[a, :2]), 6)))
        if len(pair) == 2:
            a, b = pair
            adjacency.setdefault(a, set()).add(b)
            adjacency.setdefault(b, set()).add(a)
    seen, contours = set(), []
    for first in adjacency:
        if first in seen:
            continue
        pending, component = [first], []
        seen.add(first)
        while pending:
            point = pending.pop()
            component.append(point)
            for neighbor in adjacency[point]:
                if neighbor not in seen:
                    seen.add(neighbor)
                    pending.append(neighbor)
        if len(component) >= 8 and all(len(adjacency[p]) == 2 for p in component):
            xy = np.array(component)
            center = np.mean(xy, axis=0)
            radius = float(np.quantile(np.linalg.norm(xy - center, axis=1), .75))
            contours.append((center, radius, len(component)))
    return contours


def fit_trunk_capsules(folder):
    parts = list(mesh_parts(folder, woody_only=True))
    trunk = [part for part in parts if "trunk" in part[0].lower()]
    candidates = trunk or [part for part in parts
                           if any(token in part[0].lower() for token in ("bark", "branches"))]
    if len(candidates) != 1:
        raise ValueError(f"Expected one woody primitive in {folder}")
    _, vertices, faces = candidates[0]
    total_height = float(vertices[:, 2].max())
    samples = []
    base_radius = None
    for height in np.linspace(total_height * .025, total_height * .85, 34):
        loops = cross_sections(vertices, faces, height)
        if not loops:
            break
        if not samples:
            center, radius, count = min(loops, key=lambda item: np.linalg.norm(item[0]))
            base_radius = radius
        else:
            previous = samples[-1][1]
            # The woody meshes contain detached branch contours. Stay on the
            # root-connected centerline and prefer a substantial section.
            viable = [loop for loop in loops if np.linalg.norm(loop[0] - previous) <
                      max(.08 * total_height, 4 * base_radius)]
            if not viable:
                break
            center, radius, count = min(viable, key=lambda item:
                                        np.linalg.norm(item[0] - previous) /
                                        max(base_radius, .001) + .1 * abs(math.log(item[1] / base_radius)))
            if radius > 2.5 * base_radius:
                break
        samples.append((float(height), center, radius))
    if len(samples) < 8 or samples[-1][0] < .25 * total_height:
        raise ValueError(f"Could not fit a useful trunk in {folder}")
    # Short saplings use one trunk capsule. The larger, bending tree gets three
    # segments so its upper wood is covered without broad foliage colliders.
    segments = 3 if total_height > 2.0 and len(samples) >= 24 else 1
    breaks = [round(i * (len(samples) - 1) / segments)
              for i in range(segments + 1)]
    capsules = []
    for first, last in zip(breaks, breaks[1:]):
        lower = np.array([*samples[first][1],
                          0.0 if first == 0 else samples[first][0]])
        upper = np.array([*samples[last][1], samples[last][0]])
        axis = upper - lower
        length = float(np.linalg.norm(axis))
        direction = axis / length
        rotation_axis = np.cross([0, 0, 1], direction)
        axis_length = float(np.linalg.norm(rotation_axis))
        if axis_length < 1e-9:
            quat = [1., 0., 0., 0.]
        else:
            rotation_axis /= axis_length
            angle = math.acos(float(np.clip(direction[2], -1, 1)))
            quat = [math.cos(angle / 2), *(rotation_axis * math.sin(angle / 2))]
        radius = 0.0
        for height, center, section_radius in samples[first:last + 1]:
            point = np.array([*center, height])
            along = np.clip(np.dot(point - lower, direction), 0, length)
            radius = max(radius, float(np.linalg.norm(
                point - lower - along * direction) + section_radius))
        capsules.append((radius * 1.05, length, (lower + upper) / 2, quat))
    return capsules


def rock_hull(folder, target_triangles=100):
    if not 4 <= target_triangles <= 1000:
        raise ValueError("Rock triangle target must be between 4 and 1000")
    parts = list(mesh_parts(folder))
    if len(parts) != 1:
        raise ValueError(f"Expected one rock primitive in {folder}")
    vertices = parts[0][1]
    axes = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]

    def candidate(count):
        directions = list(axes)
        for i in range(count):
            z = 1 - 2 * (i + .5) / count
            angle = i * math.pi * (3 - math.sqrt(5))
            radius = math.sqrt(1 - z * z)
            directions.append((radius * math.cos(angle), radius * math.sin(angle), z))
        chosen = sorted({int(np.argmax(vertices @ direction)) for direction in directions})
        points = vertices[chosen]
        return points, ConvexHull(points)

    maximum = min(512, max(24, 2 * target_triangles))
    stride = max(4, maximum // 20)
    best = None
    best_count = None
    for count in list(range(8, maximum + 1, stride)) + [maximum]:
        points, hull = candidate(count)
        rank = (abs(len(hull.simplices) - target_triangles), -hull.volume)
        if best is None or rank < best[0]:
            best, best_count = (rank, points, hull), count
    for count in range(max(8, best_count - stride + 1),
                       min(maximum, best_count + stride - 1) + 1):
        points, hull = candidate(count)
        rank = (abs(len(hull.simplices) - target_triangles), -hull.volume)
        if rank < best[0]:
            best = (rank, points, hull)
    _, points, hull = best
    used = sorted(set(hull.vertices.tolist()))
    remap = {old: new for new, old in enumerate(used)}
    lines = ["# Generated convex collision proxy; local Z-up metres"]
    lines += [f"v {p[0]:.9g} {p[1]:.9g} {p[2]:.9g}" for p in points[used]]
    center = points[used].mean(axis=0)
    for face in hull.simplices:
        a, b, c = points[face]
        if np.dot(np.cross(b - a, c - a), (a + b + c) / 3 - center) < 0:
            face = face[[0, 2, 1]]
        lines.append("f " + " ".join(str(remap[int(i)] + 1) for i in face))
    return "\n".join(lines) + "\n"


def generate(assets, output, rock_triangles=100):
    for name in TREES:
        capsules = fit_trunk_capsules(assets / name)
        folder = output / name
        folder.mkdir(parents=True, exist_ok=True)
        root = ET.Element("rasset", version="1")
        ET.SubElement(root, "visual", file="model.gltf")
        for radius, length, position, quat in capsules:
            ET.SubElement(root, "collision", type="capsule", radius=f"{radius:.9g}",
                          height=f"{length:.9g}",
                          position=" ".join(f"{v:.9g}" for v in position),
                          quaternion=" ".join(f"{v:.9g}" for v in quat))
        ET.indent(root)
        (folder / "model.rasset").write_text(ET.tostring(root, encoding="unicode") + "\n")
        print(name, "trunk capsules", len(capsules))
    for name in ROCKS:
        folder = output / name
        folder.mkdir(parents=True, exist_ok=True)
        proxy = rock_hull(assets / name, rock_triangles)
        (folder / "collision.obj").write_text(proxy)
        root = ET.Element("rasset", version="1")
        ET.SubElement(root, "visual", file="model.gltf")
        ET.SubElement(root, "collision", type="convex_mesh", file="collision.obj")
        ET.indent(root)
        (folder / "model.rasset").write_text(ET.tostring(root, encoding="unicode") + "\n")
        print(name, "convex collision mesh", proxy.count("\nf "), "triangles")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assets", type=Path)
    parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("--rock-triangles", type=int, default=100,
                        help="Target number of convex proxy triangles (default: 100)")
    args = parser.parse_args()
    generate(args.assets, args.output or args.assets, args.rock_triangles)
