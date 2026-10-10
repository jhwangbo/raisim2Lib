#!/usr/bin/env python3
"""Prepare the vehicle and the race venue of the rayrai off-road buggy example.

Usage: prepare_offroad_buggy_assets.py DIR [OUTPUT]
   DIR    folder given to download_offroad_buggy_assets.py (reads DIR/assets)
   OUTPUT rsc/offroad_buggy folder to write (default DIR/prepared)

* The Sketchfab "Sci - Fi Buggy" is baked into Z-up meshes in metres in the
  chassis frame of offroad_buggy_design.py (x forward, origin midway between
  the axles at wheel-centre height), one primitive per material. Its wheels,
  steering wheel and front fenders become separate meshes in the frames of
  the links that carry them; its static suspension is left out, because the
  simulated suspension replaces it.
* The Sketchfab "Coilover Shock Absorber Damper" is split into the damper
  body with the top eye and the rod with the bottom eye, scaled to the
  damper length of the design. Its coil spring is left out: rayrai_offroad_buggy
  draws the spring at its simulated length.
* The wishbones, uprights with brake calipers, brake discs, tie rods and the
  steering rack are generated from the hardpoints.
* buggy.urdf is written with offroad_buggy_design.build_urdf(), the terrain
  with generate_offroad_terrain.py and the race venue's props with
  prepare_offroad_venue.py.

Requires NumPy, SciPy and Pillow.
"""
import copy
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

import generate_offroad_terrain
import offroad_buggy_design as design
from prepare_city_assets import dump, load, node_matrix, read_float_accessor

BUGGY = 'sci_fi_buggy'
COILOVER = 'coilover'
SCALE = 0.145  # model units per metre: the 4.98-unit tyre becomes 0.722 m
# Model frame (x left, y up, z forward) to the chassis frame, before scaling.
MODEL_TO_CHASSIS = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
# Wheel-base midpoint at wheel-centre height, in model units after the turn.
MODEL_ORIGIN = np.array([-11.1065, 0.0, -0.699])
# Nodes left out of the body: the static suspension and its decals, and parts
# that become their own meshes.
SUSPENSION_NODES = ['front suspension', 'rear suspension', 'Examples_009.007_projected', '.003_Atlas']
WHEEL_NODES = {'f': ['tyre', 'rims'], 'r': ['tyre back', 'rim back']}
STEERING_WHEEL_NODE = 'steering wheel'
FENDER_NODES = ['fender', 'fender bar 2']
TEXTURE_SIZE = 2048
# The rims' metal-roughness map makes the hub caps perfect mirrors that show the sun as a white
# disc; a satin finish instead.
RIM_FINISH = {'rims': dict(pbrMetallicRoughness=dict(metallicFactor=0.85, roughnessFactor=0.38))}
# Coilover parts (node names of the model) on the damper body; the rest is on the rod.
COILOVER_BODY = ['mount2_low', 'top2_low', 'navoj_low', 'lock nut_low', 'lock nut pt2_low', 'case_low', 'brtva_low']
COILOVER_ROD = ['osovina_low', 'bump stop_low', 'spring seat_low', 'mount_low']
COILOVER_BEARINGS = 'lezajevi_low'   # one node with the bearings of both eyes
COILOVER_SPRING = 'opruga_low'
# Poly Haven ground textures (map -> size in pixels): meadow, packed track, loose dirt, gravel and rock.
GROUND_TEXTURES = {name: {'diff': 2048, 'nor_gl': 1024}
                   for name in ('rocky_terrain_02', 'dry_ground_rocks', 'brown_mud_dry', 'rocky_trail', 'rock_face_03')}


class Gltf:
    """A glTF 2.0 document with one buffer, written from NumPy arrays."""

    def __init__(self, name):
        self.name = name
        self.data = bytearray()
        self.doc = dict(asset=dict(version='2.0', generator='prepare_offroad_buggy_assets.py'), scene=0,
                        scenes=[dict(nodes=[0])], nodes=[dict(name=name, mesh=0)],
                        meshes=[dict(name=name, primitives=[])], materials=[], accessors=[], bufferViews=[])

    def _add(self, array, kind, target, bounds=False):
        while len(self.data) % 4:
            self.data.append(0)
        self.doc['bufferViews'].append(dict(buffer=0, byteOffset=len(self.data), byteLength=array.nbytes,
                                            target=target))
        self.data.extend(array.tobytes())
        accessor = dict(bufferView=len(self.doc['bufferViews']) - 1, componentType=5125 if kind == 'SCALAR' else 5126,
                        count=int(array.size if kind == 'SCALAR' else len(array)), type=kind)
        if bounds:
            accessor.update(min=array.min(0).tolist(), max=array.max(0).tolist())
        self.doc['accessors'].append(accessor)
        return len(self.doc['accessors']) - 1

    def material(self, material):
        self.doc['materials'].append(material)
        return len(self.doc['materials']) - 1

    def primitive(self, positions, normals, indices, material, uvs=None):
        attributes = dict(POSITION=self._add(np.asarray(positions, np.float32), 'VEC3', 34962, True),
                          NORMAL=self._add(np.asarray(normals, np.float32), 'VEC3', 34962))
        if uvs is not None:
            attributes['TEXCOORD_0'] = self._add(np.asarray(uvs, np.float32), 'VEC2', 34962)
        self.doc['meshes'][0]['primitives'].append(
            dict(attributes=attributes, indices=self._add(np.asarray(indices, np.uint32).ravel(), 'SCALAR', 34963),
                 material=material))

    def write(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        binary = path.with_suffix('.bin')
        binary.write_bytes(bytes(self.data))
        self.doc['buffers'] = [dict(uri=binary.name, byteLength=len(self.data))]
        dump(self.doc, path)


# ---------------------------------------------------------------------------
# Procedural geometry: each function returns (positions, normals, triangles).

def frame_for(axis):
    axis = axis / np.linalg.norm(axis)
    helper = np.array([0, 0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0, 0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    return u, np.cross(axis, u), axis


def tube(p0, p1, radius, segments=20, caps=True, radius1=None):
    """A cylinder (or a cone frustum with radius1) from p0 to p1."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    radius1 = radius if radius1 is None else radius1
    u, v, w = frame_for(p1 - p0)
    angle = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    ring = np.outer(np.cos(angle), u) + np.outer(np.sin(angle), v)
    slope = (radius - radius1) / np.linalg.norm(p1 - p0)
    normals = ring + slope * w
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    positions = np.vstack([p0 + radius * ring, p1 + radius1 * ring])
    normals = np.vstack([normals, normals])
    i = np.arange(segments)
    j = (i + 1) % segments
    triangles = np.vstack([np.stack([i, j, segments + j], 1), np.stack([i, segments + j, segments + i], 1)])
    parts = [(positions, normals, triangles)]
    if caps:
        for centre, r, n, flip in ((p0, radius, -w, True), (p1, radius1, w, False)):
            pos = np.vstack([centre, centre + r * ring])
            tri = np.stack([np.zeros(segments, int), 1 + j, 1 + i], 1) if flip else \
                np.stack([np.zeros(segments, int), 1 + i, 1 + j], 1)
            parts.append((pos, np.tile(n, (segments + 1, 1)), tri))
    return merge(parts)


def sphere(centre, radius, segments=16, rings=10):
    theta = np.linspace(0, np.pi, rings + 1)
    phi = np.linspace(0, 2 * np.pi, segments + 1)
    t, f = np.meshgrid(theta, phi, indexing='ij')
    normals = np.stack([np.sin(t) * np.cos(f), np.sin(t) * np.sin(f), np.cos(t)], -1).reshape(-1, 3)
    positions = np.asarray(centre) + radius * normals
    a = np.arange(rings)[:, None] * (segments + 1) + np.arange(segments)[None, :]
    a = a.ravel()
    triangles = np.vstack([np.stack([a, a + segments + 1, a + 1], 1),
                           np.stack([a + 1, a + segments + 1, a + segments + 2], 1)])
    return positions, normals, triangles


def box(centre, size, rotation=np.eye(3)):
    positions, normals, triangles = [], [], []
    for axis in range(3):
        for sign in (-1, 1):
            n = np.zeros(3)
            n[axis] = sign
            a, b = [k for k in range(3) if k != axis]
            corners = []
            for da, db in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                c = np.zeros(3)
                c[axis], c[a], c[b] = sign, da, db
                corners.append(c * np.asarray(size) / 2)
            base = len(positions) * 4
            quad = np.array([[0, 1, 2], [0, 2, 3]]) + base
            # Keep the winding counter-clockwise seen from outside.
            if np.dot(np.cross(corners[1] - corners[0], corners[2] - corners[0]), n) < 0:
                quad = quad[:, ::-1]
            positions.append(corners)
            normals.append([n] * 4)
            triangles.append(quad)
    positions = np.vstack(positions) @ rotation.T + np.asarray(centre)
    return positions, np.vstack(normals) @ rotation.T, np.vstack(triangles)


def merge(parts):
    positions, normals, triangles, offset = [], [], [], 0
    for p, n, t in parts:
        positions.append(p)
        normals.append(n)
        triangles.append(np.asarray(t) + offset)
        offset += len(p)
    return np.vstack(positions), np.vstack(normals), np.vstack(triangles)


# Materials of the generated parts.
def pbr(name, colour, metallic, roughness):
    return dict(name=name, pbrMetallicRoughness=dict(baseColorFactor=list(colour) + [1.0], metallicFactor=metallic,
                                                    roughnessFactor=roughness))


ARM = pbr('arm_titanium', (0.42, 0.43, 0.45), 1.0, 0.36)
STEEL = pbr('bright_steel', (0.56, 0.56, 0.55), 1.0, 0.28)
ALUMINIUM = pbr('machined_aluminium', (0.70, 0.71, 0.73), 1.0, 0.33)
UPRIGHT = pbr('anodised_orange', (0.80, 0.24, 0.02), 0.6, 0.35)
CALIPER = pbr('caliper_red', (0.70, 0.03, 0.02), 0.1, 0.35)
ADJUSTER = pbr('black_anodised', (0.02, 0.02, 0.022), 0.5, 0.4)
DISC = pbr('brake_disc', (0.32, 0.31, 0.30), 1.0, 0.45)
RUBBER = pbr('rubber_boot', (0.02, 0.02, 0.02), 0.0, 0.7)


def write_parts(path, groups):
    """groups: list of (material, [parts]); one primitive per material."""
    gltf = Gltf(path.stem)
    for material, parts in groups:
        positions, normals, triangles = merge(parts)
        gltf.primitive(positions, normals, triangles, gltf.material(copy.deepcopy(material)))
    gltf.write(path)


def wishbone(inner, ball, spread, radius, mount=None):
    """An A-arm in its link frame (origin midway between the bushings): two tubes from the
    bushings to the ball joint, a brace, bushing sleeves and the ball-joint housing."""
    front, rear = np.array([spread, 0, 0]), np.array([-spread, 0, 0])
    arm = [tube(front, ball, radius), tube(rear, ball, radius),
           tube(front + 0.42 * (ball - front), rear + 0.42 * (ball - rear), radius * 0.75)]
    sleeves = [tube(c - [0.03, 0, 0], c + [0.03, 0, 0], radius * 1.9) for c in (front, rear)]
    housing = [tube(ball - [0, 0, 0.022], ball + [0, 0, 0.022], 0.030)]
    steel = [tube(c - [0.042, 0, 0], c + [0.042, 0, 0], radius * 0.7) for c in (front, rear)]
    steel.append(sphere(ball, 0.021))
    if mount is not None:
        # Clevis for the coilover's rod eye: two plates and a bolt along x.
        down = np.array([0, 0, -0.045])
        for dx in (-0.022, 0.022):
            arm.append(box(mount + [dx, 0, 0] + down / 2, (0.008, 0.05, 0.075)))
        steel.append(tube(mount - [0.032, 0, 0], mount + [0.032, 0, 0], 0.007))
    return [(ARM, arm + sleeves), (STEEL, steel)]


def upright_parts(p, side, steer):
    """Upright in its frame (origin at the lower ball joint)."""
    o = p['lbj']
    lbj, ubj, wheel, otr, king = (p[k] - o for k in ('lbj', 'ubj', 'wheel', 'otr', 'king'))
    inboard = np.array([0, -side, 0])
    hub = wheel + 0.045 * inboard
    alu = [tube(lbj + [0, 0, -0.01], ubj + [0, 0, 0.01], 0.024),
           tube(lbj + 0.15 * (ubj - lbj), hub + [0, 0, -0.05], 0.020),
           tube(lbj + 0.85 * (ubj - lbj), hub + [0, 0, 0.05], 0.020),
           tube(king, otr, 0.013), sphere(otr, 0.019), sphere(ubj, 0.018)]
    # Hub carrier: dark steel, since its face shows through the rim's centre bore.
    hub_parts = [tube(hub + 0.035 * inboard, hub - 0.03 * inboard, 0.058)]
    # Brake caliper over the rear top of the disc, inboard of the rim.
    angle = math.radians(40)
    caliper_centre = wheel + 0.072 * inboard + 0.145 * np.array([-math.cos(angle), 0, math.sin(angle)])
    rotation = np.array([[math.cos(angle), 0, math.sin(angle)], [0, 1, 0], [-math.sin(angle), 0, math.cos(angle)]])
    caliper = [box(caliper_centre, (0.11, 0.055, 0.05), rotation),
               tube(caliper_centre - [0, 0.03 * side, 0], caliper_centre + [0, 0.03 * side, 0], 0.012)]
    return [(UPRIGHT, alu), (ADJUSTER, hub_parts), (CALIPER, caliper)]


def disc_parts(axle, side):
    """Ventilated brake disc and hat in the wheel frame."""
    inboard = np.array([0, -side, 0])
    centre = 0.072 * inboard
    return [(DISC, [tube(centre - 0.012 * inboard, centre + 0.012 * inboard, 0.165, 48)]),
            (STEEL, [tube(centre + 0.012 * inboard, 0.02 * inboard, 0.075, 32)])]


def tie_rod_parts(p):
    rod = p['otr'] - p['itr']
    return [(STEEL, [tube(0.06 * rod, 0.94 * rod, 0.010), sphere(np.zeros(3), 0.018), sphere(rod, 0.016)]),
            (ADJUSTER, [tube(0.30 * rod, 0.55 * rod, 0.014, 6)])]


def generate_parts(meshes):
    for c, axle, side in design.CORNERS:
        p = design.corner_points(axle, side)
        write_parts(meshes / f'{c}_lower_arm.gltf',
                    wishbone(p['lower_inner'], p['lbj'] - p['lower_inner'], design.LOWER_INNER['spread'], 0.017,
                             p['shock_lower'] - p['lower_inner']))
        write_parts(meshes / f'{c}_upper_arm.gltf',
                    wishbone(p['upper_inner'], p['ubj'] - p['upper_inner'], design.UPPER_INNER['spread'], 0.014))
        write_parts(meshes / f'{c}_upright.gltf', upright_parts(p, side, design.AXLES[axle]['steer']))
        write_parts(meshes / f'{c}_tie_rod.gltf', tie_rod_parts(p))
    # Steering rack: the bar moves with the rack joint, the housing is part of the chassis.
    itr = design.corner_points('f', 1)['itr'] - design.rack_origin()
    write_parts(meshes / 'steering_rack.gltf', [(STEEL, [tube(-itr - [0, 0.03, 0], itr + [0, 0.03, 0], 0.011)])])
    housing = [tube([0, -0.40, 0], [0, 0.40, 0], 0.028), tube([0.0, -0.05, 0], [0.07, -0.05, 0.06], 0.026)]
    boots = [tube([0, s * 0.40, 0], [0, s * (itr[1] - 0.035), 0], 0.024, radius1=0.014) for s in (-1, 1)]
    write_parts(meshes / 'steering_rack_housing.gltf', [(ALUMINIUM, housing), (RUBBER, boots)])


# ---------------------------------------------------------------------------
# Sketchfab models.

def flatten(document, binary, root):
    """Mesh primitives of every node with their transforms (and root) baked in, tagged with the
    names of the node and its ancestors."""
    parts = []

    def visit(index, parent, names):
        node = document['nodes'][index]
        world = parent @ node_matrix(node)
        names = names + [node.get('name', '')]
        if 'mesh' in node:
            linear = world[:3, :3]
            normal_matrix = np.linalg.inv(linear).T
            for primitive in document['meshes'][node['mesh']]['primitives']:
                attributes = primitive['attributes']
                read = lambda key: read_float_accessor(document, binary, attributes[key])
                normals = read('NORMAL') @ normal_matrix.T
                normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
                triangles = read_float_accessor(document, binary, primitive['indices']).reshape(-1, 3).astype(np.uint32)
                if np.linalg.det(linear) < 0:
                    triangles = triangles[:, ::-1]
                parts.append(dict(names=names, material=primitive.get('material'),
                                  POSITION=read('POSITION') @ linear.T + world[:3, 3], NORMAL=normals,
                                  TEXCOORD_0=read('TEXCOORD_0') if 'TEXCOORD_0' in attributes else None,
                                  indices=triangles))
        for child in node.get('children', []):
            visit(child, world, names)

    for scene_root in document['scenes'][document.get('scene', 0)]['nodes']:
        visit(scene_root, root, [])
    return parts


def select_triangles(part, keep):
    """The triangles of a part whose centroids satisfy keep(centroids), with unused vertices dropped."""
    triangles = part['indices']
    mask = keep(part['POSITION'][triangles].mean(1))
    triangles = triangles[mask]
    used, remap = np.unique(triangles, return_inverse=True)
    result = dict(part, indices=remap.reshape(-1, 3).astype(np.uint32))
    for key in ('POSITION', 'NORMAL', 'TEXCOORD_0'):
        if part[key] is not None:
            result[key] = part[key][used]
    return result


def translate(part, offset):
    return dict(part, POSITION=part['POSITION'] - offset)


class TexturedModel:
    """Writes meshes that use the materials and textures of a Sketchfab model."""

    def __init__(self, document, folder, target, texture_size):
        self.document, self.folder, self.target, self.size = document, folder, target, texture_size
        self.images = {}

    def image(self, index, alpha):
        if index not in self.images:
            from PIL import Image
            uri = self.document['images'][index]['uri']
            picture = Image.open(self.folder / uri)
            picture.thumbnail((self.size, self.size), Image.LANCZOS)
            stem = Path(uri).stem
            out = f'textures/{stem}.png' if alpha else f'textures/{stem}.jpg'
            (self.target / out).parent.mkdir(parents=True, exist_ok=True)
            if alpha:
                picture.save(self.target / out, optimize=True)
            else:
                picture.convert('RGB').save(self.target / out, quality=92)
            self.images[index] = out
        return self.images[index]

    def write(self, path, parts, material_overrides=None):
        gltf = Gltf(path.stem)
        doc = gltf.doc
        doc.update(images=[], textures=[], samplers=[dict(magFilter=9729, minFilter=9987, wrapS=10497, wrapT=10497)])
        texture_map, material_map = {}, {}

        def texture(info, alpha=False):
            source = self.document['textures'][info['index']]['source']
            if source not in texture_map:
                doc['images'].append(dict(uri=self.image(source, alpha)))
                doc['textures'].append(dict(source=len(doc['images']) - 1, sampler=0))
                texture_map[source] = len(doc['textures']) - 1
            return dict(info, index=texture_map[source])

        def material(index):
            if index not in material_map:
                m = copy.deepcopy(self.document['materials'][index])
                alpha = m.get('alphaMode') in ('BLEND', 'MASK')
                pbr_ = m.get('pbrMetallicRoughness', {})
                for key in ('baseColorTexture', 'metallicRoughnessTexture'):
                    if key in pbr_:
                        pbr_[key] = texture(pbr_[key], alpha and key == 'baseColorTexture')
                for key in ('normalTexture', 'occlusionTexture', 'emissiveTexture'):
                    if key in m:
                        m[key] = texture(m[key])
                for extension in m.get('extensions', {}).values():
                    for key, value in extension.items():
                        if key.endswith('Texture'):
                            extension[key] = texture(value)
                for key, value in (material_overrides or {}).get(m['name'], {}).items():
                    if key == 'pbrMetallicRoughness':
                        m[key] = dict({k: v for k, v in m[key].items() if k != 'metallicRoughnessTexture'}, **value)
                    else:
                        m[key] = value
                material_map[index] = gltf.material(m)
            return material_map[index]

        merged = {}
        for part in parts:
            merged.setdefault(part['material'], []).append(part)
        for index, group in merged.items():
            positions = np.vstack([p['POSITION'] for p in group])
            normals = np.vstack([p['NORMAL'] for p in group])
            uvs = np.vstack([p['TEXCOORD_0'] if p['TEXCOORD_0'] is not None else np.zeros((len(p['POSITION']), 2))
                             for p in group])
            offsets = np.cumsum([0] + [len(p['POSITION']) for p in group])[:-1]
            triangles = np.vstack([p['indices'] + o for p, o in zip(group, offsets)])
            gltf.primitive(positions, normals, triangles, material(index), uvs)
        used = sorted({e for m in doc['materials'] for e in m.get('extensions', {})})
        if used:
            doc['extensionsUsed'] = used
        gltf.write(path)


def principal_axes(points):
    centre = (points.min(0) + points.max(0)) / 2
    _, _, vt = np.linalg.svd(points - points.mean(0), full_matrices=False)
    return centre, vt  # rows: directions of decreasing spread


def prepare_buggy(source, output):
    folder = source / 'sketchfab' / BUGGY
    document = load(folder / 'scene.gltf')
    binary = (folder / document['buffers'][0]['uri']).read_bytes()
    root = np.eye(4)
    root[:3, :3] = SCALE * MODEL_TO_CHASSIS
    root[:3, 3] = -SCALE * MODEL_ORIGIN
    parts = flatten(document, binary, root)
    meshes = output / 'meshes'
    model = TexturedModel(document, folder, meshes, TEXTURE_SIZE)
    # A name matches a node by its full name or, starting with '.', by its suffix (the model has
    # nodes with garbled names such as '\ufffd\ufffd\ufffd\ufffd.003_Atlas').
    has = lambda part, names: any(node == name or name.startswith('.') and node.endswith(name)
                                  for node in part['names'] for name in names)
    body = [p for p in parts if not has(p, SUSPENSION_NODES + WHEEL_NODES['f'] + WHEEL_NODES['r']
                                        + [STEERING_WHEEL_NODE] + FENDER_NODES)]
    model.write(meshes / 'chassis.gltf', body)
    report = {}
    for c, axle, side in design.CORNERS:
        p = design.corner_points(axle, side)
        wheel = [select_triangles(part, lambda centres: centres[:, 1] * side > 0)
                 for part in parts if has(part, WHEEL_NODES[axle])]
        points = np.vstack([w['POSITION'] for w in wheel])
        centre = (points.min(0) + points.max(0)) / 2
        report[c] = dict(wheel_centre=centre.round(4).tolist(), design=p['wheel'].round(4).tolist(),
                         radius=float((points.max(0) - points.min(0))[2] / 2))
        model.write(meshes / f'{c}_wheel.gltf', [translate(w, p['wheel']) for w in wheel], RIM_FINISH)
        write_parts(meshes / f'{c}_disc.gltf', disc_parts(axle, side))
        if axle == 'f':
            # The front fenders ride on the uprights, so they steer and move with the wheels.
            fender = [select_triangles(part, lambda centres: centres[:, 1] * side > 0)
                      for part in parts if has(part, FENDER_NODES)]
            model.write(meshes / f'{c}_fender.gltf', [translate(f, p['lbj']) for f in fender])
    wheel_parts = [p for p in parts if has(p, [STEERING_WHEEL_NODE])]
    points = np.vstack([w['POSITION'] for w in wheel_parts])
    centre, axes = principal_axes(points)
    axis = axes[2] if axes[2][0] < 0 else -axes[2]  # towards the driver
    model.write(meshes / 'steering_wheel.gltf', [translate(w, centre) for w in wheel_parts])
    report['steering_wheel'] = dict(centre=centre.round(4).tolist(), axis=axis.round(4).tolist())
    shutil.copyfile(folder / 'license.txt', meshes / f'license_{BUGGY}.txt')
    return report


def prepare_coilover(source, output):
    """Damper body (top eye at the origin) and rod in the coilover link frames: z along the
    damper towards the top eye, x along the eye bolts."""
    folder = source / 'sketchfab' / COILOVER
    document = load(folder / 'scene.gltf')
    binary = (folder / document['buffers'][0]['uri']).read_bytes()
    parts = flatten(document, binary, np.eye(4))
    named = lambda name: [p for p in parts if name in p['names']]
    bearings = named(COILOVER_BEARINGS)
    split = 0.15  # the bearings of the two eyes lie below and above this height (model y)
    lower = [select_triangles(p, lambda c: c[:, 1] < split) for p in bearings]
    upper = [select_triangles(p, lambda c: c[:, 1] >= split) for p in bearings]
    eye = lambda group: (np.vstack([p['POSITION'] for p in group]).min(0)
                         + np.vstack([p['POSITION'] for p in group]).max(0)) / 2
    bottom, top = eye(lower), eye(upper)
    length = design.spring_damper('f')['length']
    scale = length / (top[1] - bottom[1])
    # Model (x, y, z) with y along the damper and z along the eye bolts -> link frame (z, x, y).
    turn = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float) * scale
    transform = lambda p: dict(p, POSITION=(p['POSITION'] - top) @ turn.T, NORMAL=p['NORMAL'] @ turn.T / scale)
    meshes = output / 'meshes'
    model = TexturedModel(document, folder, meshes, 1024)
    model.write(meshes / 'coilover_body.gltf', [transform(p) for name in COILOVER_BODY for p in named(name)]
                + [transform(p) for p in upper])
    model.write(meshes / 'coilover_rod.gltf', [transform(p) for name in COILOVER_ROD for p in named(name)]
                + [transform(p) for p in lower])
    spring = np.vstack([p['POSITION'] for p in named(COILOVER_SPRING)])
    seats = dict(lower=float((spring[:, 1].min() - top[1]) * scale), upper=float((spring[:, 1].max() - top[1]) * scale),
                 radius=float((spring[:, 0].max() - spring[:, 0].min()) / 2 * scale))
    shutil.copyfile(folder / 'license.txt', meshes / f'license_{COILOVER}.txt')
    return dict(scale=scale, eye_distance_model=float(top[1] - bottom[1]), spring=seats)


def prepare_ground_textures(source, output):
    """Colour (2K) and normal maps (1K) of the ground materials, as JPEG."""
    from PIL import Image
    target = output / 'terrain' / 'textures'
    target.mkdir(parents=True, exist_ok=True)
    for name, maps in GROUND_TEXTURES.items():
        for kind, size in maps.items():
            picture = Image.open(source / 'textures' / name / f'{name}_{kind}_2k.jpg')
            if size != 2048:
                picture = picture.resize((size, size), Image.LANCZOS)
            picture.convert('RGB').save(target / f'{name}_{kind}_{size // 1024}k.jpg', quality=90)


def main():
    import prepare_offroad_venue  # imports this module's geometry helpers
    if len(sys.argv) not in (2, 3):
        raise SystemExit('Usage: prepare_offroad_buggy_assets.py DIR [OUTPUT]')
    source = Path(sys.argv[1]) / 'assets'
    output = Path(sys.argv[2]) if len(sys.argv) == 3 else Path(sys.argv[1]) / 'prepared'
    (output / 'meshes').mkdir(parents=True, exist_ok=True)
    report = dict(buggy=prepare_buggy(source, output), coilover=prepare_coilover(source, output))
    generate_parts(output / 'meshes')
    prepare_ground_textures(source, output)
    sw = report['buggy']['steering_wheel']
    (output / 'buggy.urdf').write_text(design.build_urdf((np.array(sw['centre']), np.array(sw['axis']))))
    generate_offroad_terrain.generate(output / 'terrain')
    report['props'] = prepare_offroad_venue.prepare_venue(source, output)
    shutil.copyfile(source / 'sources.json', output / 'sources.json')
    notices = Path(__file__).resolve().parents[2] / 'rsc' / 'offroad_buggy'
    for notice in ['ATTRIBUTION.md', 'LICENSE-CC0-1.0.txt']:
        if (notices / notice).resolve() != (output / notice).resolve():
            shutil.copyfile(notices / notice, output / notice)
    files = [dict(path=str(p.relative_to(output)), bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
             for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'manifest.json'
             and not p.name.endswith('.lods') and not p.name.endswith('.rmpc')]
    (output / 'manifest.json').write_text(json.dumps(dict(license='CC-BY-4.0 and CC0-1.0, see ATTRIBUTION.md',
                                                          files=files), indent=2) + '\n')
    print(json.dumps(report, indent=1))
    print(design.summary())


if __name__ == '__main__':
    main()
