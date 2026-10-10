"""Race-venue assets of rayrai_offroad_buggy, written to rsc/offroad_buggy/props.

Called by prepare_offroad_buggy_assets.py. Every model is baked into a Z-up
glTF file in metres, centred on its footprint with its base at z = 0, with a
.rasset descriptor of simple colliders where the example needs one.

* Poly Haven (CC0): boulders, stones, a fallen trunk, tyres, chain-link fence,
  generator, jerrycan, picnic table and chair, with their 1K textures.
* Sketchfab (CC BY 4.0): hay bales, water barriers, a portable toilet and a
  bleacher, with textures reduced to 1K JPEG files.
* Generated: the start/finish gantry with its banner, feather flags, pop-up
  canopies, tyre stacks, the fence module and the track's marker posts, and
  small convex collision proxies of the posts and of rsc/city's traffic cone,
  which the example knocks around.
* Simplified copies of rsc/warehouse's tool cart and crate and rsc/city's
  traffic cone.

Dense scanned meshes are reduced with mesh_simplify.py (the normal maps keep
their detail): rayrai draws an instanced batch at one level of detail, so a
20 000-triangle jerrycan or pebble costs that much in every nearby copy.
"""
import copy
import math
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

from prepare_city_assets import flatten_scene, load
from prepare_offroad_buggy_assets import Gltf, box, merge, pbr, tube
from prepare_warehouse_assets import convert_images, flatten_node, matrix, merge_by_material, recentre, write_mesh
from mesh_simplify import simplify_part

GENERATOR = 'prepare_offroad_venue.py'
# Poly Haven props: nodes to keep (default all), colliders ('hull' for a convex proxy, 'log' for
# a capsule along the longest axis, 'bounds' for the bounding box, or a list of shapes), the share
# of triangles to keep, and material changes.
POLYHAVEN = {
    'boulder_01': dict(colliders='hull', keep=0.15),
    'namaqualand_boulder_02': dict(colliders='hull', keep=0.1),
    'rock_07': dict(colliders='hull', keep=0.1),
    'rock_09': dict(colliders=[], keep=0.08),
    'dead_tree_trunk_02': dict(colliders='log', keep=0.1),
    'portable_generator': dict(colliders='bounds', keep=0.5,
                               materials={'portable_generator_glass': dict(alphaMode='OPAQUE')}),
    'metal_jerrycan': dict(colliders=[], keep=0.1),
    'wooden_picnic_table': dict(colliders='bounds'),
    'plastic_monobloc_chair_01': dict(colliders=[]),
}
# Props of other rsc folders, copied with fewer triangles: folder and share of triangles to keep.
REUSED = {'tool_cart': ('warehouse/props/tool_cart', 0.25), 'plastic_crate_03': ('warehouse/props/plastic_crate_03', 0.12),
          'traffic_cone': ('city/traffic_cone', 0.15)}
TYRE_KEEP = 0.5
# Sketchfab props: variants (stem -> nodes, None for all), scale and the colliders of each variant.
SKETCHFAB = {
    'hay_bales': dict(scale=0.65, variants={'round': ['Cylinder_0'], 'square': ['Cube_1']},
                      colliders={'round': 'box', 'square': 'box'}),
    'water_barrier': dict(scale=1.0, variants={'red': ['WaterBarrier_RED'], 'white': ['WaterBarrier_WHITE']},
                          colliders={'red': 'box', 'white': 'box'}),
    'porta_potty': dict(scale=1.0, variants={'model': None}, colliders={'model': 'box'}),
    'bleacher': dict(scale=1.0, variants={'model': None}, colliders={'model': 'box'}),
}
TYRE = 'old_tyre'
TYRE_STACK = 3
FENCE_PANEL, FENCE_POST = 'modular_chainlink_fence_double', 'modular_chainlink_fence_post'
FENCE_SPACING = 2.0   # one panel and one post
GANTRY_SPAN = 12.6    # between the tower centres [m]
GANTRY_HEIGHT = 6.0
CANOPY_COLOURS = {'blue': (0.06, 0.22, 0.62), 'red': (0.62, 0.06, 0.05), 'white': (0.82, 0.82, 0.80)}
FLAG_COLOURS = {'orange': (242, 92, 5), 'blue': (24, 84, 190)}
BANNER_TEXT = 'RAISIM'


def convex_proxy(points, target=100):
    """A convex hull of about target triangles through the points extreme along Fibonacci-sphere
    directions, as Wavefront OBJ text."""
    best = None
    for count in range(8, 400, 4):
        z = 1 - 2 * (np.arange(count) + 0.5) / count
        angle = np.arange(count) * math.pi * (3 - math.sqrt(5))
        directions = np.stack([np.sqrt(1 - z * z) * np.cos(angle), np.sqrt(1 - z * z) * np.sin(angle), z], 1)
        directions = np.vstack([directions, np.eye(3), -np.eye(3)])
        chosen = points[np.unique(np.argmax(points @ directions.T, axis=0))]
        hull = ConvexHull(chosen)
        rank = abs(len(hull.simplices) - target)
        if best is None or rank < best[0]:
            best = (rank, chosen, hull)
    _, chosen, hull = best
    used = np.unique(hull.simplices)
    remap = {old: new for new, old in enumerate(used)}
    centre = chosen[used].mean(0)
    lines = [f'# Convex collision proxy written by {GENERATOR}; Z-up metres']
    lines += [f'v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}' for p in chosen[used]]
    for face in hull.simplices:
        a, b, c = chosen[face]
        if np.dot(np.cross(b - a, c - a), (a + b + c) / 3 - centre) < 0:
            face = face[[0, 2, 1]]
        lines.append('f ' + ' '.join(str(remap[i] + 1) for i in face))
    return '\n'.join(lines) + '\n', len(hull.simplices)


def log_capsule(points):
    """A capsule along the principal axis of a lying log: (radius, length, centre, quaternion)."""
    centre = points.mean(0)
    _, _, axes = np.linalg.svd(points - centre, full_matrices=False)
    axis = axes[0] / np.linalg.norm(axes[0])
    along = (points - centre) @ axis
    radial = np.linalg.norm(points - centre - np.outer(along, axis), axis=1)
    radius = float(np.quantile(radial, 0.8))
    lo, hi = np.quantile(along, [0.005, 0.995])
    centre = centre + axis * (lo + hi) / 2
    length = max(float(hi - lo) - 2 * radius, 0.01)
    # Quaternion (w, x, y, z) turning +z onto the axis.
    rotation_axis = np.cross([0, 0, 1], axis)
    s = np.linalg.norm(rotation_axis)
    angle = math.atan2(s, axis[2])
    q = [math.cos(angle / 2), *(rotation_axis / max(s, 1e-12) * math.sin(angle / 2))]
    return radius, length, centre, q


def write_descriptor(target, stem, colliders):
    """A .rasset next to stem.gltf. colliders: (kind, size dict, position) tuples for boxes and
    cylinders, ('capsule', radius, length, centre, quaternion) and ('convex_mesh', file)."""
    lines = ['<rasset version="1">', f'  <visual file="{stem}.gltf"/>']
    for collider in colliders:
        if collider[0] == 'convex_mesh':
            lines.append(f'  <collision type="convex_mesh" file="{collider[1]}"/>')
        elif collider[0] == 'capsule':
            _, radius, length, centre, q = collider
            lines.append(f'  <collision type="capsule" radius="{radius:.4f}" height="{length:.4f}" '
                         f'position="{" ".join(f"{v:.4f}" for v in centre)}" '
                         f'quaternion="{" ".join(f"{v:.6f}" for v in q)}"/>')
        else:
            kind, size, position = collider
            attributes = ' '.join(f'{key}="{" ".join(f"{v:.4g}" for v in value) if isinstance(value, tuple) else value}"'
                                  for key, value in size.items())
            lines.append(f'  <collision type="{kind}" {attributes} '
                         f'position="{" ".join(f"{v:.4g}" for v in position)}"/>')
    lines.append('</rasset>')
    (target / f'{stem}.rasset').write_text('\n'.join(lines) + '\n')


def bounds_box(parts):
    points = np.vstack([p['POSITION'] for p in parts])
    low, high = points.min(0), points.max(0)
    return ('box', dict(size=tuple(np.round(high - low, 3))), tuple(np.round((low + high) / 2, 3)))


def report(name, parts, size):
    print(f'{name}: {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m, '
          f'{sum(len(p["indices"]) for p in parts)} triangles', flush=True)


# ---------------------------------------------------------------------------
# Downloaded models.

def polyhaven_parts(source, name, nodes=None, root=None):
    document = load(source / 'models' / name / f'{name}_1k.gltf')
    binary = (source / 'models' / name / document['buffers'][0]['uri']).read_bytes()
    root = matrix() if root is None else root
    parts = ([p for node in nodes for p in flatten_node(document, binary, node, root)] if nodes
             else flatten_scene(document, binary, root))
    return document, parts


def copy_textures(source, target, document):
    for image in document.get('images', []):
        (target / image['uri']).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / image['uri'], target / image['uri'])


def patch_materials(document, changes):
    document = copy.deepcopy(document)
    for material in document.get('materials', []):
        material.update(changes.get(material.get('name'), {}))
    return document


def prepare_polyhaven(source, props, name, spec):
    document, parts = polyhaven_parts(source, name, spec.get('nodes'))
    parts = merge_by_material(parts)
    size = recentre(parts)
    points = np.vstack([p['POSITION'] for p in parts])
    if 'keep' in spec:
        parts = [simplify_part(p, spec['keep']) for p in parts]
    target = props / name
    copy_textures(source / 'models' / name, target, document)
    write_mesh(target, 'model', parts, patch_materials(document, spec.get('materials', {})), document['images'],
               GENERATOR)
    colliders = spec['colliders']
    if colliders == 'hull':
        text, _ = convex_proxy(points)
        (target / 'collision.obj').write_text(text)
        colliders = [('convex_mesh', 'collision.obj')]
    elif colliders == 'log':
        colliders = [('capsule', *log_capsule(points))]
    elif colliders == 'bounds':
        colliders = [bounds_box(parts)]
    if colliders:
        write_descriptor(target, 'model', colliders)
    report(name, parts, size)


def prepare_tyre_stack(source, props):
    """TYRE_STACK tyres lying on top of each other, each turned a little."""
    document, tyre = polyhaven_parts(source, TYRE)
    tyre = [simplify_part(p, TYRE_KEEP) for p in merge_by_material(tyre)]
    rng = np.random.default_rng(5)
    parts = []
    for level in range(TYRE_STACK):
        yaw = rng.uniform(0, 2 * math.pi)
        lay = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)  # tyre axis (y) to z
        turn = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]) @ lay
        for part in tyre:
            part = dict(part, POSITION=part['POSITION'] @ turn.T, NORMAL=part['NORMAL'] @ turn.T)
            low = part['POSITION'][:, 2].min()
            part['POSITION'] = part['POSITION'] + [*rng.normal(0, 0.012, 2), -low + level * 0.163]
            parts.append(part)
    parts = merge_by_material(parts)
    size = recentre(parts)
    target = props / 'tyre_stack'
    copy_textures(source / 'models' / TYRE, target, document)
    write_mesh(target, 'model', parts, document, document['images'], GENERATOR)
    write_descriptor(target, 'model', [('cylinder', dict(radius=round(size[0] / 2, 3), height=round(size[2], 3)),
                                        (0, 0, round(size[2] / 2, 3)))])
    report('tyre_stack', parts, size)


def prepare_fence(source, props):
    """A 2 m chain-link module: the double panel and the post at its +x end. Modules placed
    every FENCE_SPACING metres make a continuous fence."""
    document, panel = polyhaven_parts(source, 'modular_chainlink_fence', [FENCE_PANEL])
    _, post = polyhaven_parts(source, 'modular_chainlink_fence', [FENCE_POST])
    post_centre = np.vstack([p['POSITION'] for p in post]).mean(0)
    for part in post:
        part['POSITION'] = part['POSITION'] - [post_centre[0], post_centre[1], 0]
    parts = merge_by_material(panel + post)
    low = np.vstack([p['POSITION'] for p in parts])[:, 2].min()
    for part in parts:
        part['POSITION'] = part['POSITION'] + [FENCE_SPACING / 2, 0, -low]
    points = np.vstack([p['POSITION'] for p in parts])
    target = props / 'chainlink_fence'
    folder = source / 'models' / 'modular_chainlink_fence'
    copy_textures(folder, target, document)
    # Cut-out wire mesh instead of blending (no sorting, and it casts shadows), with the opacity map
    # as the alpha channel of its colour map.
    from PIL import Image
    images = copy.deepcopy(document['images'])
    material = next(m for m in document['materials'] if m['name'] == 'modular_chainlink_fence_wire')
    colour = images[material['pbrMetallicRoughness']['baseColorTexture']['index']]
    rgba = Image.open(folder / colour['uri']).convert('RGB')
    rgba.putalpha(Image.open(folder / 'textures' / 'modular_chainlink_fence_wire_opacity_1k.jpg').convert('L'))
    (target / colour['uri']).unlink()
    colour.update(uri='textures/modular_chainlink_fence_wire_diff_alpha_1k.png', mimeType='image/png')
    rgba.save(target / colour['uri'], optimize=True)
    wire = dict(alphaMode='MASK', alphaCutoff=0.5, doubleSided=True)
    write_mesh(target, 'model', parts, patch_materials(document, {'modular_chainlink_fence_wire': wire}), images,
               GENERATOR)
    write_descriptor(target, 'model', [('box', dict(size=(FENCE_SPACING, 0.06, 2.4)), (0, 0, 1.2))])
    report('chainlink_fence', parts, points.max(0) - points.min(0))


def prepare_sketchfab(source, props, name, spec):
    folder = source / 'sketchfab' / name
    document = load(folder / 'scene.gltf')
    binary = (folder / document['buffers'][0]['uri']).read_bytes()
    root = matrix(spec['scale'], spec.get('yaw', 0.0))
    target = props / name
    images = convert_images(document, folder, target, 1024)
    for stem, nodes in spec['variants'].items():
        parts = ([p for node in nodes for p in flatten_node(document, binary, node, root)] if nodes
                 else flatten_scene(document, binary, root))
        parts = merge_by_material(parts)
        size = recentre(parts)
        write_mesh(target, stem, parts, document, images, GENERATOR)
        if spec.get('colliders', {}).get(stem) == 'box':
            write_descriptor(target, stem, [bounds_box(parts)])
        report(f'{name}/{stem}', parts, size)
    shutil.copyfile(folder / 'license.txt', target / 'license.txt')


# ---------------------------------------------------------------------------
# Generated models.

def font(size):
    from PIL import ImageFont
    return ImageFont.load_default(size=size)


def text_centred(draw, centre, text, size, fill, stroke=0):
    face = font(size)
    left, top, right, bottom = draw.textbbox((0, 0), text, font=face, stroke_width=stroke)
    draw.text((centre[0] - (left + right) / 2, centre[1] - (top + bottom) / 2), text, font=face, fill=fill,
              stroke_width=stroke, stroke_fill=fill)


def checker(draw, x0, y0, width, height, cell):
    for i in range(int(width // cell)):
        for j in range(int(height // cell)):
            if (i + j) % 2 == 0:
                draw.rectangle([x0 + i * cell, y0 + j * cell, x0 + (i + 1) * cell - 1, y0 + (j + 1) * cell - 1],
                               fill=(240, 240, 240))


def banner_texture(path):
    """2048 x 256: checkered ends, the name in white and an orange rule."""
    from PIL import Image, ImageDraw
    image = Image.new('RGB', (2048, 256), (22, 24, 28))
    draw = ImageDraw.Draw(image)
    for x0 in (0, 2048 - 256):
        draw.rectangle([x0, 0, x0 + 255, 255], fill=(16, 16, 16))
        checker(draw, x0, 0, 256, 256, 32)
    draw.rectangle([256, 206, 2048 - 257, 222], fill=(242, 92, 5))
    text_centred(draw, (1024, 112), BANNER_TEXT, 170, (245, 245, 245), stroke=3)
    image.save(path, quality=92)


def flag_texture(path, colour):
    """256 x 1024: the name written upwards in white on a coloured feather flag."""
    from PIL import Image, ImageDraw
    image = Image.new('RGB', (1024, 256), colour)
    draw = ImageDraw.Draw(image)
    draw.rectangle([0, 0, 1023, 26], fill=(250, 250, 250))
    text_centred(draw, (560, 140), BANNER_TEXT, 120, (250, 250, 250), stroke=2)
    image.rotate(90, expand=True).save(path, quality=92)


def valance_texture(path, colour):
    from PIL import Image, ImageDraw
    rgb = tuple(int(255 * c ** (1 / 2.2)) for c in colour)
    image = Image.new('RGB', (1024, 96), rgb)
    draw = ImageDraw.Draw(image)
    ink = (30, 30, 34) if sum(colour) > 2 else (245, 245, 245)
    text_centred(draw, (512, 50), BANNER_TEXT, 64, ink, stroke=1)
    image.save(path, quality=92)


def textured(name, uri, roughness, metallic=0.0, double_sided=False):
    material = dict(name=name, pbrMetallicRoughness=dict(baseColorTexture=dict(index=0), metallicFactor=metallic,
                                                        roughnessFactor=roughness))
    if double_sided:
        material['doubleSided'] = True
    return material, dict(images=[dict(uri=uri)], textures=[dict(source=0, sampler=0)],
                          samplers=[dict(magFilter=9729, minFilter=9987, wrapS=33071, wrapT=33071)])


def two_sided(positions, normals, triangles, uvs):
    """The surface and its back, for thin fabric."""
    n = len(positions)
    return (np.vstack([positions, positions]), np.vstack([normals, -normals]),
            np.vstack([triangles, triangles[:, ::-1] + n]), np.vstack([uvs, uvs]))


def grid_surface(point, rows, cols):
    """A surface point(u, v) sampled on a grid: positions, normals, triangles, uvs."""
    u, v = np.meshgrid(np.linspace(0, 1, cols + 1), np.linspace(0, 1, rows + 1))
    positions = np.array([point(a, b) for a, b in zip(u.ravel(), v.ravel())])
    grid = positions.reshape(rows + 1, cols + 1, 3)
    du = np.gradient(grid, axis=1)
    dv = np.gradient(grid, axis=0)
    normals = np.cross(du, dv).reshape(-1, 3)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    a = (np.arange(rows)[:, None] * (cols + 1) + np.arange(cols)[None, :]).ravel()
    triangles = np.vstack([np.stack([a, a + 1, a + cols + 2], 1), np.stack([a, a + cols + 2, a + cols + 1], 1)])
    return positions, normals, triangles, np.stack([u.ravel(), 1 - v.ravel()], 1)


def truss(p0, p1, width, chord=0.024, lacing=0.011, bay=0.5):
    """A square box truss from p0 to p1: four chords, rungs and zig-zag diagonals."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    axis = p1 - p0
    length = np.linalg.norm(axis)
    w = axis / length
    helper = np.array([0, 0, 1.0]) if abs(w[2]) < 0.9 else np.array([1.0, 0, 0])
    u = np.cross(w, helper)
    u /= np.linalg.norm(u)
    v = np.cross(w, u)
    corners = [(su * u + sv * v) * width / 2 for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    parts = [tube(p0 + c, p1 + c, chord, 10) for c in corners]
    bays = max(1, int(round(length / bay)))
    for k in range(bays + 1):
        at = p0 + w * length * k / bays
        for i in range(4):
            parts.append(tube(at + corners[i], at + corners[(i + 1) % 4], lacing, 6, caps=False))
    for k in range(bays):
        a, b = p0 + w * length * k / bays, p0 + w * length * (k + 1) / bays
        for i in range(4):
            c0, c1 = corners[i], corners[(i + 1) % 4]
            parts.append(tube(a + (c0 if k % 2 else c1), b + (c1 if k % 2 else c0), lacing, 6, caps=False))
    return merge(parts)


def write_generated(target, stem, groups, extra=None):
    """groups: list of (material, (positions, normals, triangles, uvs or None)); extra: images,
    textures and samplers of the textured materials."""
    gltf = Gltf(stem)
    gltf.doc['asset']['generator'] = GENERATOR
    for material, (positions, normals, triangles, uvs) in groups:
        gltf.primitive(positions, normals, triangles, gltf.material(copy.deepcopy(material)), uvs)
    if extra:
        gltf.doc.update(copy.deepcopy(extra))
    gltf.write(target / f'{stem}.gltf')


def prepare_gantry(props):
    """Start/finish gantry across the track (the track runs along x): two truss towers on
    concrete ballast, a truss beam and the banner hanging from it."""
    target = props / 'start_gantry'
    (target / 'textures').mkdir(parents=True, exist_ok=True)
    banner_texture(target / 'textures' / 'banner.jpg')
    half, top = GANTRY_SPAN / 2, GANTRY_HEIGHT
    steel = pbr('truss_aluminium', (0.80, 0.80, 0.82), 1.0, 0.32)
    concrete = pbr('ballast_concrete', (0.42, 0.41, 0.39), 0.0, 0.85)
    frame = [truss([0, s * half, 0.5], [0, s * half, top], 0.4) for s in (-1, 1)]
    frame.append(truss([0, -half - 0.2, top - 0.2], [0, half + 0.2, top - 0.2], 0.4))
    ballast = [box([0, s * half, 0.25], (1.0, 1.0, 0.5)) for s in (-1, 1)]
    # Banner: front and back faces, the text readable from both sides, and its straps.
    width, height, bottom = 2 * half - 1.2, 1.15, top - 1.65
    frame += [tube([0, s * (width / 2 - 0.1), bottom + height], [0, s * (width / 2 - 0.1), top - 0.4], 0.008, 6)
              for s in (-1, 1)]
    faces = []
    for side in (1, -1):
        y = np.array([-width / 2, width / 2, width / 2, -width / 2]) * side
        z = np.array([bottom, bottom, bottom + height, bottom + height])
        positions = np.stack([np.full(4, 0.02 * side), y, z], 1)
        faces.append((positions, np.tile([side, 0, 0], (4, 1)), np.array([[0, 1, 2], [0, 2, 3]])))
    uvs = np.tile(np.array([[0, 1], [1, 1], [1, 0], [0, 0]], float), (2, 1))
    banner, extra = textured('banner', 'textures/banner.jpg', 0.7)
    write_generated(target, 'model', [(steel, (*merge(frame), None)), (concrete, (*merge(ballast), None)),
                                      (banner, (*merge(faces), uvs))], extra)
    colliders = [('box', dict(size=(1.0, 1.0, top)), (0, s * half, top / 2)) for s in (-1, 1)]
    write_descriptor(target, 'model', colliders)
    print(f'start_gantry: {GANTRY_SPAN:.1f} m span, {top:.1f} m high')


def prepare_flags(props):
    """Feather flags 4 m high, one model per colour."""
    target = props / 'feather_flag'
    (target / 'textures').mkdir(parents=True, exist_ok=True)
    pole = pbr('flag_pole', (0.12, 0.12, 0.13), 0.8, 0.4)
    base, top, width = 0.9, 4.0, 0.78

    def sail(u, v):
        edge = 3.0 + (top - 3.0) * math.sqrt(max(0.0, 1 - u * u))
        z = base + v * (edge - base)
        return [width * u, 0.05 * math.sin(math.pi * u) * math.sin(1.7 * math.pi * v), z]

    for name, colour in FLAG_COLOURS.items():
        flag_texture(target / 'textures' / f'{name}.jpg', colour)
        material, extra = textured(f'flag_{name}', f'textures/{name}.jpg', 0.75)
        cloth = two_sided(*grid_surface(sail, 24, 6))
        stick = tube([0, 0, -0.1], [0, 0, top + 0.05], 0.018, 10)
        write_generated(target, name, [(material, cloth), (pole, (*stick, None))], extra)
    print(f'feather_flag: {len(FLAG_COLOURS)} colours, {top:.1f} m')


def prepare_canopies(props):
    """3 m x 3 m pop-up canopies: aluminium legs and scissor frame, a pyramid roof and a valance
    with the name."""
    target = props / 'canopy'
    (target / 'textures').mkdir(parents=True, exist_ok=True)
    half, eave, peak, drop = 1.5, 2.1, 2.85, 0.24
    metal = pbr('canopy_frame', (0.75, 0.76, 0.78), 1.0, 0.35)
    legs = [tube([sx * half, sy * half, 0], [sx * half, sy * half, eave], 0.02, 8)
            for sx in (-1, 1) for sy in (-1, 1)]
    corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        legs.append(tube([x0, y0, eave - 0.3], [x1, y1, eave], 0.012, 6, caps=False))
        legs.append(tube([x0, y0, eave], [x1, y1, eave - 0.3], 0.012, 6, caps=False))
    for x, y in corners:
        legs.append(tube([x, y, eave], [x * 0.1, y * 0.1, peak - 0.06], 0.012, 6, caps=False))
    frame = (*merge(legs), None)
    # Roof: four triangles from the eaves to the apex.
    roof_parts = []
    apex = np.array([0, 0, peak])
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        a, b = np.array([x0 * 1.02, y0 * 1.02, eave]), np.array([x1 * 1.02, y1 * 1.02, eave])
        n = np.cross(b - a, apex - a)
        n /= np.linalg.norm(n)
        roof_parts.append((np.array([a, b, apex]), np.tile(n, (3, 1)), np.array([[0, 1, 2]])))
    roof_p, roof_n, roof_t = merge(roof_parts)
    roof = two_sided(roof_p, roof_n, roof_t, np.zeros((len(roof_p), 2)))
    # Valance: a band hanging from the eaves on each side, the text on the outside.
    band = []
    for k, ((x0, y0), (x1, y1)) in enumerate(zip(corners, corners[1:] + corners[:1])):
        p = np.array([[x0, y0, eave - drop], [x1, y1, eave - drop], [x1, y1, eave], [x0, y0, eave]]) * \
            [1.02, 1.02, 1]
        n = np.cross(p[1] - p[0], p[3] - p[0])
        n /= np.linalg.norm(n)
        band.append((p, np.tile(n, (4, 1)), np.array([[0, 1, 2], [0, 2, 3]]),
                     np.array([[0, 1], [1, 1], [1, 0], [0, 0]], float)))
    band_p, band_n, band_t = merge([b[:3] for b in band])
    valance = two_sided(band_p, band_n, band_t, np.vstack([b[3] for b in band]))
    for name, colour in CANOPY_COLOURS.items():
        valance_texture(target / 'textures' / f'{name}.jpg', colour)
        fabric = pbr(f'canopy_{name}', colour, 0.0, 0.8)
        fabric['doubleSided'] = True
        material, extra = textured(f'valance_{name}', f'textures/{name}.jpg', 0.8, double_sided=True)
        write_generated(target, name, [(metal, frame), (fabric, (*roof[:3], None)), (material, valance)], extra)
        write_descriptor(target, name, [('cylinder', dict(radius=0.03, height=eave),
                                         (sx * half, sy * half, eave / 2)) for sx in (-1, 1) for sy in (-1, 1)])
    print(f'canopy: {len(CANOPY_COLOURS)} colours')


def prepare_cone_proxy(props):
    """A collision hull of about 40 triangles for rsc/city's traffic cone: a dynamic body with the
    full mesh's hull costs RaiSim far more per contact with the height map."""
    cone = Path(__file__).resolve().parents[2] / 'rsc' / 'city' / 'traffic_cone'
    document = load(cone / 'model.gltf')
    binary = (cone / document['buffers'][0]['uri']).read_bytes()
    points = np.vstack([p['POSITION'] for p in flatten_scene(document, binary, np.eye(4))])
    text, triangles = convex_proxy(points, 40)
    (props / 'traffic_cone').mkdir(parents=True, exist_ok=True)
    (props / 'traffic_cone' / 'collision.obj').write_text(text)
    print(f'traffic_cone: collision proxy of {triangles} triangles')


def prepare_reused(props, name, folder, keep):
    """A copy of a prepared prop of another rsc folder (Z-up, base at z = 0) with fewer triangles,
    its textures and its .rasset."""
    folder = Path(__file__).resolve().parents[2] / 'rsc' / folder
    document = load(folder / 'model.gltf')
    binary = (folder / document['buffers'][0]['uri']).read_bytes()
    parts = [simplify_part(p, keep) for p in merge_by_material(flatten_scene(document, binary, np.eye(4)))]
    target = props / name
    copy_textures(folder, target, document)
    write_mesh(target, 'model', parts, document, document['images'], GENERATOR)
    if (folder / 'model.rasset').exists():
        shutil.copyfile(folder / 'model.rasset', target / 'model.rasset')
    points = np.vstack([p['POSITION'] for p in parts])
    report(name, parts, points.max(0) - points.min(0))


def marker_texture(path):
    """64 x 256: an orange post with two white reflective bands, and the black rubber of its base at
    the bottom."""
    from PIL import Image, ImageDraw
    image = Image.new('RGB', (64, 256), (236, 96, 12))
    draw = ImageDraw.Draw(image)
    for top in (18, 58):
        draw.rectangle([0, top, 63, top + 24], fill=(242, 242, 236))
    draw.rectangle([0, 232, 63, 255], fill=(18, 18, 18))
    image.save(path, quality=92)


def prepare_marker_post(props):
    """A flexible delineator 1 m high on a rubber base, all on one textured material (rayrai's
    instanced batches would tint an untextured part), and a convex collision proxy."""
    target = props / 'marker_post'
    (target / 'textures').mkdir(parents=True, exist_ok=True)
    marker_texture(target / 'textures' / 'marker.jpg')
    sides, radius, base, height = 16, 0.028, 0.13, 1.0

    def band(r0, r1, z0, z1, v0, v1):
        a = np.linspace(0, 2 * np.pi, sides + 1)
        ring = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], 1)
        positions = np.vstack([ring * r0 + [0, 0, z0], ring * r1 + [0, 0, z1]])
        slope = (r0 - r1) / max(z1 - z0, 1e-9)
        normals = ring + [0, 0, slope]
        normals = np.vstack([normals, normals]) / np.linalg.norm(normals, axis=1).max()
        uvs = np.vstack([np.stack([a / (2 * np.pi), np.full_like(a, v0)], 1), np.stack([a / (2 * np.pi), np.full_like(a, v1)], 1)])
        i = np.arange(sides)
        triangles = np.vstack([np.stack([i, i + 1, sides + 2 + i], 1), np.stack([i, sides + 2 + i, sides + 1 + i], 1)])
        return positions, normals, triangles, uvs

    parts = [band(base, base, 0.0, 0.035, 0.99, 0.95), band(base, 0.04, 0.035, 0.07, 0.95, 0.92),
             band(radius, radius, 0.07, height, 0.91, 0.0), band(radius, 0.0, height, height + 0.01, 0.0, 0.0)]
    offsets = np.cumsum([0] + [len(p[0]) for p in parts[:-1]])
    positions = np.vstack([p[0] for p in parts])
    normals = np.vstack([p[1] for p in parts])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    triangles = np.vstack([p[2] + o for p, o in zip(parts, offsets)])
    uvs = np.vstack([p[3] for p in parts])
    material, extra = textured('marker_post', 'textures/marker.jpg', 0.55)
    extra['samplers'][0].update(wrapS=10497)
    write_generated(target, 'model', [(material, (positions, normals, triangles, uvs))], extra)
    text, triangles = convex_proxy(positions, 32)
    (target / 'collision.obj').write_text(text)
    print(f'marker_post: {height:.1f} m, collision proxy of {triangles} triangles')


def prepare_venue(source, output):
    """Write output/props from the downloads in source (DIR/assets)."""
    props = output / 'props'
    if props.exists():
        shutil.rmtree(props)
    props.mkdir(parents=True)
    for name, spec in POLYHAVEN.items():
        prepare_polyhaven(source, props, name, spec)
    prepare_tyre_stack(source, props)
    prepare_fence(source, props)
    for name, spec in SKETCHFAB.items():
        prepare_sketchfab(source, props, name, spec)
    prepare_gantry(props)
    prepare_flags(props)
    prepare_canopies(props)
    for name, (folder, keep) in REUSED.items():
        prepare_reused(props, name, folder, keep)
    prepare_cone_proxy(props)
    prepare_marker_post(props)
    sizes = {p.name: sum(f.stat().st_size for f in p.rglob('*') if f.is_file()) for p in props.iterdir()}
    print('props: %.1f MB' % (sum(sizes.values()) / 1e6))
    return {name: round(size / 1e6, 2) for name, size in sorted(sizes.items())}
