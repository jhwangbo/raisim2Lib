#!/usr/bin/env python3
"""Download the source assets of the rayrai off-road buggy example into DIR/assets.

The buggy, its coilovers, and the hay bales, water barriers, portable toilet
and bleacher of the race venue are Sketchfab models under CC-BY-4.0, credited
in rsc/offroad_buggy/ATTRIBUTION.md. Sketchfab downloads need the API token of
a free account (https://sketchfab.com/settings/password), read from the
SKETCHFAB_API_TOKEN environment variable or ~/.sketchfab_api_token. The ground
textures, boulders, stones, a fallen trunk, tyres, fence and pit props come
from Poly Haven (CC0). The trees, mossy rocks and grass are reused from
rsc/forest, the sky and traffic cones from rsc/city and the barrels, tool cart
and crates from rsc/warehouse. Only Python's standard library is used.

Usage: download_offroad_buggy_assets.py DIR
"""
import hashlib
import io
import json
import os
import pathlib
import sys
import urllib.request
import zipfile

USER_AGENT = {'User-Agent': 'RaisimOffroadBuggyExample/1.0'}
# Poly Haven texture id -> resolution; colour and OpenGL normal maps.
TEXTURES = {'rocky_terrain_02': '2k', 'rocky_trail': '2k', 'dry_ground_rocks': '2k', 'brown_mud_dry': '2k',
            'rock_face_03': '2k'}
TEXTURE_MAPS = ['Diffuse', 'nor_gl']
# Poly Haven models (glTF, 1K textures).
MODELS = ['boulder_01', 'namaqualand_boulder_02', 'rock_07', 'rock_09', 'dead_tree_trunk_02', 'old_tyre',
          'modular_chainlink_fence', 'portable_generator', 'wooden_picnic_table', 'metal_jerrycan',
          'plastic_monobloc_chair_01']
# Extra maps of models (1K JPEG): the fence's wire is cut out by its opacity map, which the glTF
# download leaves out.
MODEL_MAPS = {'modular_chainlink_fence': ['wire_opacity']}
# Sketchfab models, CC-BY-4.0: local name -> model uid.
SKETCHFAB = {
    'sci_fi_buggy': '085bd97876a64eacbc041299ac945e9c',  # Sci - Fi Buggy, TiyaMakes
    'coilover': '92e61757780840a683abdd38c4e8a700',      # [Free] Coilover Shock Absorber Damper, Robert Prispilovic
    'hay_bales': '6b775049e57e4d9da87977905493ebfb',     # Hay Bales, FrodoUndead
    'water_barrier': 'b39c313f0b2148dca48bf113529fb06e', # Water Barriers, xplanepilot
    'porta_potty': 'b970702ea74a456e89673e73cfb6d873',   # Porta Potty, foon.
    'bleacher': '4960023d1ea340bfb07625aaa7f9713b',      # Bleacher, JanStano
}


def read(url, headers=None):
    request = urllib.request.Request(url, headers={**USER_AGENT, **(headers or {})})
    return urllib.request.urlopen(request, timeout=300).read()


def sketchfab_token():
    token = os.environ.get('SKETCHFAB_API_TOKEN', '').strip()
    path = pathlib.Path.home() / '.sketchfab_api_token'
    if not token and path.exists():
        token = path.read_text().strip()
    if not token:
        raise SystemExit('The buggy and the coilover come from Sketchfab, which needs an API token: set '
                         'SKETCHFAB_API_TOKEN or write the token to ~/.sketchfab_api_token '
                         '(https://sketchfab.com/settings/password).')
    return token


def fetch(root, manifest, relative, url, md5=None):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_bytes(read(url))
    data = path.read_bytes()
    if md5 is not None and hashlib.md5(data).hexdigest() != md5:
        raise RuntimeError('Checksum mismatch: ' + str(path))
    manifest.append(dict(path=relative, url=url, sha256=hashlib.sha256(data).hexdigest(), bytes=len(data)))


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: download_offroad_buggy_assets.py DIR')
    root = pathlib.Path(sys.argv[1]) / 'assets'
    root.mkdir(parents=True, exist_ok=True)
    manifest = []
    print('Powered by Poly Haven: https://polyhaven.com', flush=True)
    for name, resolution in TEXTURES.items():
        files = json.loads(read('https://api.polyhaven.com/files/' + name))
        for key in TEXTURE_MAPS:
            entry = files[key][resolution]['jpg']
            fetch(root, manifest, f'textures/{name}/' + entry['url'].rsplit('/', 1)[1], entry['url'], entry['md5'])
        print(name, flush=True)
    for name in MODELS:
        files = json.loads(read('https://api.polyhaven.com/files/' + name))
        gltf = files['gltf']['1k']['gltf']
        entries = {f'{name}_1k.gltf': gltf, **gltf['include']}
        for relative, entry in entries.items():
            fetch(root, manifest, f'models/{name}/{relative}', entry['url'], entry['md5'])
        for key in MODEL_MAPS.get(name, []):
            entry = files[key]['1k']['jpg']
            fetch(root, manifest, f'models/{name}/textures/' + entry['url'].rsplit('/', 1)[1], entry['url'], entry['md5'])
        print(name, '%.1f MiB' % (sum(f['size'] for f in entries.values()) / 2**20), flush=True)
    # Sketchfab hands out short-lived links, so the manifest records the model page.
    token = None
    for name, uid in SKETCHFAB.items():
        target = root / 'sketchfab' / name
        if not (target / 'scene.gltf').exists():
            token = token or sketchfab_token()
            links = json.loads(read(f'https://api.sketchfab.com/v3/models/{uid}/download',
                                    {'Authorization': 'Token ' + token}))
            zipfile.ZipFile(io.BytesIO(read(links['gltf']['url']))).extractall(target)
        page = f'https://sketchfab.com/3d-models/{uid}'
        for path in sorted(target.rglob('*')):
            if path.is_file():
                data = path.read_bytes()
                manifest.append(dict(path=str(path.relative_to(root)), url=page,
                                     sha256=hashlib.sha256(data).hexdigest(), bytes=len(data)))
        print('Sketchfab:', name, flush=True)
    (root / 'sources.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
