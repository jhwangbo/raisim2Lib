"""Split Poly Haven's rock lineup into six centered, Z-up, radius-one meshes."""
import copy
import json
import math
import shutil
from pathlib import Path


def prepare_rocks(source, output):
    source, output = Path(source), Path(output)
    original = json.loads((source/'rock_moss_set_01_1k.gltf').read_text())
    shared = output/'rock_moss_set_01'
    shared.mkdir(exist_ok=True)
    for buffer in original['buffers']:
        shutil.copyfile(source/buffer['uri'], shared/buffer['uri'])
    shutil.copytree(source/'textures', shared/'textures', dirs_exist_ok=True)
    names = []
    for index, node in enumerate(original['nodes']):
        scene = copy.deepcopy(original)
        mesh = scene['meshes'][node['mesh']]
        bounds = [scene['accessors'][p['attributes']['POSITION']] for p in mesh['primitives']]
        low = [min(a['min'][i] for a in bounds) for i in range(3)]
        high = [max(a['max'][i] for a in bounds) for i in range(3)]
        cx, cz = (low[0]+high[0])/2, (low[2]+high[2])/2
        scale = 1/math.hypot((high[0]-low[0])/2, (high[2]-low[2])/2)
        scene.update(meshes=[mesh], nodes=[dict(mesh=0,
            rotation=[math.sqrt(.5),0,0,math.sqrt(.5)], scale=[scale]*3,
            translation=[-cx*scale,cz*scale,-low[1]*scale])], scenes=[{'nodes':[0]}], scene=0)
        for item in scene['buffers'] + scene['images']:
            item['uri'] = '../rock_moss_set_01/' + item['uri']
        name = 'rock_moss_' + str(index)
        target = output/name
        target.mkdir(exist_ok=True)
        (target/'model.gltf').write_text(json.dumps(scene,separators=(',',':')))
        names.append(name)
    return names
