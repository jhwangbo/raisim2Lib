"""Check shipped resources, glTF links, and the actual lowest vertex of each rooted plant."""
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

root = Path(sys.argv[1])
manifest = json.loads((root / 'manifest.json').read_text())
assert len(manifest['plants']) == 10
for record in manifest['files']:
    path = root / record['path']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == record['sha256'], path
assert len(manifest['rocks']) == 6
for name in manifest['plants'] + manifest['rocks']:
    folder = root / name
    scene = json.loads((folder / 'model.gltf').read_text())
    assert len(scene['nodes']) == 1 and len(scene['meshes']) == 1
    node = scene['nodes'][0]
    assert all(abs(a-b) < 1e-7 for a,b in zip(node['rotation'],[math.sqrt(.5),0,0,math.sqrt(.5)]))
    minimum = float('inf')
    for image in scene['images']:
        assert (folder / image['uri']).is_file()
    for primitive in scene['meshes'][0]['primitives']:
        accessor = scene['accessors'][primitive['attributes']['POSITION']]
        assert accessor['componentType'] == 5126 and accessor['type'] == 'VEC3'
        view = scene['bufferViews'][accessor['bufferView']]
        buffer = (folder / scene['buffers'][view['buffer']]['uri']).read_bytes()
        offset = view.get('byteOffset',0)+accessor.get('byteOffset',0)
        stride = view.get('byteStride',12)
        assert offset+(accessor['count']-1)*stride+12 <= len(buffer)
        for i in range(accessor['count']):
            x,y,z = struct.unpack_from('<fff',buffer,offset+i*stride)
            assert math.isfinite(x+y+z)
            minimum = min(minimum,y*node.get('scale',[1,1,1])[1]+node['translation'][2])
            if name in manifest['rocks']:
                px=x*node['scale'][0]+node['translation'][0]
                py=-z*node['scale'][2]+node['translation'][1]
                assert math.hypot(px,py)<=1.00001, (name,px,py)
    assert abs(minimum) < 1e-5, (name,minimum)
print('Verified 10 rooted plants and 6 normalized rocks, texture links, and asset checksums')
