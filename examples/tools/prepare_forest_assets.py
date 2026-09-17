"""Prepare compact, root-centered, Z-up runtime meshes; no runtime importer tools needed."""
import copy, hashlib, json, math, pathlib, shutil, sys
ROOT=pathlib.Path(sys.argv[1])
OUT=ROOT/'prepared'
OUT.mkdir(exist_ok=True)
selections = [('pine_sapling_small',1),('fir_sapling',0),('tree_small_02',0),('grass_bermuda_01',9),('grass_medium_01',9),('grass_medium_02',4),('fern_02',0),('dandelion_01',0),('nettle_plant',0),('periwinkle_plant',0)]
for name,selected in selections:
 src=ROOT/'assets'/name; dst=OUT/name;dst.mkdir(exist_ok=True)
 d=json.loads((src/(name+'_1k.gltf')).read_text()); node=d['nodes'][selected];mesh=d['meshes'][node['mesh']]
 # Each selected mesh is a single rooted plant, not the supplier's display lineup.
 amin=min(d['accessors'][p['attributes']['POSITION']]['min'][1] for p in mesh['primitives'])
 amax=max(d['accessors'][p['attributes']['POSITION']]['max'][1] for p in mesh['primitives'])
 views={};accessors={};data=bytearray();av=[];bv=[]
 def accessor(i):
  if i in accessors:return accessors[i]
  a=copy.deepcopy(d['accessors'][i]);old=a['bufferView']
  if old not in views:
   v=copy.deepcopy(d['bufferViews'][old]);b=(src/d['buffers'][v['buffer']]['uri']).read_bytes()
   while len(data)%4:data.append(0)
   start=v.get('byteOffset',0);data.extend(b[start:start+v['byteLength']]);v['byteOffset']=len(data)-v['byteLength'];v['buffer']=0
   views[old]=len(bv);bv.append(v)
  a['bufferView']=views[old];accessors[i]=len(av);av.append(a);return accessors[i]
 for p in mesh['primitives']:
  p['indices']=accessor(p['indices']);p['attributes']={k:accessor(v) for k,v in p['attributes'].items()}
 for m in d['materials']:
  m['doubleSided']=True
  if any(s in m.get('name','') for s in ['twig','leaves','grass']):
   m.setdefault('extensions',{})['KHR_materials_specular']={'specularColorFactor':[1,1,1]}
   m['extras']={'rayrai':{'foliage':{'type':'leaf','twoSidedLighting':True,'transmission':0.4,'transmissionColor':[.65,.85,.28]}}}
   if 'twig' in m.get('name',''):m['alphaMode']='OPAQUE'
 d.update(meshes=[mesh],nodes=[{'mesh':0,'rotation':[math.sqrt(.5),0,0,math.sqrt(.5)],'translation':[0,0,-amin]}],scenes=[{'nodes':[0]}],scene=0,accessors=av,bufferViews=bv,buffers=[{'uri':'mesh.bin','byteLength':len(data)}])
 (dst/'mesh.bin').write_bytes(data);(dst/'model.gltf').write_text(json.dumps(d,separators=(',',':')))
 shutil.copytree(src/'textures',dst/'textures',dirs_exist_ok=True)
 print(name,'height',amax-amin,'MiB',len(data)/1048576)
shutil.copy2(ROOT/'assets/sources.json',OUT/'sources.json')
shutil.copytree(ROOT/'assets/ground',OUT/'ground',dirs_exist_ok=True)
# Keep licensing alongside regenerated resources.
for notice in ['LICENSE-CC0-1.0.txt','ATTRIBUTION.md']:
 shutil.copyfile(pathlib.Path(__file__).resolve().parents[1]/'rsc/forest'/notice,OUT/notice)
from prepare_forest_rocks import prepare_rocks
rocks = prepare_rocks(ROOT/'assets/rock_moss_set_01', OUT)
files=[dict(path=str(p.relative_to(OUT)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
       for p in sorted(OUT.rglob('*')) if p.is_file() and p.name!='manifest.json']
(OUT/'manifest.json').write_text(json.dumps(dict(license='CC0-1.0',license_url='https://polyhaven.com/license',
    plants=[name for name,_ in selections],rocks=rocks,files=files),indent=2)+'\n')
