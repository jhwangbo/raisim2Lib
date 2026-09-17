import hashlib, json, pathlib, urllib.request, sys
root=pathlib.Path(sys.argv[1])/'assets'
root.mkdir(parents=True,exist_ok=True)
manifest=[]
print("Powered by Poly Haven: https://polyhaven.com", flush=True)
def read(url):
 return urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'RaisimForestExample/1.0'}),timeout=120).read()
for name in ['pine_sapling_small','fir_sapling','tree_small_02','grass_bermuda_01','grass_medium_01','grass_medium_02','fern_02','dandelion_01','nettle_plant','periwinkle_plant','rock_moss_set_01']:
 info=json.loads(read('https://api.polyhaven.com/files/'+name))['gltf']['1k']['gltf']
 files={name+'_1k.gltf':info,**info['include']}
 print(name,sum(f['size'] for f in files.values())/1048576,flush=True)
 for rel,f in files.items():
  p=root/name/rel;p.parent.mkdir(parents=True,exist_ok=True)
  if not p.exists():p.write_bytes(read(f['url']))
  assert hashlib.md5(p.read_bytes()).hexdigest()==f['md5']
  manifest.append(dict(path=str(p.relative_to(root)),url=f['url'],sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size))
ground=json.loads(read('https://api.polyhaven.com/files/mud_forest'))
for key in ['Diffuse','nor_gl']:
 f=ground[key]['1k']['jpg'];p=root/'ground'/f['url'].rsplit('/',1)[1]
 p.parent.mkdir(exist_ok=True)
 if not p.exists():p.write_bytes(read(f['url']))
 assert hashlib.md5(p.read_bytes()).hexdigest()==f['md5']
 manifest.append(dict(path=str(p.relative_to(root)),url=f['url'],sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size))
(root/'sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
