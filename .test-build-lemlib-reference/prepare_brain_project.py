import hashlib
import json
import shutil
import zipfile
from pathlib import Path

root = Path.cwd()
dest = root / '.test-build-lemlib-brain'
commit = '3388145e0d90ee3c7c17d7267a30ef64367df4cb'
prefix = 'LemLib-' + commit + '/'
if dest.exists():
    raise SystemExit('Destination already exists; refusing to overwrite the isolated project')
dest.mkdir()
for folder in ['include/pros', 'include/liblvgl', 'firmware']:
    shutil.copytree(root / folder, dest / folder)
for file in ['include/api.h', 'include/main.h', 'common.mk']:
    shutil.copy2(root / file, dest / file)
project = json.loads((root / 'project.pros').read_text())
project['py/state']['project_name'] = 'LEMLIBTEST'
project['py/state']['upload_options'] = {'slot': 2}
(dest / 'project.pros').write_text(json.dumps(project, indent=2) + '\n')
manifest = {'upstream': 'LemLib v0.5.6', 'commit': commit,
            'source_url': 'https://github.com/LemLib/LemLib/tree/' + commit,
            'sdk': 'PROS kernel 4.2.1 and liblvgl 9.2.0 copied from the current project',
            'files': {}}
with zipfile.ZipFile(root / '.test-build-lemlib-reference/pinned-source.zip') as archive:
    for entry in archive.namelist():
        if not entry.startswith(prefix) or entry.endswith('/'):
            continue
        rel = entry[len(prefix):]
        if not (rel.startswith(('src/lemlib/', 'include/lemlib/', 'include/fmt/')) or rel == 'LICENSE'):
            continue
        data = archive.read(entry)
        path = dest / ('LICENSE-LemLib' if rel == 'LICENSE' else rel)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        manifest['files'][str(path.relative_to(dest)).replace('\\', '/')] = hashlib.sha256(data).hexdigest()
odom = dest / 'src/lemlib/chassis/odom.cpp'
assert hashlib.sha256(odom.read_bytes()).hexdigest() == '26a84c6a3448c0443a398fef437ae6597d44cb3d0d5485748c3f4604005df5f6'
(dest / 'UPSTREAM-MANIFEST.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Created isolated project; upstream files:', len(manifest['files']))
