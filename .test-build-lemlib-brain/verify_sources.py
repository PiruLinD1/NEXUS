"""Read-only verification of the pinned upstream sources and built app artifacts."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parent
upstream = json.loads((root / 'UPSTREAM-MANIFEST.json').read_text())
for name, expected in upstream['files'].items():
    actual = hashlib.sha256((root / name).read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit('Upstream mismatch: ' + name)
build = json.loads((root / 'BUILD-MANIFEST.json').read_text())
for name, expected in build['sha256'].items():
    actual = hashlib.sha256((root / name).read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit('Build artifact mismatch: ' + name)
project = json.loads((root / 'project.pros').read_text())['py/state']
assert project['project_name'] == 'LEMLIBTEST'
assert project['upload_options']['slot'] == 2
print('PASS:', len(upstream['files']), 'upstream files and', len(build['sha256']), 'build/source artifacts verified.')
