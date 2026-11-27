#!/usr/bin/env python3
"""Capture the actual dirty source baseline without commits, reset or calibration edits.

Snapshot contains owned source/config/docs text, diff, git/submodule IDs and dependency
versions. Official submodules, binary assets and native solver build products are NOT
copied; restore them from the pinned repository/environment before overlaying source.
"""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import time

ROOT = Path(__file__).resolve().parents[1]
SUFFIXES = {'.py','.yaml','.yml','.xml','.xacro','.mjcf','.world','.sdf','.cpp','.hpp',
            '.h','.cc','.txt','.md','.cfg','.ini'}
EXCLUDED = {'pinky_pro','BehaviorTree.ROS2'}


def command(args):
    result = subprocess.run(args,cwd=ROOT,text=True,capture_output=True)
    return dict(returncode=result.returncode,stdout=result.stdout.strip(),stderr=result.stderr.strip())


def digest(data):
    return hashlib.sha256(data).hexdigest()


def collect():
    files = set()
    for package in (ROOT/'src').iterdir():
        if not package.is_dir() or package.name in EXCLUDED:
            continue
        for path in package.rglob('*'):
            if '__pycache__' in path.parts or path.is_symlink() or not path.is_file():
                continue
            if path.suffix in SUFFIXES or path.name in ('CMakeLists.txt','COLCON_IGNORE'):
                files.add(path)
    for base in ('scripts','docs'):
        for path in (ROOT/base).rglob('*'):
            if '__pycache__' in path.parts or 'validation' in path.parts or not path.is_file():
                continue
            if path.suffix in SUFFIXES:
                files.add(path)
    for name in ('README.md','.gitmodules','identification_mujoco.yaml','identification_mujoco_zero_delay.yaml'):
        if (ROOT/name).is_file():files.add(ROOT/name)
    return sorted(files)


def verify(directory):
    manifest=json.loads((directory/'manifest.json').read_text())
    archive=directory/'source.tar.gz'
    checks={'archive_sha256':digest(archive.read_bytes())==manifest['archive_sha256']}
    with tarfile.open(archive,'r:gz') as tf:
        for member in tf.getmembers():
            data=tf.extractfile(member).read()
            checks[member.name]=digest(data)==manifest['files'][member.name]['sha256']
    return checks


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--check',type=Path)
    parser.add_argument('--request',type=Path)
    args=parser.parse_args()
    if args.check:
        checks=verify(args.check)
        print(json.dumps(dict(checks=len(checks),passed=all(checks.values())),indent=2))
        return 0 if all(checks.values()) else 1
    if not args.output:parser.error('--output or --check is required')
    output=args.output
    output.mkdir(parents=True,exist_ok=False)
    manifest=dict(created_unix=time.time(),repository=str(ROOT),files={})
    archive=output/'source.tar.gz'
    with archive.open('wb') as raw,gzip.GzipFile(fileobj=raw,mode='wb',mtime=0) as compressed:
        with tarfile.open(fileobj=compressed,mode='w') as tf:
            for path in collect():
                data=path.read_bytes();name=str(path.relative_to(ROOT))
                info=tarfile.TarInfo(name)
                info.size=len(data);info.mode=path.stat().st_mode & 0o777;info.mtime=0
                tf.addfile(info,io.BytesIO(data))
                manifest['files'][name]=dict(sha256=digest(data),bytes=len(data),mode=info.mode)
    manifest['archive_sha256']=digest(archive.read_bytes())
    manifest['git']={k:command(v) for k,v in {
        'head':['git','rev-parse','HEAD'],'branch':['git','branch','--show-current'],
        'status':['git','status','--short'],'submodules':['git','submodule','status','--recursive']}.items()}
    for name,args2 in [('working_tree.patch',['git','diff','--binary']),
                       ('index.patch',['git','diff','--cached','--binary'])]:
        (output/name).write_text(command(args2)['stdout']+'\n')
    manifest['versions']=command([str(ROOT/'.venv/bin/python'),'-c',
        "import sys,importlib.metadata as m,json;print(json.dumps({'python':sys.version,**{p:m.version(p) for p in ['numpy','scipy','casadi','mujoco','psutil']}}))"])
    # Small explicit runtime inputs only; never guess or rewrite a 'latest' profile.
    for rel in ('src/pinky_soccer_dynamics/config/dynamics.yaml',
        'src/pinky_soccer_sim/config/calibration/mujoco_profile.yaml',
        'src/pinky_soccer_sim/config/calibration/gazebo_profile.yaml'):
        if (ROOT/rel).exists():manifest.setdefault('parameter_sources',[]).append(rel)
    if args.request:
        request=args.request.read_bytes()
        (output/'request.md').write_bytes(request)
        manifest['request_sha256']=digest(request)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (output/'RESTORE.md').write_text('''# Legacy source baseline\n\nUse a separate clean checkout of the recorded HEAD and pinned submodules.\nOverlay source.tar.gz there; never extract over the active workspace.\nThe archive includes uncommitted source/config changes. Verify the archive with\n`python scripts/capture_control_baseline.py --check PATH`.\nBinary assets, calibration sessions and native build products are not bundled.\nRebuild in the recorded environment and verify the selected runtime profile hashes.\nGit patches are supplementary; the archive is the dirty source reference.\n''')
    checks=verify(output)
    print(json.dumps(dict(files=len(manifest['files']),archive_bytes=archive.stat().st_size,
                          verification=all(checks.values())),indent=2))


if __name__=='__main__':
    raise SystemExit(main())
