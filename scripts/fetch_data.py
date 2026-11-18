#!/usr/bin/env python3
"""Download, verify and unpack the shared experiment data without GitHub credentials."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, help='Verify and unpack an already downloaded archive')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    release = json.loads((root / 'data/release.json').read_text())
    dest = root / 'data/downloads'
    dest.mkdir(parents=True, exist_ok=True)
    archive = args.archive or dest / release['archive']

    def valid(path):
        if not path.is_file() or path.stat().st_size != release['bytes']:
            return False
        with path.open('rb') as stream:
            digest = hashlib.sha256()
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        return digest.hexdigest() == release['sha256']

    if not valid(archive) and not args.archive:
        temporary = archive.with_suffix(archive.suffix + '.part')
        print('Downloading:', release['url'], flush=True)
        request = urllib.request.Request(release['url'], headers={'User-Agent': 'd435i-team-data'})
        with urllib.request.urlopen(request, timeout=60) as response, temporary.open('wb') as output:
            shutil.copyfileobj(response, output)
        if not valid(temporary):
            raise SystemExit('下载文件校验失败；保留 .part 文件供检查，请重新运行。')
        temporary.replace(archive)
    if not valid(archive):
        raise SystemExit('SHA256/大小不匹配，请重新下载数据附件。')
    expected_root = dest / 'd435i-data-20261118'
    if expected_root.exists():
        manifest_file = expected_root / 'MANIFEST.json'
        if manifest_file.is_file():
            manifest = json.loads(manifest_file.read_text())
            for item in manifest['files']:
                relative = Path(item['path'])
                if relative.is_absolute() or '..' in relative.parts:
                    raise SystemExit('已有解压目录的清单包含异常路径。')
                existing = expected_root / relative
                if not existing.is_file() or existing.is_symlink() or hashlib.sha256(existing.read_bytes()).hexdigest() != item['sha256']:
                    raise SystemExit(f'已有文件发生变化，请先备份目录再重新下载：{existing}')
            print('Archive and existing extracted files verified:', expected_root)
            return
        raise SystemExit(f'解压目录已存在但没有完整清单，请先检查或备份：{expected_root}')
    with tarfile.open(archive, 'r:gz') as bundle:
        members = bundle.getmembers()
        for entry in members:
            path = Path(entry.name)
            if path.is_absolute() or '..' in path.parts or not path.parts or path.parts[0] != expected_root.name:
                raise SystemExit('拒绝解压包含异常路径的数据包。')
            if not (entry.isfile() or entry.isdir()):
                raise SystemExit('数据包只能包含普通文件和目录。')
            target = dest / path
            if target.exists() or target.is_symlink():
                raise SystemExit('解压路径已存在：' + str(target))
        bundle.extractall(dest, members=members)
    print('SHA256 verified. Extracted:', expected_root)


if __name__ == '__main__':
    main()
