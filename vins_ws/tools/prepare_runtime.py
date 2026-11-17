#!/usr/bin/env python3
"""Create a per-run VINS config with absolute paths resolved on this computer."""
import argparse
import re
import shutil
from pathlib import Path


def prepare(ws, run):
    ws = Path(ws).resolve()
    run = Path(run).resolve()
    run.mkdir(parents=True, exist_ok=True)
    (run / 'pose_graph').mkdir(exist_ok=True)
    source = ws / 'config/d435i'
    for name in ('left.yaml', 'right.yaml'):
        shutil.copy2(source / name, run / name)
    text = (source / 'depth_imu.yaml').read_text()
    for key, value in [('output_path', str(run) + '/'),
                       ('pose_graph_save_path', str(run / 'pose_graph') + '/')]:
        text, count = re.subn(r'^' + key + r':.*$', key + ': "' + value + '"', text, flags=re.M)
        if count != 1:
            raise ValueError('Expected exactly one ' + key)
    config = run / 'depth_imu.yaml'
    config.write_text(text)
    return config


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    args = parser.parse_args()
    print(prepare(Path(__file__).resolve().parents[1], args.run_dir))
