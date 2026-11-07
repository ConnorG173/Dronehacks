"""Read the planner's course config (planner/README.md "Inputs") and DHMAP map files.

Same flat format the planner parses: ``Key: value`` lines, ``#`` comments, numbers
separated by spaces or commas, optional ``[ ]``. A key may repeat (``Checkpoint``).
Only the keys path following needs are interpreted; the rest are kept as text so a
derived config (e.g. a different ``MaxVelMag``) can be written back for the planner.
"""
import os
import re
import shutil
import struct
from dataclasses import dataclass, field

import numpy as np

_NUM_SPLIT = re.compile(r'[\s,\[\]]+')


def parse_numbers(text):
    return [float(t) for t in _NUM_SPLIT.split(text.strip()) if t]


def read_config_lines(path):
    """[(key, raw_value)] in file order, comments stripped."""
    out = []
    with open(path, 'r', encoding='utf-8') as handle:
        for line in handle:
            body = line.split('#', 1)[0].strip()
            if not body:
                continue
            if ':' not in body:
                raise ValueError(f'{path}: not a "Key: value" line: {line.rstrip()}')
            key, value = body.split(':', 1)
            out.append((key.strip(), value.strip()))
    return out


@dataclass
class Box:
    center: np.ndarray
    half: np.ndarray

    def contains(self, p):
        return bool(np.all(np.abs(np.asarray(p) - self.center) <= self.half))


@dataclass
class Course:
    path: str
    lines: list
    map_file: str
    map_bound: tuple
    voxel_width: float
    dilate_radius: float
    start_pos: np.ndarray
    start_heading_rad: float
    hover_height: float
    checkpoints: list = field(default_factory=list)
    goal: np.ndarray = None
    out_dir: str = None

    def value(self, key):
        for k, v in self.lines:
            if k == key:
                return v
        raise KeyError(key)

    @property
    def hover_point(self):
        return self.start_pos + np.array([0.0, 0.0, self.hover_height])

    def write_derived(self, path, overrides):
        """Write a copy of this config with some keys replaced. Relative paths are
        rewritten relative to the copy's directory (the planner splits values on
        whitespace, so absolute paths with spaces would break)."""
        base = os.path.dirname(os.path.abspath(self.path))
        new_base = os.path.dirname(os.path.abspath(path))
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(f'# derived from {os.path.abspath(self.path)} by team_path_following\n')
            for key, value in self.lines:
                if key in overrides:
                    value = str(overrides[key])
                elif key in ('MapFile', 'OutDir'):
                    full = value if os.path.isabs(value) else os.path.join(base, value)
                    value = os.path.relpath(full, new_base)
                    if key == 'MapFile' and any(c.isspace() for c in value):
                        # the planner cannot read a path with spaces: copy the map next to the config
                        local = os.path.join(new_base, os.path.basename(full).replace(' ', '_'))
                        shutil.copyfile(full, local)
                        value = os.path.basename(local)
                handle.write(f'{key}: {value}\n')


def load_course(path):
    lines = read_config_lines(path)
    base = os.path.dirname(os.path.abspath(path))

    def one(key):
        vals = [v for k, v in lines if k == key]
        if len(vals) != 1:
            raise ValueError(f'{path}: expected exactly one {key}, got {len(vals)}')
        return vals[0]

    def nums(key, n):
        v = parse_numbers(one(key))
        if len(v) != n:
            raise ValueError(f'{path}: {key} needs {n} numbers, got {len(v)}')
        return v

    checkpoints = []
    for k, v in lines:
        if k == 'Checkpoint':
            c = parse_numbers(v)
            if len(c) != 6:
                raise ValueError(f'{path}: Checkpoint needs 6 numbers')
            checkpoints.append(Box(np.array(c[:3]), np.array(c[3:])))

    map_file = one('MapFile')
    if not os.path.isabs(map_file):
        map_file = os.path.normpath(os.path.join(base, map_file))
    out_dir = one('OutDir')
    if not os.path.isabs(out_dir):
        out_dir = os.path.normpath(os.path.join(base, out_dir))

    return Course(
        path=os.path.abspath(path),
        lines=lines,
        map_file=map_file,
        map_bound=tuple(nums('MapBound', 6)),
        voxel_width=nums('VoxelWidth', 1)[0],
        dilate_radius=nums('DilateRadius', 1)[0],
        start_pos=np.array(nums('StartPos', 3)),
        start_heading_rad=np.deg2rad(nums('StartHeadingDeg', 1)[0]),
        hover_height=nums('HoverHeight', 1)[0],
        checkpoints=checkpoints,
        goal=np.array(nums('Goal', 3)),
        out_dir=out_dir,
    )


def read_dhmap(path):
    """DHMAP v1 -> (N, 3) float32 array of occupied surface points, map frame."""
    with open(path, 'rb') as handle:
        raw = handle.read()
    if raw[:8] != b'DHMAP\0\0\0':
        raise ValueError(f'{path}: not a DHMAP file')
    version, _, n = struct.unpack_from('<IIQ', raw, 8)
    if version != 1:
        raise ValueError(f'{path}: DHMAP version {version} not supported')
    pts = np.frombuffer(raw, '<f4', count=3 * n, offset=24).reshape(-1, 3)
    return pts[np.all(np.isfinite(pts), axis=1)]


def write_dhmap(path, pts):
    pts = np.asarray(pts, dtype='<f4').reshape(-1, 3)
    with open(path, 'wb') as handle:
        handle.write(b'DHMAP\0\0\0' + struct.pack('<IIQ', 1, 0, len(pts)))
        handle.write(pts.tobytes())
