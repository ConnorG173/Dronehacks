"""Read-only DHPATH decoding and explicit pad-NED to course-FLU conversion.

Importing this module never starts a node or sends a command.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import struct


COLUMNS = ('t','x','y','z','vx','vy','vz','ax','ay','az','yaw','yaw_rate')
FLIPPED = frozenset((2,3,5,6,8,9,10,11))


def read_dhpath(path):
    raw = Path(path).read_bytes()
    if len(raw) < 32 or raw[:8] != b'DHPATH\0\0':
        raise ValueError('not a DHPATH file')
    version, cols, count, dt = struct.unpack('<IIQd', raw[8:32])
    if version != 1 or cols != 12 or not 2 <= count <= 4000:
        raise ValueError('expected DHPATH v1, 12 columns, 2–4000 rows')
    if not math.isfinite(dt) or dt <= 0 or len(raw) != 32 + count*96:
        raise ValueError('invalid DHPATH sample period or exact file size')
    rows = list(struct.iter_unpack('<12d', raw[32:]))
    if not all(all(math.isfinite(v) for v in row) for row in rows):
        raise ValueError('non-finite trajectory sample')
    if rows[0][0] < 0 or any(b[0] <= a[0] for a,b in zip(rows,rows[1:])):
        raise ValueError('trajectory times must increase strictly')
    if any(abs(v) > 1000 for row in rows for v in row[1:4]):
        raise ValueError('trajectory coordinates exceed 1000 m')
    return rows, dict(format='DHPATH_v1', source=str(Path(path).resolve()),
                      sha256=hashlib.sha256(raw).hexdigest(), dt_s=dt, rows=count)


def to_course_flu(rows):
    # Same proper 3D rotation for position, velocity and acceleration; heading
    # becomes counterclockwise/left-positive instead of clockwise/right-positive.
    return [[-v if i in FLIPPED else v for i,v in enumerate(row)] for row in rows]


def document(rows, name):
    return dict(version=1,frame='course_flu',units='m',name=name,
                waypoints=[list(r[1:4]) for r in rows],times_s=[r[0] for r in rows])


def describe(rows, max_xy=1., max_z=.3):
    xy=max(math.hypot(*r[4:6]) for r in rows)
    vz=max(abs(r[6]) for r in rows)
    stretch=max(1.,xy/max_xy,vz/max_z)
    return dict(duration_s=rows[-1][0]-rows[0][0],
                first_xyz=list(rows[0][1:4]),last_xyz=list(rows[-1][1:4]),
                xyz_bounds=[[min(r[i] for r in rows),max(r[i] for r in rows)] for i in (1,2,3)],
                length_m=sum(math.dist(a[1:4],b[1:4]) for a,b in zip(rows,rows[1:])),
                max_radius_m=max(math.hypot(*r[1:3]) for r in rows),
                peak_xy_mps=xy,peak_abs_z_mps=vz,
                minimum_uniform_time_stretch=stretch,
                speed_limited_duration_s=(rows[-1][0]-rows[0][0])*stretch,
                timing_applied=False,execution_supported=False)


def main():
    import argparse
    from course_state import CourseState
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',required=True)
    p.add_argument('--destination',required=True)
    p.add_argument('--name',default='course_v3_20261107')
    args=p.parse_args()
    rows,meta=read_dhpath(args.source)
    converted=to_course_flu(rows)
    dest=Path(args.destination)
    ok,reason=CourseState(dest).import_route(document(converted,args.name))
    if not ok: raise SystemExit(reason)
    # Original binary and CSV stay untouched. Retain all converted derivative
    # columns for later timing work, separately from the display document.
    table=dest.with_name(dest.stem+'_flu.csv')
    with table.open('w',newline='') as handle:
        writer=csv.writer(handle);writer.writerow(COLUMNS);writer.writerows(converted)
    report=dict(source=meta,conversion='[x, -y, -z]; negate yaw and yaw_rate',
                course_frame='origin at launch mark; +X pad nose/map +X, +Y left, +Z up',
                route=describe(converted),destination=str(dest),converted_table=str(table))
    dest.with_name(dest.stem+'_import.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
