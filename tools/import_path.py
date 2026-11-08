# usage: import_path.py <path.csv> <name> <token>
# PX4 NED-at-pad CSV (t,x,y,z,...) -> course_flu [x,-y,-z], claim, import, release.
import csv, json, sys, urllib.request
src, name, token = sys.argv[1], sys.argv[2], sys.argv[3]
rows = list(csv.DictReader(open(src)))
t = [float(r['t']) for r in rows]
wp = [[float(r['x']), -float(r['y']), -float(r['z'])] for r in rows]
assert all(b > a for a, b in zip(t, t[1:])), 'times must increase'
doc = dict(version=1, frame='course_flu', units='m', name=name, waypoints=wp, times_s=t)
json.dump(doc, open(name + '.json', 'w'))
print(f'{len(wp)} points, {t[-1]:.2f} s, start {wp[0]}, end {[round(v, 3) for v in wp[-1]]}, '
      f'z {min(p[2] for p in wp):.2f}..{max(p[2] for p in wp):.2f} m')
import os
base, C = os.environ.get('DASH_URL', 'http://100.64.16.93:8091'), 'path-import'
def post(path, body):
    req = urllib.request.Request(f'{base}{path}?k={token}', json.dumps(dict(body, c=C)).encode(),
                                 {'Content-Type': 'application/json'})
    j = json.load(urllib.request.urlopen(req, timeout=15))
    print(path, body.get('op'), '->', j['ok'], j['msg'])
    return j
if not post('/api/cmd', dict(op='claim'))['ok']:
    sys.exit('could not take control (someone holds it)')
try:
    j = post('/api/course', dict(op='import', trajectory=doc))
    c = j['state'].get('course') or {}
    print('dashboard course:', c.get('name'), c.get('status'), c.get('original_count'), c.get('reason'))
finally:
    post('/api/cmd', dict(op='release'))
