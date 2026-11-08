# usage: import_return.py <return.json from convert_hop.py> <token>   (claim, import_return, release)
import json, sys, urllib.request
doc, token = json.load(open(sys.argv[1])), sys.argv[2]
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
    j = post('/api/course', dict(op='import_return', trajectory=doc))
    print('return hop on dashboard:', (j.get('state') or {}).get('course_return'))
finally:
    post('/api/cmd', dict(op='release'))
