#!/bin/bash
# Upload an edited course CSV to the drone as the main course (the return hop stays loaded).
# usage: upload_course.sh [file.csv]     default: newest *edited*.csv in ~/Downloads
# env: REF=<original csv> (start/end check), DASH_URL, ssh host alias "skybeta" for the token
set -e
cd "$(dirname "$0")"
F="${1:-$(ls -t ~/Downloads/*edited*.csv 2>/dev/null | head -1)}"
[ -f "$F" ] || { echo "no edited CSV found (pass one as an argument)"; exit 1; }
echo "uploading: $F"
REF="${REF:-../flight/courses/course_20261108_path9/path (9).csv}"   # route the edit started from
python3 - "$F" "$REF" <<'PY'
import csv, math, sys
a=list(csv.DictReader(open(sys.argv[2]))); b=list(csv.DictReader(open(sys.argv[1])))
P=lambda r:[float(r['x']),float(r['y']),float(r['z'])]
print(f"  {len(b)} rows, {float(b[-1]['t']):.2f} s, height {min(-float(r['z']) for r in b):.2f}-{max(-float(r['z']) for r in b):.2f} m")
e=math.dist(P(a[-1])[:2],P(b[-1])[:2]); s=math.dist(P(a[0])[:2],P(b[0])[:2])
print(f"  start moved {s:.3f} m, end moved {e:.3f} m vs the reference route" + ("  <-- end moved: the return hop may no longer join (limit 0.30 m incl. 0.09 m)" if e > .2 else ""))
PY
T=$(ssh skybeta 'docker exec skybeta-stack cat /root/team_ws/claude/dash/dash_token.txt')
NAME="course_$(date +%Y%m%d_%H%M)_edited"
python3 -I import_path.py "$F" "$NAME" "$T"
echo "done: press Set start here on the launch mark, then Take off · course + return hop"
