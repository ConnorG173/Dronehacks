# usage: convert_hop.py <main path.csv> <main route.txt> <hop path.csv> <hop route.txt> <out name>
# Puts the hop (planned from its own pad + heading) into the MAIN course's frame via the shared map frame.
# Each plan's pad is its route.txt first point (hover height above it); its heading comes from the
# start->goal vector in both frames (path rows are NED at the pad, route.txt is the map frame, z up).
import csv, json, math, sys
def load(csvp, txtp):
    rows = list(csv.DictReader(open(csvp)))
    m = [list(map(float, l.split())) for l in open(txtp) if l.strip()]
    x, y = float(rows[-1]['x'])-float(rows[0]['x']), float(rows[-1]['y'])-float(rows[0]['y'])
    h = math.atan2(m[-1][1]-m[0][1], m[-1][0]-m[0][0]) - math.atan2(-y, x)
    pad = (m[0][0]-(float(rows[0]['x'])*math.cos(h)+float(rows[0]['y'])*math.sin(h)),
           m[0][1]-(float(rows[0]['x'])*math.sin(h)-float(rows[0]['y'])*math.cos(h)))
    return rows, pad, h
main, pad9, h9 = load(*sys.argv[1:3])
hop, pad11, h11 = load(*sys.argv[3:5])
wp = []
for r in hop:
    x, y, z = float(r['x']), float(r['y']), float(r['z'])
    mx = pad11[0] + x*math.cos(h11) + y*math.sin(h11)          # hop NED -> map
    my = pad11[1] + x*math.sin(h11) - y*math.cos(h11)
    dx, dy = mx-pad9[0], my-pad9[1]                              # map -> main NED
    X, Y = dx*math.cos(h9) + dy*math.sin(h9), dx*math.sin(h9) - dy*math.cos(h9)
    wp.append([X, -Y, -z])                                        # course_flu, as import_path.py
t = [float(r['t']) for r in hop]
end = [float(main[-1]['x']), -float(main[-1]['y']), -float(main[-1]['z'])]
print(f'main pad {pad9} heading {math.degrees(h9):.2f}; hop pad {pad11} heading {math.degrees(h11):.2f}')
print(f'hop {len(wp)} pts {t[-1]:.2f} s; start {[round(v,3) for v in wp[0]]} vs main end {[round(v,3) for v in end]} '
      f'(gap {math.dist(wp[0][:2], end[:2]):.3f} m); end {[round(v,3) for v in wp[-1]]} (from main start {math.dist(wp[-1][:2],[0,0]):.3f} m); '
      f'height {min(p[2] for p in wp):.2f}-{max(p[2] for p in wp):.2f} m')
json.dump(dict(version=1, frame='course_flu', units='m', name=sys.argv[5], waypoints=wp, times_s=t), open(sys.argv[5]+'.json', 'w'))
