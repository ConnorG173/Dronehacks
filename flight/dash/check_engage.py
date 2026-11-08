import json, urllib.request
tok = open("/root/team_ws/claude/dash/dash_token.txt").read().strip()
B = "http://100.64.16.93:8091"
def post(op, **kw):
    r = urllib.request.Request(f"{B}/api/cmd?k={tok}", data=json.dumps(dict(c="t2", op=op, **kw)).encode(),
                               headers={"Content-Type": "application/json"}, method="POST")
    return json.loads(urllib.request.urlopen(r, timeout=3).read())
print("claim:", post("claim")["msg"])
j = post("engage", alt=1.0)
st = j["state"]
print("engage:", j["ok"], "-", j["msg"])
print("floor_offset:", st["floor_offset"], "| ekf:", st["ekf"], "| battery:", st["batt"], "| vio:", st["vio"])
print("release:", post("release")["msg"])
