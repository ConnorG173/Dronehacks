#!/usr/bin/env python3
"""Generate the sim frame `skybeta_frame`: PX4's x500 (dynamics, motors, sensors unchanged) with
collisions replaced by Sky Beta's footprint, so the sim drone is not wider than the real one.

    python3 sim/tools/make_skybeta_frame.py [--prop-in 7] [--px4 ~/PX4-Autopilot]

Writes sim/gz/models/skybeta_base/ and sim/gz/models/skybeta_frame/ (generated, gitignored).

Sky Beta geometry, from the organizers' px4/params/07_airframe_geometry.params (CA_ROTOR*_PX/PY,
FRD, metres from the IMU): front motors (+0.08, +/-0.16), rear motors (-0.15, +/-0.14).
ASSUMPTION: 7" props (not in the params; adjacent motors 0.23 m apart allow at most ~9").
Result: reaches ~0.29 m from the centre, vs ~0.39 m for the stock x500.

Why only collisions: moving the x500's rotors would change its dynamics and PX4's control
allocation (the airframe's CA_ROTOR*). Prop discs are static collisions on base_link, the real rotor
links keep only a negligible collision.
"""
import argparse
import os
import xml.etree.ElementTree as ET

# FRD (forward, right) metres from IMU -> gz FLU (forward, left): y_flu = -y_frd
SKYBETA_MOTORS_FRD = [(0.08, 0.16), (-0.15, -0.14), (0.08, -0.16), (-0.15, 0.14)]
BODY_BOX = (0.20, 0.20, 0.05)     # central plate + electronics, conservative
PROP_Z = 0.03                    # disc height above base_link origin (x500 rotors sit ~0.023)


def el(tag, text=None, **attrib):
    e = ET.Element(tag, attrib)
    if text is not None:
        e.text = text
    return e


def disc_collision(name, x, y, r):
    c = el("collision", name=name)
    c.append(el("pose", f"{x:.4f} {y:.4f} {PROP_Z} 0 0 0"))
    g = el("geometry")
    cyl = el("cylinder")
    cyl.append(el("radius", f"{r:.4f}"))
    cyl.append(el("length", "0.015"))
    g.append(cyl)
    c.append(g)
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prop-in", type=float, default=7.0)
    ap.add_argument("--px4", default=os.path.expanduser("~/PX4-Autopilot"))
    a = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    models = os.path.normpath(os.path.join(here, "..", "gz", "models"))
    src = os.path.join(a.px4, "Tools", "simulation", "gz", "models")
    r = a.prop_in * 0.0254 / 2

    tree = ET.parse(os.path.join(src, "x500_base", "model.sdf"))
    model = tree.getroot().find("model")
    model.set("name", "skybeta_base")
    for link in model.findall("link"):
        name = link.get("name")
        for col in link.findall("collision"):
            box = col.find("geometry/box/size")
            if name == "base_link" and col.get("name") == "base_link_collision_0" and box is not None:
                box.text = " ".join(str(v) for v in BODY_BOX)
            elif name.startswith("rotor_") and box is not None:
                box.text = "0.001 0.001 0.001"
        if name == "base_link":
            for i, (px, py) in enumerate(SKYBETA_MOTORS_FRD):
                link.append(disc_collision(f"skybeta_prop_disc_{i}", px, -py, r))
    os.makedirs(os.path.join(models, "skybeta_base"), exist_ok=True)
    tree.write(os.path.join(models, "skybeta_base", "model.sdf"), encoding="unicode", xml_declaration=True)

    with open(os.path.join(src, "x500", "model.sdf")) as f:
        top = f.read()
    top = top.replace("model://x500_base", "model://skybeta_base").replace("<model name='x500'>", "<model name='skybeta_frame'>")
    top = top.replace('<model name="x500">', '<model name="skybeta_frame">')
    os.makedirs(os.path.join(models, "skybeta_frame"), exist_ok=True)
    with open(os.path.join(models, "skybeta_frame", "model.sdf"), "w") as f:
        f.write(top)
    for d, desc in (("skybeta_base", "x500 base with Sky Beta collisions"), ("skybeta_frame", "x500 dynamics, Sky Beta footprint")):
        with open(os.path.join(models, d, "model.config"), "w") as f:
            f.write(f'<?xml version="1.0"?>\n<model><name>{d}</name><version>1.0</version>'
                    f'<sdf version="1.9">model.sdf</sdf><description>{desc} (generated)</description></model>\n')
    reach = max((px * px + py * py) ** 0.5 for px, py in SKYBETA_MOTORS_FRD) + r
    print(f"wrote skybeta_base + skybeta_frame: {a.prop_in:.1f} in props, max reach {reach:.3f} m")


if __name__ == "__main__":
    main()
