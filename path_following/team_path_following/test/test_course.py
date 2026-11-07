import filecmp
import os

import numpy as np

from conftest import REPO
from team_path_following.course import load_course, read_dhmap


def test_course_v3_parses(tmp_path):
    c = load_course(os.path.join(REPO, 'planner', 'config', 'course_v3.yaml'))
    assert len(c.checkpoints) >= 1 and c.goal.shape == (3,)
    assert os.path.exists(c.map_file)
    pts = read_dhmap(c.map_file)
    assert pts.shape[1] == 3 and len(pts) > 1000
    d = os.path.join(tmp_path, 'sub dir')
    os.makedirs(d)
    out = os.path.join(d, 'derived.yaml')
    c.write_derived(out, {'MaxVelMag': 1.1, 'OutDir': '.'})
    c2 = load_course(out)
    assert c2.value('MaxVelMag') == '1.1'
    assert filecmp.cmp(c2.map_file, c.map_file, shallow=False)
    assert np.allclose(c2.start_pos, c.start_pos)
    assert ' ' not in c2.value('MapFile')
