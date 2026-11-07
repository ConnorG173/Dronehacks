"""End to end, offline: the Follower in each adapter's configuration flies course_v3 against the PX4
position-controller model, from landed on the pad to the end of the run (closed_loop.py)."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import closed_loop as CL  # noqa: E402


@pytest.fixture(scope="module")
def cpath():
    return CL.load()


@pytest.mark.parametrize("mode,final", [("sim", "DONE"), ("real", "FINISH")])
def test_follows_course_v3_without_stopping(cpath, mode, final):
    log = CL.run(cpath, mode, 1.0)
    m = CL.metrics(cpath, log)
    assert m["final_mode"] == final, log["follower"].reason
    assert m["stalls"] == 0 and m["xtrack_max"] < 0.10 and m["err_max"] < 0.15
    assert m["t_follow"] < cpath.duration / log["follower"].scale + 1.5
    assert m["vmax"] < 1.15


@pytest.mark.parametrize("mode", ["sim", "real"])
@pytest.mark.parametrize("kw", [{"kp_xy": 0.7}, {"kp_xy": 1.3}])
def test_wrong_kp_slows_but_does_not_stop(cpath, mode, kw):
    m = CL.metrics(cpath, CL.run(cpath, mode, 1.0, kw))
    assert m["final_mode"] in ("DONE", "FINISH") and m["stalls"] == 0 and m["xtrack_max"] < 0.15


def test_sim_full_speed_with_feedforward(cpath):
    m = CL.metrics(cpath, CL.run(cpath, "sim", 10.0))
    assert m["final_mode"] == "DONE" and m["stalls"] == 0 and m["xtrack_max"] < 0.30
