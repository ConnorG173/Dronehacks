"""Run the GCOPTER planner (planner/, not modified) on a course config with overrides.

The planner rejects unknown keys and requires all keys, so overrides are applied by
writing a derived copy of the config (relative paths made absolute) into the output
directory. Exit code 0 + path.bin present = a validated plan (planner/README.md).
"""
import os
import shlex
import subprocess
import time

from .course import load_course


def run_planner(binary, config, out_dir, overrides=None, attempts=3, timeout_s=60.0,
                wrapper=None, log=print):
    """Returns (path_bin, summary_text). Raises RuntimeError after `attempts` failures.

    wrapper: optional command prefix (list) for running the binary elsewhere, e.g. a
    `docker run ...` prefix on a laptop. Paths must then be valid inside it.
    """
    course = load_course(config)
    os.makedirs(out_dir, exist_ok=True)
    derived = os.path.join(out_dir, 'planner_input.yaml')
    ov = dict(overrides or {})
    ov['OutDir'] = '.'
    course.write_derived(derived, ov)
    cmd = list(wrapper or []) + [binary, derived, '--out', os.path.abspath(out_dir)]
    last = ''
    for k in range(attempts):
        t0 = time.monotonic()
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        last = proc.stdout + proc.stderr
        path_bin = os.path.join(out_dir, 'path.bin')
        if proc.returncode == 0 and os.path.exists(path_bin):
            log(f'planner OK in {time.monotonic() - t0:.1f} s (attempt {k + 1})')
            return path_bin, last
        log(f'planner attempt {k + 1}/{attempts} failed: {last.strip().splitlines()[-1:] or "?"}')
    raise RuntimeError(f'planner failed {attempts} times: {shlex.join(cmd)}\n{last}')
