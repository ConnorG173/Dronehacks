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
        why = [l for l in last.splitlines() if l.startswith('ERROR')] or last.strip().splitlines()[-1:] or \
            [f'exit {proc.returncode}']
        log(f'planner attempt {k + 1}/{attempts} failed (exit {proc.returncode}): {why[0]}')
    raise RuntimeError(f'planner failed {attempts} times: {shlex.join(cmd)}\n{last}')


def plan_duration(path_bin):
    """Total duration of a DHPATH file (last row's t)."""
    from .pathfile import read_dhpath
    rows, _ = read_dhpath(path_bin)
    return float(rows[-1, 0])


def plan_best(binary, config, out_dir, overrides=None, candidates=3, budget_s=45.0,
              wrapper=None, log=print):
    """Run the (randomised) planner up to `candidates` successful times within `budget_s`
    and keep the fastest validated plan as out_dir/path.bin. Every candidate already
    passed the planner's own validation, so picking the shortest is safe."""
    import shutil
    t0 = time.monotonic()
    best, best_t, summary, n_ok, errors = None, float('inf'), '', 0, []
    k = 0
    while n_ok < max(1, candidates) and (time.monotonic() - t0 < budget_s or best is None) and k < 3 * max(1, candidates):
        cand = os.path.join(out_dir, f'cand{k}')
        k += 1
        try:
            path, text = run_planner(binary, config, cand, overrides, attempts=1, wrapper=wrapper, log=log,
                                     timeout_s=max(10.0, budget_s))
        except (RuntimeError, subprocess.TimeoutExpired) as exc:
            errors.append(str(exc).splitlines()[0])
            continue
        n_ok += 1
        dur = plan_duration(path)
        log(f'candidate {k}: {dur:.2f} s')
        if dur < best_t:
            best, best_t, summary = path, dur, text
    if best is None:
        raise RuntimeError(f'no valid plan in {k} attempts: {errors[-1:] or "?"}')
    final = os.path.join(out_dir, 'path.bin')
    shutil.copyfile(best, final)
    log(f'best of {n_ok} plans: {best_t:.2f} s ({time.monotonic() - t0:.1f} s planning)')
    return final, summary
