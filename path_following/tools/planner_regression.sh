#!/usr/bin/env bash
# Build the planner in pf-dev:humble (arm64, OMPL 1.7 as on the drone) and run planner tests 5-6
# (random course that used to segfault, course_v3). "all" runs the full planner/tests/run_tests.py.
# The repo is mounted at /repo: the planner config parser cannot read paths with spaces.
#   bash path_following/tools/planner_regression.sh [all]
set -eo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
WHICH="${1:-quick}"
docker run --rm --network none -e WHICH="$WHICH" -v "$REPO:/repo" -w /repo/planner pf-dev:humble bash -c '
set -e
cmake -S . -B build-test -DCMAKE_PREFIX_PATH=/opt/ros/humble >/dev/null
cmake --build build-test -j8 >/dev/null
if [ "$WHICH" = all ]; then
  python3 tests/run_tests.py build-test
else
  python3 -c "import sys; sys.argv=[\"t\", \"build-test\"]; sys.path.insert(0, \"tests\"); import run_tests as t; t.straight_legs(); t.course_v3(); print(\"failures:\", t.failures); sys.exit(1 if t.failures else 0)"
fi'
