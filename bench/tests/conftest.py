# Lets pytest import benchtest/ from this folder however it is invoked
# (python3 -m pytest -q tests  from bench/, or  python3 -m pytest -q bench/tests  from the repo root).
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
