#!/usr/bin/env python3
"""Redraw every figure this repository produces.

The figures come from three sources, all run by this script:

* the figure scripts under ``generators/``, listed in
  :data:`shfm.generators.GENERATORS`, which read the results shipped under
  ``data/``;
* the notebooks;
* the examples under ``examples/``.

    python scripts/make_all_figures.py                 # everything
    python scripts/make_all_figures.py --notebooks --only 05 07   # those notebooks
    python scripts/make_all_figures.py --generators    # the scripts only
    python scripts/make_all_figures.py --examples      # the examples only

The notebooks are executed in place with nbconvert, so running this also checks
that they still run top to bottom. Everything that solves needs a CUDA device,
which is notebooks 00, 03, 04, 05, 07 and 08, the examples, and two of the
figure scripts, ``nav_function`` and ``state_system``. Whatever measures a path
needs the Bench-MR bindings, from ``scripts/setup.sh``: notebooks 03, 05, 06, 07
and 08, and the figure scripts of the studies. The examples do not.
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from shfm import generators as GEN  # noqa: E402

NOTEBOOKS = ["00_overview", "01_vehicle_and_kinematics",
             "02_collision_and_configuration_space",
             "03_hamiltonian_fast_marching", "04_state_system",
             "05_precision_docking", "06_parameter_study", "07_benchmarks",
             "08_benchmark_metrics", "09_scheme_step_by_step"]
EXAMPLE_SCRIPTS = ["hello_world"]
EXAMPLE_NOTEBOOKS = ["your_own_problem"]


def run_generators(names=None):
    ok = True
    for name in (names or sorted(GEN.GENERATORS)):
        start = time.time()
        try:
            GEN.run(name)
            print("  %-18s ok    %5.1f s" % (name, time.time() - start))
        except Exception as exc:                       # noqa: BLE001
            ok = False
            print("  %-18s FAILED %s" % (name, str(exc)[:200]))
    return ok


def strip_execution_metadata(path):
    """Drop the timestamps nbconvert writes into every executed cell."""
    with open(path, encoding="utf-8") as f:
        nb = json.load(f)
    for cell in nb["cells"]:
        cell.get("metadata", {}).pop("execution", None)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(nb, f, indent=1, ensure_ascii=False)
        f.write("\n")


def run_notebooks(names=None, folder="notebooks"):
    ok = True
    for stem in (names or NOTEBOOKS):
        path = os.path.join(HERE, folder, stem + ".ipynb")
        start = time.time()
        # The kernelspec on most machines launches a bare ``python``, which is
        # whatever comes first on PATH and not necessarily the interpreter this
        # script runs under. We put the directory of that interpreter first on
        # PATH, so that the notebooks see the same environment, cupy included.
        env = dict(os.environ)
        env.pop("MPLBACKEND", None)      # inline figures, stored in the notebook
        env["PATH"] = os.path.dirname(sys.executable) + os.pathsep + env.get("PATH", "")
        done = subprocess.run(
            [sys.executable, "-m", "jupyter", "nbconvert", "--to", "notebook",
             "--execute", "--inplace", "--ExecutePreprocessor.timeout=7200",
             path],
            capture_output=True, env=env, cwd=os.path.join(HERE, folder))
        if done.returncode == 0:
            strip_execution_metadata(path)
            print("  %-38s ok    %6.1f s" % (stem, time.time() - start))
        else:
            ok = False
            print("  %-38s FAILED\n%s" % (stem,
                  done.stderr.decode("utf-8", "replace")[-1500:]))
    return ok


def run_examples():
    ok = True
    for stem in EXAMPLE_SCRIPTS:
        start = time.time()
        done = subprocess.run([sys.executable, os.path.join(HERE, "examples", stem + ".py")],
                              capture_output=True, cwd=HERE)
        if done.returncode == 0:
            print("  %-38s ok    %6.1f s" % (stem, time.time() - start))
        else:
            ok = False
            print("  %-38s FAILED\n%s" % (stem,
                  done.stderr.decode("utf-8", "replace")[-1500:]))
    return run_notebooks(EXAMPLE_NOTEBOOKS, folder="examples") and ok


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", nargs="*", default=None,
                    help="notebook prefixes, e.g. 05 07")
    ap.add_argument("--generators", action="store_true",
                    help="run the standalone generators and stop")
    ap.add_argument("--notebooks", action="store_true",
                    help="run the notebooks, then the examples unless --only is given")
    ap.add_argument("--examples", action="store_true",
                    help="run the examples and stop")
    args = ap.parse_args()

    ok = True
    if args.examples:
        print("examples")
        return 0 if run_examples() else 1
    if not args.notebooks:
        print("generators")
        ok &= run_generators()
    if not args.generators:
        chosen = None
        if args.only:
            chosen = [n for n in NOTEBOOKS
                      if any(n.startswith(p) for p in args.only)]
        print("notebooks")
        ok &= run_notebooks(chosen)
        if not args.only:
            print("examples")
            ok &= run_examples()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
