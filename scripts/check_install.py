#!/usr/bin/env python3
"""Check that this machine meets the requirements, part by part.

    python scripts/check_install.py

Run it from the environment the package will use. Each line says what was
checked and what it is needed for:

* the package itself, which imports, builds environments and draws on any
  machine, but does not plan;
* the GPU stack, needed by every solve;
* the vendored Bench-MR, built by ``scripts/setup.sh``, needed by anything that
  measures a path and by the comparison plates;
* jupyter, to run the notebooks.

The exit status is non-zero when a part is missing.
"""
import importlib
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

# The versions environment.yml pins. A different version is reported, not
# refused: it may well work, but it is not what the results were produced with.
PINNED = {"numpy": "1.23", "scipy": "1.10", "matplotlib": "3.7", "PIL": "9.4",
          "cupy": "12.0.0"}

failed = []


def report(ok, what, detail=""):
    print("  %-7s %-42s %s" % ("ok" if ok else "MISSING", what, detail))
    if not ok:
        failed.append(what)


def version_note(module, name):
    v = getattr(module, "__version__", "?")
    pin = PINNED.get(name)
    return v if not pin or v.startswith(pin) else "%s (environment.yml pins %s)" % (v, pin)


def section(title):
    print("\n" + title)


section("Package")
ok = sys.version_info[:2] >= (3, 9)
report(ok, "Python 3.9 or later", sys.version.split()[0])
for name in ("numpy", "scipy", "matplotlib", "PIL"):
    try:
        mod = importlib.import_module(name)
        report(True, name, version_note(mod, name))
    except ImportError as exc:
        report(False, name, str(exc))
try:
    import shfm  # noqa: F401
    from shfm.solver import agd_location
    where = os.path.relpath(agd_location(), HERE)
    report(where.startswith(os.path.join("external", "agd")), "shfm, with the vendored agd", where)
except Exception as exc:                                      # noqa: BLE001
    report(False, "shfm, with the vendored agd", str(exc))

section("GPU (every solve)")
try:
    import cupy
    n = cupy.cuda.runtime.getDeviceCount()
    props = cupy.cuda.runtime.getDeviceProperties(0) if n else {}
    name = props.get("name", b"").decode() if n else ""
    report(n > 0, "cupy and a CUDA device",
           "%s, %s, runtime %d, driver %d" % (version_note(cupy, "cupy"), name,
                                             cupy.cuda.runtime.runtimeGetVersion(),
                                             cupy.cuda.runtime.driverGetVersion()))
    free, total = cupy.cuda.Device(0).mem_info
    print("          %.1f GB free of %.1f GB; a solve at 200x200x100 needs about 1.25 GB"
          % (free / 1e9, total / 1e9))
except Exception as exc:                                      # noqa: BLE001
    report(False, "cupy and a CUDA device", str(exc).splitlines()[0])
if "cupy and a CUDA device" not in failed:
    try:
        import numpy as np
        from shfm import from_occupancy, Planner, REFERENCE_COEFFICIENTS
        walls = np.zeros((40, 40), bool)
        walls[[0, -1], :] = walls[:, [0, -1]] = True
        env = from_occupancy(walls, zoom=2, n_theta=16)
        r = Planner(env, k1=10, k2=50, **REFERENCE_COEFFICIENTS).solve(
            seed=[12, 20, 0, 0], tips=[[28, 20, 0, 0]])
        p = r.path(0)
        ok = len(p) > 1 and np.hypot(*(p[-1, :2] - [28, 20])) < 1.0
        report(ok, "a small solve, end to end", "%d points" % len(p))
    except Exception as exc:                                  # noqa: BLE001
        report(False, "a small solve, end to end", str(exc).splitlines()[0])

section("Bench-MR (metrics, plates, campaigns), built by scripts/setup.sh")
from shfm import benchmark as B                              # noqa: E402
report(os.path.exists(B.BINARY), "benchmark binary", os.path.relpath(B.BINARY, HERE))
report(os.path.exists(B.TEMPLATE), "settings template", os.path.relpath(B.TEMPLATE, HERE))
try:
    from shfm.metrics import bindings, path_length, quiet
    bindings()
    with quiet():
        ok = abs(path_length([[0, 0], [3, 4]]) - 5.0) < 1e-9
    report(ok, "metric bindings, for this Python")
except Exception as exc:                                      # noqa: BLE001
    report(False, "metric bindings, for this Python", str(exc).splitlines()[0])
for name in ("click", "bitarray", "psutil", "tqdm"):
    try:
        mod = importlib.import_module(name)
        report(True, name + " (Bench-MR plotting tools)", getattr(mod, "__version__", ""))
    except ImportError as exc:
        report(False, name + " (Bench-MR plotting tools)", str(exc))

section("Notebooks")
done = subprocess.run([sys.executable, "-m", "jupyter", "nbconvert", "--version"],
                      capture_output=True, text=True)
report(done.returncode == 0, "jupyter nbconvert", done.stdout.strip())

print()
if failed:
    print("Missing: %s." % ", ".join(failed))
    print("See docs/install.md.")
    sys.exit(1)
print("Everything is in place.")
