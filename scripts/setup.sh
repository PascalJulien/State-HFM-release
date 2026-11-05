#!/usr/bin/env bash
#
# Build the vendored Bench-MR benchmark binary and its Python metric bindings.
#
#   conda activate shfm
#   ./scripts/setup.sh
#
# Run it from the environment that will use the package: the bindings are
# compiled for the Python found first on PATH, or for $PYTHON when it is set.
#
# Bench-MR supplies every metric this package reports, through shfm.metrics, so
# it is needed by anything that measures a path: notebooks 03, 05, 06, 07
# and 08, the acquisition scripts, and the figure scripts of the studies.
# Notebooks 00 to 02, 04 and 09 plan and draw without it.
#
# What it needs from the system, found the standard way, through CMake and
# pkg-config: a C++17 compiler, CMake, pkg-config, Eigen 3, Boost (serialization,
# filesystem, system, program_options), OMPL 1.5 and SBPL. pybind11 comes from
# the Python environment (environment.yml). Libraries installed outside the
# default prefixes are found through CMAKE_PREFIX_PATH (OMPL, Eigen, Boost) and
# PKG_CONFIG_PATH (SBPL); sourcing a ROS setup.bash sets both. Further CMake
# options can be passed in CMAKE_ARGS.
#
# Qt5 is not needed: Bench-MR only uses it for an interactive viewer, so it is
# left out of the build. Set SHFM_WITH_QT=1 to let CMake look for it.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bench="$here/external/bench-mr"

python="${PYTHON:-$(command -v python || command -v python3 || true)}"

# Check everything first, so that a missing piece is reported by name rather
# than as a CMake error twenty lines into the configure step.
missing=()
if [ -z "$python" ]; then
  missing+=("a Python interpreter (activate the shfm environment)")
elif ! "$python" -c 'import sys; sys.exit(0 if sys.version_info[:2] >= (3, 9) else 1)'; then
  missing+=("Python 3.9 or later, found $("$python" -V 2>&1) at $python")
fi
command -v cmake >/dev/null 2>&1 || missing+=("cmake")
command -v pkg-config >/dev/null 2>&1 || missing+=("pkg-config")
command -v "${CXX:-c++}" >/dev/null 2>&1 || missing+=("a C++17 compiler")
if command -v pkg-config >/dev/null 2>&1 && ! pkg-config --exists sbpl; then
  missing+=("SBPL: pkg-config cannot find sbpl.pc (set PKG_CONFIG_PATH)")
fi
pybind11_dir=""
if [ -n "$python" ]; then
  pybind11_dir="$("$python" -m pybind11 --cmakedir 2>/dev/null || true)"
  [ -n "$pybind11_dir" ] || missing+=("pybind11 in the Python environment at $python")
fi
if [ "${#missing[@]}" -gt 0 ]; then
  echo "Cannot build Bench-MR, missing:"
  printf '  - %s\n' "${missing[@]}"
  echo "See docs/install.md, section Requirements."
  exit 1
fi

python="$(command -v "$python")"
cmake_args=(
  -DCMAKE_BUILD_TYPE=Release
  -Dpybind11_DIR:PATH="$pybind11_dir"
  # pybind11 2.x reads the first, 3.x the others; all three name the
  # interpreter the bindings are compiled for.
  -DPYTHON_EXECUTABLE:FILEPATH="$python"
  -DPython_EXECUTABLE:FILEPATH="$python"
  -DPython3_EXECUTABLE:FILEPATH="$python"
)
if [ "${SHFM_WITH_QT:-0}" = 1 ]; then
  [ -z "${QT5_DIR:-}" ] || cmake_args+=(-DQt5_DIR="$QT5_DIR")
else
  cmake_args+=(-DCMAKE_DISABLE_FIND_PACKAGE_Qt5=ON)
fi

mkdir -p "$bench/build"
cd "$bench/build"
# A cache left by a failed configure, or written for another interpreter, keeps
# steering CMake towards it: start from a clean one in either case.
if [ ! -f Makefile ] || ! grep -qxF "PYTHON_EXECUTABLE:FILEPATH=$python" CMakeCache.txt 2>/dev/null; then
  rm -f CMakeCache.txt
fi
# CMAKE_ARGS is split on spaces: it holds options, not paths with spaces in
# them.
# shellcheck disable=SC2086
cmake .. "${cmake_args[@]}" ${CMAKE_ARGS:-} >/dev/null

jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
cmake --build . --target benchmark -- -j"$jobs"

# metrics_bindings is a separate CMake target, not a dependency of benchmark, so
# it has to be asked for by name. Without it shfm.metrics cannot import, and
# nothing that measures a path will run.
cmake --build . --target metrics_bindings -- -j"$jobs"

# The binary writes its own settings template on startup; the config builder
# in shfm.benchmark starts from it, so make sure it exists.
if [ ! -f "$bench/bin/benchmark_template.json" ]; then
  ( cd "$bench/bin" && ./benchmark >/dev/null 2>&1 || true )
fi

# Import the bindings the way the package does, with the interpreter they were
# built for.
if ! "$python" -c "import sys; sys.path.insert(0, sys.argv[1]); from shfm.metrics import bindings; bindings()" "$here" 2>/dev/null; then
  echo
  echo "The build finished but $python cannot import the metric bindings from"
  echo "  external/bench-mr/python/"
  echo "The planner will run; anything that measures a path will not."
  exit 1
fi

echo
echo "Done."
echo "  binary   external/bench-mr/bin/benchmark"
echo "  bindings external/bench-mr/python/metrics_bindings (for $("$python" -V 2>&1))"
