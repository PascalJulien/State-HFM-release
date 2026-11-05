# Installation

## Requirements

* **Linux on x86-64, an NVIDIA GPU and its driver.** The eikonal solver runs on
  the GPU through cupy; there is no practical CPU path. conda installs the CUDA
  runtime, so only the driver comes from the system. A 200x200x100 grid takes
  about 1.25 GB of GPU memory at peak.
* **conda** (or mamba), for the Python environment of `environment.yml`:
  Python 3.9 and the versions the results were produced with. The `agd`
  library by J.-M. Mirebeau, which provides the anisotropic solver and its CUDA
  kernels, is vendored under `external/agd`.
* **To measure paths**, that is for notebooks 03, 05, 06, 07 and 08, for the
  acquisition scripts and for the figure scripts that measure paths or draw
  Bench-MR plates, the vendored Bench-MR has to be built. The build requires,
  from the system: a C++17 compiler, CMake, pkg-config, Eigen 3, Boost
  (serialization, filesystem, system, program_options), OMPL 1.5 and SBPL. Qt
  is not needed. Distributions that ship an older OMPL (Ubuntu 20.04 ships
  1.4) need it built from source; SBPL comes from source or from ROS Noetic
  (`ros-noetic-sbpl`).
* `pdftoppm` (poppler-utils), for the PNG copies of two trajectory plates.

The configuration every result here was produced with: Ubuntu 20.04, GCC 9.4,
CMake 3.27, OMPL 1.5.2 built from source, SBPL 1.3.1 from ROS Noetic, Boost
1.71, Eigen 3.3.7; an NVIDIA T1200 with 4 GB, driver 565 (CUDA 12.7), cupy
12.0.0 on a CUDA 10.2 runtime.

## The Python environment

```bash
git clone https://github.com/PascalJulien/State-HFM
cd State-HFM
conda env create -f environment.yml
conda activate shfm
pip install -e .
python scripts/check_install.py
```

With conda older than 23.10, whose default solver is the classic one, solving
this environment can take longer than half an hour; use `mamba` or
`micromamba` with the same file, or `conda env create --solver=libmamba` where
that solver is installed.

Both external libraries are vendored under `external/`, so there is nothing
else to fetch. The package reads `external/`, `data/` and `generators/` from
the clone, which is why it is installed in editable mode; the notebooks and
the scripts also work from the clone without installing it.
`scripts/check_install.py` reports, part by part, what this machine can run:
the package, a solve on the GPU, the Bench-MR metrics, the notebooks.

## Bench-MR, for the metrics

If you only want to plan paths, the above is enough. The metrics come from
Bench-MR, so anything that measures a path needs it built:

```bash
./scripts/setup.sh
```

Run it with the environment active: it builds the benchmark binary and the
metric bindings for that Python, and writes the settings template the
configuration builder starts from. It checks Python, CMake, pkg-config, the
compiler, SBPL and pybind11 first and names the missing ones; a missing OMPL,
Boost or Eigen shows up as a CMake error. Libraries installed outside the default prefixes are found
through `CMAKE_PREFIX_PATH` (OMPL, Eigen, Boost) and `PKG_CONFIG_PATH` (SBPL);
sourcing a ROS `setup.bash` sets both. Further CMake options go in
`CMAKE_ARGS`.
