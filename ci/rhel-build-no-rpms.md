# Building SCRIMMAGE on RHEL 8 (no sudo)

You can build and run SCRIMMAGE on RHEL 8 / Rocky 8 entirely in your home
directory using conda — no root, no `dnf`, no RPMs. If you have
[Miniforge](https://github.com/conda-forge/miniforge) (it installs into your
home dir), you're set.

## Quickstart

```bash
# 1. Create the env with everything SCRIMMAGE needs
git clone <this-repo> scrimmage && cd scrimmage
conda env create -f rhel-environment-build.yml
conda activate scrimmage

# 2. Build — a plain cmake && make, no flags needed
mkdir build && cd build && cmake .. && make -j$(nproc)

# 3. Run (source the setup script once per shell)
source ~/.scrimmage/setup.bash
scrimmage ../missions/straight-no-gui.xml
```

The dependency list lives in `rhel-environment-build.yml` at the repo root. With
the env active, conda puts everything on `CMAKE_PREFIX_PATH`, so `cmake` finds
all the dependencies on its own.

## Building an overlay repo against it

Got a separate repo that does `find_package(scrimmage)`? Build it the same way,
from its own `build/` dir with the env active — it finds this build tree
automatically. Depending on what the overlay vendors, you may need two flags:

```bash
cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DEIGEN3_INCLUDE_DIR=$CONDA_PREFIX/include/eigen3 ..
```

The first helps if the overlay bundles an old dependency (like an old
googletest) that conda's newer cmake would otherwise reject; the second if it
reads the legacy `EIGEN3_INCLUDE_DIR` (conda's Eigen only sets the modern
target). Neither is a SCRIMMAGE problem — just conda's newer cmake/Eigen.

## Good to know

- **Boost is pinned to 1.83.** Newer Boost drops the compiled `boost_system`
  library that SCRIMMAGE still links against.
- **gRPC, JSBSim, and the GUI are off.** They aren't needed for the core build;
  cmake auto-disables the plugins that use them. Use `-no-gui` missions.

## About the dockerfile

`ci/dockerfiles/rockylinux8-conda` is just proof this works — it builds
SCRIMMAGE from a stock `rockylinux:8` image the same way. Docker isn't part of
running SCRIMMAGE on your box; the quickstart above is.
