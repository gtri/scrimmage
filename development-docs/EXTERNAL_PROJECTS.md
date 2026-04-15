# SCRIMMAGE External Projects (Overlay Pattern)

SCRIMMAGE supports an "overlay" or "external project" pattern — building SCRIMMAGE as a dependency and creating a separate repository on top for custom plugins, missions, and configurations.

## Overview

```
~/.scrimmage/setup.bash        # Sources all project setenv files
    ├── env/scrimmage-setenv   # Base SCRIMMAGE paths
    └── env/MyProject-setenv   # Overlay project paths (additive)

At runtime, SCRIMMAGE searches merged paths:
  SCRIMMAGE_PLUGIN_PATH  = /scrimmage/plugins:/MyProject/plugins
  SCRIMMAGE_MISSION_PATH = /scrimmage/missions:/MyProject/missions
```

## Creating an External Project

Use the built-in template generator:

```bash
# From SCRIMMAGE root
./scripts/create-scrimmage-project.py MyProject ~/projects/
```

This creates a standalone project:

```
MyProject/
├── CMakeLists.txt              # Uses find_package(scrimmage)
├── .devcontainer/              # VS Code devcontainer for sibling SCRIMMAGE repo
├── .vscode/                    # Build/debug tasks and LLDB launch configs
├── cmake/Modules/              # CMake helpers
├── include/MyProject/plugins/  # Plugin headers + XML configs
│   └── autonomy/
│       └── ExamplePlugin/
│           ├── ExamplePlugin.h
│           └── ExamplePlugin.xml
├── src/plugins/                # Plugin implementations
│   └── autonomy/
│       └── ExamplePlugin/
│           ├── CMakeLists.txt
│           └── ExamplePlugin.cpp
├── missions/                   # Project mission XML files
│   └── example.xml
├── msgs/                       # Custom protobuf messages
├── setup/
└── test/
```

## VS Code Dev Container Workflow

The generated template now includes a `.devcontainer/devcontainer.json` and `.vscode/{tasks,launch}.json` so an external project gets the same clangd, CMake Tools, and LLDB workflow as the main SCRIMMAGE repository.

It assumes the repositories are siblings on the host:

```text
/code/scrimmage
/code/MyProject
```

Inside the container those mounts become:

```text
/root/scrimmage
/root/MyProject
```

The generated setup does four important things:

1. Mounts the sibling SCRIMMAGE checkout into the container at `/root/scrimmage`
2. Points clangd at the overlay project's `build/compile_commands.json`
3. Configures CMake Tools and the default build tasks for install-tree builds with `SETUP_*_CONFIG=OFF`
4. Points the overlay project at the installed SCRIMMAGE package with `-Dscrimmage_DIR=/root/scrimmage/build/install/share/cmake/scrimmage`

That means the recommended devcontainer workflow is:

```bash
# Inside the container, build and install SCRIMMAGE first
cd /root/scrimmage
mkdir -p build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug \
  -DSETUP_HOME_CONFIG=OFF -DCMAKE_INSTALL_PREFIX=$PWD/install ..
make -j$(nproc) install

# Then build and install the external project
cd /root/MyProject
mkdir -p build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug \
  -DSETUP_LOCAL_CONFIG_DIR=OFF \
  -DCMAKE_INSTALL_PREFIX=$PWD/install \
  -Dscrimmage_DIR=/root/scrimmage/build/install/share/cmake/scrimmage ..
make -j$(nproc) install

# Source both install trees when running from a terminal
source /root/scrimmage/build/install/etc/scrimmage/env/scrimmage-setenv
source /root/MyProject/build/install/etc/MyProject/env/MyProject-setenv
```

The LLDB launch configuration runs `/root/scrimmage/build/install/bin/scrimmage` and uses the installed overlay plugin and library paths, so `F5` matches the install-tree workflow instead of the bare metal build-tree path layout.

## How It Works

### 1. CMake Integration

The overlay project's `CMakeLists.txt` finds the base SCRIMMAGE installation:

```cmake
find_package(scrimmage REQUIRED)

include(GenerateSetEnv)
GenerateSetEnv(
  SETUP_LOCAL_CONFIG_DIR ${SETUP_LOCAL_CONFIG_DIR}
  SETENV_IN_FILE ${SCRIMMAGE_CMAKE_MODULES}/setenv.in
  MISSION_PATH ${PROJECT_SOURCE_DIR}/missions
  PLUGIN_PATH ${PROJECT_BINARY_DIR}/plugin_libs
              ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/plugins
)
```

### 2. Environment Path Merging

When building the overlay project, `GenerateSetEnv()`:

1. Creates `~/.scrimmage/env/MyProject-setenv` containing:
   ```bash
   export SCRIMMAGE_MISSION_PATH=${SCRIMMAGE_MISSION_PATH}:/path/to/MyProject/missions
   export SCRIMMAGE_PLUGIN_PATH=${SCRIMMAGE_PLUGIN_PATH}:/path/to/MyProject/plugin_libs
   ```

2. Appends a source line to `~/.scrimmage/setup.bash`:
   ```bash
   source ~/.scrimmage/env/MyProject-setenv
   ```

Paths are **additive** — the overlay project's paths are appended to the base SCRIMMAGE paths.

### 3. Plugin Resolution

Missions can reference plugins from both:

- **Base SCRIMMAGE**: `SimpleAircraft`, `SimpleAircraftControllerPID`, `Straight`, etc.
- **Overlay project**: `ExamplePlugin`, custom autonomy/controllers/sensors

```xml
<!-- missions/example.xml -->
<entity>
  <motion_model>SimpleAircraft</motion_model>         <!-- From base SCRIMMAGE -->
  <controller>SimpleAircraftControllerPID</controller> <!-- From base SCRIMMAGE -->
  <autonomy>ExamplePlugin</autonomy>                   <!-- From overlay project -->
</entity>
```

## Complete Workflow (Bare Metal / Non-Container)

This workflow is for building SCRIMMAGE from source on the host machine.

```bash
# 1. Clone and build base SCRIMMAGE
git clone https://github.com/gtri/scrimmage.git
cd scrimmage
mkdir build && cd build
cmake .. && make -j$(nproc)
source ~/.scrimmage/setup.bash

# 2. Create the overlay project
cd /path/to/scrimmage
./scripts/create-scrimmage-project.py MyDrones ~/projects/

# 3. Build the overlay project
cd ~/projects/MyDrones
mkdir build && cd build
cmake .. && make -j$(nproc)

# 4. Source environment (includes both projects automatically)
source ~/.scrimmage/setup.bash

# 5. Run custom missions
scrimmage ~/projects/MyDrones/missions/example.xml
```

## CI / Container Workflow (Using `ghcr.io/gtri/scrimmage-24.04`)

When using a pre-built SCRIMMAGE image, each project uses its own install tree with `SETUP_LOCAL_CONFIG_DIR=OFF`:

```
/root/scrimmage/build/install/etc/scrimmage/env/scrimmage-setenv  # Base SCRIMMAGE
/root/MyDrones/build/install/etc/MyDrones/env/MyDrones-setenv     # External project
```

### Step 1: Create the External Project (on host)

```bash
# Clone SCRIMMAGE to use the project generator
git clone https://github.com/gtri/scrimmage.git
cd scrimmage

# Create the external project
./scripts/create-scrimmage-project.py MyDrones ~/projects/
```

### Step 2: Start Container with Project Mounted

```bash
xhost +local:docker  # For GUI support on Linux

docker run -it --rm \
  -v ~/projects/MyDrones:/root/MyDrones \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  ghcr.io/gtri/scrimmage-24.04:latest \
  bash
```

### Step 3: Build and Install External Project (inside container)

```bash
cd /root/MyDrones
mkdir -p build && cd build

# Build with SETUP_LOCAL_CONFIG_DIR=OFF (no ~/.scrimmage/ modifications)
cmake .. -DSETUP_LOCAL_CONFIG_DIR=OFF \
         -DCMAKE_INSTALL_PREFIX=$PWD/install \
         -Dscrimmage_DIR=/root/scrimmage/build/install/share/cmake/scrimmage
make -j$(nproc) install

# Source both install trees
source /root/scrimmage/build/install/etc/scrimmage/env/scrimmage-setenv
source /root/MyDrones/build/install/etc/MyDrones/env/MyDrones-setenv

# Run missions
scrimmage /root/MyDrones/missions/example.xml
```

### Key Differences from Bare Metal Workflow

| Aspect | Bare Metal | Container |
|--------|------------|-----------|
| `SETUP_LOCAL_CONFIG_DIR` | `ON` (default) | `OFF` |
| Base SCRIMMAGE setenv | `~/.scrimmage/env/scrimmage-setenv` | `/root/scrimmage/build/install/etc/scrimmage/env/scrimmage-setenv` |
| External project setenv | `~/.scrimmage/env/MyDrones-setenv` | `/root/MyDrones/build/install/etc/MyDrones/env/MyDrones-setenv` |
| Requires `make install` | No | Yes |

---

## Adding New Plugins

Use the plugin templates in `scripts/templates/`:

```
scripts/templates/
├── autonomy/       # Autonomy plugin template
├── controller/     # Controller plugin template
├── motion/         # Motion model template
├── sensor/         # Sensor plugin template
├── interaction/    # Entity interaction template
├── metrics/        # Metrics plugin template
├── network/        # Network plugin template
└── gpu_motion/     # GPU-accelerated motion template
```

Copy and adapt these templates into the overlay project's `src/plugins/` and `include/MyProject/plugins/` directories.