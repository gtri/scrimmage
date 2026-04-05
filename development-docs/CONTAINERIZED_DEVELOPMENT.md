# Containerized Development Guide

This guide covers all ways to run SCRIMMAGE in containers (Docker, enroot, etc.).

## Quick Reference

| Option | Image Source | Your Code | Use Case |
|--------|-------------|-----------|----------|
| A | Build locally | Mounted | Dev (VS Code devcontainer) |
| B | Pull from GHCR | Baked in | Just run, no dev |
| C | GHCR → enroot | Baked in | HPC/Slurm |

---

## Option A: VS Code Dev Container (Build Locally)

Builds the dependency image from the Dockerfile in this repo.

1. Clone: `git clone -b Ubuntu-24.04 https://github.com/gtri/scrimmage.git`
2. Open folder in VS Code → "Reopen in Container"
3. Build:
   ```bash
   mkdir -p build && cd build
   cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .. && make -j$(nproc)
   ```
4. Run:
   ```bash
   source ~/.scrimmage/setup.bash
   scrimmage missions/straight-no-gui.xml
   ```

This automatically configures clangd intellisense and X11 forwarding (Linux).

---

## Option B: Run Pre-built Image (No Development)

For running scrimmage without development. Everything is baked in.

```bash
docker run -it ghcr.io/gtri/scrimmage-24.04:latest bash
source /root/scrimmage/build/install/etc/scrimmage/env/scrimmage-setenv
scrimmage /root/scrimmage/missions/straight-no-gui.xml
```

For GUI support:
```bash
xhost +local:docker
docker run -it --rm \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  ghcr.io/gtri/scrimmage-24.04:latest bash
```

---

## Option C: HPC with enroot

For Slurm clusters using enroot:

```bash
# Import and create container
enroot import docker://ghcr.io/gtri/scrimmage-24.04:latest
enroot create --name scrimmage-24.04 scrimmage-24.04+latest.sqsh

# Run interactively
enroot start scrimmage-24.04
source /root/scrimmage/build/install/etc/scrimmage/env/scrimmage-setenv
scrimmage /root/scrimmage/missions/straight-no-gui.xml

# Or run a mission directly
enroot start scrimmage-24.04 bash -c "
  source /root/scrimmage/build/install/etc/scrimmage/env/scrimmage-setenv
  scrimmage /root/scrimmage/missions/straight-no-gui.xml
"

# Cleanup
enroot remove scrimmage-24.04
rm scrimmage-24.04+latest.sqsh
```

---

## Manual Docker (without VS Code)

### Build the Image

```bash
cd /path/to/scrimmage
docker build -f ci/dockerfiles/ubuntu-24.04-slim-dependency-only -t scrimmage-dev:24.04 .
```

### Run with Mounted Code

```bash
docker run -it --rm \
  -v /path/to/scrimmage:/root/scrimmage \
  scrimmage-dev:24.04 \
  bash
```

For GUI support on Linux:
```bash
xhost +local:docker
docker run -it --rm \
  -v /path/to/scrimmage:/root/scrimmage \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  scrimmage-dev:24.04 \
  bash
```

### Build and Run

```bash
cd /root/scrimmage && mkdir -p build && cd build
cmake .. && make -j$(nproc)
source ~/.scrimmage/setup.bash
scrimmage /root/scrimmage/missions/straight-no-gui.xml
```

---

## Clangd Setup

Generate `compile_commands.json` for intellisense:
```bash
cd /root/scrimmage/build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
```

---

## Debugging with LLDB (VS Code)

The devcontainer includes LLDB and the CodeLLDB extension for integrated debugging.

### Setup (one-time)

Build with debug symbols:
```bash
cd /root/scrimmage/build
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
make -j$(nproc)
```

Or use the VS Code task: `Ctrl+Shift+P` → "Tasks: Run Task" → "cmake-configure-debug"

### Debug a Mission

1. Set breakpoints in your code (click left gutter)
2. Press `F5` or go to Run → Start Debugging
3. Select "Debug: scrimmage" (uses `missions/straight-no-gui.xml`)

For other missions: use "Debug: scrimmage (pick mission)"

### Debug Tests

1. Open a test file (e.g., `test/test_quaternion.cpp`)
2. Set breakpoints
3. Run "Debug: Current Test File" from the debug dropdown

### Available Debug Configurations

Found in the **debug dropdown** (top of Run and Debug panel, `Ctrl+Shift+D`):

| Config | Description |
|--------|-------------|
| Debug: scrimmage | Run scrimmage with straight-no-gui.xml |
| Debug: scrimmage (pick mission) | Choose from common mission files |
| Debug: Current Test File | Debug test executable matching open file (e.g., `test_quaternion.cpp` → `build/test/test_quaternion`) |
| Debug: Attach to Process | Attach to a running scrimmage process |

### Available Build Tasks

Run via `Ctrl+Shift+P` → "Tasks: Run Task":

| Task | Description |
|------|-------------|
| cmake-configure | Configure with selectable build type |
| cmake-configure-debug | Configure with Debug build type |
| build | Build with parallel jobs (default task) |
| test | Run all tests with ctest |
| clean | Clean build artifacts |
| rebuild | Clean then build |

### Terminal Debugging (without VS Code)

```bash
source ~/.scrimmage/setup.bash
cd /root/scrimmage
lldb build/bin/scrimmage -- missions/straight-no-gui.xml
# step through debug with keybindings
```

---

## Neovim LSP with Docker

Configure clangd to exec into a running container:

```lua
config = {
  clangd = {
    cmd = {
      "docker", "exec", "-i", "<container>",  -- container name/ID
      "clangd", "--background-index",
      "--path-mappings=/home/<user>/scrimmage=/root/scrimmage",  -- local=container
      "--compile-commands-dir=/root/scrimmage/build",
      "--completion-style=bundled", "--header-insertion=iwyu",
    },
  },
}
```
