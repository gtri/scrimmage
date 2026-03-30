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
