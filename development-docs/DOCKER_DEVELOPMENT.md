# Docker Development Guide

## VS Code Dev Container

Open the repository in VS Code and click "Reopen in Container" when prompted, or run "Dev Containers: Reopen in Container" from the Command Palette.

This automatically configures clangd intellisense and X11 forwarding (Linux).

Build and run:
```bash
cd /root/scrimmage && mkdir -p build && cd build
cmake .. && make -j$(nproc) && make install
source ~/.scrimmage/setup.bash
scrimmage /root/scrimmage/missions/straight-no-gui.xml
```

## Manual Docker

### Prerequisites

- Docker installed
- SCRIMMAGE repository cloned locally

### Build the Image

```bash
cd /path/to/scrimmage
docker build -f ci/dockerfiles/ubuntu-24.04-slim-dependency-only -t scrimmage-dev:24.04 .
```

### Run the Container

```bash
docker run -it --rm \
  -v /path/to/scrimmage:/root/scrimmage \
  scrimmage-dev:24.04 \
  bash
```

For GUI support on Linux, enable X11 forwarding:
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
cmake .. && make -j$(nproc) && make install
source ~/.scrimmage/setup.bash
scrimmage /root/scrimmage/missions/straight-no-gui.xml
```

### Clangd Setup

Generate `compile_commands.json` for intellisense:
```bash
cd /root/scrimmage/build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
```
