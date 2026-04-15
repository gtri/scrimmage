# (>>>PROJECT_NAME<<<)

This template is intended to sit next to a SCRIMMAGE checkout.

```text
/code/scrimmage
/code/(>>>PROJECT_NAME<<<)
```

## VS Code / Dev Container

The generated project includes:

- `.devcontainer/devcontainer.json` with clangd, CMake Tools, and CodeLLDB
- `.vscode/tasks.json` for configuring and installing SCRIMMAGE plus this overlay project
- `.vscode/launch.json` for debugging the installed `scrimmage` binary against this project's missions and plugins

The devcontainer mounts the sibling SCRIMMAGE repository at `/root/scrimmage` and configures this project against the SCRIMMAGE install tree with `-Dscrimmage_DIR=/root/scrimmage/build/install/share/cmake/scrimmage`.

The visible install tasks in VS Code open a fresh terminal after the install finishes and source both install-tree env files automatically, so you can run `scrimmage` immediately in that shell. There is also an `open sourced shell` task when you want that environment without rebuilding first.

## Build

Build and install SCRIMMAGE first:

```bash
cd /code/scrimmage
mkdir -p build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug \
    -DSETUP_HOME_CONFIG=OFF -DCMAKE_INSTALL_PREFIX=$PWD/install ..
make -j$(nproc) install
```

Then build and install this project:

```bash
cd /code/(>>>PROJECT_NAME<<<)
mkdir -p build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug \
    -DSETUP_LOCAL_CONFIG_DIR=OFF \
    -DCMAKE_INSTALL_PREFIX=$PWD/install \
    -Dscrimmage_DIR=/code/scrimmage/build/install/share/cmake/scrimmage ..
make -j$(nproc) install
```

Source both setenv files before running from a terminal:

```bash
source /code/scrimmage/build/install/etc/scrimmage/env/scrimmage-setenv
source /code/(>>>PROJECT_NAME<<<)/build/install/etc/(>>>PROJECT_NAME<<<)/env/(>>>PROJECT_NAME<<<)-setenv
scrimmage /code/(>>>PROJECT_NAME<<<)/missions/example.xml
```

The launch configurations use the installed binary and installed plugin libraries, so `F5` matches the install-tree workflow.
    
