# (>>>PROJECT_NAME<<<)

This template is intended to sit next to a SCRIMMAGE checkout.

```text
/code/scrimmage
/code/(>>>PROJECT_NAME<<<)
```

## VS Code / Dev Container

The generated project includes:

- `.devcontainer/devcontainer.json` with clangd, CMake Tools, and CodeLLDB
- `.vscode/tasks.json` for configuring and building SCRIMMAGE plus this overlay project
- `.vscode/launch.json` for debugging `scrimmage` against this project's missions and plugins

The devcontainer mounts the sibling SCRIMMAGE repository at `/root/scrimmage` and configures this project against the SCRIMMAGE build tree with `-Dscrimmage_DIR=/root/scrimmage/build`.

## Build

Build SCRIMMAGE first:

```bash
cd /code/scrimmage
mkdir -p build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```

Then build this project:

```bash
cd /code/(>>>PROJECT_NAME<<<)
mkdir -p build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug \
    -Dscrimmage_DIR=/code/scrimmage/build ..
make -j$(nproc)
```

If you prefer to use an install tree instead of the SCRIMMAGE build tree, point `scrimmage_DIR` at the installed `share/cmake/scrimmage` directory and update the devcontainer/task settings to match.
    
