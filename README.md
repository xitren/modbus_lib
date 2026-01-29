# modbus_lib

[![Coverage](https://github.com/xitren/modbus_lib/actions/workflows/cmake-multi-platform.yml/badge.svg?branch=main)](https://github.com/xitren/modbus_lib/actions/workflows/cmake-multi-platform.yml)

## Build and test in Ubuntu devcontainer

Use the Ubuntu devcontainer (`.devcontainer/devcontainer.ubuntu.json`) to
build and run tests in a glibc-based environment.

### VS Code (Dev Containers)

1. Open this repository in VS Code.
2. Run `Dev Containers: Open Folder in Container...`.
3. Select the configuration `Simplified env (Ubuntu/glibc for sanitizers)`.
4. After the container starts, open a terminal in the container and run:

```bash
cmake -S . -B build -G Ninja -DBUILD_TESTS=ON
cmake --build build
ctest --test-dir build
```

### CLI (docker build/run)

If you prefer plain Docker, you can build the Ubuntu devcontainer image and
use it to run the same commands:

```bash
docker build -f .devcontainer/Dockerfile.ubuntu -t modbus-lib-ubuntu .
docker run --rm -it -v "$(pwd):/home" -w /home modbus-lib-ubuntu bash
```

Then, inside the container:

```bash
cmake -S . -B build -G Ninja -DBUILD_TESTS=ON
cmake --build build
ctest --test-dir build
```