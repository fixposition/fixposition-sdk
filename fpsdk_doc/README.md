# Fixposition SDK

[![CI](https://github.com/fixposition/fixposition_utility/actions/workflows/ci.yml/badge.svg)](https://github.com/fixposition/fixposition_utility/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/Documentation-781808)](https://fixposition.github.io/fixposition_utility/fixposition-sdk-docs)

---
## Overview

- [fpsdk_common](../fpsdk_common/README.md) c++ library with common functions
- [fpsdk_ros1](../fpsdk_ros1/README.md) c++ library with common ROS 1 functions
- [fpsdk_ros2](../fpsdk_ros2/README.md) c++ library with common ROS 2 functions
- [fpsdk_apps](../fpsdk_apps/README.md) various apps and examples using the above libraries
- [ros1_fpsdk_demo](../ros1_fpsdk_demo/README.md) Demo of using the SDK for a ROS1 node
- [ros2_fpsdk_demo](../ros2_fpsdk_demo/README.md) Demo of using the SDK for a ROS2 node

---
## Dependencies

For building the libraries and apps:

- **Linux**, GCC, glibc, cmake, bash, etc. (tested with Ubuntu 22.04, Ubuntu 24.04 and Debian Bookworm)
- yaml-cpp        (≥ ?,      tested with 0.6.2)
- boost           (≥ ?,      tested with 0.71.0)
- zlib1g          (≥ ?,      tested with 1.2.11)
- Eigen3          (≥ ?,      tested with 3.3.7)
- Cap'n Proto (*) (≥ 1.0.2,  tested with 1.0.2)
- PROJ        (*) (≥ 9.?.?,  tested with 9.4.1)
- ROS1        (*) (Noetic,   tested with Noetic), or
- ROS2        (*) (Humble,   tested with Humble and Jazzy)

(*) Optional dependencies. Without these some functionality will be unavailable in the built libraries and apps.

For development additionally:

- clang-format (≥ 17, tested with 17)
- Doxygen      (≥ 1.11.0, tested with 1.11.0)
- GTest        (≥ 1.12.0, tested with 1.12.1 and 1.13.0)

<img src="fpsdk-overview.drawio.svg" width="400">

---
## Building

> *tl;dr*
> ```sh
> ./docker/docker.sh pull noetic-dev       # Or "docker.sh build noetic-dev" to build the image locally
> ./docker/docker.sh run noetic-dev bash
> # Now inside Docker do:
> make install
> ./fpsdk/bin/fpltool -h
> ```

### VSCode devcontainer

Open the fpsdk.code-workspace, change to one of the provided devcontainers, and in a terminal do:

```sh
make install
./fpsdk/bin/fpltool
```

### Command-line "devcontainer"

```sh
docker pull ghcr.io/fixposition/fixposition-sdk:noetic-dev
./docker.sh run bash
source /opt/ros/noetic/setup.bash
make install
./fpsdk/bin/fpltool
```

To run all CI:

```sh
./docker/docker.sh run noetic-ci ./docker/ci.sh
```

### Manually

This details the manual setup of the dependencies and building the SDK on a ROS1 system (e.g., Ubuntu 20.04 with ROS
Noetic). It works similarly for ROS2 (e.g., Ubuntu 22.04 with ROS Humble) or non-ROS (e.g., Debian Bookworm) based
systems. Refer to the [Docker configration files and scripts](./docker) on installing the required dependencies.


1. Setup build system, install dependencies

    The exact steps required depend on your system. You'll need the dependencies mentioned above installed system wide
    or otherwise tell CMake where to find them.

    ```sh
    sudo apt install  libyaml-cpp-dev libboost-all-dev zlib1g-dev libeigen3-dev                     # For building...
    sudo apt install libgtest-dev clang-format doxygen pre-commit                                   # For development...
    source /opt/ros/noetic/setup.bash                                                               # If you have ROS1...
    ```

3. Configure

    ```sh
    cmake -B build -DCMAKE_INSTALL_PREFIX=~/fpsdk
    ```

    Additional parameters include (see CMakeList.txt files of the projects for details):

    - Build type: `-DCMAKE_BUILD_TYPE=Debug` or `-DCMAKE_BUILD_TYPE=Release` (default)
    - Force ROS1 package path: `-DROS_PACKAGE_PATH=/path/to/ros` (default: auto-detect)
    - Explicitly enable or disable testing: `-DBUILD_TESTING=OFF` or `-DBUILD_TESTING=ON`. Default is to automatically
      enable testing if a suitable GTest library is found.
    - Explicitly enable or disable use of the PROJ library: `-DUSE_PROJ=ON` or `-DUSE_PROJ=OFF`. Default is to
      automatically use the PROJ library if a suitable version is found
    - Explicitly enable or disable use of the CapnProto library: `-DUSE_CAPNP=ON` or `-DUSE_CAPNP=OFF`. Default is to
      automatically use the CapnProto library if a suitable version is found

4. Build

    ```sh
    cmake --build build
    ```

5. Install

    ```sh
    cmake --install build
    ```

6. Enjoy!

    For example:

    ```sh
    ~/fpsdk/bin/fpltool -h
    ```

Alternatively, you can do `catkin build fpsdk_apps` in your ROS workspace.

The packages can be built individually. See the instructions in the [fpsdk_common](fpsdk_common/README.md),
[fpsdk_ros1](fpsdk_ros1/README.md) and [fpsdk_apps](fpsdk_apps/README.md) packages.

Refer to the CI workflow configuration ([ci.yaml](./.github/workflows/ci.yml)) and the CI script
([ci.sh](./docker/ci.sh)) for more options, such as different build configurations or running tests.


## Documentation

```sh
make doc INSTALL_PREFIX=dummy BUILD_TYPE=Release
(cd build/Release/doc && python -m http.server 8000)
```

---
## License

Unless otherwise noted, the content of this project is licensed under the MIT License.
See the [LICENSE](LICENSE) file for details.
