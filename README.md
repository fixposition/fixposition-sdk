# Fixposition Utilities and SDK

This is a collection of utilities that may be useful to use with Fixposition products.
The SDK part of this repo refers to C++ libraries and apps.


---
## Utilities

- [Software Update](software_update/README.md)
- [Recording utilities](record/README.md)


---
## SDK

[![CI](https://github.com/fixposition/fixposition_utility/actions/workflows/main.yml/badge.svg)](https://github.com/fixposition/fixposition_utility/actions/workflows/main.yml)

### Overview

- [fpcommon](fpcommon/README.md) c++ library with common functions
- [fpros1](fpros1/README.md) c++ library with common ROS functions
- [fpapps](fpapps/README.md) various apps

### Dependencies

- **Linux**, GCC, glibc, cmake, bash, ... (tested with Ubuntu 20.04 and Debian Bookworm)
- ROS (tested with [noetic-ros-base](https://hub.docker.com/_/ros/))
    - Note that some parts can be built without ROS. However, in this case some functionality will be unavailable
      in the built libraries and apps.
- yaml-cpp
  (tested with 0.6.2 and 0.7.0)
- boost
  (tested with 0.71.0 and 0.74.0)
- zlib1g
  (tested with 1.2.11 and 1.2.13)
- Eigen3
  (tested with 3.3.7)
- GTest
  (tested with 1.13.0)

This should install the necessary dependencies on Ubuntu or Debian:

```sh
apt install build-essential cmake libyaml-cpp-dev libboost-dev libboost-stacktrace-dev zlib1g-dev
```

### Building

> *tl;dr*:
>
> ```sh
> source /opt/ros/noetic/setup.bash
> make install INSTALL_PREFIX=~/fpsdk
> ```

1. Source ROS environment (optional, but required for some functionality)

    ```sh
    source /opt/ros/noetic/setup.bash
    ```

2. Configure

    ```sh
    cmake -B build -DCMAKE_INSTALL_PREFIX=~/fpsdk
    ```

    Additional parameters include: `-DCMAKE_BUILD_TYPE=Debug`, `-DROS_PACKAGE_PATH=/path/to/ros`, `-DBUILD_TESTING=OFF`

3. Build

    ```sh
    cmake --build build
    ```

4. Install

    ```sh
    cmake --install build
    ```

5. Enjoy!

    For example:

    ```sh
    ~/fpsdk/bin/fpltool -h
    ```

Alternatively, you can do `catkin build fpapps` in your ROS workspace.

The packages can be build individually. See the instructions in the [fpcommon](fpcommon/README.md),
[fpros1](fpros1/README.md) and [fpapps](fpapps/README.md) packages.

---
## License

Unless otherwise noted, the content of this project is licensed under the MIT License.
See the [LICENSE](LICENSE) file for details.
