/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Documentation
 */

#ifndef __FPSDK_BUILD_HPP__
#define __FPSDK_BUILD_HPP__

namespace fpsdk {
/* ****************************************************************************************************************** */
// clang-format off

/*!

    @page FPSDK_BUILD_DOC Fixposition SDK: Build and installation

    @section FPSDK_BUILD_DEPS Dependencies

    For building the libraries and apps:

    - **Linux**, GCC (C++-17), glibc, cmake, bash, etc. (tested with Ubuntu 20.04/22.04/24.04 and Debian Bookworm)
    - boost            (≥ 1.71.0, tested with 1.71.0, 1.74.1, 1.83.0)
    - curl             (≥ 7.68.0, tested with 7.68.0, 7.88.1, 8.5.0)
    - Eigen3           (≥ 3.3.7,  tested with 3.3.7, 3.4.0)
    - yaml-cpp         (≥ 0.6.2,  tested with 0.6.2, 0.7.0, 0.8.0)
    - zlib1g           (≥ 1.2.11, tested with 1.2.11, 1.2.13, 1.3)
    - OpenSSL (libssl) (≥ 1.1.x,  tested with 1.1.1, 3.0.2, 3.0.13, 3.0.15)
    - nlohmann-json3   (≥ 3.7.3,  tested with 3.7.3, 3.11.3)
    - PROJ         (*) (≥ 9.4.x,  tested with 9.4.0, 9.4.1)
    - ROS1         (*) (Noetic), or
    - ROS2         (*) (Humble or Jazzy)

    (*) Optional dependencies. Without these some functionality in the libraries and apps is unavailable (compiled out).

    For development additionally:

    - clang-format (≥ 17, tested with 17)
    - Doxygen      (≥ 1.11.0, tested with 1.11.0)
    - GTest        (≥ 1.12.0, tested with 1.12.1 and 1.13.0)

    <!-- trick doxygen -->

    @page FPSDK_BUILD_DOC

    @section FPSDK_BUILD_BUILD Building

    @subsection FPSDK_BUILD_BUILD_TLDR tl;dr

    @code{sh}
    ./docker/docker.sh pull noetic-dev       # Or "docker.sh build noetic-dev" to build the image locally
    ./docker/docker.sh run noetic-dev bash
    # Now inside Docker do:
    source /opt/ros/noetic/setup.bash
    make install
    ./fpsdk/bin/fpltool -h
    @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_BUILD_DEVCONTAINER VSCode devcontainer

    Open the fpsdk.code-workspace, change to one of the provided devcontainers, and in a terminal do:

    @code{sh}
    make install
    ./fpsdk/bin/fpltool
    @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_BUILD_DOCKER Docker container

    Docker images are provided that include all the dependencies:

    @code{sh}
    ./docker/docker.sh pull noetic-dev       # Or "docker.sh build noetic-dev" to build the image locally
    ./docker/docker.sh run noetic-dev bash
    source /opt/ros/noetic/setup.bash
    make install
    ./fpsdk/bin/fpltool
    @endcode

    Note that the "docker.sh" script does not give you a suitable ROS runtime (or development, playground, ...)
    environment! Its only purpose is to run the CI for this repo and to demonstrate the building here. For a ROS
    run-time environment please setup a docker *container* yourself elsewhere (perhaps using the docker *image* provided
    here). See ROS and Docker documentation and the Internet for help.

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_BUILD_CI Run CI

    @code{sh}
    ./docker/docker.sh run noetic-ci ./docker/ci.sh
    ./docker/docker.sh run humble-ci ./docker/ci.sh
    ./docker/docker.sh run jazzy-ci ./docker/ci.sh
    @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_BUILD_MANUAL Manual build

    This details the manual setup of the dependencies and building the SDK on a ROS1 system (for example, Ubuntu 20.04
    with ROS Noetic). It works similarly for ROS2 (for example, Ubuntu 22.04 with ROS Humble) or non-ROS (for example,
    Debian Bookworm) based systems. Refer to the Docker configration files and scripts in the docker/ folder on
    installing the required dependencies.

    1. Setup build system, install dependencies

        The exact steps required depend on your system. You'll need the dependencies mentioned above installed system
        wide or otherwise tell CMake where to find them.

        @code{sh}
        sudo apt install libyaml-cpp-dev libboost-all-dev zlib1g-dev libeigen3-dev linux-libc-dev       # For building
        sudo apt install libgtest-dev clang-format doxygen pre-commit                                   # For development
        source /opt/ros/noetic/setup.bash                                                               # If you have ROS1
        @endcode

    3. Configure

        @code{sh}
        cmake -B build -DCMAKE_INSTALL_PREFIX=~/fpsdk
        @endcode

        Additional parameters include (see CMakeList.txt files of the projects for details):

        - Build type: `-DCMAKE_BUILD_TYPE=Debug` or `-DCMAKE_BUILD_TYPE=Release` (default)
        - Force ROS1 package path: `-DROS_PACKAGE_PATH=/path/to/ros` (default: auto-detect)
        - Explicitly enable or disable testing: `-DBUILD_TESTING=OFF` or `-DBUILD_TESTING=ON`.
          The default is to automatically enable testing if a suitable GTest library is found.
          Note that the instead of the standard BUILD_TESTING variable the variable `FPSDK_BUILD_TESTING=...`
          can be used.
        - Explicitly enable or disable use of the PROJ library: `-DUSE_PROJ=ON` or `-DUSE_PROJ=OFF`.
          The default is to automatically use the PROJ library if a suitable version is found

    4. Build

        @code{sh}
        cmake --build build
        @endcode

    5. Install

        @code{sh}
        cmake --install build
        @endcode

    6. Enjoy!

        For example:

        @code{sh}
        ~/fpsdk/bin/fpltool -h
        @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_DOC_DOXYGEN Documentation

    @code{sh}
    make doc
    @endcode


    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_PACKAGE Individual packages

    For example to build the fpsdk_common package (library):

    @code{sh}
    cmake -B build -S fpsdk_common -DCMAKE_INSTALL_PREFIX=~/fpsdk
    cmake --build build
    cmake --install build
    @endcode


    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_ROS ROS workspace

    The packages build in a ROS workspace using catkin (ROS1) resp. colcon (ROS2). For example:

    @code{sh}
    catkin build fpsdk_common
    @endcode

    Note that if you clone this repository directly to your `ros_workspace/src` directory, you'll have to place
    CATKIN_IGNORE resp. COLCON_IGNORE files in some places. For example:

    @code{sh}
    # ROS1 catkin workspace
    touch src/fixposition-sdk/examples/CATKIN_IGNORE                   # Ignore all examples, or
    touch src/fixposition-sdk/examples/ros1_fpsdk_demo/CATKIN_IGNORE   # Ignore only this example
    touch src/fixposition-sdk/fpsdk_ros2/CATKIN_IGNORE
    @endcode

    @code{sh}
    # ROS2 colcon workspace
    touch src/fixposition-sdk/examples/COLCON_IGNORE                   # Ignore all examples, or
    touch src/fixposition-sdk/examples/ros2_fpsdk_demo/COLCON_IGNORE   # Ignore only this example
    touch src/fixposition-sdk/fpsdk_ros1/COLCON_IGNORE
    @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_DETAILS Details

    Refer to the various CMakeList.txt files, the CI workflow configuration (`.github/workflows/ci.yml`)
    and the CI script (`docker/ci.sh`) for details on how things are done.



*/

// clang-format on
/* ****************************************************************************************************************** */
}  // namespace fpsdk
#endif  // __FPSDK_BUILD_HPP__
