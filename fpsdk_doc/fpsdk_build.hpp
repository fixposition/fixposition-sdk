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

    @section FPSDK_BUILD_DOC_OVERVIEW Overview

    This page describes how the Fixposition SDK can be built and installed. This is intended for users who are familiar
    with developing c++ code and building software and who want to build their own applications using the @ref
    FPSDK_COMMON_DOC.

    For users interested in simply *using* the @ref FPSDK_APPS_DOC it is recommended to use the @ref FPSDK_RUN_DOC
    instead of building and installing those themselves.


    @section FPSDK_BUILD_DEPS Dependencies

    The Fixposition SDK is made for **Linux**. To build it a **GCC** (C++-17) toolchain is required. You may have some
    luck using clang, but you're on your own there. Besides a glibc and standard Linux tools, such as CMake, make, bash,
    xxd, sed, awk, etc. are required to build. This is tested and known to work with Ubuntu 20.04, 22.04, 24.04 and
    26.04, as well as Debian Trixie.

    For building the libraries and apps the following dependencies are used. Some are required and some are optional.

    | Dependency         | Version   | Required | CMake (1)           | Optional use             |
    |--------------------|-----------|----------|---------------------|--------------------------|
    | boost              | ≥ 1.71.0  | required |                     |                          |
    | BZip2              | ≥ 1.0.8   | optional | FPSDK_USE_BZ2       | fpsdk::common::ros1::BagWriter  |
    | curl               | ≥ 7.68.0  | required |                     |                          |
    | Eigen3             | ≥ 3.3.7   | required |                     |                          |
    | yaml-cpp           | ≥ 0.6.2   | required |                     |                          |
    | zlib1g             | ≥ 1.2.11  | required |                     |                          |
    | OpenSSL (libssl)   | ≥ 1.1.x   | required |                     |                          |
    | nlohmann-json3     | ≥ 3.7.3   | required |                     |                          |
    | PROJ               | ≥ 9.4.x   | optional | FPSDK_USE_PROJ      | fpsdk::common::trafo::Transformer |
    | FFmpeg         (2) | = 7.1.x   | optional | FPSDK_USE_FFMPEG    | fpsdk::common::video, @ref FPSDK_APPS_FPLTOOL |
    | ROS1           (3) | Noetic    | optional |                     | fpsdk::common::ros1      |
    | ROS2           (3) | Humble, Jazzy or Lyrical | optional|       | fpsdk::common::ros2, @ref FPSDK_APPS_FPLTOOL |
    | clang-format   (4) | ≥ 19      | optional |                     | for development          |
    | Doxygen        (4) | ≥ 1.14.0  | optional |                     | for development          |
    | GTest          (4) | ≥ 1.13.0  | optional | FPSDK_BUILD_TESTING | for development          |

    (1) These CMake arguments are available to control these dependencies. By default the dependency is automatically
        used when a suitable library is found. To explicitly enable or disable the use of such a dependency one can set
        the CMake argument accordingly. For example: `-DFPSDK_USE_PROJ=OFF` to disable the use of the PROJ library or
        `-DFPSDK_USE_PROJ=ON` to required the use of the PROJ library. See also @ref FPSDK_BUILD_BUILD_MANUAL below.

    (2) The FFmpeg libraries must be configured with --disable-gpl and --disable-nonfree in order to comply
        with the Fixposition SDK license. The CMake tests for that and refuses to use non-free/gpl builds of the FFmpeg
        libraries.

    (3) ROS support is optional, and it's either ROS1 *or* ROS2. To enable, build in a ROS environment using catkin
        resp. colcon and the availability of ROS is detected automatically.

    (4) These are only needed for development. That is, they are not required for building the @ref FPSDK_COMMON_DOC
        and @ref FPSDK_APPS_DOC.

    See @ref FPSDK_BUILD_CIVERSIONS for the versions used in the CI builds.

    <!-- trick doxygen -->

    @page FPSDK_BUILD_DOC

    @section FPSDK_BUILD_BUILD Building

    @subsection FPSDK_BUILD_BUILD_TLDR tl;dr

    @code{sh}
    ./docker/docker.sh pull trixie-dev       # Or "docker.sh build trixie-dev" to build the image locally
    ./docker/docker.sh run trixie-dev bash
    # Now inside Docker do:
    make install
    ./fpsdk/bin/fpltool -h
    @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_BUILD_DEVCONTAINER VSCode devcontainer

    Open the fpsdk.code-workspace, change to one of the provided devcontainers (recommended: trixie), and in a terminal
    do:

    @code{sh}
    make install
    ./fpsdk/bin/fpltool
    @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_BUILD_DOCKER Docker container

    Docker images are provided that include all the dependencies:

    @code{sh}
    ./docker/docker.sh pull trixie-dev       # Or "docker.sh build trixie-dev" to build the image locally
    ./docker/docker.sh run trixie-dev bash
    make install
    ./fpsdk/bin/fpltool
    @endcode

    Note that for the containers with ROS (Noetic, Lyrical, etc.) you'll have to source the ROS stuff. For example:

    @code{sh}
    ./docker/docker.sh pull noetic-dev       # Or "docker.sh build noetic-dev" to build the image locally
    ./docker/docker.sh run noetic-dev bash
    . /opt/ros/noetic/setup.bash
    make install
    ./fpsdk/bin/fpltool
    @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_BUILD_CI Run CI

    @code{sh}
    ./docker/docker.sh run trixie-ci ./docker/ci.sh
    ./docker/docker.sh run noetic-ci ./docker/ci.sh
    ./docker/docker.sh run humble-ci ./docker/ci.sh
    ./docker/docker.sh run jazzy-ci ./docker/ci.sh
    ./docker/docker.sh run lyrical-ci ./docker/ci.sh
    @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_BUILD_MANUAL Manual build

    This details the manual setup of the dependencies and building the SDK on a ROS1 system (for example, Ubuntu 20.04
    with ROS Noetic). It works similarly for ROS2 (for example, Ubuntu 22.04 with ROS Humble) or non-ROS (for example,
    Debian Trixie) based systems. Refer to the Docker configration files and scripts in the docker/ folder on installing
    the required dependencies.

    1. Setup build system, install dependencies

        The exact steps required depend on your system. You'll need the dependencies mentioned above installed system
        wide or otherwise tell CMake where to find them. Refer to the provided Docker configuration and helper scripts,
        namely "install_apt_base.sh", to check which packages can be installed in Ubuntu or Debian.

        Something like this should work:

        @code{sh}
        sudo apt install libyaml-cpp-dev libboost-all-dev zlib1g-dev libeigen3-dev linux-libc-dev xxd   # For building
        sudo apt install libgtest-dev clang-format doxygen pre-commit                                   # For development
        source /opt/ros/lyrical/setup.bash                                                              # If you have ROS2 Lyrical
        @endcode

    3. Configure

        @code{sh}
        cmake -B build -DCMAKE_INSTALL_PREFIX=~/fpsdk
        @endcode

        Additional parameters, such as `-DFPSDK_USE_PROJ=OFF`, can be given. See @ref FPSDK_BUILD_DEPS above.

        The build type can be selected by `-DCMAKE_BUILD_TYPE=Debug` or `-DCMAKE_BUILD_TYPE=Release` (default).
        Add `-DCMAKE_PREFIX_PATH=...` to hint at non-standard installation paths, such as `/path/to/ffmpeg-lgpl`.

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

    Note that if you clone this repository directly to your `ros_workspace/src` directory, you may have to place
    CATKIN_IGNORE resp. COLCON_IGNORE files in some places. For example:

    @code{sh}
    # ROS1 catkin workspace
    touch src/fixposition-sdk/examples/CATKIN_IGNORE                   # Ignore all examples, or
    touch src/fixposition-sdk/examples/parser_intro/CATKIN_IGNORE     # Ignore only this example
    @endcode

    @code{sh}
    # ROS2 colcon workspace
    touch src/fixposition-sdk/examples/COLCON_IGNORE                   # Ignore all examples, or
    touch src/fixposition-sdk/examples/parser_intro/COLCON_IGNORE      # Ignore only this example
    @endcode

    <!-- trick doxygen -->

    @subsection FPSDK_BUILD_DETAILS Details

    Refer to the various CMakeList.txt files, the CI workflow configuration (`.github/workflows/ci.yml`)
    and the CI script (`docker/ci.sh`) for details on how things are done.


    @section FPSDK_BUILD_CIVERSIONS Dependency versions


    The builds using Debian 13 "Trixie" currently use:

    @include fpsdk_common_versions_trixie/fpsdk_common/fpsdk_common_versions.txt


    The builds using ROS Noetic (Ubuntu 20.04.6 LTS "Focal") currently use:

    @include fpsdk_common_versions_noetic/fpsdk_common/fpsdk_common_versions.txt


    The builds using ROS Humble (Ubuntu 22.04 LTS "Jammy") currently use:

    @include fpsdk_common_versions_humble/fpsdk_common/fpsdk_common_versions.txt


    The builds using ROS Jazzy (Ubuntu 24.04 LTS "Noble") currently use:

    @include fpsdk_common_versions_jazzy/fpsdk_common/fpsdk_common_versions.txt


    The builds using ROS Lyrical (Ubuntu 26.04 LTS "Raccoon") currently use:

    @include fpsdk_common_versions_lyrical/fpsdk_common/fpsdk_common_versions.txt
*/

// clang-format on
/* ****************************************************************************************************************** */
}  // namespace fpsdk
#endif  // __FPSDK_BUILD_HPP__
