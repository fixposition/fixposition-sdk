#!/bin/bash
set -eEu
set -o pipefail
set -o errtrace

SCRIPTDIR=$(dirname $(readlink -f $0))

# ----------------------------------------------------------------------------------------------------------------------

# Variables set in our Dockerfiles. Try to make it work in other environments.
if [ -z "${FPSDK_IMAGE:-}" ]; then
    export FPSDK_IMAGE=other
    if [ -z "${ROS_DISTRO:-}" ]; then
        if [ -d /opt/ros/noetic ]; then
            export ROS_DISTRO=noetic
        elif [ -d /opt/ros/humble ]; then
            export ROS_DISTRO=humble
        elif [ -d /opt/ros/jazzy ]; then
            export ROS_DISTRO=jazzy
        else
            export ROS_DISTRO=
        fi
    fi

    echo "Not using a Fixposition SDK Docker image (ROS_DISTRO=${ROS_DISTRO})"

else
    echo "Using Fixposition SDK Docker image (FPSDK_IMAGE=${FPSDK_IMAGE}, ROS_DISTRO=${ROS_DISTRO})"
fi

# Sources will be here
if [ -n "${GITHUB_WORKSPACE:-}" ]; then
    FPSDK_SRC_DIR=${GITHUB_WORKSPACE}
    CMAKE_BUILD_ARGS="--parallel 4"
# For running it locally via docker.sh...
else
    FPSDK_SRC_DIR=${SCRIPTDIR}/..
    CMAKE_BUILD_ARGS="--parallel $(nproc --ignore=2)"
fi
echo "FPSDK_SRC_DIR=${FPSDK_SRC_DIR}"

# Optional single command-line argument to ci.sh to select only one (or some) step(s)
STEP_FILT=${1:-}

# Unset the ROS variables used to detect ROS (compare fpsdk_common/cmake/setup.cmake), so that we can run (the non-ROS
# builds in) this script in a ROS devcontainer, too. We'll re-load the ROS environment later (below) based on
# $ROS_DISTRO from the image ENV variables resp. set above.
unset ROS_VERSION
unset ROS_PACKAGE_PATH

# ----------------------------------------------------------------------------------------------------------------------

ERROR_COUNT=0
ERROR_NAMES=
NSTEPS=0
declare -A TITLES
function do_step
{
    local func=$1

    # Skip?
    if [[ ! ${func} =~ ${STEP_FILT} ]]; then
        echo "Skip ${TITLES[$func]}"
        return 0
    fi

    local res=0
    echo "::group::${TITLES[$func]} ($func)"
    echo "----- $func: ${TITLES[$func]} -----"
    ((NSTEPS=${NSTEPS} + 1))

    if ! ${func}; then
        res=1
        ((ERROR_COUNT=${ERROR_COUNT} + 1))
        ERROR_NAMES="${ERROR_NAMES} ${func}"
    fi

    echo "::endgroup::"
    if [ ${res} -ne 0 ]; then
        echo "::warning title=${FPSDK_IMAGE} ${func} failed::${TITLES[$func]} ($func)"
    fi

    return ${res}
}


########################################################################################################################

TITLES["pre_commit_check"]="Pre-commit checks"
function pre_commit_check
{
    cd ${FPSDK_SRC_DIR}
    make pre-commit
}

########################################################################################################################

TITLES["build_toplevel_release_noros"]="Build top-level project (release, without ROS)"
function build_toplevel_release_noros
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_release_noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

TITLES["test_toplevel_release_noros"]="Test top-level project (release, without ROS)"
function test_toplevel_release_noros
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_release_noros # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_toplevel_debug_noros"]="Build top-level project (debug, without ROS)"
function build_toplevel_debug_noros
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_debug_noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

TITLES["test_toplevel_debug_noros"]="Test top-level project (debug, without ROS)"
function test_toplevel_debug_noros
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_debug_noros # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_projs_release_noros"]="Build individual projects (release, without ROS)"
function build_projs_release_noros
{
    local buildname=${FPSDK_IMAGE}_build_projs_release_noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}

    cmake -B build/${buildname}/fpsdk_common -S fpsdk_common \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpsdk_common ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_common || return 1

    cmake -B build/${buildname}/fpsdk_apps -S fpsdk_apps \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpsdk_apps ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_apps || return 1

    install/${buildname}/bin/fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["doxygen_release_noros"]="Doxygen (release, without ROS)"
function doxygen_release_noros
{
    local buildname=${FPSDK_IMAGE}_doxygen_release_noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make doc \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_toplevel_release_clang"]="Build top-level project (release, clang instead of gcc)"
function build_toplevel_release_clang
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_release_clang

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        CC=clang CXX=clang++ \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

TITLES["build_toplevel_debug_clang"]="Build top-level project (debug, clang instead of gcc)"
function build_toplevel_debug_clang
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_debug_clang

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        CC=clang CXX=clang++ \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

########################################################################################################################

TITLES["build_toplevel_release_noros_mindeps"]="Build top-level project (release, without ROS, minimal deps)"
function build_toplevel_release_noros_mindeps
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_release_noros_mindeps

    cmake -B build/${buildname} -S . \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release \
        -DFPSDK_USE_PROJ=OFF || return 1
    cmake --build build/${buildname} ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname} || return 1

    install/${buildname}/bin/fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_projs_release_noros_mindeps"]="Build individual projects (release, without ROS, minimal deps)"
function build_projs_release_noros_mindeps
{
    local buildname=${FPSDK_IMAGE}_build_projs_release_noros_mindeps

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}

    cmake -B build/${buildname}/fpsdk_common -S fpsdk_common \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release \
        -DFPSDK_USE_PROJ=OFF || return 1
    cmake --build build/${buildname}/fpsdk_common ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_common || return 1

    cmake -B build/${buildname}/fpsdk_apps -S fpsdk_apps \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpsdk_apps ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_apps || return 1

    install/${buildname}/bin/fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_examples"]="Build examples"
function build_examples
{
    local ok=0
    # Note: the fpsdk_ros[12]_demo examples are built elsewhere
    cd ${FPSDK_SRC_DIR}/examples
    make build -k

    return ${ok}
}

########################################################################################################################

TITLES["build_toplevel_release_ros1"]="Build top-level project (release, with ROS1)"
function build_toplevel_release_ros1
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_release_ros1

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

TITLES["test_toplevel_release_ros1"]="Test top-level project (release, with ROS1)"
function test_toplevel_release_ros1
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_release_ros1 # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_toplevel_debug_ros1"]="Build top-level project (debug, with ROS1)"
function build_toplevel_debug_ros1
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_debug_ros1

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

TITLES["test_toplevel_debug_ros1"]="Test top-level project (debug, with ROS1)"
function test_toplevel_debug_ros1
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_debug_ros1 # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_projs_release_ros1"]="Build individual projects (release, with ROS1)"
function build_projs_release_ros1
{
    local buildname=${FPSDK_IMAGE}_build_projs_release_ros1

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}

    cmake -B build/${buildname}/fpsdk_common -S fpsdk_common \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpsdk_common ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_common || return 1

    cmake -B build/${buildname}/fpsdk_ros1 -S fpsdk_ros1 \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpsdk_ros1 ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_ros1 || return 1

    cmake -B build/${buildname}/fpsdk_apps -S fpsdk_apps \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpsdk_apps ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_apps || return 1

    install/${buildname}/bin/fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_catkin_release"]="Build catkin (release, with ROS1)"
function build_catkin_release
{
    local buildname=${FPSDK_IMAGE}_build_catkin_release

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    mkdir -p build/${buildname}/src
    cd build/${buildname}/src
    ln -s ../../../fpsdk_common .
    ln -s ../../../fpsdk_ros1 .
    ln -s ../../../fpsdk_apps .
    ln -s ../../../examples/ros1_fpsdk_demo .
    cd ..
    catkin init || return 1
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release || return 1
    catkin build || return 1
    source devel/setup.bash || return 1
    fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["doxygen_release_ros1"]="Doxygen (release, with ROS1)"
function doxygen_release_ros1
{
    local buildname=${FPSDK_IMAGE}_doxygen_release_ros1

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make doc \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

########################################################################################################################

TITLES["build_toplevel_release_ros2"]="Build top-level project (release, with ROS2)"
function build_toplevel_release_ros2
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_release_ros2

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

TITLES["test_toplevel_release_ros2"]="Test top-level project (release, with ROS2)"
function test_toplevel_release_ros2
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_release_ros2 # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_toplevel_debug_ros2"]="Build top-level project (debug, with ROS2)"
function build_toplevel_debug_ros2
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_debug_ros2

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

TITLES["test_toplevel_debug_ros2"]="Test top-level project (debug, with ROS2)"
function test_toplevel_debug_ros2
{
    local buildname=${FPSDK_IMAGE}_build_toplevel_debug_ros2 # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_projs_release_ros2"]="Build individual projects (release, with ROS2)"
function build_projs_release_ros2
{
    local buildname=${FPSDK_IMAGE}_build_projs_release_ros2

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}

    cmake -B build/${buildname}/fpsdk_common -S fpsdk_common \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpsdk_common ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_common || return 1

    cmake -B build/${buildname}/fpsdk_ros2 -S fpsdk_ros2 \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpsdk_ros2 ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_ros2 || return 1

    cmake -B build/${buildname}/fpsdk_apps -S fpsdk_apps \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpsdk_apps ${CMAKE_BUILD_ARGS} || return 1
    cmake --install build/${buildname}/fpsdk_apps || return 1

    install/${buildname}/bin/fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_colcon_release"]="Build colcon (release, with ROS2)"
function build_colcon_release
{
    local buildname=${FPSDK_IMAGE}_build_colcon_release

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    mkdir -p build/${buildname}/src
    cd build/${buildname}/src
    ln -s ../../../fpsdk_common .
    ln -s ../../../fpsdk_ros2 .
    ln -s ../../../fpsdk_apps .
    ln -s ../../../examples/ros2_fpsdk_demo .
    cd ..
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release || return 1
    set +u
    source install/setup.bash || return 1
    set -u
    fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["doxygen_release_ros2"]="Doxygen (release, with ROS2)"
function doxygen_release_ros2
{
    local buildname=${FPSDK_IMAGE}_doxygen_release_ros2

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make doc \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

########################################################################################################################

# Build stuff without ROS first
echo "===== non-ROS builds ====="
if [ "${FPSDK_IMAGE%-*}" = "trixie" ]; then
    do_step pre_commit_check           || true # continue
fi
do_step build_toplevel_release_noros   || true # continue
do_step test_toplevel_release_noros    || true # continue
do_step build_toplevel_debug_noros     || true # continue
do_step test_toplevel_debug_noros      || true # continue
do_step build_projs_release_noros      || true # continue
if [ "${FPSDK_IMAGE%-*}" = "trixie" ]; then
    do_step doxygen_release_noros      || true # continue
fi
do_step build_toplevel_release_noros_mindeps   || true # continue
do_step build_projs_release_noros_mindeps      || true # continue

do_step build_examples                 || true # continue

# Some minimal checks with clang
echo "===== clang ====="
if [ "${FPSDK_IMAGE%-*}" = "trixie" ]; then
    do_step build_toplevel_release_clang  || true # continue
    do_step build_toplevel_debug_clang    || true # continue
fi

# Build ROS stuff resp. stuff with the ROS environment loaded last
# - Either ROS 1
if [ "${ROS_DISTRO}" = "noetic" ]; then
    echo "===== ROS1 builds ====="
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u

    do_step build_toplevel_release_ros1   || true # continue
    do_step test_toplevel_release_ros1    || true # continue
    do_step build_toplevel_debug_ros1     || true # continue
    do_step test_toplevel_debug_ros1      || true # continue
    do_step build_projs_release_ros1      || true # continue
    do_step build_catkin_release          || true # continue
    #do_step doxygen_release_ros1          || true # continue

# - Or ROS 2
elif [ "${ROS_DISTRO}" = "humble" -o "${ROS_DISTRO}" = "jazzy" ]; then
    echo "===== ROS2 builds ====="
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u

    do_step build_toplevel_release_ros2   || true # continue
    do_step test_toplevel_release_ros2    || true # continue
    do_step build_toplevel_debug_ros2     || true # continue
    do_step test_toplevel_debug_ros2      || true # continue
    do_step build_projs_release_ros2      || true # continue
    do_step build_colcon_release          || true # continue
    #do_step doxygen_release_ros2          || true # continue
fi

########################################################################################################################

# Are we happy?
if [ ${ERROR_COUNT} -eq 0 ]; then
    echo "::notice TITLES=CI success::Successfully completed ${NSTEPS} steps"
    exit 0
else
    echo "::error TITLES=CI failure::Failed ${ERROR_COUNT} of ${NSTEPS} steps: ${ERROR_NAMES}"
    exit 1
fi

########################################################################################################################
