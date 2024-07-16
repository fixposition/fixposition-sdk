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
# For running it locally via docker.sh...
else
    FPSDK_SRC_DIR=${SCRIPTDIR}/..
fi
echo "FPSDK_SRC_DIR=${FPSDK_SRC_DIR}"

# ----------------------------------------------------------------------------------------------------------------------

NERRORS=0
NSTEPS=0
declare -A TITLES
function do_step
{
    local func=$1
    local res=0
    echo "::group::${TITLES[$func]}"
    echo "----- ${TITLES[$func]} -----"
    ((NSTEPS=${NSTEPS} + 1))

    if ! ${func}; then
        res=1
        ((NERRORS=${NERRORS} + 1))
    fi

    echo "::endgroup::"
    if [ ${res} -ne 0 ]; then
        echo "::warning title=${FPSDK_IMAGE} ${func} failed::${TITLES[$func]}"
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
    local buildname=${FPSDK_IMAGE}_toplevel-release-noros

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
    local buildname=${FPSDK_IMAGE}_toplevel-release-noros # re-using build

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
    local buildname=${FPSDK_IMAGE}_toplevel-debug-noros

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
    local buildname=${FPSDK_IMAGE}_toplevel-debug-noros # re-using build

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
    local buildname=${FPSDK_IMAGE}_toplevel-release-noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}

    cmake -B build/${buildname}/fpcommon -S fpcommon \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpcommon || return 1
    cmake --install build/${buildname}/fpcommon || return 1

    cmake -B build/${buildname}/fpapps -S fpapps \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpapps || return 1
    cmake --install build/${buildname}/fpapps || return 1

    install/${buildname}/bin/fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["doxygen_release_noros"]="Doxygen (release, without ROS)"
function doxygen_release_noros
{
    local buildname=${FPSDK_IMAGE}_doxygen-release-noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make doc \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

########################################################################################################################

TITLES["build_toplevel_release_ros1"]="Build top-level project (release, with ROS1)"
function build_toplevel_release_ros1
{
    local buildname=${FPSDK_IMAGE}_toplevel-release-ros1

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
    local buildname=${FPSDK_IMAGE}_toplevel-release-ros1 # re-using build

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
    local buildname=${FPSDK_IMAGE}_toplevel-debug-ros1

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
    local buildname=${FPSDK_IMAGE}_toplevel-debug-ros1 # re-using build

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
    local buildname=${FPSDK_IMAGE}_toplevel-release-noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}

    cmake -B build/${buildname}/fpcommon -S fpcommon \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpcommon || return 1
    cmake --install build/${buildname}/fpcommon || return 1

    cmake -B build/${buildname}/fpros1 -S fpros1 \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpros1 || return 1
    cmake --install build/${buildname}/fpros1 || return 1

    cmake -B build/${buildname}/fpapps -S fpapps \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpapps || return 1
    cmake --install build/${buildname}/fpapps || return 1

    install/${buildname}/bin/fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_catkin_release"]="Build catkin (release, with ROS1)"
function build_catkin_release
{
    local buildname=${FPSDK_IMAGE}_catkin-release

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    mkdir -p build/${buildname}/src
    cd build/${buildname}/src
    ln -s ../../../fpcommon .
    ln -s ../../../fpros1 .
    ln -s ../../../fpapps .
    cd ..
    catkin init || return 1
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release || return 1
    catkin build fpapps || return 1
    source devel/setup.bash || return 1
    fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["doxygen_release_ros1"]="Doxygen (release, with ROS1)"
function doxygen_release_ros1
{
    local buildname=${FPSDK_IMAGE}_doxygen-release-ros1

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
    local buildname=${FPSDK_IMAGE}_toplevel-release-ros2

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
    local buildname=${FPSDK_IMAGE}_toplevel-release-ros2 # re-using build

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
    local buildname=${FPSDK_IMAGE}_toplevel-debug-ros2

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
    local buildname=${FPSDK_IMAGE}_toplevel-debug-ros2 # re-using build

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
    local buildname=${FPSDK_IMAGE}_toplevel-release-noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}

    cmake -B build/${buildname}/fpcommon -S fpcommon \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpcommon || return 1
    cmake --install build/${buildname}/fpcommon || return 1

    # TODO: not working yet
    # cmake -B build/${buildname}/fpros2 -S fpros2 \
    #     -DCMAKE_INSTALL_PREFIX=install/${buildname} \
    #     -DCMAKE_BUILD_TYPE=Release || return 1
    # cmake --build build/${buildname}/fpros2 || return 1
    # cmake --install build/${buildname}/fpros2 || return 1

    cmake -B build/${buildname}/fpapps -S fpapps \
        -DCMAKE_INSTALL_PREFIX=install/${buildname} \
        -DCMAKE_BUILD_TYPE=Release || return 1
    cmake --build build/${buildname}/fpapps || return 1
    cmake --install build/${buildname}/fpapps || return 1

    install/${buildname}/bin/fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

TITLES["build_colcon_release"]="Build colcon (release, with ROS2)"
function build_colcon_release
{
    local buildname=${FPSDK_IMAGE}_catkin-release

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    mkdir -p build/${buildname}/src
    cd build/${buildname}/src
    ln -s ../../../fpcommon .
    ln -s ../../../fpapps .
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
    local buildname=${FPSDK_IMAGE}_doxygen-release-ros2

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
do_step pre_commit_check               || true # continue
do_step build_toplevel_release_noros   || true # continue
do_step test_toplevel_release_noros    || true # continue
do_step build_toplevel_debug_noros     || true # continue
do_step test_toplevel_debug_noros      || true # continue
do_step build_projs_release_noros      || true # continue
do_step doxygen_release_noros          || true # continue

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
    do_step doxygen_release_ros1          || true # continue

# - Or ROS 2
elif [ -n "${ROS_DISTRO}" -a "${ROS_VERSION:-}" = "2" ]; then
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
    do_step doxygen_release_ros2          || true # continue
fi

########################################################################################################################

# Are we happy?
if [ ${NERRORS} -eq 0 ]; then
    echo "::notice TITLES=CI success::Successfully completed ${NSTEPS} steps"
    exit 0
else
    echo "::error TITLES=CI failure::Failed ${NERRORS} of ${NSTEPS} steps"
    exit 1
fi

########################################################################################################################
