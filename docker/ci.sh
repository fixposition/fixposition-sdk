#!/bin/bash
set -eEu
set -o pipefail
set -o errtrace

SCRIPTDIR=$(dirname $(readlink -f $0))

# ----------------------------------------------------------------------------------------------------------------------

# Variables set in Dockerfile resp. from Github actions
if [ -z "${FPSDK_IMAGE:-}" ]; then
    export FPSDK_IMAGE=other
    if [ -z "${ROS_DISTRO:-}" ]; then
        if [ -d /opt/ros/noetic ]; then
            export ROS_DISTRO=noetic
        elif [ -d /opt/ros/humble ]; then
            export ROS_DISTRO=humble
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
TITLE=
function do_step
{
    local func=$1
    local res=0
    echo "::group::${TITLE}"
    echo "----- ${TITLE} -----"
    ((NSTEPS=${NSTEPS} + 1))

    if ! ${func}; then
        res=1
        ((NERRORS=${NERRORS} + 1))
    fi

    echo "::endgroup::"
    if [ ${res} -eq 0 ]; then
        echo "::notice title=${TITLE}::${FPSDK_IMAGE} ${func} success"
    else
        echo "::warning title=${TITLE}::${FPSDK_IMAGE} ${func} failed"
    fi

    return ${res}
}


########################################################################################################################

function pre_commit_check
{
    TITLE="Pre-commit checks"

    cd ${FPSDK_SRC_DIR}
    pre-commit run --all-files --hook-stage manual || return 1
}

########################################################################################################################

function build_toplevel_release_noros
{
    TITLE="Build top-level project (release, without ROS)"
    local buildname=toplevel-release-noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

function test_toplevel_release_noros
{
    TITLE="Test top-level project (release, without ROS)"
    local buildname=toplevel-release-noros # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

function build_toplevel_debug_noros
{
    TITLE="Build top-level project (debug, without ROS)"
    local buildname=toplevel-debug-noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

function test_toplevel_debug_noros
{
    TITLE="Test top-level project (debug, without ROS)"
    local buildname=toplevel-debug-noros # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

function build_projs_release_noros
{
    TITLE="Build individual projects (release, without ROS)"
    local buildname=toplevel-release-noros

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

function doxygen_release_noros
{
    TITLE="Doxygen (release, without ROS)"
    local buildname=doxygen-release-noros

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make doc \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

########################################################################################################################

function build_toplevel_release_ros1
{
    TITLE="Build top-level project (release, with ROS1)"
    local buildname=toplevel-release-ros1

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

function test_toplevel_release_ros1
{
    TITLE="Test top-level project (release, with ROS1)"
    local buildname=toplevel-release-ros1 # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Release \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

function build_toplevel_debug_ros1
{
    TITLE="Build top-level project (debug, with ROS1)"
    local buildname=toplevel-debug-ros1

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    make install \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
    install/${buildname}/bin/fpltool -V || return 1
}

function test_toplevel_debug_ros1
{
    TITLE="Test top-level project (debug, with ROS1)"
    local buildname=toplevel-debug-ros1 # re-using build

    cd ${FPSDK_SRC_DIR}
    make test \
        INSTALL_PREFIX=install/${buildname} \
        BUILD_TYPE=Debug \
        BUILD_DIR=build/${buildname} || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

function build_projs_release_ros1
{
    TITLE="Build individual projects (release, with ROS1)"

    local buildname=toplevel-release-noros

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

function build_catkin_release
{
    TITLE="Build catkin (release, with ROS1)"
    local buildname=catkin-release

    cd ${FPSDK_SRC_DIR}
    rm -rf build/${buildname}
    mkdir -p build/${buildname}/src
    cd build/${buildname}/src
    ln -s ../../../fpcommon .
    ln -s ../../../fpros1 .
    ln -s ../../../fpapps .
    cd ..
    catkin init || return 1
    catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release || return 1
    catkin build fpapps || return 1
    source devel/setup.bash || return 1
    fpltool -V || return 1
}

# ----------------------------------------------------------------------------------------------------------------------

function doxygen_release_ros1
{
    TITLE="Doxygen (release, with ROS1)"
    local buildname=doxygen-release-ros1

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

# Build ROS stuff last
if [ "${ROS_DISTRO}" = "noetic" ]; then
    echo "===== ROS1 builds ====="
    source /opt/ros/${ROS_DISTRO}/setup.bash

    do_step build_toplevel_release_ros1   || true # continue
    do_step test_toplevel_release_ros1    || true # continue
    do_step build_toplevel_debug_ros1     || true # continue
    do_step test_toplevel_debug_ros1      || true # continue
    do_step build_projs_release_ros1      || true # continue
    do_step build_catkin_release          || true # continue
    do_step doxygen_release_ros1          || true # continue

elif [ "${ROS_DISTRO}" = "humble" ]; then
    echo "===== ROS2 builds ====="
    source /opt/ros/${ROS_DISTRO}/setup.bash

    # TODO...
fi

if [ ${NERRORS} -eq 0 ]; then
    echo "::notice title=CI success::Successfully completed ${NSTEPS} steps"
    exit 0
else
    echo "::error title=CI failure::Failed ${NERRORS} of ${NSTEPS} steps"
fi

########################################################################################################################
