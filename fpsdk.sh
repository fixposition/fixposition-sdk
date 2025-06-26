#!/bin/bash
########################################################################################################################
# ___    ___
# \  \  /  /
#  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
#  /  /\  \   License: see the LICENSE file
# /__/  \__\
#
########################################################################################################################
#
# Script to run fpsdk apps in a temporary container
#
########################################################################################################################
set -eEu
set -o pipefail
set -o errtrace

SCRIPTDIR=$(dirname $(readlink -f $0))
DEBUG=0

function main
{
    # Get command line options
    OPTERR=1
    local image=trixie
    local update=0
    local volume_args=
    local docker_args=
    while getopts ":hduti:v:" opt; do
        case $opt in
            h)
                echo
                echo "Run Fixposition SDK apps"
                echo
                echo "This uses Docker images with pre-built binaries of the Fixposition SDK apps."
                echo
                echo "Usage: $0 [-d] [-u] [-i <image>] [-v <volume> ...] <command> ..."
                echo
                echo "Where:"
                echo
                echo "    -d          Enable debug output (of this script)"
                echo "    -u          Update (pull) the necessary Docker <image>"
                echo "    -t          Executes <command> with docker run --tty"
                echo "    -i <image>  Specifies which Docker image to use (default: trixie). Available images are:"
                echo "                trixie    -- Debian Trixie (no ROS, some functionality not available)"
                echo "                noetic    -- ROS1 Noetic (additional ROS1 functionality available)"
                echo "                humble    -- ROS2 Humble (additional ROS2 functionality available)"
                echo "                jazzy     -- ROS2 Jazzy (additional ROS2 functionality available)"
                echo "    -v <dir>    Additional directory to mount as volumes in docker (see examples below)"
                echo "    <command>   The command to run,"
                echo "    ...         and its arguments (if any)"
                echo
                echo "Examples:"
                echo
                echo "    Show version information:"
                echo
                echo "        $0 fpltool -V"
                echo
                echo "    Extract .fpl to ROS1 .bag file in current directory:"
                echo
                echo "        $0 -i noetic fpltool rosbag some.fpl"
                echo
                echo "    If the .fpl file is in another directory, we must mount that explicitly:"
                echo
                echo "        $0 -i noetic -v ~/Downloads fpltool rosbag ~/Downloads/some.fpl"
                echo
                echo "    User fpltool to extract a .fpl file and then use the parsertool to extract the"
                echo "    NMEA and FP_A messages from the output messages:"
                echo
                echo "        $0 fpltool extract some.fpl"
                echo "        $0 parsertool -f NMEA,FP_A -c some_userio.raw > some_userio.txt"
                echo
                echo "    Run an interactive shell:"
                echo
                echo "        $0 -t -i noetic bash"
                echo
                exit 0
                ;;
            d)
                DEBUG=1
                ;;
            u)
                update=1
                ;;
            t)
                docker_args="--tty"
                ;;
            i)
                image=${OPTARG}
                ;;
            v)
                volume_args="${volume_args} --volume ${OPTARG}:${OPTARG}"
                ;;
            *)
                exit_fail "Illegal option -${OPTARG}!"
                ;;
        esac
    done
    if [ ${OPTIND} -gt 1 ]; then
        shift $(expr $OPTIND - 1)
    fi
    local have_command=0
    if [ $# -gt 0 ]; then
        have_command=1
    fi

    debug "SCRIPTDIR=${SCRIPTDIR} image=${image} docker_args=${docker_args} volume_args=${volume_args} have_command=${have_command} command=$@"

    # Check if we're running inside a container
    # PID 2 will be kthreadd on host Linux, and no kthreadd will be visible in a container
    if ! grep -q kthreadd /proc/2/status 2>/dev/null; then
        error "You cannot run this in a container"
        exit 1
    fi

    # Check for common issues
    if ! command -v docker >/dev/null ]; then
        warning "Docker does not seem to be installed, this probably doesn't work"
    fi
    if [ $(id -u) -eq 0 ]; then
        warning "You probably should not run this as root"
    fi
    if ! id -nG | grep -qw docker; then
        echo "You're not in the docker group, this may not work"
    fi

    local res=0

    # The image (see docker/docker-compose.yaml, which we're not using to let this script to run stand-alone)
    image="ghcr.io/fixposition/fixposition-sdk:${image}-run"
    if ! docker image inspect ${image} >/dev/null 2>&1; then
        update=1
    fi

    # Pull image
    if [ ${update} -gt 0 ]; then
        info "Pulling image ${image}..."
        if ! docker pull ${image}; then
            error "Failed to pull ${image} :-("
            exit 1
        fi
        info "Image is updated"
        if [ ${have_command} -eq 0 ]; then
            exit 0
        fi
    fi

    if [ ${have_command} -eq 0 ]; then
        exit_fail "Need a command"
    fi

    # Docker run command
    local args="${docker_args}"
    # - single-use container, no funny network
    args="${args} --rm --network host"
    # - connect stdin
    args="${args} --interactive"
    # - Mount current directory as /data and run command inside docker from there
    args="${args} --volume ${PWD}:/data --workdir /data"
    # - Additional mounts
    args="${args} ${volume_args}"
    # - Set user, mount passwd and group file
    args="${args} --user $(id -u):$(id -g) --volume /etc/passwd:/etc/passwd:ro --volume /etc/group:/etc/group:ro"


    [ ${DEBUG} -gt 0 ] && set -x
    if ! docker run ${args} ${image} "$@"; then
        res=1
    fi
    [ ${DEBUG} -gt 0 ] && set +x

    # Happy?
    debug "res=${res}"
    if [ ${res} -eq 0 ]; then
        exit 0
    else
        exit 1
    fi
}

function exit_fail
{
    error "$@"
    echo "Try '$0 -h' for help." 1>&2
    exit 1
}

function notice
{
    echo -e "\033[1;37m$@\033[m" 1>&2
}

function info
{
    echo -e "\033[0m$@\033[m" 1>&2
}

function warning
{
    echo -e "\033[1;33mWarning: $@\033[m" 1>&2
}

function error
{
    echo -e "\033[1;31mError: $@\033[m" 1>&2
}

function debug
{
    if [ ${DEBUG} -gt 0 ]; then
        echo -e "\033[0;36mDebug: $@\033[m" 1>&2
    fi
}

function panic
{
    local res=$?
    echo -e "\033[1;35mPanic at ${BASH_SOURCE[1]}:${BASH_LINENO[0]}! ${BASH_COMMAND} (res=$res)\033[m" 1>&2
    exit $res
}

main "$@"
exit 99

########################################################################################################################
