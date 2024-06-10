#!/bin/bash
set -eEu
set -o pipefail
set -o errtrace

if [ $(id -u) -eq 0 ]; then
    echo "Thou shalt not sudo!"
    exit 1
fi
if ! id -nG | grep -qw docker; then
    echo "You're not in the docker group!"
    exit 1
fi

SCRIPTDIR=$(dirname $(readlink -f $0))
DEBUG=0
IMAGE=ghcr.io/fixposition/fixposition/sdk:ci

function main
{
    # Get command line options
    OPTERR=1
    local asroot=0
    while getopts "hdri:" opt; do
        case $opt in
            r)
                asroot=1
                ;;
            i)
                IMAGE=${OPTARG}
                ;;
            d)
                DEBUG=1
                ;;
            h)
                echo "$0 [-d]   build  |  push  |  [-r] run <command> [<arguments> ...]"
                echo
                exit 0
                ;;
            *)
                error "Illegal option '$opt'!" 1>&2
                exit 1
                ;;
        esac
    done
    if [ ${OPTIND} -gt 1 ]; then
        shift $(expr $OPTIND - 1)
    fi

    notice "Hello"
    debug "SCRIPTDIR=${SCRIPTDIR} IMAGE=${IMAGE} asroot=${asroot}"

    local res=0

    if [ ${res} -eq 0 -a $# -ge 1 ]; then
        local command=$1
        shift 1
        debug "args: $@"
        case "${command}" in
            # Build docker image
            build)
                if ! do_build; then
                    res=1
                fi
                ;;
            push)
                if ! do_push; then
                    res=1
                fi
                ;;
            run)
                if ! do_run ${asroot} "$@"; then
                    res=1
                fi
                ;;
            *)
                res=1
                ;;
        esac
    fi

    # Happy?
    debug "res=${res}"
    if [ ${res} -eq 0 ]; then
        exit 0
    else
        error "Ouch"
        exit 1
    fi
}

function do_build
{
    cd ${SCRIPTDIR}
    if docker build -t ${IMAGE} -f Dockerfile .; then
        return 0
    else
        return 1
    fi
}

function do_push
{
    cd ${SCRIPTDIR}
    if docker push ${IMAGE}; then
        return 0
    else
        return 1
    fi
}

function do_run
{
    local asroot=$1
    shift 1

    local flags=""
    flags="${flags} -it"
    flags="${flags} --rm"
    if [ ${asroot} -gt 0 ]; then
        flags="${flags} --user root"
    else
        flags="${flags} --user $(id -u):$(id -g)"
    fi
    flags="${flags} --network host --dns 172.22.0.1"
    flags="${flags} --mount type=bind,src=${SCRIPTDIR},dst=${SCRIPTDIR}"
    flags="${flags} --workdir ${SCRIPTDIR}"
    if docker run ${flags} ${IMAGE} "$@"; then
        return 0
    else
        return 1
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
