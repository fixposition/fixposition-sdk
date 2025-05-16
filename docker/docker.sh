#!/bin/bash
########################################################################################################################
# ___    ___
# \  \  /  /
#  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
#  /  /\  \   License: see the LICENSE file
# /__/  \__\
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
    local asroot=0
    while getopts "hdr" opt; do
        case $opt in
            r)
                asroot=1
                ;;
            d)
                DEBUG=1
                ;;
            h)
                echo "$0 [-d]  build  [<service>]"
                echo "$0 [-d]  pull   [<service>]"
                echo "$0 [-d]  push   [<service>]"
                echo "$0 [-d]  run     <service>  <command> [<arguments> ...]"
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

    # Need an action
    local action=
    if [ $# -lt 1 ]; then
        exit_fail "Missing arguments"
    else
        action=$1
        shift 1
    fi

    # Service
    local service=
    if [ $# -gt 0 ]; then
        service=$1
        shift 1
    fi

    debug "SCRIPTDIR=${SCRIPTDIR} asroot=${asroot} action=${action} service=${service} args=$@"

    local res=0

    case "${action}" in
        # Build docker image
        build)
            if ! do_build ${service}; then
                res=1
            fi
            ;;
        pull)
            if ! do_pull ${service}; then
                res=1
            fi
            ;;
        push)
            if ! do_push ${service}; then
                res=1
            fi
            ;;
        run)
            if [ -n "${service}" ]; then
                if ! do_run ${service} ${asroot} "$@"; then
                    res=1
                fi
            else
                exit_fail "Missing arguments"
            fi
            ;;
        *)
            res=1
            ;;
    esac

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
    local service=${1:-}
    cd ${SCRIPTDIR}
    local args=
    if [ ${DEBUG} -gt 0 ]; then
        args="${args} --progress plain"
    fi
    if docker compose build ${args} ${service}; then
        return 0
    else
        return 1
    fi
}

function do_pull
{
    local service=${1:-}
    cd ${SCRIPTDIR}
    if docker compose pull ${service}; then
        return 0
    else
        return 1
    fi
}

function do_push
{
    local service=${1:-}
    cd ${SCRIPTDIR}
    if docker compose push ${service}; then
        return 0
    else
        return 1
    fi
}

function do_run
{
    local service=$1
    local asroot=$2
    shift 2

    local flags=""
    flags="${flags} -it"
    flags="${flags} --rm"
    if [ ${asroot} -gt 0 ]; then
        flags="${flags} --user root"
    else
        flags="${flags} --user $(id -u):$(id -g)"
    fi

    local hostname=fpsdk-${service}-$(id -un)
    mkdir -p /tmp/home-${hostname}
    flags="${flags} --hostname ${hostname} --network host"
    flags="${flags} --mount type=bind,src=${SCRIPTDIR}/..,dst=${SCRIPTDIR}/.."
    flags="${flags} --mount type=bind,src=/tmp/home-${hostname},dst=/tmp/home-${hostname}"
    flags="${flags} --env HOME=/tmp/home-${hostname}"
    flags="${flags} --workdir ${SCRIPTDIR}/.."
    if docker run ${flags} ghcr.io/fixposition/fixposition-sdk:${service} "$@"; then
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

########################################################################################################################
