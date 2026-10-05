#!/bin/bash
set -eEu -o pipefail

scriptdir=$(dirname $(realpath $0))

command=${1:-}
workspace=${2:-}
shift 2

# echo "scriptdir=${scriptdir}"
# echo "command=${command}"
# echo "workspace=${workspace}"
# echo "args=$*"

PERSISTENT_DIRS=".local .claude .bash_history.d .vscode-server"

case ${command} in
    initializeCommand)
        # Make the persistent directories (see compose.yaml .services.dev.volumes)
        for dir in ${PERSISTENT_DIRS}; do
            if [ ! -d ${workspace}/.devcontainer/.persist/${dir} ]; then
                echo "Making ${workspace}/.devcontainer/.persist/${dir}"
                mkdir -p ${workspace}/.devcontainer/.persist/${dir}
            fi
        done
        # Thank you Claudine, for using .claude AND .claude.json... :-/
        if [ ! -f ${workspace}/.devcontainer/.persist/.claude.json ]; then
            echo "Making ${workspace}/.devcontainer/.persist/.claude.json"
            echo "{}" > ${workspace}/.devcontainer/.persist/.claude.json
        fi
        ;;
    onCreateCommand)
        id
        ;;
    *)
        echo "bad command: ${command}"
        exit 1
        ;;
esac

exit 0
