#!/bin/bash
set -eEu
####################################################################################################
# Install pre-commit hooks (for user running the script)

tmpdir=$(mktemp -d)
cd ${tmpdir}
git init
cp /tmp/.pre-commit-config.yaml .
pre-commit install-hooks
cd /
rm -rf ${tmpdir}

####################################################################################################
