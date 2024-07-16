#!/bin/bash
set -eEu
####################################################################################################
# Add user for devcontainer, see ../.devcontainer/devcontainer.json

adduser fpsdk

echo "fpsdk ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/fpsdk
chmod 0440 /etc/sudoers.d/fpsdk

####################################################################################################
