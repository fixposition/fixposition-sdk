#!/bin/bash
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
# Add user for devcontainer, see ../.devcontainer/devcontainer.json
#
########################################################################################################################
set -eEu
#
# We deliberately use UID and GID 1000 as that is typically the first user on any (Debian/Ubuntu)
# system. Since we want to use vscode's devcontainer remoteUser/updateRemoteUserUID feature, we want
# to make sure that the the host system's user UID/GID (which very likely is 1000:1000, too) can be
# mapped to the fpsdk user inside the container. However, the base image (hello, Ubuntu Noble!) may
# already have a "user 1000", in which case we wipe that one and take its place.

if id 1000; then
    deluser --remove-home $(id -un 1000)
fi

addgroup --gid 1000 fpsdk
adduser --uid 1000 --gid 1000 fpsdk

echo "fpsdk ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/fpsdk
chmod 0440 /etc/sudoers.d/fpsdk

########################################################################################################################
