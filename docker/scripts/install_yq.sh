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
# Download and install yq
#
########################################################################################################################
set -eEu

curl -L https://github.com/mikefarah/yq/releases/download/v4.44.2/yq_linux_amd64 -o /tmp/yq
echo "8f083b9e8dc7898321fc1499e310dc8c73bfda69 /tmp/yq" | sha1sum --check

install -m 0755 /tmp/yq /usr/local/bin/yq
rm -f /tmp/yq

########################################################################################################################
