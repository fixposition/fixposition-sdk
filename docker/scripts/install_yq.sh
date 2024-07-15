#!/bin/bash
set -eEu
####################################################################################################
# Download and install yq

curl -o /usr/local/bin/yq \
    -L https://github.com/mikefarah/yq/releases/download/v4.44.2/yq_linux_amd64
chmod 0755 /usr/local/bin/yq

####################################################################################################
