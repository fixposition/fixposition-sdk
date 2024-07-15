#!/bin/bash
set -eEu
####################################################################################################
# Download and install mcap

curl -o /usr/local/bin/mcap \
    -L https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.47/mcap-linux-amd64
chmod 0755 /usr/local/bin/mcap

####################################################################################################
