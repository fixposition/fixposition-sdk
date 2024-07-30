#!/bin/bash
set -eEu
####################################################################################################
# Download and install mcap

curl -L https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.47/mcap-linux-amd64 -o /tmp/mcap
echo "0ebddf1ed63c68a2784d7bc47511e2845e2fa18e /tmp/mcap" | sha1sum --check

install -m 0755 /tmp/mcap /usr/local/bin/mcap
rm -f /tmp/mcap

####################################################################################################
