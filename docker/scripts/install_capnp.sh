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
# Download, build and install capnp
#
########################################################################################################################
set -eEu

curl -L https://capnproto.org/capnproto-c++-1.0.2.tar.gz -o /tmp/capnp.tar.gz
echo "a7517fd137ca3a81b11c7dd936708b6759caa7a3 /tmp/capnp.tar.gz" | sha1sum --check

mkdir /tmp/capnp
cd /tmp/capnp
tar --strip-components=1 -xzvf ../capnp.tar.gz
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF
cmake --build build --parallel 4
cmake --install build
cd /
rm -rf /tmp/capnp.tar.gz /tmp/capnp

########################################################################################################################
