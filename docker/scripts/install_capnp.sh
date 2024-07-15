#!/bin/bash
set -eEu
####################################################################################################
# Download, build and install capnp

curl -L https://capnproto.org/capnproto-c++-1.0.2.tar.gz -o /tmp/capnp.tar.gz
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

####################################################################################################
