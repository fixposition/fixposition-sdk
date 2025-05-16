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
# Download, build and install PROJ
#
########################################################################################################################
set -eEu

curl -L https://download.osgeo.org/proj/proj-9.4.1.tar.gz -o /tmp/proj.tar.gz
echo "e4d0d227c183e47b4ba570ff132806e882b49453 /tmp/proj.tar.gz" | sha1sum --check

mkdir /tmp/proj
cd /tmp/proj
tar --strip-components=1 -xzvf ../proj.tar.gz
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF
    cmake --build build --parallel 4
cmake --install build
cd /
rm -rf /tmp/proj.tar.gz /tmp/proj

########################################################################################################################
