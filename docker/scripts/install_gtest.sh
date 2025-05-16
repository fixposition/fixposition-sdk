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
# Download, build and install googletest
#
########################################################################################################################
set -eEu

curl -L https://github.com/google/googletest/archive/refs/tags/v1.13.0.tar.gz -o /tmp/gtest.tar.gz
echo "bfa4b5131b6eaac06962c251742c96aab3f7aa78 /tmp/gtest.tar.gz" | sha1sum --check

mkdir /tmp/gtest
cd /tmp/gtest
tar --strip-components=1 -xzvf ../gtest.tar.gz
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_SHARED_LIBS=ON
cmake --build build --parallel 4
cmake --install build
cd /
rm -rf /tmp/gtest.tar.gz /tmp/gtest

########################################################################################################################
