#!/bin/bash
set -eEu
####################################################################################################
# Download, build and install googletest

curl -L https://github.com/google/googletest/archive/refs/tags/v1.13.0.tar.gz -o /tmp/gtest.tar.gz
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

####################################################################################################
