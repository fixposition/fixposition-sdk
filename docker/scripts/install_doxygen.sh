#!/bin/bash
set -eEu
####################################################################################################
# Download, build and install Doxygen

curl -L https://www.doxygen.nl/files/doxygen-1.11.0.src.tar.gz -o /tmp/doxygen.tar.gz
mkdir /tmp/doxygen
cd /tmp/doxygen
tar --strip-components=1 -xzvf ../doxygen.tar.gz
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build --parallel 4
cmake --install build
cd /
rm -rf /tmp/doxygen.tar.gz /tmp/doxygen

####################################################################################################
