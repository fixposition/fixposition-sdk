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
# Download, build and install Doxygen
#
########################################################################################################################
set -eEu

curl -L https://www.doxygen.nl/files/doxygen-1.11.0.src.tar.gz -o /tmp/doxygen.tar.gz
echo "1668d086dadd5b36f07fb1f0e211e700f3be17a7 /tmp/doxygen.tar.gz" | sha1sum --check

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

########################################################################################################################
