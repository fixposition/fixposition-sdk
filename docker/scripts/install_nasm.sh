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
# Download, build nasm
#
########################################################################################################################
set -eEu


curl -L https://www.nasm.us/pub/nasm/releasebuilds/3.01/nasm-3.01.tar.xz -o /tmp/nasm.tar.gz
echo "f0531dfe9728192fe190cc4932df44a26f1bde7c /tmp/nasm.tar.gz" | sha1sum --check

mkdir /tmp/nasm
cd /tmp/nasm
tar --strip-components=1 -xvf ../nasm.tar.gz

./configure --prefix=/usr/local
make -j4
make install

cd /
rm -rf /tmp/nasm.tar.gz /tmp/nasm


########################################################################################################################
