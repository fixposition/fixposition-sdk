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
# Download, build and install clang-format
#
########################################################################################################################
set -eEu

case ${FPSDK_IMAGE} in
    humble-*)
        distro=jammy
        ;;
    jazzy-*)
        distro=noble
        ;;
    noetic-*)
        distro=focal
        ;;
esac

curl https://apt.llvm.org/llvm-snapshot.gpg.key > /etc/apt/trusted.gpg.d/llvm.asc
echo "deb http://apt.llvm.org/${distro}/ llvm-toolchain-${distro}-19 main" > /etc/apt/sources.list.d/llvm.list
export DEBIAN_FRONTEND=noninteractive
apt-get -y update
apt-get -y --no-install-recommends install \
    clang-format-19
apt-get clean
ln -s /usr/bin/clang-format-19 /usr/bin/clang-format
ln -s /usr/bin/clang-format-diff-19 /usr/bin/clang-format-diff

########################################################################################################################
