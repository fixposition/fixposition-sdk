#!/bin/bash
set -eEu
####################################################################################################
# Download, build and install clang-format

case ${FPSDK_IMAGE} in
    bookworm-*)
        distro=bookworm
        ;;
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
echo "deb http://apt.llvm.org/${distro}/ llvm-toolchain-${distro}-17 main" > /etc/apt/sources.list.d/llvm.list
export DEBIAN_FRONTEND=noninteractive
apt-get -y update
apt-get -y --no-install-recommends install \
    clang-format-17
apt-get clean
ln -s /usr/bin/clang-format-17 /usr/bin/clang-format
ln -s /usr/bin/clang-format-diff-17 /usr/bin/clang-format-diff

####################################################################################################
