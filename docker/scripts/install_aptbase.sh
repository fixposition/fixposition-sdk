#!/bin/bash
set -eEu
####################################################################################################
# Install stuff for the *-base images (minimal dependencies to build stuff)

# List of packages, with filter for the different images we make
packages=$(awk -v filt=${FPSDK_IMAGE%-*} '$1 ~ filt { print $2 }' <<EOF
    noetic.humble.jazzy.bookworm    bison
    noetic.humble.jazzy.bookworm    build-essential
    noetic.humble.jazzy.bookworm    cmake
    noetic.humble.jazzy.bookworm    curl
    noetic.humble.jazzy.bookworm    fakeroot
    noetic.humble.jazzy.bookworm    flex
    noetic.humble.jazzy.bookworm    gawk
    noetic.humble.jazzy.bookworm    git
    ..............jazzy.bookworm    gnupg2
    ..............jazzy.bookworm    googletest
    noetic.humble.jazzy.bookworm    graphviz
    noetic.humble.jazzy.bookworm    libboost-all-dev                  # This is not small... :-/
    noetic.humble.jazzy.bookworm    libcurl4-openssl-dev
    noetic.humble.jazzy.bookworm    libeigen3-dev
    ..............jazzy.bookworm    libgtest-dev
    noetic.humble.jazzy.bookworm    libsqlite3-dev
    noetic.humble.jazzy.bookworm    libtiff-dev
    noetic.humble.jazzy.bookworm    libyaml-cpp-dev
    noetic.humble.jazzy.bookworm    netbase
    .......humble.jazzy.bookworm    pre-commit
    noetic......................    python3-catkin-tools
    noetic.humble.jazzy.bookworm    python-is-python3
    noetic.humble.jazzy.bookworm    python3-osrf-pycommon
    noetic.humble.jazzy.bookworm    python3-pip
    noetic......................    ros-noetic-eigen-conversions
    noetic......................    ros-noetic-tf
    noetic......................    ros-noetic-tf-conversions
    noetic......................    ros-noetic-tf2-ros
    noetic......................    ros-noetic-tf2-tools
    noetic......................    ros-noetic-tf2-geometry-msgs
    noetic.humble.jazzy.bookworm    sqlite3
    noetic.humble.jazzy.bookworm    sudo
    ..............jazzy.........    unminimize
    noetic.humble.jazzy.bookworm    zlib1g-dev
EOF
)

echo "Installing: ${packages}"

export DEBIAN_FRONTEND=noninteractive

apt-get -y update
apt-get -y --with-new-pkgs upgrade
apt-get -y --no-install-recommends install ${packages}
apt-get -y autoremove
apt-get clean

####################################################################################################
