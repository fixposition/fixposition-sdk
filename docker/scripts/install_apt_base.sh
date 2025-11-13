#!/bin/bash
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
# Install stuff for the *-base images (minimal dependencies to build stuff)
#
########################################################################################################################
set -eEu

# List of packages, with filter for the different images we make
packages=$(awk -v filt=${FPSDK_IMAGE%-*} '$1 ~ filt { print $2 }' <<EOF
    noetic.humble.jazzy.trixie      bison
    noetic.humble.jazzy.trixie      build-essential
    .......humble.jazzy.trixie      clang
    ....................trixie      clang-format
    noetic.humble.jazzy.trixie      cmake
    noetic.humble.jazzy.trixie      curl
    noetic.humble.jazzy.trixie      fakeroot
    noetic.humble.jazzy.trixie      flex
    noetic.humble.jazzy.trixie      gawk
    noetic.humble.jazzy.trixie      git
    ..............jazzy.trixie      gnupg2
    noetic.humble.jazzy.trixie      gnuplot
    ..............jazzy.trixie      googletest
    noetic.humble.jazzy.trixie      graphviz
    noetic.humble.jazzy.trixie      jq
    ....................trixie      libbacktrace-dev                  # GCC has its own, but clang needs this
    noetic.humble.jazzy.trixie      libboost-all-dev                  # This is not small... :-/
    noetic.humble.jazzy.trixie      libclone-perl
    noetic.humble.jazzy.trixie      libcurl4-openssl-dev
    noetic.humble.jazzy.trixie      libeigen3-dev
    ..............jazzy.trixie      libgtest-dev
    noetic.humble.jazzy.trixie      libjson-xs-perl
    noetic.humble.jazzy.trixie      libmime-base64-perl
    noetic.humble.jazzy.trixie      libpath-tiny-perl
    ..............jazzy.trixie      libproj25
    ..............jazzy.trixie      libproj-dev
    noetic.humble.jazzy.trixie      libssl-dev
    noetic.humble.jazzy.trixie      libsqlite3-dev
    noetic.humble.jazzy.trixie      libtiff-dev
    noetic.humble.jazzy.trixie      libyaml-cpp-dev
    noetic.humble.jazzy.trixie      netbase
    noetic.humble.jazzy.trixie      nlohmann-json3-dev
    .......humble.jazzy.trixie      pre-commit
    ..............jazzy.trixie      proj-data
    ..............jazzy.trixie      proj-bin
    noetic....................      python3-catkin-tools
    noetic.humble.jazzy.trixie      python-is-python3
    noetic.humble.jazzy.trixie      python3-osrf-pycommon
    noetic.humble.jazzy.trixie      python3-pip
    noetic.humble.jazzy.trixie      python3-venv
    .......humble.............      ros-humble-rosbag2-storage-mcap
    ..............jazzy.......      ros-jazzy-rosbag2-storage-mcap
    noetic....................      ros-noetic-eigen-conversions
    noetic....................      ros-noetic-tf
    noetic....................      ros-noetic-tf-conversions
    noetic....................      ros-noetic-tf2-ros
    noetic....................      ros-noetic-tf2-tools
    noetic....................      ros-noetic-tf2-geometry-msgs
    noetic.humble.jazzy.trixie      sqlite3
    noetic.humble.jazzy.trixie      sudo
    ..............jazzy.......      unminimize
    noetic.humble.jazzy.trixie      unzip
    noetic.humble.jazzy.trixie      zlib1g-dev
    noetic.humble.jazzy.trixie      zip
EOF
)

echo "Installing: ${packages}"

export DEBIAN_FRONTEND=noninteractive

apt-get -y update
apt-get -y --with-new-pkgs upgrade
apt-get -y --no-install-recommends install ${packages}
apt-get -y autoremove
apt-get clean

########################################################################################################################
