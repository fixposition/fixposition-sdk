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
    noetic.humble.jazzy.lyrical.trixie      bison
    noetic.humble.jazzy.lyrical.trixie      build-essential
    .......humble.jazzy.lyrical.trixie      clang
    ....................lyrical.trixie      clang-format
    ....................lyrical.trixie      clang-tools
    ....................lyrical.trixie      clang-tidy
    noetic.humble.jazzy.lyrical.trixie      cmake
    noetic.humble.jazzy.lyrical.trixie      curl
    ............................trixie      doxygen
    noetic.humble.jazzy.lyrical.trixie      fakeroot
    noetic.humble.jazzy.lyrical.trixie      flex
    noetic.humble.jazzy.lyrical.trixie      gawk
    noetic.humble.jazzy.lyrical.trixie      git
    ..............jazzy.lyrical.trixie      gnupg2
    noetic.humble.jazzy.lyrical.trixie      gnuplot
    ..............jazzy.lyrical.trixie      googletest
    noetic.humble.jazzy.lyrical.trixie      graphviz
    noetic.humble.jazzy.lyrical.trixie      jq
    ....................lyrical.trixie      libbacktrace-dev                  # GCC has its own, but clang needs this
    noetic.humble.jazzy.lyrical.trixie      libboost-all-dev                  # This is not small... :-/
    noetic.humble.jazzy.lyrical.trixie      libbz2-dev                        # For compressing ROS1 bags
    noetic.humble.jazzy.lyrical.trixie      libclone-perl
    noetic.humble.jazzy.lyrical.trixie      libcurl4-openssl-dev
    noetic.humble.jazzy.lyrical.trixie      libeigen3-dev
    ..............jazzy.lyrical.trixie      libgtest-dev
    noetic.humble.jazzy.lyrical.trixie      libjson-xs-perl
    noetic.humble.jazzy.lyrical.trixie      libmime-base64-perl
    noetic.humble.jazzy.lyrical.trixie      libpath-tiny-perl
    ..............jazzy.lyrical.trixie      libproj25
    ..............jazzy.lyrical.trixie      libproj-dev
    noetic.humble.jazzy.lyrical.trixie      libssl-dev
    ....................lyrical.......      libstdc++-16-dev                  # For clang
    noetic.humble.jazzy.lyrical.trixie      libsqlite3-dev
    noetic.humble.jazzy.lyrical.trixie      libtiff-dev
    noetic.humble.jazzy.lyrical.trixie      libva-dev                         # automatically installs va-driver-all?
    noetic.humble.jazzy.lyrical.trixie      libyaml-cpp-dev
    noetic.humble.jazzy.lyrical.trixie      nasm
    noetic.humble.jazzy.lyrical.trixie      netbase
    noetic.humble.jazzy.lyrical.trixie      nlohmann-json3-dev
    .......humble.jazzy.lyrical.trixie      pre-commit
    ..............jazzy.lyrical.trixie      proj-data
    ..............jazzy.lyrical.trixie      proj-bin
    noetic............................      python3-catkin-tools
    noetic.humble.jazzy.lyrical.trixie      python-is-python3
    noetic.humble.jazzy.lyrical.trixie      python3-osrf-pycommon
    noetic.humble.jazzy.lyrical.trixie      python3-pip
    noetic.humble.jazzy.lyrical.trixie      python3-venv
    .......humble.....................      ros-humble-rosbag2-storage-mcap
    .......humble.....................      ros-humble-lifecycle
    .......humble.....................      ros-humble-diagnostic-msgs
    .......humble.....................      ros-humble-diagnostic-updater
    ..............jazzy...............      ros-jazzy-rosbag2-storage-mcap
    ..............jazzy...............      ros-jazzy-lifecycle
    ..............jazzy...............      ros-jazzy-diagnostic-msgs
    ..............jazzy...............      ros-jazzy-diagnostic-updater
    ....................lyrical.......      ros-lyrical-rosbag2-storage-mcap
    ....................lyrical.......      ros-lyrical-lifecycle
    ....................lyrical.......      ros-lyrical-diagnostic-msgs
    ....................lyrical.......      ros-lyrical-diagnostic-updater
    noetic............................      ros-noetic-eigen-conversions
    noetic............................      ros-noetic-tf
    noetic............................      ros-noetic-tf-conversions
    noetic............................      ros-noetic-tf2-ros
    noetic............................      ros-noetic-tf2-tools
    noetic............................      ros-noetic-tf2-geometry-msgs
    noetic.humble.jazzy.lyrical.trixie      sqlite3
    noetic.humble.jazzy.lyrical.trixie      sudo
    ..............jazzy.lyrical.......      unminimize
    noetic.humble.jazzy.lyrical.trixie      unzip
    noetic.humble.jazzy.lyrical.trixie      zlib1g-dev
    noetic.humble.jazzy.lyrical.trixie      zip
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
