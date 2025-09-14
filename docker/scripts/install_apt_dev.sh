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
# Install stuff for the *-dev images (devcontainer)
#
########################################################################################################################
set -eEu

if [ "${FPSDK_IMAGE%-*}" = "trixie" ]; then
    # TODO: use new .sources format
    echo deb http://deb.debian.org/debian           trixie              main non-free contrib non-free-firmware  > /etc/apt/sources.list.d/debian.list
    echo deb http://deb.debian.org/debian-security  trixie-security     main non-free contrib non-free-firmware >> /etc/apt/sources.list.d/debian.list
    echo deb http://deb.debian.org/debian           trixie-backports    main non-free contrib non-free-firmware >> /etc/apt/sources.list.d/debian.list
    echo deb http://deb.debian.org/debian           trixie-updates      main non-free contrib non-free-firmware >> /etc/apt/sources.list.d/debian.list
    rm /etc/apt/sources.list.d/debian.sources
fi

# List of packages, with filter for the different images we make
packages=$(awk -v filt=${FPSDK_IMAGE%-*} '$1 ~ filt { print $2 }' <<EOF
    noetic.humble.jazzy.trixie      ack
    noetic.humble.jazzy.trixie      aptitude
    noetic.humble.jazzy.trixie      bash-completion
    noetic.humble.jazzy.trixie      bind9-dnsutils
    noetic.humble.jazzy.trixie      bsdmainutils
    noetic.humble.jazzy.trixie      can-utils
    noetic.humble.jazzy.trixie      ccache
    noetic.humble.jazzy.trixie      chrpath
    ....................trixie      clang-tools
    ....................trixie      clang-tidy
    ....................trixie      clangd
    noetic.humble.jazzy.trixie      curl
    noetic.humble.jazzy.trixie      dlocate
    noetic.humble.jazzy.trixie      evtest
    noetic.humble.jazzy.trixie      file
    noetic.humble.jazzy.trixie      flip
    noetic.humble.jazzy.trixie      gdb
    noetic.humble.jazzy.trixie      git-lfs
    noetic.humble.jazzy.trixie      htop
    noetic.humble.jazzy.trixie      iproute2
    noetic.humble.jazzy.trixie      iputils-ping
    noetic.humble.jazzy.trixie      less
    noetic.humble.jazzy.trixie      libboost-doc
    noetic.humble.jazzy.trixie      libevdev-dev
    noetic.humble.jazzy.trixie      libgpiod-dev
    noetic.humble.jazzy.trixie      libiio-dev
    noetic.humble.jazzy.trixie      libssl-doc
    ....................trixie      linux-perf
    noetic.humble.jazzy.......      linux-tools-common                          # perf
    noetic.humble.jazzy.trixie      lsb-release
    noetic.humble.jazzy.trixie      man
    noetic.humble.jazzy.trixie      man-db
    noetic.humble.jazzy.trixie      manpages
    noetic.humble.jazzy.trixie      manpages
    noetic.humble.jazzy.trixie      manpages-dev
    noetic.humble.jazzy.trixie      manpages-dev
    noetic.humble.jazzy.trixie      manpages-posix
    noetic.humble.jazzy.trixie      manpages-posix-dev
    noetic.humble.jazzy.trixie      moreutils
    noetic.humble.jazzy.trixie      ncdu
    noetic.humble.jazzy.trixie      net-tools
    noetic.humble.............      netcat
    ..............jazzy.trixie      netcat-openbsd
    noetic.humble.jazzy.trixie      openssh-client
    noetic.humble.jazzy.trixie      psmisc
    noetic.humble.jazzy.trixie      pv
    noetic.humble.jazzy.trixie      sl
    noetic.humble.jazzy.trixie      rsync
    noetic.humble.jazzy.trixie      socat
    noetic.humble.jazzy.trixie      strace
    noetic.humble.jazzy.trixie      systemd-journal-remote
    noetic.humble.jazzy.trixie      tcpdump
    noetic.humble.jazzy.trixie      tig
    noetic.humble.jazzy.trixie      valgrind
    noetic.humble.jazzy.trixie      vim
    noetic.humble.jazzy.trixie      wget
    noetic.humble.jazzy.trixie      xauth
    noetic.humble.jazzy.trixie      xxd
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
