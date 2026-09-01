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
    noetic.humble.jazzy.lyrical.trixie      ack
    noetic.humble.jazzy.lyrical.trixie      aptitude
    noetic.humble.jazzy.lyrical.trixie      bash-completion
    noetic.humble.jazzy.lyrical.trixie      bind9-dnsutils
    noetic.humble.jazzy.lyrical.trixie      bsdmainutils
    noetic.humble.jazzy.lyrical.trixie      can-utils
    noetic.humble.jazzy.lyrical.trixie      ccache
    noetic.humble.jazzy.lyrical.trixie      chrpath
    ....................lyrical.trixie      clangd
    noetic.humble.jazzy.lyrical.trixie      curl
    noetic.humble.jazzy.lyrical.trixie      dlocate
    noetic.humble.jazzy.lyrical.trixie      evtest
    noetic.humble.jazzy.lyrical.trixie      file
    noetic.humble.jazzy.lyrical.trixie      flip
    noetic.humble.jazzy.lyrical.trixie      gdb
    noetic.humble.jazzy.lyrical.trixie      git-lfs
    noetic.humble.jazzy.lyrical.trixie      htop
    noetic.humble.jazzy.lyrical.trixie      imagemagick
    noetic.humble.jazzy.lyrical.trixie      iproute2
    noetic.humble.jazzy.lyrical.trixie      iputils-ping
    noetic.humble.jazzy.lyrical.trixie      less
    noetic.humble.jazzy.lyrical.trixie      libboost-doc
    noetic.humble.jazzy.lyrical.trixie      libevdev-dev
    noetic.humble.jazzy.lyrical.trixie      libgpiod-dev
    noetic.humble.jazzy.lyrical.trixie      libiio-dev
    noetic.humble.jazzy.lyrical.trixie      libssl-doc
    ....................lyrical.trixie      linux-perf
    noetic.humble.jazzy...............      linux-tools-common                          # perf
    noetic.humble.jazzy.lyrical.trixie      lsb-release
    noetic.humble.jazzy.lyrical.trixie      man
    noetic.humble.jazzy.lyrical.trixie      man-db
    noetic.humble.jazzy.lyrical.trixie      manpages
    noetic.humble.jazzy.lyrical.trixie      manpages
    noetic.humble.jazzy.lyrical.trixie      manpages-dev
    noetic.humble.jazzy.lyrical.trixie      manpages-dev
    noetic.humble.jazzy.lyrical.trixie      manpages-posix
    noetic.humble.jazzy.lyrical.trixie      manpages-posix-dev
    noetic.humble.jazzy.lyrical.trixie      moreutils
    noetic.humble.jazzy.lyrical.trixie      ncdu
    noetic.humble.jazzy.lyrical.trixie      net-tools
    noetic.humble.....................      netcat
    ..............jazzy.lyrical.trixie      netcat-openbsd
    noetic.humble.jazzy.lyrical.trixie      openssh-client
    noetic.humble.jazzy.lyrical.trixie      psmisc
    noetic.humble.jazzy.lyrical.trixie      pv
    noetic.humble.jazzy.lyrical.trixie      sl
    noetic.humble.jazzy.lyrical.trixie      rsync
    noetic.humble.jazzy.lyrical.trixie      socat
    noetic.humble.jazzy.lyrical.trixie      strace
    noetic.humble.jazzy.lyrical.trixie      systemd-journal-remote
    noetic.humble.jazzy.lyrical.trixie      tcpdump
    noetic.humble.jazzy.lyrical.trixie      tig
    noetic.humble.jazzy.lyrical.trixie      va-driver-all
    noetic.humble.jazzy.lyrical.trixie      vainfo
    noetic.humble.jazzy.lyrical.trixie      valgrind
    noetic.humble.jazzy.lyrical.trixie      vim
    noetic.humble.jazzy.lyrical.trixie      wget
    noetic.humble.jazzy.lyrical.trixie      xauth
    noetic.humble.jazzy.lyrical.trixie      xxd
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
