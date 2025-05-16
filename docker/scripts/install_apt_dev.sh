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

if [ "${FPSDK_IMAGE%-*}" = "bookworm" ]; then
    echo deb http://deb.debian.org/debian           bookworm            main non-free contrib non-free-firmware  > /etc/apt/sources.list.d/debian.list
    echo deb http://deb.debian.org/debian-security  bookworm-security   main non-free contrib non-free-firmware >> /etc/apt/sources.list.d/debian.list
    echo deb http://deb.debian.org/debian           bookworm-backports  main non-free contrib non-free-firmware >> /etc/apt/sources.list.d/debian.list
    echo deb http://deb.debian.org/debian           bookworm-updates    main non-free contrib non-free-firmware >> /etc/apt/sources.list.d/debian.list
    rm /etc/apt/sources.list.d/debian.sources
fi

# List of packages, with filter for the different images we make
packages=$(awk -v filt=${FPSDK_IMAGE%-*} '$1 ~ filt { print $2 }' <<EOF
    noetic.humble.jazzy.bookworm    ack
    noetic.humble.jazzy.bookworm    aptitude
    noetic.humble.jazzy.bookworm    bash-completion
    noetic.humble.jazzy.bookworm    bind9-dnsutils
    noetic.humble.jazzy.bookworm    bsdmainutils
    noetic.humble.jazzy.bookworm    can-utils
    noetic.humble.jazzy.bookworm    ccache
    noetic.humble.jazzy.bookworm    chrpath
    noetic.humble.jazzy.bookworm    curl
    noetic.humble.jazzy.bookworm    dlocate
    noetic.humble.jazzy.bookworm    evtest
    noetic.humble.jazzy.bookworm    file
    noetic.humble.jazzy.bookworm    flip
    noetic.humble.jazzy.bookworm    gdb
    noetic.humble.jazzy.bookworm    git-lfs
    noetic.humble.jazzy.bookworm    htop
    noetic.humble.jazzy.bookworm    iproute2
    noetic.humble.jazzy.bookworm    iputils-ping
    noetic.humble.jazzy.bookworm    less
    noetic.humble.jazzy.bookworm    libboost-doc
    noetic.humble.jazzy.bookworm    libevdev-dev
    noetic.humble.jazzy.bookworm    libgpiod-dev
    noetic.humble.jazzy.bookworm    libiio-dev
    noetic.humble.jazzy.bookworm    libssl-doc
    ....................bookworm    linux-perf
    noetic.humble.jazzy.........    linux-tools-common                          # perf
    noetic.humble.jazzy.bookworm    lsb-release
    noetic.humble.jazzy.bookworm    man
    noetic.humble.jazzy.bookworm    man-db
    noetic.humble.jazzy.bookworm    manpages
    noetic.humble.jazzy.bookworm    manpages
    noetic.humble.jazzy.bookworm    manpages-dev
    noetic.humble.jazzy.bookworm    manpages-dev
    noetic.humble.jazzy.bookworm    manpages-posix
    noetic.humble.jazzy.bookworm    manpages-posix-dev
    noetic.humble.jazzy.bookworm    moreutils
    noetic.humble.jazzy.bookworm    ncdu
    noetic.humble.jazzy.bookwork    net-tools
    noetic.humble...............    netcat
    ..............jazzy.bookworm    netcat-openbsd
    noetic.humble.jazzy.bookworm    openssh-client
    noetic.humble.jazzy.bookworm    psmisc
    noetic.humble.jazzy.bookworm    pv
    noetic.humble.jazzy.bookworm    sl
    noetic.humble.jazzy.bookworm    rsync
    noetic.humble.jazzy.bookworm    socat
    noetic.humble.jazzy.bookworm    strace
    noetic.humble.jazzy.bookworm    systemd-journal-remote
    noetic.humble.jazzy.bookworm    tcpdump
    noetic.humble.jazzy.bookworm    tig
    noetic.humble.jazzy.bookworm    valgrind
    noetic.humble.jazzy.bookworm    vim
    noetic.humble.jazzy.bookworm    wget
    noetic.humble.jazzy.bookworm    xauth
    noetic.humble.jazzy.bookworm    xxd
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
