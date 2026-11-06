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
# Set timezone, sane language and locale, see also ENV in the Dockerfiles
#
########################################################################################################################
set -eEu

set -e

echo "en_GB.UTF-8 UTF-8" >  /etc/locale.gen
echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen
locale-gen en_GB.UTF-8 en_US.UTF-8
update-locale LANG=en_GB.UTF-8

ln -snf /usr/share/zoneinfo/$TZ /etc/localtime

########################################################################################################################
