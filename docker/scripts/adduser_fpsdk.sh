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
# Add user for devcontainer, see ../.devcontainer/devcontainer.json
#
########################################################################################################################
set -eEu

groupadd -g 12345 fpsdk
useradd -m -u 12345 -g 12345 -s /bin/bash fpsdk

echo "fpsdk ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/fpsdk
chmod 0440 /etc/sudoers.d/fpsdk

sudo -u fpsdk cp /tmp/adduser_fpsdk/.bashrc /tmp/adduser_fpsdk/.bash_logout /home/fpsdk/

sudo -u fpsdk git clone https://github.com/magicmonty/bash-git-prompt.git /home/fpsdk/.bash-git-prompt
sudo -u fpsdk git -C /home/fpsdk/.bash-git-prompt checkout 51080c22b2cebb63111379f4eacd22cda199684b
cp /tmp/adduser_fpsdk/fpsdk.bgptheme /home/fpsdk/.bash-git-prompt/themes


########################################################################################################################
