#!/bin/bash
set -eEu
####################################################################################################
# Install git-bash-prompt

user=$(id -un)
if [ "${user}" = "root" ]; then
    home=/root
else
    home=/home/${user}
fi

git clone https://github.com/magicmonty/bash-git-prompt.git ${home}/.bash-git-prompt
git -C ${home}/.bash-git-prompt checkout 51080c22b2cebb63111379f4eacd22cda199684b
cp /tmp/fpsdk.bgptheme ${home}/.bash-git-prompt/themes

####################################################################################################
