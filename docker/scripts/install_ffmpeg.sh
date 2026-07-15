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
# Download, build and install FFmpeg's libavcodec, libavutil and libswscale
# Note the use of --disable-gpl  and --disable-nonfree to make this LGPL and
# compatible with the Fixposition SDK license.
#
########################################################################################################################
set -eEu

# curl -L https://ffmpeg.org/releases/ffmpeg-8.1.1.tar.xz -o /tmp/ffmpeg.tar.gz
# echo "0b8318579556aaa736096ceb1e8333d2260b63bf /tmp/ffmpeg.tar.gz" | sha1sum --check

# LTS
curl -L https://ffmpeg.org/releases/ffmpeg-7.1.5.tar.xz -o /tmp/ffmpeg.tar.gz
echo "280615051f0546371d20d8b119f3ff357e0fb948 /tmp/ffmpeg.tar.gz" | sha1sum --check


mkdir /tmp/ffmpeg
cd /tmp/ffmpeg
tar --strip-components=1 -xvf ../ffmpeg.tar.gz


./configure \
  --disable-everything \
  --disable-programs \
  --disable-doc \
  --disable-swresample \
  --disable-network \
  --disable-podpages \
  --disable-iconv \
  --disable-lzma \
  --disable-bzlib \
  --disable-zlib \
  --disable-libxcb \
  \
  --enable-avcodec \
  --enable-avformat \
  --enable-avfilter \
  --enable-filters \
  --enable-decoder=h264 \
  --enable-decoder=hevc \
  --enable-decoder=mjpeg \
  --enable-parser=h264 \
  --enable-parser=hevc \
  --enable-parser=mjpeg \
  --enable-vaapi \
  \
  --disable-static \
  --enable-shared \
  \
  --disable-gpl \
  --disable-nonfree \
  \
  --prefix=/usr/local

#  --disable-avformat
#  --disable-avdevice

# --enable-vulkan
# --enable-nvdec

#./configure --disable-gpl --disable-nonfree --enable-shared --disable-static --prefix=/usr/local

make -j4
make install

cd /
rm -rf /tmp/ffmpeg.tar.gz /tmp/ffmpeg


########################################################################################################################
