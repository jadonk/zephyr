#!/bin/sh

URL='https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2019q3/RC1.1/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2?revision=c34d758a-be0c-476e-a2de-af8c6e16a8a2?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,8-2019-q3-update'
TBZ2='gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2'
DIR='gcc-arm-none-eabi-8-2018-q4-major'

set -e

curl -L "${URL}" -o ${TBZ2}
mkdir -p /opt/arm
tar xpf ${TBZ2} -C /opt/arm
