#!/bin/sh

set -e

cd /
git clone --depth 1 https://git.kernel.org/pub/scm/utils/dtc/dtc.git
cd dtc
make -j`nproc --all`
cp dtc /usr/local/bin

