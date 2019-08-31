#!/bin/sh

PN=cmake
PV=3.15.2
PR=1
DEB="${PN}_${PV}-${PR}.deb"
URL="https://github.com/cfriedt/${PN}-debs/releases/download/v${PV}/${DEB}"

set -e

cd /
curl -L "${URL}" -o "${DEB}"
dpkg -i "${DEB}"
