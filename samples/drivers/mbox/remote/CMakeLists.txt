#
# Copyright (c) 2021 Carlo Caione <ccaione@baylibre.com>
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.0)

if("${BOARD}" STREQUAL "nrf5340dk_nrf5340_cpunet")
  message(INFO " ${BOARD} compile as remote in this sample")
else()
  message(FATAL_ERROR "${BOARD} is not supported for this sample")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mbox_ipc_remote)

target_sources(app PRIVATE src/main.c)
