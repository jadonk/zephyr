# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(google_iot_mqtt)

if(NOT EXISTS ${APPLICATION_SOURCE_DIR}/src/private_info/key.c)
  message(FATAL_ERROR "!!!!!! Generate key file before continuing.  See README !!!!!")
endif()

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE ${APPLICATION_SOURCE_DIR}/src/private_info/key.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/src/tls_config)
