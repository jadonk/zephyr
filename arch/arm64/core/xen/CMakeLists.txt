# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2021 EPAM Systems

# Needed to separate definitions in common Xen headers
zephyr_compile_options($<$<COMPILE_LANGUAGE:ASM>:-D__ASSEMBLY__>)

# Xen interface version used in headers for correct definition
zephyr_compile_options(-D__XEN_INTERFACE_VERSION__=0x00040e00)

zephyr_library_sources(hypercall.S)
