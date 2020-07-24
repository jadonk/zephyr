# Copyright (c) 2019 Brett Witherspoon
#
# SPDX-License-Identifier: Apache-2.0

# #include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)

#board_runner_args(jlink "--device=CC1352R1F3" "--iface=jtag" "--tool-opt=-jtagconf -1,-1 -autoconnect 1")
#include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

board_set_flasher_ifnset(misc-flasher)
board_finalize_runner_args(misc-flasher)
