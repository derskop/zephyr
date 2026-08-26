# Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=n32g457ve" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
