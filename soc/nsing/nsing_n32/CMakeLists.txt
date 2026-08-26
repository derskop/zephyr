# Nations Technologies N32 SoC family

# Copyright (c) 2024 Nations Technologies
# SPDX-License-Identifier: Apache-2.0

# 全家族公共代码（nsing/common/）
add_subdirectory(common)

# 根据 Kconfig 字符串 SOC_SERIES（如 "n32g45x"）进入对应系列目录
add_subdirectory(${SOC_SERIES})
