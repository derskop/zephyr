# Copyright (c) 2024 Nations Technologies Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(pinctrl_nsing_l3)

# Custom state "drive" from pinctrl-names ("default", "drive"): the
# framework numbers private states from PINCTRL_STATE_PRIV_START on,
# in pinctrl-names order, so "drive" gets id 2. Only "default" and
# "sleep" have framework-defined macros; custom ones must be defined
# by the user (see tests/drivers/pinctrl/api). Injected as a compile
# definition so the shared ../common/test_device.c sees it too.
target_compile_definitions(app PRIVATE PINCTRL_STATE_DRIVE=PINCTRL_STATE_PRIV_START)

target_sources(app PRIVATE ../common/test_device.c)
target_sources(app PRIVATE src/main.c)
