# Copyright (c) 2026 Nations Technologies
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(reset_nsing)

target_sources(app PRIVATE src/main.c)
