#
# Copyright (c) 2024, Nationstech
# SPDX-License-Identifier: Apache-2.0
#

zephyr_include_directories(.)

# Shared GPIO port layer, used by the GPIO and pinctrl drivers
zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_N32G45X n32_gpioport.c)


