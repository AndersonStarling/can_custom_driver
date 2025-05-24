# SPDX-License-Identifier: Apache-2.0
zephyr_library()

zephyr_library_sources (${ZEPHYR_BASE}/drivers/can/can_common.c)
zephyr_library_sources_ifdef(CONFIG_CAN_STM32_CUSTOM can_stm32.c)




