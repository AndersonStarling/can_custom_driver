# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/can.h)



zephyr_library_sources(can_common.c)
zephyr_library_sources(can_stm32.c)


