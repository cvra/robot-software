/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

/**
 * OS detection
 */
#if !(UAVCAN_STM32_CHIBIOS)
# error "CAN driver depends on ChibiOS"
#endif

/**
 * Number of interfaces must be enabled explicitly
 */
#if !defined(UAVCAN_STM32_NUM_IFACES) || (UAVCAN_STM32_NUM_IFACES != 1 && UAVCAN_STM32_NUM_IFACES != 2)
# error "UAVCAN_STM32_NUM_IFACES must be set to either 1 or 2"
#endif
