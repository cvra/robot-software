/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32/build_config.hpp>
#include <hal.h>

/**
 * Debug output
 */
#ifndef UAVCAN_STM32_LOG
# if 0
#  define UAVCAN_STM32_LOG(fmt, ...)  lowsyslog("uavcan_stm32: " fmt "\n", ##__VA_ARGS__)
# else
#  define UAVCAN_STM32_LOG(...)       ((void)0)
# endif
#endif

/**
 * IRQ handler macros
 */
#define UAVCAN_STM32_IRQ_HANDLER(id)  CH_IRQ_HANDLER(id)
#define UAVCAN_STM32_IRQ_PROLOGUE()    CH_IRQ_PROLOGUE()
#define UAVCAN_STM32_IRQ_EPILOGUE()    CH_IRQ_EPILOGUE()

/**
 * Priority mask for timer and CAN interrupts.
 */
#ifndef UAVCAN_STM32_IRQ_PRIORITY_MASK
# define UAVCAN_STM32_IRQ_PRIORITY_MASK  CORTEX_MAX_KERNEL_PRIORITY
#endif

/**
 * Glue macros
 */
#define UAVCAN_STM32_GLUE2_(A, B)       A##B
#define UAVCAN_STM32_GLUE2(A, B)        UAVCAN_STM32_GLUE2_(A, B)

#define UAVCAN_STM32_GLUE3_(A, B, C)    A##B##C
#define UAVCAN_STM32_GLUE3(A, B, C)     UAVCAN_STM32_GLUE3_(A, B, C)

namespace uavcan_stm32
{

struct CriticalSectionLocker
{
    CriticalSectionLocker() { chSysSuspend(); }
    ~CriticalSectionLocker() { chSysEnable(); }
};

namespace clock
{

uavcan::uint64_t getUtcUSecFromCanInterrupt();

}

}
