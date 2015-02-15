/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_stm32/thread.hpp>
#include <uavcan_stm32/clock.hpp>
#include <uavcan_stm32/can.hpp>
#include "internal.hpp"

namespace uavcan_stm32
{

/*
 * BusEvent
 */
bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
    static const uavcan::int64_t MaxDelayMSec = 0x000FFFFF;

    const uavcan::int64_t msec = duration.toMSec();
    msg_t ret = msg_t();

    if (msec <= 0)
    {
        ret = sem_.wait(TIME_IMMEDIATE);
    }
    else
    {
        ret = sem_.wait((msec > MaxDelayMSec) ? MS2ST(MaxDelayMSec) : MS2ST(msec));
    }
    return ret == MSG_OK;
}

void BusEvent::signal()
{
    sem_.signal();
}

void BusEvent::signalFromInterrupt()
{
    chSysLockFromISR();
    sem_.signalI();
    chSysUnlockFromISR();
}

/*
 * Mutex
 */
void Mutex::lock()
{
    mtx_.lock();
}

void Mutex::unlock()
{
    chibios_rt::BaseThread::unlockMutex(&mtx_);
}

}
