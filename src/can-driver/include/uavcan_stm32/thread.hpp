/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32/build_config.hpp>
#include <ch.hpp>
#include <uavcan/uavcan.hpp>

namespace uavcan_stm32
{

class CanDriver;

class BusEvent
{
    chibios_rt::CounterSemaphore sem_;

public:
    BusEvent(CanDriver& can_driver)
        : sem_(0)
    {
        (void)can_driver;
    }

    bool wait(uavcan::MonotonicDuration duration);

    void signal();

    void signalFromInterrupt();
};

class Mutex
{
    chibios_rt::Mutex mtx_;

public:
    void lock();
    void unlock();
};

class MutexLocker
{
    Mutex& mutex_;

public:
    MutexLocker(Mutex& mutex)
        : mutex_(mutex)
    {
        mutex_.lock();
    }
    ~MutexLocker()
    {
        mutex_.unlock();
    }
};

}
