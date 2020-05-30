#include <cstdlib>
#include <error/error.h>
#include <mutex>
#include <parameter/parameter_port.h>

static std::mutex parameter_lock;

void parameter_port_lock(void)
{
    parameter_lock.lock();
}

void parameter_port_unlock(void)
{
    parameter_lock.unlock();
}

void parameter_port_assert(int condition)
{
    if (!condition) {
        ERROR("parameter_assert()");
    }
}

void* parameter_port_buffer_alloc(size_t size)
{
    return new uint8_t[size];
}

void parameter_port_buffer_free(void* buffer)
{
    uint8_t* ptr = (uint8_t*)buffer;
    delete[] ptr;
}
