#include <cstdlib>
#include <error/error.h>
#include <mutex>

static std::mutex parameter_lock;

extern "C" void parameter_port_lock(void)
{
    parameter_lock.lock();
}

extern "C" void parameter_port_unlock(void)
{
    parameter_lock.unlock();
}

extern "C" void parameter_port_assert(int condition)
{
    if (!condition) {
        ERROR("parameter_assert()");
    }
}

extern "C" void* parameter_port_buffer_alloc(size_t size)
{
    return new uint8_t[size];
}

extern "C" void parameter_port_buffer_free(void* buffer)
{
    uint8_t* ptr = (uint8_t*)buffer;
    delete[] ptr;
}
