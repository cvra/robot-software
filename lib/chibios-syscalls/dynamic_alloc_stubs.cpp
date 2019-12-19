#include <ch.h>
#include <hal.h>

#ifndef CVRA_NO_DYNAMIC_ALLOCATION
#define CVRA_NO_DYNAMIC_ALLOCATION 0
#endif

#if CVRA_NO_DYNAMIC_ALLOCATION
void* operator new(size_t)
{
    osalSysHalt("operator new");
    return reinterpret_cast<void*>(0xFFFFFFFF);
}

void* operator new[](size_t)
{
    osalSysHalt("operator new[]");
    return reinterpret_cast<void*>(0xFFFFFFFF);
}

void operator delete(void*)
{
    osalSysHalt("operator delete");
}

void operator delete[](void*)
{
    osalSysHalt("operator delete[]");
}
#else
void* operator new(size_t s)
{
    void* p = malloc(s);
    if (!p) {
        osalSysHalt("operator new");
    }
    return p;
}

void* operator new[](size_t s)
{
    void* p = malloc(s);
    if (!p) {
        osalSysHalt("operator new[]");
    }
    return p;
}

void operator delete(void* p)
{
    free(p);
}

void operator delete[](void* p)
{
    free(p);
}
#endif /* CVRA_NO_DYNAMIC_ALLOCATION */
