#include <ch.h>
#include <hal.h>

#include <stdio.h>
#include <errno.h>

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
#endif /* CVRA_NO_DYNAMIC_ALLOCATION */
