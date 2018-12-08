#ifndef ASSERT_H
#define ASSERT_H

#include <ch.h>

#define assert(x)                          \
    do {                                   \
        if (!(x)) {                        \
            chSysHalt("assertion failed"); \
        }                                  \
    } while (0)

#endif
