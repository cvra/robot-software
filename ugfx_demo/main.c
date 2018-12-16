#include "gfx.h"

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    gfxInit();
    gdispClear(Silver);

    gdispFillArea(10, 10, 100, 100, Black);

    while (1) {
        gfxSleepMilliseconds(1000);
    }

    return 0;
}
