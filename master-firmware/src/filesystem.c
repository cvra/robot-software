#include <ch.h>
#include <hal.h>
#include <ff.h>
#include <error/error.h>
#include "filesystem.h"

static FATFS SDC_FS;

void filesystem_start(void)
{
    FRESULT err;
    sdcStart(&SDCD1, NULL);
    sdcConnect(&SDCD1);

    err = f_mount(&SDC_FS, "", 1);
    if (err != FR_OK) {
        WARNING("Cannot mount SD card!");
    }
}
