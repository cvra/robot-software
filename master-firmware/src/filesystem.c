#include <ch.h>
#include <hal.h>
#include <ff.h>
#include <error/error.h>
#include "filesystem.h"

static FATFS SDC_FS;

void filesystem_start(void)
{
    FRESULT err;

    /* Enable SD power if card present */
    if (!palReadPad(GPIOA, GPIOA_SD_DETECT)) {
        palClearPad(GPIOA, GPIOA_SD_PWR);
        chThdSleepMilliseconds(100);
    }

    sdcStart(&SDCD1, NULL);
    sdcConnect(&SDCD1);

    err = f_mount(&SDC_FS, "", 1);
    if (err != FR_OK) {
        WARNING("Cannot mount SD card!");
    }
}
