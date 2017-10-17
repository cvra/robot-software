#include <ch.h>
#include <hal.h>
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"


int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength,
                uint8 *readBuffer)
{
    spiSelect(&SPID1);
    spiSend(&SPID1, headerLength, headerBuffer);
    spiReceive(&SPID1, readlength, readBuffer);
    spiUnselect(&SPID1);

    // Does not appear to be used nor documented by Decawave
    return 0;
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer,
               uint32 bodyLength, const uint8 *bodyBuffer)
{
    spiSelect(&SPID1);
    spiSend(&SPID1, headerLength, headerBuffer);
    spiSend(&SPID1, bodyLength, bodyBuffer);
    spiUnselect(&SPID1);

    // Does not appear to be used nor documented by Decawave
    return 0;
}

void deca_sleep(unsigned int time_ms)
{
    chThdSleepMilliseconds(time_ms);
}

decaIrqStatus_t decamutexon(void)
{
    int irq_enabled = port_irq_enabled(port_get_irq_status());

    if (irq_enabled) {
        chSysLock();
    }

    return irq_enabled;
}

void decamutexoff(decaIrqStatus_t enable_irq)
{
    if (enable_irq) {
        chSysUnlock();
    }
}

void decawave_start(void)
{
    static SPIConfig spi_cfg = {
        .end_cb = NULL,
        .ssport = GPIOA,
        .sspad = GPIOA_UWB_CS_N,
        /* 2.65 Mhz */
        .cr1 = SPI_CR1_BR_2
    };
    spiStart(&SPID1, &spi_cfg);

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        chSysHalt("dwm1000 init fail");
    }
    dwt_setlnapamode(1, 1);
    dwt_setleds(DWT_LEDS_ENABLE);


    /* Configuration example taken straight from decawave's example. */
    /* TODO: Make it a bit more easy to configure */
    static dwt_config_t config = {
        2,               /* Channel number. */
        DWT_PRF_64M,     /* Pulse repetition frequency. */
        DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
        DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_110K,     /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        (1025 + 64 - 32)
    };

    dwt_configure(&config);
}
