#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <trace/trace.h>
#include "decawave_interface.h"
#include "exti.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "main.h"
#include "uwb_protocol.h"
#include "usbconf.h"

int readfromspi(uint16 headerLength, const uint8* headerBuffer, uint32 readlength, uint8* readBuffer)
{
    spiSelect(&SPID1);
    spiSend(&SPID1, headerLength, headerBuffer);
    spiReceive(&SPID1, readlength, readBuffer);
    spiUnselect(&SPID1);

    // Does not appear to be used nor documented by Decawave
    return 0;
}

int writetospi(uint16 headerLength, const uint8* headerBuffer, uint32 bodyLength, const uint8* bodyBuffer)
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
    /* As all the Decawave related code runs in the same thread, no need for
     * locking. */
    return 0;
}

void decamutexoff(decaIrqStatus_t enable_irq)
{
    (void)enable_irq;
}

uint64_t uwb_timestamp_get(void)
{
    return ((uint64_t)dwt_readsystimestamphi32()) << 8;
}

uint64_t decawave_get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

void uwb_transmit_frame(uint64_t tx_timestamp, uint8_t* frame, size_t frame_size)
{
    dwt_writetxdata(frame_size, frame, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(frame_size, 0, 0); /* Zero offset in TX buffer. */

    dwt_setrxaftertxdelay(0);

    if (tx_timestamp == UWB_TX_TIMESTAMP_IMMEDIATE) {
        dwt_starttx(DWT_RESPONSE_EXPECTED);
    } else {
        dwt_setdelayedtrxtime(tx_timestamp >> 8);
        dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    }
}

void decawave_start(void)
{
    int res;
    static SPIConfig spi_cfg = {
        .end_cb = NULL,
        .ssport = GPIOA,
        .sspad = GPIOA_UWB_CS_N,
        /* 2.65 Mhz */
        .cr1 = SPI_CR1_BR_2};

    spiStart(&SPID1, &spi_cfg);

    palClearPad(GPIOC, GPIOC_UWB_RST_N);
    chThdSleepMilliseconds(3); // DW1000 reset cycle
    palSetPad(GPIOC, GPIOC_UWB_RST_N);

    res = dwt_initialise(DWT_LOADUCODE);
    chDbgAssert(res == DWT_SUCCESS, "dwm1000 init fail");

    /* Enable DWM1000 LEDs */
    dwt_setlnapamode(1, 1);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configuration example taken straight from decawave's example. */
    /* TODO: Make it a bit more easy to configure */
    static dwt_config_t config = {
        5, /* Channel number. */
        DWT_PRF_64M, /* Pulse repetition frequency. */
        DWT_PLEN_64, /* Preamble length. Used in TX only. */
        DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
        9, 9, /* Preamble codes (RX, TX) */
        1, /* Non standard Start Frame Delimiter */
        DWT_BR_6M8, /* Data rate. */
        DWT_PHRMODE_EXT, /* PHY header mode. */
        /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        (64 + 64 - 8)};

    dwt_configure(&config);

    static dwt_txconfig_t txconfig = {
        0xC0, /* Pulse generator delay value */
        0x25456585, /* TX power, Channel 5 and 64Mhz smart power enabled */
    };

    dwt_configuretxrf(&txconfig);
    dwt_setsmarttxpower(1);
}
