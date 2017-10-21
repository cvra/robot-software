#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "exti.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "main.h"

messagebus_topic_t rtt_topic;
MUTEX_DECL(rtt_topic_lock);
CONDVAR_DECL(rtt_topic_condvar);




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
    int irq_enabled = 0;

    /* TODO Proper locking ((prevent DWM1000 interrupts from arriving).
     *
     * Cannot use chSysLock because it uses SPI transfers in critical sections.*/

    return irq_enabled;
}

void decamutexoff(decaIrqStatus_t enable_irq)
{
    /* TODO Proper locking (see above) */
    (void) enable_irq;
}

/* TODO: Handle RX errors as well. */
static void frame_rx_cb(const dwt_cb_data_t *data)
{
    static unsigned char frame[100];
    static uint32_t last_tx = 0;

    static char tx_msg[] = "hello world";

    /* For testing */

    dwt_setrxaftertxdelay(0);

    /* We need to pass sizeof + 2, because those functions assume the
     * checksum size is included. */
    dwt_writetxdata(sizeof(tx_msg)+2, tx_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg)+2, 0, 0); /* Zero offset in TX buffer, no ranging. */

    /* Start transmission. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    messagebus_topic_publish(&rtt_topic, NULL, 0);

    //chprintf((BaseSequentialStream *)&SD2, "delay: %d\r\n", ST2MS(new_tx  - last_tx));

    //dwt_readrxdata(frame, data->datalength, 0);
    //chprintf((BaseSequentialStream *)&SD2,
    //        "Received a frame of length %d \"%s\"\r\n", data->datalength, frame);

    /* We need to manually re enable the receiver once a frame was processed. */
    //dwt_rxenable(DWT_START_RX_DELAYED);
}

static void frame_tx_done_cb(const dwt_cb_data_t *data)
{
    (void) data;
}

static void decawave_thread(void *p)
{
    (void) p;

    chRegSetThreadName("decawave_drv");

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
        5,               /* Channel number. */
        DWT_PRF_64M,     /* Pulse repetition frequency. */
        DWT_PLEN_64,   /* Preamble length. Used in TX only. */
        DWT_PAC8,       /* Preamble acquisition chunk size. Used in RX only. */
        /* Preamble codes (RX, TX) */
        5, 5,
        1,  /* Non standard SFD */
        DWT_BR_6M8,     /* Data rate. */
        DWT_PHRMODE_EXT, /* PHY header mode. */
        /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        (1025 + 64 - 32)
    };

    dwt_configure(&config);

    /* Enable interrupt on frame RX */
    uint32_t interrupts = DWT_INT_RFCG | DWT_INT_TFRS;
    dwt_setinterrupt(interrupts, 1);

    /* Configure interrupt listener. */
    event_listener_t uwb_int;
    chEvtRegisterMaskWithFlags(&exti_events, &uwb_int,
                               (eventmask_t)EXTI_EVENT_UWB_INT,
                               (eventflags_t)EXTI_EVENT_UWB_INT);

    dwt_setcallbacks(frame_tx_done_cb, frame_rx_cb, NULL, NULL);

    messagebus_topic_init(&rtt_topic, &rtt_topic_lock, &rtt_topic_condvar,
            NULL, 0);
    messagebus_advertise_topic(&bus, &rtt_topic, "/rtt");

    while (1) {
        /* Wait for an interrupt coming from the UWB module. */
        chEvtWaitAny(EXTI_EVENT_UWB_INT);

        /* Process the interrupt. */
        dwt_isr();
    }
}

void decawave_start(void)
{
    static THD_WORKING_AREA(thd_wa, 512);
    chThdCreateStatic(thd_wa, sizeof(thd_wa), HIGHPRIO, decawave_thread, NULL);
}
