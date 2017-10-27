#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "decawave_interface.h"
#include "exti.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "main.h"
#include "uwb_protocol.h"

static messagebus_topic_t ranging_topic;
static MUTEX_DECL(ranging_topic_lock);
static CONDVAR_DECL(ranging_topic_condvar);
static range_msg_t ranging_topic_buffer;

static uwb_protocol_handler_t handler;

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

uint64_t uwb_timestamp_get(void)
{
#warning not implemented
    return 0; // TODO
}

void uwb_transmit_frame(uint64_t tx_timestamp, uint8_t *frame, size_t frame_size)
{
    // TODO apparently only the high 32 bits ared used, what does that mean
#warning not implemented
    // dwt_setdelayedtrxtime(tx_timestamp);
    dwt_writetxdata(frame_size, frame, 1); /* Zero offset in TX buffer. */
    dwt_setrxaftertxdelay(0);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

/* TODO: Handle RX errors as well. */
static void frame_rx_cb(const dwt_cb_data_t *data)
{
    static uint8_t frame[1024];

    dwt_readrxdata(frame, data->datalength, 0);
    uwb_process_incoming_frame(&handler, frame, data->datalength, 0);
}

static void frame_tx_done_cb(const dwt_cb_data_t *data)
{
    (void) data;
}

static void ranging_found_cb(uint16_t addr, uint64_t time)
{
    range_msg_t msg;
    msg.timestamp = ST2MS(chVTGetSystemTime());
    msg.anchor_addr = addr;
    msg.range = time;

    messagebus_topic_publish(&ranging_topic, &msg, sizeof(msg));
}

static void decawave_thread(void *p)
{
    (void) p;

    chRegSetThreadName("decawave_drv");

    uwb_protocol_handler_init(&handler);
    handler.address = 0xcafe;
    handler.pan_id = 0xcaff;
    handler.ranging_found_cb = ranging_found_cb;

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

    messagebus_topic_init(&ranging_topic, &ranging_topic_lock, &ranging_topic_condvar,
                          &ranging_topic_buffer, sizeof(ranging_topic_buffer));
    messagebus_advertise_topic(&bus, &ranging_topic, "/ranging");

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
