#include <ch.h>
#include <hal.h>

#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"

#include "main.h"
#include "decawave_interface.h"
#include "ranging_thread.h"
#include "uwb_protocol.h"
#include "exti.h"

/** Speed of light in Decawave units */
#define SPEED_OF_LIGHT             (299792458.0 / (128 * 499.2e6))

/* Default antenna delay values for 64 MHz PRF. */
#define RX_ANT_DLY                 2 * 16436

#define EVENT_UWB_INT               (1 << 0)
#define EVENT_ADVERTISE_TIMER       (1 << 1)
#define EVENT_ANCHOR_POSITION_TIMER (1 << 2)

#define UWB_ADVERTISE_TIMER_PERIOD      MS2ST(250)
#define UWB_ANCHOR_POSITION_TIMER_PERIOD S2ST(1)


static uwb_protocol_handler_t handler;

static messagebus_topic_t ranging_topic;
static MUTEX_DECL(ranging_topic_lock);
static CONDVAR_DECL(ranging_topic_condvar);
static range_msg_t ranging_topic_buffer;
static EVENTSOURCE_DECL(advertise_timer_event);
static EVENTSOURCE_DECL(anchor_position_timer_event);

static struct {
    parameter_namespace_t ns;
    parameter_t mac_addr;
    parameter_t pan_id;
    struct {
        parameter_namespace_t ns;
        parameter_t is_anchor;
        struct {
            parameter_namespace_t ns;
            parameter_t x,y,z;
        } position;
    } anchor;
} uwb_params;

static void ranging_thread(void *p);
static void ranging_found_cb(uint16_t addr, uint64_t time);
static void topics_init(void);
static void parameters_init(void);
static void hardware_init(void);
static void events_init(void);

static void advertise_timer_cb(void *t);
static void anchor_position_timer_cb(void *t);
static void frame_tx_done_cb(const dwt_cb_data_t *data);
static void frame_rx_cb(const dwt_cb_data_t *data);

void ranging_start(void)
{
    static THD_WORKING_AREA(ranging_wa, 512);
    chThdCreateStatic(ranging_wa, sizeof(ranging_wa), HIGHPRIO, ranging_thread, NULL);
}

static void ranging_thread(void *p)
{
    (void) p;
    chRegSetThreadName("ranging");

    uwb_protocol_handler_init(&handler);
    handler.ranging_found_cb = ranging_found_cb;

    parameters_init();
    topics_init();
    hardware_init();
    events_init();

    static uint8_t frame[64];

    while (1) {
        /* Wait for an interrupt coming from the UWB module. */
        eventmask_t flags = chEvtWaitOne(ALL_EVENTS);

        /* Copy parameters */
        handler.is_anchor = parameter_boolean_get(&uwb_params.anchor.is_anchor);
        handler.address = parameter_integer_get(&uwb_params.mac_addr);
        handler.pan_id = parameter_integer_get(&uwb_params.pan_id);

        if (flags & EVENT_ADVERTISE_TIMER) {
            if (handler.is_anchor) {
                /* First disable transceiver */
                dwt_forcetrxoff();

                /* Then send a measurement frame */
                uwb_send_measurement_advertisement(&handler, frame);
            }
        } else if (flags & EVENT_ANCHOR_POSITION_TIMER) {
            /* First disable transceiver */
            dwt_forcetrxoff();

            uwb_send_anchor_position(&handler, 10, 20, 30, frame);
        } else if (flags & EVENT_UWB_INT) {
            /* Process the interrupt. */
            dwt_isr();
        }
    }

}

static void frame_tx_done_cb(const dwt_cb_data_t *data)
{
    (void) data;
}

/* TODO: Handle RX errors as well, especially timeouts. */
static void frame_rx_cb(const dwt_cb_data_t *data)
{
    static uint8_t frame[1024];
    uint64_t rx_ts = decawave_get_rx_timestamp_u64();

    dwt_readrxdata(frame, data->datalength, 0);

    uwb_process_incoming_frame(&handler, frame, data->datalength, rx_ts);

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void ranging_found_cb(uint16_t addr, uint64_t time)
{
    range_msg_t msg;
    /* TODO: For some reason the macro ST2US creates an overflow. */
    uint32_t ts = chVTGetSystemTime() * (1000000 / CH_CFG_ST_FREQUENCY);

    msg.timestamp = ts;
    msg.anchor_addr = addr;
    msg.range = time * SPEED_OF_LIGHT;

    palTogglePad(GPIOB, GPIOB_LED_DEBUG);

    messagebus_topic_publish(&ranging_topic, &msg, sizeof(msg));
}

static void advertise_timer_cb(void *t)
{
    virtual_timer_t *timer = (virtual_timer_t *)t;

    chSysLockFromISR();
    chVTSetI(timer, UWB_ADVERTISE_TIMER_PERIOD, advertise_timer_cb, t);
    chEvtBroadcastI(&advertise_timer_event);
    chSysUnlockFromISR();
}

static void anchor_position_timer_cb(void *t)
{
    virtual_timer_t *timer = (virtual_timer_t *)t;

    chSysLockFromISR();
    chVTSetI(timer, UWB_ANCHOR_POSITION_TIMER_PERIOD, anchor_position_timer_cb, t);
    chEvtBroadcastI(&anchor_position_timer_event);
    chSysUnlockFromISR();
}

static void parameters_init(void)
{
    /* Prepare parameters. */
    parameter_namespace_declare(&uwb_params.ns, &parameter_root, "uwb");
    parameter_integer_declare_with_default(&uwb_params.mac_addr, &uwb_params.ns, "mac_addr", 0x00);
    parameter_integer_declare_with_default(&uwb_params.pan_id, &uwb_params.ns, "pan_id", 0x00);

    parameter_namespace_declare(&uwb_params.anchor.ns, &uwb_params.ns, "anchor");
    parameter_boolean_declare_with_default(&uwb_params.anchor.is_anchor, &uwb_params.anchor.ns, "is_anchor",
                                           false);
    parameter_namespace_declare(&uwb_params.anchor.position.ns, &uwb_params.anchor.ns, "position");
    parameter_scalar_declare_with_default(&uwb_params.anchor.position.x, &uwb_params.anchor.position.ns, "x", 0.);
    parameter_scalar_declare_with_default(&uwb_params.anchor.position.y, &uwb_params.anchor.position.ns, "y", 0.);
    parameter_scalar_declare_with_default(&uwb_params.anchor.position.z, &uwb_params.anchor.position.ns, "z", 0.);
}

static void topics_init(void)
{
    /* Prepare topic for range information */
    messagebus_topic_init(&ranging_topic, &ranging_topic_lock, &ranging_topic_condvar,
                          &ranging_topic_buffer, sizeof(ranging_topic_buffer));
    messagebus_advertise_topic(&bus, &ranging_topic, "/range");
}

static void hardware_init(void)
{
    decawave_start();

    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_TFRS, true);
    dwt_setcallbacks(frame_tx_done_cb, frame_rx_cb, NULL, NULL);

    dwt_setrxantennadelay(RX_ANT_DLY);

    dwt_setrxtimeout(0); // Disable RX timeout
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void events_init(void)
{
    /* Setup a virtual timer to schedule measurement advertisement. */
    static virtual_timer_t advertise_timer;
    chVTObjectInit(&advertise_timer);
    chVTSet(&advertise_timer, MS2ST(500), advertise_timer_cb, &advertise_timer);

    /* Setup a virtual timer to schedule measurement advertisement. */
    static virtual_timer_t anchor_position_timer;
    chVTObjectInit(&anchor_position_timer);
    chVTSet(&anchor_position_timer, MS2ST(200), anchor_position_timer_cb, &anchor_position_timer);

    /* Register event listeners */
    static event_listener_t uwb_int_listener, advertise_timer_listener, anchor_position_listener;
    chEvtRegisterMask(&exti_uwb_event, &uwb_int_listener, EVENT_UWB_INT);
    chEvtRegisterMask(&advertise_timer_event, &advertise_timer_listener, EVENT_ADVERTISE_TIMER);
    chEvtRegisterMask(&anchor_position_timer_event, &anchor_position_listener, EVENT_ANCHOR_POSITION_TIMER);
}
