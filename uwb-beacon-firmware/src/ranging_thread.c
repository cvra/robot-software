#include <ch.h>

#include <string.h>

#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"

#include "main.h"
#include "decawave_interface.h"
#include "ranging_thread.h"
#include "uwb_protocol.h"
#include "exti.h"
#include "state_estimation_thread.h"
#include "trace_points.h"

/** Speed of light in Decawave units */
#define SPEED_OF_LIGHT (299792458.0 / (128 * 499.2e6))

/* Antenna delay for our UWB board. */
#define RX_ANT_DLY 32915

#define EVENT_UWB_INT (1 << 0)
#define EVENT_ADVERTISE_TIMER (1 << 1)
#define EVENT_ANCHOR_POSITION_TIMER (1 << 2)
#define EVENT_TAG_POSITION_TIMER (1 << 3)
#define EVENT_DATA_PACKET_READY (1 << 4)

/* TODO: Put this in parameters. */
#define UWB_ANCHOR_POSITION_TIMER_PERIOD TIME_S2I(1)
#define UWB_TAG_POSITION_TIMER_PERIOD TIME_MS2I(300)

static uwb_protocol_handler_t handler;

static messagebus_topic_t ranging_topic;
static MUTEX_DECL(ranging_topic_lock);
static CONDVAR_DECL(ranging_topic_condvar);
static range_msg_t ranging_topic_buffer;

static messagebus_topic_t anchor_position_topic;
static MUTEX_DECL(anchor_position_topic_lock);
static CONDVAR_DECL(anchor_position_topic_condvar);
static anchor_position_msg_t anchor_position_topic_buffer;

static messagebus_topic_t tag_position_topic;
static MUTEX_DECL(tag_position_topic_lock);
static CONDVAR_DECL(tag_position_topic_condvar);
static tag_position_msg_t tag_position_topic_buffer;

static messagebus_topic_t data_packet_topic;
static MUTEX_DECL(data_packet_topic_lock);
static CONDVAR_DECL(data_packet_topic_condvar);
static data_packet_msg_t data_packet_topic_buffer;

static EVENTSOURCE_DECL(advertise_timer_event);
static EVENTSOURCE_DECL(anchor_position_timer_event);
static EVENTSOURCE_DECL(tag_position_timer_event);
static EVENTSOURCE_DECL(data_packet_ready_event);

static BSEMAPHORE_DECL(data_packet_sent, true);
static MUTEX_DECL(data_packet_lock);
static const uint8_t* data_packet;
static uint16_t data_packet_dst_mac;
static size_t data_packet_size;

static struct {
    parameter_namespace_t ns;
    parameter_t mac_addr;
    parameter_t pan_id;
    parameter_t antenna_delay;
    struct {
        parameter_namespace_t ns;
        parameter_t is_anchor;
        parameter_t advertisement_period_ms;
        struct {
            parameter_namespace_t ns;
            parameter_t x, y, z;
        } position;
    } anchor;
} uwb_params;

static void ranging_thread(void* p);
static void ranging_found_cb(uint16_t addr, uint64_t time);
static void anchor_position_received_cb(uint16_t addr, float x, float y, float z);
static void data_packet_received_cb(const uint8_t* msg, size_t size, uint16_t src, uint16_t dst);
static void tag_position_received_cb(uint16_t addr, float x, float y);
static void topics_init(void);
static void parameters_init(void);
static void hardware_init(void);
static void events_init(void);

static void advertise_timer_cb(void* t);
static void anchor_position_timer_cb(void* t);
static void tag_position_timer_cb(void* t);
static void frame_tx_done_cb(const dwt_cb_data_t* data);
static void frame_rx_cb(const dwt_cb_data_t* data);
static void frame_rx_timeout_cb(const dwt_cb_data_t* data);
static void frame_rx_error_cb(const dwt_cb_data_t* data);

void ranging_start(void)
{
    static THD_WORKING_AREA(ranging_wa, 512);
    chThdCreateStatic(ranging_wa, sizeof(ranging_wa), HIGHPRIO, ranging_thread, NULL);
}

static void ranging_thread(void* p)
{
    (void)p;
    chRegSetThreadName("ranging");

    uwb_protocol_handler_init(&handler);
    handler.ranging_found_cb = ranging_found_cb;
    handler.anchor_position_received_cb = anchor_position_received_cb;
    handler.tag_position_received_cb = tag_position_received_cb;
    handler.user_data_received_cb = data_packet_received_cb;

    parameters_init();
    topics_init();
    hardware_init();
    events_init();

    static uint8_t frame[1024];

    int current_anchor_mac_index = 0;
    uint16_t anchor_macs[] = {7, 10, 11, 14};
    int nb_anchor_macs = sizeof(anchor_macs) / sizeof(anchor_macs[0]);

    while (1) {
        /* Wait for an interrupt coming from the UWB module. */
        eventmask_t flags = chEvtWaitOne(ALL_EVENTS);

        /* Copy parameters */
        handler.is_anchor = parameter_boolean_get(&uwb_params.anchor.is_anchor);
        handler.address = parameter_integer_get(&uwb_params.mac_addr);
        handler.pan_id = parameter_integer_get(&uwb_params.pan_id);

        if (parameter_changed(&uwb_params.antenna_delay)) {
            dwt_setrxantennadelay(parameter_integer_get(&uwb_params.antenna_delay));
        }

        if (flags & EVENT_ADVERTISE_TIMER) {
            if (!handler.is_anchor && nb_anchor_macs > 0) {
                /* First disable transceiver */
                dwt_forcetrxoff();

                /* Initiate measurement sequence */
                uwb_initiate_measurement(&handler, frame, anchor_macs[current_anchor_mac_index]);

                current_anchor_mac_index++;
                if (current_anchor_mac_index == nb_anchor_macs) {
                    current_anchor_mac_index = 0;
                }
            }
        }

        if (flags & EVENT_ANCHOR_POSITION_TIMER) {
            /* Make sure we are an anchor before we send an anchor position
             * message. */
            if (!handler.is_anchor) {
                continue;
            }

            /* First disable transceiver */
            dwt_forcetrxoff();

            float x, y, z;
            x = parameter_scalar_get(&uwb_params.anchor.position.x);
            y = parameter_scalar_get(&uwb_params.anchor.position.y);
            z = parameter_scalar_get(&uwb_params.anchor.position.z);

            uwb_send_anchor_position(&handler, x, y, z, frame);
        }

        if (flags & EVENT_DATA_PACKET_READY) {
            dwt_forcetrxoff();
            uwb_send_data_packet(&handler, data_packet_dst_mac, data_packet, data_packet_size, frame);
        }

        if (flags & EVENT_UWB_INT) {
            /* Process the interrupt. */
            dwt_isr();
        }
    }
}

static void frame_tx_done_cb(const dwt_cb_data_t* data)
{
    (void)data;
    trace(TRACE_POINT_UWB_TX_DONE);
}

/* TODO: Handle RX errors as well, especially timeouts. */
static void frame_rx_cb(const dwt_cb_data_t* data)
{
    static uint8_t frame[1024];
    trace(TRACE_POINT_UWB_RX);
    uint64_t rx_ts = decawave_get_rx_timestamp_u64();

    dwt_readrxdata(frame, data->datalength, 0);

    uwb_process_incoming_frame(&handler, frame, data->datalength, rx_ts);

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void frame_rx_timeout_cb(const dwt_cb_data_t* data)
{
    (void)data;

    board_led_toggle(BOARD_LED_DEBUG);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void frame_rx_error_cb(const dwt_cb_data_t* data)
{
    (void)data;

    board_led_toggle(BOARD_LED_DEBUG);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void anchor_position_received_cb(uint16_t addr, float x, float y, float z)
{
    anchor_position_msg_t msg;

    /* TODO: For some reason the macro ST2US creates an overflow. */
    uint32_t ts = chVTGetSystemTime() * (1000000 / CH_CFG_ST_FREQUENCY);

    msg.timestamp = ts;
    msg.anchor_addr = addr;
    msg.x = x;
    msg.y = y;
    msg.z = z;

    messagebus_topic_publish(&anchor_position_topic, &msg, sizeof(msg));
}

static void tag_position_received_cb(uint16_t addr, float x, float y)
{
    tag_position_msg_t msg;

    /* TODO: For some reason the macro ST2US creates an overflow. */
    uint32_t ts = chVTGetSystemTime() * (1000000 / CH_CFG_ST_FREQUENCY);

    msg.timestamp = ts;
    msg.tag_addr = addr;
    msg.x = x;
    msg.y = y;

    messagebus_topic_publish(&tag_position_topic, &msg, sizeof(msg));
}

static void ranging_found_cb(uint16_t addr, uint64_t time)
{
    range_msg_t msg;
    /* TODO: For some reason the macro ST2US creates an overflow. */
    uint32_t ts = chVTGetSystemTime() * (1000000 / CH_CFG_ST_FREQUENCY);

    msg.timestamp = ts;
    msg.anchor_addr = addr;
    msg.range = time * SPEED_OF_LIGHT;

    messagebus_topic_publish(&ranging_topic, &msg, sizeof(msg));
}

static void data_packet_received_cb(const uint8_t* data, size_t size, uint16_t src, uint16_t dst)
{
    static data_packet_msg_t msg;

    msg.dst_mac = dst;
    msg.src_mac = src;
    msg.data_size = size;
    memcpy(msg.data, data, size);

    messagebus_topic_publish(&data_packet_topic, &msg, sizeof(msg));
}

static void advertise_timer_cb(void* t)
{
    virtual_timer_t* timer = (virtual_timer_t*)t;

    chSysLockFromISR();
    int period = uwb_params.anchor.advertisement_period_ms.value.i;
    chVTSetI(timer, TIME_MS2I(period), advertise_timer_cb, t);
    chEvtBroadcastI(&advertise_timer_event);
    chSysUnlockFromISR();
}

static void anchor_position_timer_cb(void* t)
{
    virtual_timer_t* timer = (virtual_timer_t*)t;

    chSysLockFromISR();
    chVTSetI(timer, UWB_ANCHOR_POSITION_TIMER_PERIOD, anchor_position_timer_cb, t);
    chEvtBroadcastI(&anchor_position_timer_event);
    chSysUnlockFromISR();
}

static void tag_position_timer_cb(void* t)
{
    virtual_timer_t* timer = (virtual_timer_t*)t;

    chSysLockFromISR();
    chVTSetI(timer, UWB_TAG_POSITION_TIMER_PERIOD, tag_position_timer_cb, t);
    chEvtBroadcastI(&tag_position_timer_event);
    chSysUnlockFromISR();
}

static void parameters_init(void)
{
    /* Prepare parameters. */
    parameter_namespace_declare(&uwb_params.ns, &parameter_root, "uwb");
    parameter_integer_declare_with_default(&uwb_params.mac_addr, &uwb_params.ns, "mac_addr", 0x00);
    parameter_integer_declare_with_default(&uwb_params.pan_id, &uwb_params.ns, "pan_id", 0x00);
    parameter_integer_declare_with_default(&uwb_params.antenna_delay,
                                           &uwb_params.ns,
                                           "antenna_delay",
                                           RX_ANT_DLY);

    parameter_namespace_declare(&uwb_params.anchor.ns, &uwb_params.ns, "anchor");
    parameter_boolean_declare_with_default(&uwb_params.anchor.is_anchor,
                                           &uwb_params.anchor.ns,
                                           "is_anchor",
                                           false);
    parameter_integer_declare_with_default(&uwb_params.anchor.advertisement_period_ms,
                                           &uwb_params.anchor.ns,
                                           "advertisement_period_ms",
                                           100);

    parameter_namespace_declare(&uwb_params.anchor.position.ns, &uwb_params.anchor.ns, "position");
    parameter_scalar_declare_with_default(&uwb_params.anchor.position.x,
                                          &uwb_params.anchor.position.ns,
                                          "x",
                                          0.);
    parameter_scalar_declare_with_default(&uwb_params.anchor.position.y,
                                          &uwb_params.anchor.position.ns,
                                          "y",
                                          0.);
    parameter_scalar_declare_with_default(&uwb_params.anchor.position.z,
                                          &uwb_params.anchor.position.ns,
                                          "z",
                                          0.);
}

static void topics_init(void)
{
    /* Prepare topic for range information */
    messagebus_topic_init(&ranging_topic, &ranging_topic_lock, &ranging_topic_condvar,
                          &ranging_topic_buffer, sizeof(ranging_topic_buffer));
    messagebus_advertise_topic(&bus, &ranging_topic, "/range");

    /* Prepare topic for anchor positions */
    messagebus_topic_init(&anchor_position_topic,
                          &anchor_position_topic_lock,
                          &anchor_position_topic_condvar,
                          &anchor_position_topic_buffer,
                          sizeof(anchor_position_topic_buffer));
    messagebus_advertise_topic(&bus, &anchor_position_topic, "/anchors_pos");

    /* Prepare topic for tag positions */
    messagebus_topic_init(&tag_position_topic,
                          &tag_position_topic_lock,
                          &tag_position_topic_condvar,
                          &tag_position_topic_buffer,
                          sizeof(tag_position_topic_buffer));
    messagebus_advertise_topic(&bus, &tag_position_topic, "/tags_pos");

    /* Prepare topic for received data packets */
    messagebus_topic_init(&data_packet_topic,
                          &data_packet_topic_lock,
                          &data_packet_topic_condvar,
                          &data_packet_topic_buffer,
                          sizeof(data_packet_topic_buffer));
    messagebus_advertise_topic(&bus, &data_packet_topic, "/uwb_data");
}

static void hardware_init(void)
{
    decawave_start();

    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_TFRS | DWT_INT_RPHE | DWT_INT_SFDT | DWT_INT_RFSL | DWT_INT_RFCE | DWT_INT_ARFE | DWT_INT_RFTO | DWT_INT_RXPTO, true);
    dwt_setcallbacks(frame_tx_done_cb, frame_rx_cb, frame_rx_timeout_cb, frame_rx_error_cb);

    dwt_setrxantennadelay(parameter_integer_get(&uwb_params.antenna_delay));

    dwt_setrxtimeout(0); // Disable RX timeout
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void events_init(void)
{
    /* Setup a virtual timer to schedule measurement advertisement. */
    static virtual_timer_t advertise_timer;
    chVTObjectInit(&advertise_timer);
    chVTSet(&advertise_timer, TIME_MS2I(500), advertise_timer_cb, &advertise_timer);

    /* Setup a virtual timer to schedule anchor position broadcasts. */
    static virtual_timer_t anchor_position_timer;
    chVTObjectInit(&anchor_position_timer);
    chVTSet(&anchor_position_timer, TIME_MS2I(200), anchor_position_timer_cb, &anchor_position_timer);

    /* Setup a virtual timer to schedule anchor position broadcasts. */
    static virtual_timer_t tag_position_timer;
    chVTObjectInit(&tag_position_timer);
    chVTSet(&tag_position_timer, TIME_MS2I(400), tag_position_timer_cb, &tag_position_timer);

    /* Register event listeners */
    static event_listener_t uwb_int_listener, advertise_timer_listener, anchor_position_listener,
        tag_position_listener, data_packet_ready_listener;
    chEvtRegisterMask(&uwb_event, &uwb_int_listener, EVENT_UWB_INT);
    chEvtRegisterMask(&advertise_timer_event, &advertise_timer_listener, EVENT_ADVERTISE_TIMER);
    chEvtRegisterMask(&anchor_position_timer_event,
                      &anchor_position_listener,
                      EVENT_ANCHOR_POSITION_TIMER);

    chEvtRegisterMask(&data_packet_ready_event, &data_packet_ready_listener, EVENT_DATA_PACKET_READY);

    chEvtRegisterMask(&tag_position_timer_event,
                      &tag_position_listener,
                      EVENT_TAG_POSITION_TIMER);
}

void ranging_send_data_packet(const uint8_t* data, size_t data_size, uint16_t dst_mac)
{
    chMtxLock(&data_packet_lock);

    data_packet = data;
    data_packet_size = data_size;

    data_packet_dst_mac = dst_mac;

    /* Tell the ranging thread that we want to send a packet. */
    chEvtBroadcastI(&data_packet_ready_event);

    /* wait for the data packet to be handled by the module */
    chBSemWait(&data_packet_sent);

    chMtxUnlock(&data_packet_lock);
}
