#include <lwip/api.h>
#include <string.h>
#include <serial-can-bridge/can_frame.h>
#include <serial-datagram/serial_datagram.h>
#include <serial-can-bridge/serial_can_bridge.h>

#define CAN_BRIDGE_STACKSIZE 2048
THD_WORKING_AREA(wa_can_bridge, CAN_BRIDGE_STACKSIZE);

#define CAN_BRIDGE_RX_STACKSIZE 1024
#define CAN_BRIDGE_TX_STACKSIZE 1024

/** Thread running the CAN bridge. */
msg_t can_bridge_thread(void *p);
msg_t can_bridge_rx_thread(void *p);
msg_t can_bridge_tx_thread(void *p);
bool can_interface_receive(struct can_frame *frame);

struct can_bridge_instance_t {
    struct netconn *conn;
    binary_semaphore_t tx_finished;
};

static const CANConfig can1_config = {
    .mcr = (1 << 6)  /* Automatic bus-off management enabled. */
         | (1 << 2), /* Message are prioritized by order of arrival. */

    /* APB Clock is 42 Mhz, bitrate 1Mhz */
    .btr = (1 << 0)  /* Baudrate prescaler (10 bits) */
         | (11 << 16)/* Time segment 1 (3 bits) */
         | (7 << 20) /* Time segment 2 (3 bits) */
         | (0 << 24) /* Resync jump width (2 bits) */

#if 0
         | (1 << 30) /* Loopback mode enabled */
#endif
};

void can_bridge_init(void)
{
    canStart(&CAND1, &can1_config);
    chThdCreateStatic(wa_can_bridge,
                      CAN_BRIDGE_STACKSIZE,
                      CAN_BRIDGE_PRIO,
                      can_bridge_thread,
                      NULL);
}

msg_t can_bridge_thread(void *p)
{
    struct netconn *conn, *client_conn;
    struct can_bridge_instance_t *instance;
    int error;

    (void)p;

    chRegSetThreadName("can_bridge_thd");

    /* Creates a TCP server on port 1337 */
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, IP_ADDR_ANY, 1337);
    netconn_listen(conn);


    while (1) {
        error = netconn_accept(conn, &client_conn);

        if (error != ERR_OK) {
            continue;
        }

        instance = malloc(sizeof(struct can_bridge_instance_t));
        instance->conn = client_conn;

        chBSemObjectInit(&instance->tx_finished, true);

        chThdCreateFromHeap(NULL, /* Use system heap */
                            CAN_BRIDGE_RX_STACKSIZE,
                            CAN_BRIDGE_PRIO,
                            can_bridge_rx_thread,
                            (void *)instance);

        chThdCreateFromHeap(NULL, /* Use system heap */
                            CAN_BRIDGE_TX_STACKSIZE,
                            CAN_BRIDGE_PRIO,
                            can_bridge_tx_thread,
                            (void *)instance);
    }
    return MSG_OK;
}

msg_t can_bridge_tx_thread(void *p)
{
    chRegSetThreadName("can_bridge_tx");
    struct can_bridge_instance_t *instance = (struct can_bridge_instance_t *)p;

    struct netbuf *buf;
    char *data;
    u16_t len;
    err_t err;

    serial_datagram_rcv_handler_t rcv;

    static char datagram_buf[32];

    serial_datagram_rcv_handler_init(
        &rcv,
        datagram_buf,
        sizeof(datagram_buf),
        can_bridge_datagram_rcv_cb,
        NULL);

    while (1) {
        /* Tries to receive something from the connection. */
        err = netconn_recv(instance->conn, &buf);

        /* If connection closed, exit the thread. */
        if (err != ERR_OK) {
            break;
        }

        do {
            netbuf_data(buf, (void **)&data, &len);
            err = serial_datagram_receive(&rcv, data, len);
        } while (netbuf_next(buf) >= 0);
        netbuf_delete(buf);
    }

    chBSemSignal(&instance->tx_finished);

    return MSG_OK;
}

void serial_write(void *arg, const void *p, size_t len)
{
    struct can_bridge_instance_t *instance = (struct can_bridge_instance_t *)arg;
    if (len != 0) {
        netconn_write(instance->conn, p, len, NETCONN_COPY);
    }
}

msg_t can_bridge_rx_thread(void *p)
{
    chRegSetThreadName("can_bridge_rx");
    struct can_bridge_instance_t *instance = (struct can_bridge_instance_t *)p;

    struct can_frame frame;

    static uint8_t outbuf[32];
    size_t outlen;

    /* Wait as long as the thread is not finished (semaphore not taken) */
    while (chBSemWaitTimeout(&instance->tx_finished, TIME_IMMEDIATE) == MSG_TIMEOUT) {
        if (!can_interface_receive(&frame)) {
            continue;
        }

        outlen = sizeof(outbuf);
        if (can_bridge_frame_write(&frame, outbuf, &outlen)) {
            serial_datagram_send(outbuf, outlen, serial_write, instance);
        }
    }

    netconn_close(instance->conn);
    netconn_delete(instance->conn);
    free(instance);

    return MSG_OK;
}

void can_interface_send(struct can_frame *frame)
{
    canmbx_t mailbox = 0;
    systime_t timeout = TIME_INFINITE;

    CANTxFrame ctf;

    ctf.DLC = frame->dlc;

    if (frame->id & CAN_FRAME_RTR_FLAG) {
        ctf.RTR = 1;
    } else {
        ctf.RTR = 0;
    }

    if (frame->id & CAN_FRAME_EXT_FLAG) {
        ctf.IDE = 1;
        ctf.EID = frame->id & CAN_FRAME_EXT_ID_MASK;
    } else {
        ctf.IDE = 0;
        ctf.SID = frame->id & CAN_FRAME_STD_ID_MASK;
    }

    ctf.data32[0] = frame->data.u32[0];
    ctf.data32[1] = frame->data.u32[1];

    canTransmit(&CAND1, mailbox, &ctf, timeout);
}

bool can_interface_receive(struct can_frame *frame)
{
    CANRxFrame crf;
    canmbx_t mailbox = 0;
    systime_t timeout = 1;

    msg_t retval;

    retval = canReceive(&CAND1, mailbox, &crf, timeout);

    if (retval != MSG_OK || !can_bridge_id_passes_filter(frame->id)) {
        return false;
    }

    frame->dlc = crf.DLC;

    if (crf.IDE == 1) {
        frame->id = crf.EID | CAN_FRAME_EXT_FLAG;
    } else {
        frame->id = crf.SID;
    }

    if (crf.RTR) {
        frame->id |= CAN_FRAME_RTR_FLAG;
    }

    frame->data.u32[0] = crf.data32[0];
    frame->data.u32[1] = crf.data32[1];

    return true;
}
