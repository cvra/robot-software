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

void can_bridge_init(void)
{
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
