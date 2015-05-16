#include <lwip/api.h>
#include <string.h>
#include <serial-can-bridge/can_frame.h>
#include <serial-datagram/serial_datagram.h>
#include <serial-can-bridge/serial_can_bridge.h>

#define CAN_BRIDGE_STACKSIZE 2048
THD_WORKING_AREA(wa_can_bridge, CAN_BRIDGE_STACKSIZE);

#define CAN_BRIDGE_RX_STACKSIZE 1024
#define CAN_BRIDGE_TX_STACKSIZE 1024

#define CAN_BRIDGE_RX_QUEUE_SIZE    32
#define CAN_BRIDGE_TX_QUEUE_SIZE    512

memory_pool_t can_bridge_rx_pool;
memory_pool_t can_bridge_tx_pool;
mailbox_t can_bridge_rx_queue;
mailbox_t can_bridge_tx_queue;
SEMAPHORE_DECL(can_bridge_is_initialized, 0);

msg_t rx_mbox_buf[CAN_BRIDGE_RX_QUEUE_SIZE];
struct can_frame rx_pool_buf[CAN_BRIDGE_RX_QUEUE_SIZE];
msg_t tx_mbox_buf[CAN_BRIDGE_TX_QUEUE_SIZE];
struct can_frame tx_pool_buf[CAN_BRIDGE_TX_QUEUE_SIZE];

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
    // default: only standard frames pass the filter
    can_bridge_filter_id = 0;
    can_bridge_filter_mask = CAN_FRAME_EXT_FLAG;

    // rx queue
    chMBObjectInit(&can_bridge_rx_queue, rx_mbox_buf, CAN_BRIDGE_RX_QUEUE_SIZE);
    chPoolObjectInit(&can_bridge_rx_pool, sizeof(struct can_frame), NULL);
    chPoolLoadArray(&can_bridge_rx_pool, rx_pool_buf, sizeof(rx_pool_buf)/sizeof(struct can_frame));

    // tx queue
    chMBObjectInit(&can_bridge_tx_queue, tx_mbox_buf, CAN_BRIDGE_TX_QUEUE_SIZE);
    chPoolObjectInit(&can_bridge_tx_pool, sizeof(struct can_frame), NULL);
    chPoolLoadArray(&can_bridge_tx_pool, tx_pool_buf, sizeof(tx_pool_buf)/sizeof(struct can_frame));

    // signal to uavcan thread that bridge is initialized.
    chSemSignal(&can_bridge_is_initialized);

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
        if (instance == NULL) {
            chSysHalt("malloc() failed.");
        }
        instance->conn = client_conn;

        chBSemObjectInit(&instance->tx_finished, true);

        thread_t *tx, *rx;
        rx = chThdCreateFromHeap(NULL, /* Use system heap */
                            CAN_BRIDGE_RX_STACKSIZE,
                            CAN_BRIDGE_PRIO,
                            can_bridge_rx_thread,
                            (void *)instance);

        tx = chThdCreateFromHeap(NULL, /* Use system heap */
                            CAN_BRIDGE_TX_STACKSIZE,
                            CAN_BRIDGE_PRIO,
                            can_bridge_tx_thread,
                            (void *)instance);
        chThdWait(tx);
        chThdWait(rx);
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

    struct can_frame *framep;

    static uint8_t outbuf[32];
    size_t outlen;

    /* Wait as long as the thread is not finished (semaphore not taken) */
    while (chBSemWaitTimeout(&instance->tx_finished, TIME_IMMEDIATE) == MSG_TIMEOUT) {
        msg_t m = chMBFetch(&can_bridge_rx_queue, (msg_t *)&framep, MS2ST(100));
        if (m != MSG_OK) {
            continue;
        }

        outlen = sizeof(outbuf);
        if (can_bridge_frame_write(framep, outbuf, &outlen)) {
            serial_datagram_send(outbuf, outlen, serial_write, instance);
        }

        chPoolFree(&can_bridge_rx_pool, framep);
    }

    netconn_close(instance->conn);
    netconn_delete(instance->conn);
    free(instance);

    return MSG_OK;
}

void can_interface_send(struct can_frame *frame)
{
    struct can_frame *tx = (struct can_frame *)chPoolAlloc(&can_bridge_tx_pool);
    if (tx == NULL) {
        return;
    }
    tx->id = frame->id;
    tx->dlc = frame->dlc;
    tx->data.u32[0] = frame->data.u32[0];
    tx->data.u32[1] = frame->data.u32[1];
    if (chMBPost(&can_bridge_tx_queue, (msg_t)tx, MS2ST(100)) != MSG_OK) {
        // couldn't post, free memory
        chPoolFree(&can_bridge_tx_pool, tx);
    }
    return;
}
