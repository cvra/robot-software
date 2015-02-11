#include <lwip/api.h>
#include <string.h>

#define CAN_BRIDGE_STACKSIZE 2048
THD_WORKING_AREA(wa_can_bridge, CAN_BRIDGE_STACKSIZE);

#define CAN_BRIDGE_RX_STACKSIZE 1024
#define CAN_BRIDGE_TX_STACKSIZE 1024

/** Thread running the CAN bridge. */
msg_t can_bridge_thread(void *p);
msg_t can_bridge_rx_thread(void *p);
msg_t can_bridge_tx_thread(void *p);

struct can_bridge_instance_t {
    struct netconn *conn;
    binary_semaphore_t tx_finished;
    mutex_t lock;
};

const char *data = "Hello, world\n";

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
        chMtxObjectInit(&instance->lock);


        chThdCreateFromHeap(NULL, /* Use system heap */
                            CAN_BRIDGE_RX_STACKSIZE,
                            CAN_BRIDGE_PRIO,
                            can_bridge_rx_thread,
                            (void *)instance);

        chThdCreateFromHeap(NULL, /* Use system heap */
                            CAN_BRIDGE_TX_STACKSIZE,
                            CAN_BRIDGE_PRIO,
                            can_bridge_rx_thread,
                            (void *)instance);
    }
    return MSG_OK;
}

msg_t can_bridge_tx_thread(void *p)
{
    chRegSetThreadName("can_bridge_tx");
    struct can_bridge_instance_t *instance = (struct can_bridge_instance_t *)p;

    /* XXX Do something */

    chBSemSignal(&instance->tx_finished);

    return MSG_OK;
}

msg_t can_bridge_rx_thread(void *p)
{
    chRegSetThreadName("can_bridge_rx");
    struct can_bridge_instance_t *instance = (struct can_bridge_instance_t *)p;

    /* Do something with the conn */
    chMtxLock(&instance->lock);
    netconn_write(instance->conn, data, strlen(data), NETCONN_NOCOPY);
    chMtxUnlock(&instance->lock);

    /* Do not delete connection before tx thread is finished too. */
    chBSemWait(&instance->tx_finished);

    netconn_close(instance->conn);
    netconn_delete(instance->conn);

    return MSG_OK;
}

