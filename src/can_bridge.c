#include <lwip/api.h>
#include <string.h>

#define CAN_BRIDGE_STACKSIZE 2048
THD_WORKING_AREA(wa_can_bridge, CAN_BRIDGE_STACKSIZE);

/** Thread running the CAN bridge. */
msg_t can_bridge_thread(void *p);

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

        netconn_write(client_conn, data, strlen(data), NETCONN_NOCOPY);
        netconn_close(client_conn);

        netconn_delete(client_conn);
    }
    return MSG_OK;
}

