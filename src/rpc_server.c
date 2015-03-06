#include <lwip/api.h>
#include <simplerpc/service_call.h>
#include <serial-datagram/serial_datagram.h>

uint8_t input_buffer[1024];
uint8_t output_buffer[1024];

#define RPC_SERVER_STACKSIZE 2048
#define RPC_SERVER_PORT 20001
THD_WORKING_AREA(wa_rpc_server, RPC_SERVER_STACKSIZE);

bool method_called;

static void ping_cb(int argc, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) argc;
    (void) input;
    cmp_write_str(output, "pong", 4);
}

service_call_method service_call_callbacks[] = {
    {.name = "ping", .cb = ping_cb}
};


/* Callback fired when a serial datagram is received via TCP. */
static void serial_datagram_recv_cb(const void *data, size_t len)
{
    method_called = true;

    service_call_process(data, len, output_buffer, sizeof output_buffer,
                         service_call_callbacks,
                         sizeof(serial_datagram_rcv_handler_init) / sizeof (service_call_method));

}

msg_t rpc_server_thread(void *p)
{
    struct netconn *conn, *client_conn;
    int error;

    (void)p;

    struct netbuf *buf;
    char *data;
    u16_t len;
    err_t err;

    serial_datagram_rcv_handler_t handler;

    chRegSetThreadName("rpc_server");

    /* Creates a TCP server */
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, IP_ADDR_ANY, RPC_SERVER_PORT);
    netconn_listen(conn);

    while (1) {
        serial_datagram_rcv_handler_init(&handler,
                                         input_buffer, sizeof input_buffer,
                                         serial_datagram_recv_cb);

        error = netconn_accept(conn, &client_conn);

        if (error != ERR_OK) {
            continue;
        }

        /* method_called will be set to true once a callback is fired. */
        method_called = false;

        while (!method_called) {
            /* Tries to receive something from the connection. */
            err = netconn_recv(client_conn, &buf);

            /* If connection closed early, exit the thread. */
            if (err != ERR_OK) {
                break;
            }

            do {
                netbuf_data(buf, (void **)&data, &len);
                err = serial_datagram_receive(&handler, data, len);
            } while (netbuf_next(buf) >= 0);
            netbuf_delete(buf);
        }
    }

    return MSG_OK;
}
