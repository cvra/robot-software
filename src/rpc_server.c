#include <lwip/api.h>
#include <simplerpc/service_call.h>
#include <simplerpc/message.h>
#include <serial-datagram/serial_datagram.h>
#include "rpc_server.h"

#define RPC_SERVER_STACKSIZE 2048
#define RPC_SERVER_PORT 20001
#define MSG_SERVER_PORT 20000

THD_WORKING_AREA(wa_rpc_server, RPC_SERVER_STACKSIZE);

static uint8_t input_buffer[1024];
static uint8_t output_buffer[1024];

static bool method_called;
static size_t output_bytes_written;

static void ping_cb(int argc, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) argc;
    (void) input;
    cmp_write_str(output, "pong", 4);
}

service_call_method service_call_callbacks[] = {
    {.name = "ping", .cb = ping_cb}
};

/** Adapts the serial datagram send callback API to the netconn one.
 *
 * @param [in] arg Pointer to the struct netconn instance to use, cast to void *.
 * @param [in] buffer Buffer to transmit, cast to (void *).
 * @param [in] buffer_len Length of the input buffer, in bytes.
 */
static void netconn_serial_datagram_tx_adapter(void *arg, const void *buffer, size_t buffer_len)
{
    struct netconn *conn = (struct netconn *)arg;

    /* We don't know the lifetime of buffer so we must use NETCONN_COPY. */
    netconn_write(conn, buffer, buffer_len, NETCONN_COPY);
}

static void message_cb(int argc, cmp_ctx_t *input)
{
    (void) argc;
    bool res;
    cmp_read_bool(input, &res);

    if (res) {
        palClearPad(GPIOC, GPIOC_LED);
    } else {
        palSetPad(GPIOC, GPIOC_LED);
    }

    uint8_t request[100];
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;
    ip_addr_t server;
    const int port = 20000;

    message_encode(&ctx, &mem, request, sizeof request, "demo", 1);
    cmp_write_str(&ctx, "heil", 4);
    IP4_ADDR(&server, 192, 168, 2, 1);

    message_transmit(request, cmp_mem_access_get_pos(&mem), &server, port);
}

static message_method_t message_callbacks[] = {
    {.name = "test", .cb = message_cb}
};


/* Callback fired when a serial datagram is received via TCP. */
static void serial_datagram_recv_cb(const void *data, size_t len, void *arg)
{
    (void) arg;
    method_called = true;

    output_bytes_written = service_call_process(data, len, output_buffer, sizeof output_buffer,
                                                service_call_callbacks,
                                                sizeof(service_call_callbacks) / sizeof (service_call_callbacks[0]));

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
                                         serial_datagram_recv_cb,
                                         NULL);

        error = netconn_accept(conn, &client_conn);

        if (error != ERR_OK) {
            continue;
        }

        /* method_called will be set to true once a callback is fired. */
        method_called = false;
        output_bytes_written = 0;

        while (!method_called) {
            /* Tries to receive something from the connection. */
            err = netconn_recv(client_conn, &buf);

            /* If connection closed early */
            if (err != ERR_OK) {
                break;
            }

            do {
                netbuf_data(buf, (void **)&data, &len);
                err = serial_datagram_receive(&handler, data, len);
            } while (netbuf_next(buf) >= 0);
            netbuf_delete(buf);
        }

        if (output_bytes_written > 0) {
            netconn_write(client_conn, output_buffer, output_bytes_written, NETCONN_COPY);
        }

        netconn_close(client_conn);
        netconn_delete(client_conn);
    }

    return MSG_OK;
}

void rpc_server_init(void)
{
    chThdCreateStatic(wa_rpc_server,
                      RPC_SERVER_STACKSIZE,
                      RPC_SERVER_PRIO,
                      rpc_server_thread,
                      NULL);
}


size_t rpc_transmit(uint8_t *input_buffer, size_t input_buffer_size,
                    uint8_t *output_buffer, size_t output_buffer_size,
                    ip_addr_t *addr, uint16_t port)
{
    struct netconn *conn;
    int err;

    cmp_ctx_t ctx; /* For cmp_mem_access. */
    cmp_mem_access_t mem;
    struct netbuf *buf;

    u16_t len;
    char *data;


    conn = netconn_new(NETCONN_TCP);

    if (conn == NULL) {
        return -1;
    }

    err = netconn_connect(conn, addr, port);

    if (err != ERR_OK) {
        goto fail;
    }

    serial_datagram_send((void *)input_buffer, input_buffer_size,
                         netconn_serial_datagram_tx_adapter, (void *)conn);

    cmp_mem_access_init(&ctx, &mem, output_buffer, output_buffer_size);

    while (1) {
        err = netconn_recv(conn, &buf);

        /* If connection was closed by server, abort */
        if (err != ERR_OK) {
            break;
        }

        do {
            netbuf_data(buf, (void **)&data, &len);

            /* Append data to buffer. */
            ctx.write(&ctx, data, len);

        } while (netbuf_next(buf) >= 0);
        netbuf_delete(buf);
    }

    netconn_delete(conn);
    return cmp_mem_access_get_pos(&mem);

fail:
    netconn_delete(conn);
    return -1;
}

msg_t message_server_thread(void *arg)
{
  struct netconn *conn;
  struct netbuf *buf;
  static uint8_t buffer[4096];
  err_t err;
  LWIP_UNUSED_ARG(arg);

  conn = netconn_new(NETCONN_UDP);
  netconn_bind(conn, NULL, 20000);

  while (1) {
    err = netconn_recv(conn, &buf);

    if (err == ERR_OK) {
        netbuf_copy(buf, buffer, buf->p->tot_len);
        message_process(buffer, buf->p->tot_len, message_callbacks, 1);
    }
    netbuf_delete(buf);
  }
}

void message_server_init(void)
{
    static THD_WORKING_AREA(wa_msg_server, 2048);

    chThdCreateStatic(wa_msg_server,
                      2048,
                      RPC_SERVER_PRIO,
                      message_server_thread,
                      NULL);

}

void message_transmit(uint8_t *input_buffer, size_t input_buffer_size, ip_addr_t *addr, uint16_t port)
{
    struct netconn *conn;
    struct netbuf *buf;

    conn = netconn_new(NETCONN_UDP);
    buf = netbuf_new();

    netbuf_ref(buf, input_buffer, input_buffer_size);

    netconn_sendto(conn, buf, addr, port);

    netbuf_delete(buf);
    netconn_delete(conn);
}
