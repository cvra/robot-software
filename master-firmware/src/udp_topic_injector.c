#include <ch.h>
#include <lwip/api.h>
#include "udp_topic_injector.h"
#include "msgbus_protobuf.h"
#include "main.h"

static void udp_topic_injector_thd(void *p)
{
    (void)p;

    struct netconn *conn;

    /* Listens to UDP on port 10'000 */
    conn = netconn_new(NETCONN_UDP);
    chDbgAssert(conn != NULL, "Cannot create a connection object");
    netconn_bind(conn, IPADDR_ANY, 10000);

    while (true) {
        struct netbuf *buf;

        /* Read a datagram */
        if (netconn_recv(conn, &buf) != ERR_OK) {
            continue;
        }

        /* TODO: Better than a static object ? */
        static uint8_t msg[1024];
        netbuf_copy(buf, msg, sizeof(msg));

        messagebus_inject_encoded_message(&bus, msg, netbuf_len(buf));

        netbuf_delete(buf);
    }
}

void udp_topic_injector_start(void)
{
    static THD_WORKING_AREA(wa, 2048);
    chThdCreateStatic(wa, sizeof(wa), NORMALPRIO, udp_topic_injector_thd, NULL);
}
