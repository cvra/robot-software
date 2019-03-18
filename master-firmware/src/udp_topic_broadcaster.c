#include <ch.h>

#include <lwip/api.h>

#include "main.h"
#include "error/error.h"
#include "msgbus_protobuf.h"
#include "udp_topic_broadcaster.h"

static messagebus_watchgroup_t watchgroup;
static MUTEX_DECL(watchgroup_lock);
static CONDVAR_DECL(watchgroup_condvar);

#define MSG_MAX_LENGTH 128
#define MSG_BUF_SIZE 16

struct encoded_message {
    uint16_t len;
    uint8_t buf[MSG_MAX_LENGTH];
};

static struct encoded_message msg_buffer[MSG_BUF_SIZE];
static char *msg_mailbox_buf[MSG_BUF_SIZE];
static MAILBOX_DECL(msg_mailbox, msg_mailbox_buf, MSG_BUF_SIZE);
static MEMORYPOOL_DECL(msg_pool, sizeof(struct encoded_message), PORT_NATURAL_ALIGN, NULL);

static void new_topic_cb(messagebus_t *bus, messagebus_topic_t *topic, void *arg)
{
    (void)bus;
    (void)arg;
    topic_metadata_t *metadata = (topic_metadata_t *)topic->metadata;
    NOTICE("Registered topic: %s", topic->name);
    messagebus_watchgroup_watch(&metadata->udp_watcher, &watchgroup, topic);
}

static void udp_topic_encode_thd(void *p)
{
    (void)p;

    messagebus_topic_t *topic;

    uint8_t object_buf[512];

    /* We need to have max priority to avoid skipping messages */
    chThdSetPriority(HIGHPRIO);
    chRegSetThreadName(__FUNCTION__);

    NOTICE("UDP topic broadcaster is ready!");

    while (true) {
        struct encoded_message *msg = chPoolAlloc(&msg_pool);

        if (msg == NULL) {
            WARNING("Dropping a messsage.");
            continue;
        }

        topic = messagebus_watchgroup_wait(&watchgroup);

        msg->len = messagebus_encode_topic_message(topic,
                                                   msg->buf,
                                                   MSG_MAX_LENGTH,
                                                   object_buf,
                                                   sizeof(object_buf));

        if (msg->len > 0) {
            chMBPostTimeout(&msg_mailbox, (msg_t)msg, TIME_IMMEDIATE);
        } else {
            chPoolFree(&msg_pool, msg);
        }
    }
}

static void udp_topic_send_thd(void *p)
{
    (void)p;
    chThdSetPriority(NORMALPRIO);
    chRegSetThreadName(__FUNCTION__);

    struct netconn *conn;
    conn = netconn_new(NETCONN_UDP);

    chDbgAssert(conn != NULL, "Could not create connection");

    while (true) {
        struct encoded_message *msg;
        struct netbuf *buf;

        msg_t res = chMBFetchTimeout(&msg_mailbox, (msg_t *)&msg, TIME_INFINITE);

        if (res != MSG_OK) {
            continue;
        }

        buf = netbuf_new();

        if (buf == NULL) {
            chPoolFree(&msg_pool, msg);
            continue;
        }

        netbuf_ref(buf, msg->buf, msg->len);

        // TODO: take those from parameter tree
        ip_addr_t addr;
        LWIP_GATEWAY(&addr);
        const int port = 10000;

        netconn_sendto(conn, buf, &addr, port);

        netbuf_delete(buf);
        chPoolFree(&msg_pool, msg);
    }
}

void udp_topic_broadcast_start(void)
{
    chPoolLoadArray(&msg_pool, msg_buffer, MSG_BUF_SIZE);

    static messagebus_new_topic_cb_t cb;
    messagebus_watchgroup_init(&watchgroup, &watchgroup_lock, &watchgroup_condvar);
    messagebus_new_topic_callback_register(&bus, &cb, new_topic_cb, NULL);

    static THD_WORKING_AREA(encode_wa, 2048);
    chThdCreateStatic(encode_wa, sizeof(encode_wa), HIGHPRIO, udp_topic_encode_thd, NULL);

    static THD_WORKING_AREA(send_wa, 2048);
    chThdCreateStatic(send_wa, sizeof(send_wa), NORMALPRIO, udp_topic_send_thd, NULL);
}
