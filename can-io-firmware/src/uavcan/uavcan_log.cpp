#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/protocol/debug/LogLevel.hpp>
#include "uavcan_log.h"
#include "error/error.h"

/* XXX We could increase this a bit. */
#define MSG_BUFFER_SIZE 1

static struct {
    char msg[91];
    char source[32];
    int level;
} msg_buffer[MSG_BUFFER_SIZE];

static msg_t free_msg_queue_buf[MSG_BUFFER_SIZE];
static msg_t msg_queue_buf[MSG_BUFFER_SIZE];

static MAILBOX_DECL(msg_queue, msg_queue_buf, sizeof(msg_queue_buf));
static MAILBOX_DECL(free_msg_queue, free_msg_queue_buf, sizeof(free_msg_queue_buf));

int uavcan_log_start(void)
{
    int i;
    for (i = 0; i < MSG_BUFFER_SIZE; i++) {
        chMBPost(&free_msg_queue, i, TIME_IMMEDIATE);
    }

    return 0;
}

int uavcan_log_spin(Node &node)
{
    msg_t msg_index;
    static uavcan::Publisher<uavcan::protocol::debug::LogMessage> pub(node);

    while (chMBFetch(&msg_queue, &msg_index, TIME_IMMEDIATE) == MSG_OK) {
        /* Construct the UAVCAN data type. */
        uavcan::protocol::debug::LogMessage msg;
        msg.text = msg_buffer[msg_index].msg;
        msg.source = msg_buffer[msg_index].source;

        switch (msg_buffer[msg_index].level) {
            case ERROR_SEVERITY_ERROR:
                msg.level.value = uavcan::protocol::debug::LogLevel::ERROR;
                break;

            case ERROR_SEVERITY_WARNING:
                msg.level.value = uavcan::protocol::debug::LogLevel::WARNING;
                break;
                break;

            case ERROR_SEVERITY_NOTICE:
                break;

            case ERROR_SEVERITY_DEBUG:
                break;
        }

        /* Send the message */
        pub.broadcast(msg);

        /* Put the message handler back in the queue. */
        chMBPost(&free_msg_queue, msg_index, TIME_IMMEDIATE);
    }

    return 0;
}

void uavcan_log_write(const struct error *e, va_list args)
{
    msg_t msg_index;

    /* Try to get a message slot, otherwise drop the message. */
    if (chMBFetch(&free_msg_queue, &msg_index, TIME_IMMEDIATE) != MSG_OK) {
        return;
    }

    snprintf(msg_buffer[msg_index].msg, sizeof(msg_buffer[msg_index].msg) - 1,
             "%s:%d", strrchr(e->file, '/'), e->line);

    vsnprintf(msg_buffer[msg_index].msg, sizeof(msg_buffer[msg_index].msg) - 1,
              e->text, args);

    chMBPost(&msg_queue, msg_index, TIME_IMMEDIATE);
}
