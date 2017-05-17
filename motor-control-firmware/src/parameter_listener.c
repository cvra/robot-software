#include <ch.h>
#include <chprintf.h>
#include <parameter/parameter.h>
#include <parameter/parameter_msgpack.h>
#include <serial-datagram/serial_datagram.h>
#include "main.h"

static void error_cb(void *arg, const char *id, const char *err)
{
    (void)arg;
    (void)id;
    (void)err;
}

static void parameter_decode_cb(const void *dtgrm, size_t len, void *arg)
{
    int ret = parameter_msgpack_read(&parameter_root_ns, (char*)dtgrm, len, error_cb, arg);
    chprintf(ch_stdout, "ok %d\n", ret);
    // parameter_print(&parameter_root_ns);
}

static THD_WORKING_AREA(parameter_listener_wa, 512);
static THD_FUNCTION(parameter_listener, arg)
{
    static char rcv_buf[200];
    static serial_datagram_rcv_handler_t rcv_handler;
    serial_datagram_rcv_handler_init(&rcv_handler, &rcv_buf, sizeof(rcv_buf), parameter_decode_cb, NULL);
    while (1) {
        char c = chSequentialStreamGet((BaseSequentialStream*)arg);
        int ret = serial_datagram_receive(&rcv_handler, &c, 1);
        if (ret != SERIAL_DATAGRAM_RCV_NO_ERROR) {
            chprintf(ch_stdout, "serial datagram error %d\n", ret);
        }
        (void)ret; // ingore errors
    }
}

void parameter_listener_start(BaseSequentialStream *dev)
{
    chThdCreateStatic(parameter_listener_wa, sizeof(parameter_listener_wa), LOWPRIO, parameter_listener, dev);
}
