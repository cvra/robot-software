#include <lwip/api.h>
#include <simplerpc/service_call.h>
#include <lwip/netif.h>
#include <hal.h>
#include <chprintf.h>
#include <string.h>
#include "rpc_server.h"
#include "config.h"
#include "interface_panel.h"
#include "commands.h"
#include "panic_log.h"
#include "unix_timestamp.h"
#include "timestamp/timestamp.h"
#include "bus_enumerator.h"
#include "uavcan_node.h"
#include "node_tracker.h"
#include "msgbus/messagebus.h"
#include "main.h"
#include "base/encoder.h"
#include "base/polar.h"
#include "base/odometry.h"
#include "base/position_manager.h"



static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
    size_t n, size;

    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: mem\r\n");
        return;
    }
    n = chHeapStatus(NULL, &size);
    chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
    chprintf(chp, "heap fragments   : %u\r\n", n);
    chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
    static const char *states[] = {CH_STATE_NAMES};
    thread_t *tp;

    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: threads\r\n");
        return;
    }
    chprintf(chp, "    addr    stack prio refs     state       time\r\n");
    tp = chRegFirstThread();
    do {
        chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %10lu %s\r\n",
                (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
                (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
                states[tp->p_state], (uint32_t)tp->p_time, tp->p_name);
        tp = chRegNextThread(tp);
    } while (tp != NULL);
}

static void cmd_ip(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    (void) argc;

    struct netif *n; /* used for iteration. */

    for (n = netif_list; n != NULL; n = n->next) {
        /* Converts the IP adress to a human readable format. */
        char ip[17], gw[17], nm[17];
        ipaddr_ntoa_r(&n->ip_addr, ip, 17);
        ipaddr_ntoa_r(&n->netmask, nm, 17);
        ipaddr_ntoa_r(&n->gw, gw, 17);

        chprintf(chp, "%s%d: %s, nm: %s, gw:%s\r\n", n->name, n->num, ip, nm, gw);
    }
}

static void cmd_crashme(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    (void) argc;
    (void) chp;

    chSysHalt(__FUNCTION__);
}

static void cmd_reboot(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    (void) argc;
    (void) chp;
    NVIC_SystemReset();
}

static void cmd_time(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argv;
    (void) argc;

    unix_timestamp_t ts;
    int h, m;

    /* Get current time */
    int now = timestamp_get();
    ts = timestamp_local_us_to_unix(now);
    chprintf(chp, "Current scheduler tick:      %12ld\r\n", now);
    chprintf(chp, "Current UNIX timestamp:      %12ld\r\n", ts.s);
    chprintf(chp, "current ChibiOS time (ms):   %12ld\r\n", ST2MS(chVTGetSystemTime()));
    chprintf(chp, "current timestamp time (us): %12ld\r\n", timestamp_get());

    /* Get time since start of day */
    ts.s = ts.s % (24 * 60 * 60);

    h = ts.s / 3600;
    ts.s = ts.s % 3600;

    m = ts.s / 60;
    ts.s = ts.s % 60;

    chprintf(chp, "Current time: %02d:%02d:%02d\r\n", h, m, ts.s);
}

static void cmd_rpc_client_test(BaseSequentialStream *chp, int argc, char **argv)
{
    uint8_t request[30];
    uint8_t output[30];

    cmp_ctx_t ctx;
    cmp_mem_access_t mem;
    ip_addr_t server;

    (void) chp;
    (void) argc;
    (void) argv;

    const int port = 20001;

    IP4_ADDR(&server, 192, 168, 2, 1);

    service_call_write_header(&ctx, &mem, request, sizeof request, "demo");
    cmp_write_nil(&ctx);

    rpc_transmit(request, cmp_mem_access_get_pos(&mem), output, sizeof output,
                 &server, port);
}

static void tree_indent(BaseSequentialStream *out, int indent)
{
    int i;
    for (i = 0; i < indent; ++i) {
        chprintf(out, "  ");
    }
}

static void show_config_tree(BaseSequentialStream *out, parameter_namespace_t *ns, int indent)
{
    parameter_t *p;

    tree_indent(out, indent);
    chprintf(out, "%s\r\n", ns->id);

    for (p=ns->parameter_list; p!=NULL; p=p->next) {
        tree_indent(out, indent + 1);
        if (parameter_defined(p)) {
            switch (p->type) {
                case _PARAM_TYPE_SCALAR:
                    chprintf(out, "%s: %f\r\n", p->id, parameter_scalar_get(p));
                    break;

                case _PARAM_TYPE_INTEGER:
                    chprintf(out, "%s: %d\r\n", p->id, parameter_integer_get(p));
                    break;

                default:
                    chprintf(out, "%s: unknown type %d\r\n", p->id, p->type);
                    break;
            }
        } else {
            chprintf(out, "%s: [not set]\r\n", p->id);
        }
    }

    if (ns->subspaces) {
        show_config_tree(out, ns->subspaces, indent + 1);
    }

    if (ns->next) {
        show_config_tree(out, ns->next, indent);
    }
}

static void cmd_config_tree(BaseSequentialStream *chp, int argc, char **argv)
{
    parameter_namespace_t *ns;
    if (argc != 1) {
        ns = &global_config;
    } else {
        ns = parameter_namespace_find(&global_config, argv[0]);
        if (ns == NULL) {
            chprintf(chp, "Cannot find subtree.\r\n");
            return;
        }

        ns = ns->subspaces;

        if (ns == NULL) {
            chprintf(chp, "This tree is empty.\r\n");
            return;
        }
    }

    show_config_tree(chp, ns, 0);
}

static void cmd_node(BaseSequentialStream *chp, int argc, char **argv)
{
    if (argc != 1) {
            chprintf(chp, "usage: node node_name.\r\n");
            return;
    }
    uint8_t id =  bus_enumerator_get_can_id(&bus_enumerator, argv[0]);
    chprintf(chp, "Node ID: %s = %d.\r\n", argv[0],id);
}

static void cmd_uavcan_node_reboot(BaseSequentialStream *chp, int argc, char **argv)
{
    int id = 0xff;
    if (argc == 1) {
        id = atoi(argv[0]);
    }
    if (argc > 1 || id < 1 || id > 0xff) {
        chprintf(chp, "usage: node_reboot [1-127|255]\r\n");
        return;
    }
    uavcan_node_send_reboot(id);
}

static void cmd_node_tracker(BaseSequentialStream *chp, int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uint64_t d, hi;
    node_tracker_get_and_clear(&d, &hi);
    chprintf(chp, "UAVCAN: tracking active nodes...\n");
    chThdSleepMilliseconds(2000);
    node_tracker_get_and_clear(&d, &hi);
    chprintf(chp, "present nodes: ");
    int id;
    for (id = 0; id < 128; id++) {
        if (d & 1) {
            chprintf(chp, "%u ", id);
        }
        d >>= 1;
        if (d == 0) {
            if (id < 64) {
                id = 63;
                d = hi;
            } else {
                break;
            }
        }
    }
    chprintf(chp, "\r\n");
}

static void cmd_topics(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    chprintf(chp, "available topics:\r\n");

    MESSAGEBUS_TOPIC_FOREACH(&bus, topic) {
        chprintf(chp, "%s\r\n", topic->name);
    }
}

static void cmd_encoders(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    messagebus_topic_t *encoders_topic;
    encoders_msg_t values;

    encoders_topic = messagebus_find_topic_blocking(&bus, "/encoders");
    messagebus_topic_wait(encoders_topic, &values, sizeof(values));

    chprintf(chp, "left: %ld\r\nright: %ld\r\n", values.left, values.right);
}

static void cmd_position(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    messagebus_topic_t *position_topic;
    pose2d_t pos;

    position_topic = messagebus_find_topic_blocking(&bus, "/position");
    messagebus_topic_wait(position_topic, &pos, sizeof(pos));

    chprintf(chp, "x: %f [m]\r\ny: %f [m]\r\na: %f [deg]\r\n", pos.x, pos.y, DEGREES(pos.heading));
}

static void cmd_position_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 3) {
        float x = atof(argv[0]);
        float y = atof(argv[0]);
        float heading = atof(argv[0]);

        position_manager_reset(x, y, heading);
        chprintf(chp, "New pos x: %f [m]\r\ny: %f [m]\r\na: %f [deg]\r\n", x, y, heading);
    } else {
        chprintf(chp, "Usage: pos_reset x[m] y[m] heading[deg]\r\n");
    }
}

static void cmd_wheel_correction(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 2) {
        float left, right;
        position_manager_get_wheel_correction(&left, &right);

        if (!strcmp("left", argv[0])) {
            left = atof(argv[1]);
        } else if (!strcmp("right", argv[0])) {
            right = atof(argv[1]);
        }

        position_manager_set_wheel_correction(left, right);
    } else {
        chprintf(chp, "Usage: wheel_corr {left|right} factor\r\n");
    }
}

const ShellCommand commands[] = {
    {"crashme", cmd_crashme},
    {"config_tree", cmd_config_tree},
    {"encoders", cmd_encoders},
    {"ip", cmd_ip},
    {"mem", cmd_mem},
    {"node", cmd_node},
    {"node_reboot", cmd_uavcan_node_reboot},
    {"node_tracker", cmd_node_tracker},
    {"pos", cmd_position},
    {"pos_reset", cmd_position_reset},
    {"reboot", cmd_reboot},
    {"rpc_client_demo", cmd_rpc_client_test},
    {"threads", cmd_threads},
    {"time", cmd_time},
    {"topics", cmd_topics},
    {"wheel_corr", cmd_wheel_correction},
    {NULL, NULL}
};
