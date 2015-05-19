#include <lwip/api.h>
#include <simplerpc/service_call.h>
#include <lwip/netif.h>
#include <hal.h>
#include <test.h>
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
#include "robot_pose.h"

/** Stack size for the unit test thread. */
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)


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

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
    thread_t *tp;

    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: test\r\n");
        return;
    }
    tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriorityX(),
            TestThread, chp);
    if (tp == NULL) {
        chprintf(chp, "out of memory\r\n");
        return;
    }
    chThdWait(tp);
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

static void cmd_panic_log(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    (void) argc;
    const char *message;

    message = panic_log_read();

    if (message == NULL) {
        chprintf(chp, "Did not reboot after a panic.");
    } else {
        chprintf(chp, "%s", message);
    }
    chprintf(chp, "\r\n");
}

static void cmd_time(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argv;
    (void) argc;

    unix_timestamp_t ts;
    int h, m;

    /* Get current time */
    int now = ST2US(chVTGetSystemTime());
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

    service_call_encode(&ctx, &mem, request, sizeof request, "demo", 0);

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
    chprintf(chp, "%s %d\r\n", bus_enumerator.str_to_can->str_id, bus_enumerator.str_to_can->can_id);
    chprintf(chp, "%s %d\r\n", bus_enumerator.can_to_str->str_id, bus_enumerator.can_to_str->can_id);

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

static void cmd_pos(BaseSequentialStream *chp, int argc, char **argv)
{
    float x, y, theta;

    chMtxLock(&robot_pose_lock);
    x = robot_pose.x;
    y = robot_pose.y;
    theta = robot_pose.theta;
    chMtxUnlock(&robot_pose_lock);

    chprintf(chp, "%.3f;%.3f%.3f\r\n", x, y, theta);


}

const ShellCommand commands[] = {
    {"mem", cmd_mem},
    {"ip", cmd_ip},
    {"config_tree", cmd_config_tree},
    {"threads", cmd_threads},
    {"test", cmd_test},
    {"panic_log", cmd_panic_log},
    {"crashme", cmd_crashme},
    {"time", cmd_time},
    {"rpc_client_demo", cmd_rpc_client_test},
    {"node", cmd_node},
    {"pos", cmd_pos},
    {"node_reboot", cmd_uavcan_node_reboot},
    {NULL, NULL}
};
