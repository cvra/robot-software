#include <lwip/api.h>
#include <simplerpc/service_call.h>
#include <lwip/netif.h>
#include <hal.h>
#include <test.h>
#include <chprintf.h>
#include "rpc_server.h"
#include "motor_control.h"
#include "config.h"
#include "interface_panel.h"
#include "commands.h"
#include "panic_log.h"
#include "unix_timestamp.h"

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
    chprintf(chp, "Current scheduler tick: %12ld\r\n", now);
    chprintf(chp, "Current UNIX timestamp: %12ld\r\n", ts.s);
    chprintf(chp, "current time (ms):      %12ld\r\n", ST2MS(chVTGetSystemTime()));

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

static void cmd_fwd(BaseSequentialStream *chp, int argc, char **argv)
{
    if (argc != 1) {
        chprintf(chp, "usage: fwd <speed>\n");
        return;
    }
    float cmd = strtof(argv[0], NULL);
    m1_vel_setpt = cmd;
    m2_vel_setpt = -cmd;
}

static void cmd_vel_setpt(BaseSequentialStream *chp, int argc, char **argv)
{
    (void)argv;
    if (argc > 0) {
        chprintf(chp, "usage: vel_setpt\n");
        return;
    }
    int m1_vel_setpt_int = (int) 1000 * m1_vel_setpt;
    int m2_vel_setpt_int = (int) 1000 * m2_vel_setpt;
    chprintf(chp, "m1_vel_setpt: %d\n", m1_vel_setpt_int);
    chprintf(chp, "m2_vel_setpt: %d\n", m2_vel_setpt_int);
}

static void cmd_config_get(BaseSequentialStream *chp, int argc, char **argv)
{
    parameter_t *param;

    if (argc != 1) {
        chprintf(chp, "Usage: config_get key\r\n");
        return;
    }

    param = parameter_find(&global_config, argv[0]);

    if (param == NULL) {
        chprintf(chp, "Cannot find key: \"%s\"\r\n", argv[0]);
        return;
    }

    chprintf(chp, "Value: %d\r\n", (int)parameter_scalar_get(param));
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
        chprintf(out, "%s: %f\r\n", p->id, parameter_scalar_get(p));
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

static void cmd_interface_panel_test(BaseSequentialStream *chp, int argc, char **argv)
{
    (void)argc;
    (void)argv;
    chprintf(chp, "testing interface panel...\n");
    chprintf(chp, "press both buttons to exit\n");
    interface_panel_test();
}

const ShellCommand commands[] = {
    {"mem", cmd_mem},
    {"ip", cmd_ip},
    {"config_get", cmd_config_get},
    {"config_tree", cmd_config_tree},
    {"threads", cmd_threads},
    {"test", cmd_test},
    {"panic_log", cmd_panic_log},
    {"crashme", cmd_crashme},
    {"time", cmd_time},
    {"rpc_client_demo", cmd_rpc_client_test},
    {"fwd", cmd_fwd},
    {"vel_setpt", cmd_vel_setpt},
    {"interface_panel_test", cmd_interface_panel_test},
    {NULL, NULL}
};
