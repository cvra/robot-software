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
#include "msgbus/messagebus.h"
#include "main.h"
#include "cvra/cvra_motors.h"
#include "base/encoder.h"
#include "base/polar.h"
#include "base/odometry.h"
#include "base/base_controller.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include <trace/trace.h>


#define DEGREES(x) (x * 180.f / M_PI)
#define RADIANS(x) (x * M_PI / 180.f)

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

    float x, y, a;

    x = position_get_x_float(&robot.pos);
    y = position_get_y_float(&robot.pos);
    a = position_get_a_rad_float(&robot.pos);

    chprintf(chp, "x: %f [mm]\r\ny: %f [mm]\r\na: %f [rad]\r\n", x, y, a);
}

static void cmd_position_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 3) {
        float x = atof(argv[0]);
        float y = atof(argv[1]);
        float a = atof(argv[2]);

        position_set(&robot.pos, x, y, a);

        chprintf(chp, "New pos x: %f [mm]\r\ny: %f [mm]\r\na: %f [rad]\r\n", x, y, a);
    } else {
        chprintf(chp, "Usage: pos_reset x[mm] y[mm] a[rad]\r\n");
    }
}

static void cmd_traj_forward(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 1) {
        robot.mode = BOARD_MODE_DISTANCE_ONLY;

        int32_t distance;
        distance = atoi(argv[0]);
        trajectory_d_rel(&robot.traj, distance);

        robot.mode = BOARD_MODE_ANGLE_DISTANCE;
    } else {
        chprintf(chp, "Usage: forward distance\r\n");
    }
}

static void cmd_traj_rotate(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 1) {
        robot.mode = BOARD_MODE_ANGLE_ONLY;

        int32_t angle;
        angle = atoi(argv[0]);
        trajectory_a_rel(&robot.traj, angle);

        robot.mode = BOARD_MODE_ANGLE_DISTANCE;
    } else {
        chprintf(chp, "Usage: rotate angle\r\n");
    }
}

static void cmd_traj_goto(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 3) {
        float x, y, a;
        x = atof(argv[0]);
        y = atof(argv[1]);
        a = atof(argv[2]);
        chprintf(chp, "Going to x: %.1fmm y: %.1fmm a: %.1fdeg\r\n", x, y, a);

        trajectory_goto_xy_abs(&robot.traj, x, y);
        chThdSleepMilliseconds(100);

        while (trajectory_finished(&robot.traj) == 0) {
            chThdSleepMilliseconds(10);
        }

        trajectory_a_abs(&robot.traj, a);
    } else {
        chprintf(chp, "Usage: goto x y a\r\n");
    }
}


void add_rectangular_obstacle(int x, int y, int half_dx, int half_dy)
{
    poly_t *obstacle = oa_new_poly(4);

    oa_poly_set_point(obstacle, x + half_dx, y + half_dy, 0);
    oa_poly_set_point(obstacle, x + half_dx, y - half_dy, 1);
    oa_poly_set_point(obstacle, x - half_dx, y - half_dy, 2);
    oa_poly_set_point(obstacle, x - half_dx, y + half_dy, 3);
}


static void cmd_pathplanner(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 3) {
        /* Get goal pos */
        int x, y;
        float a;
        x = atoi(argv[0]);
        y = atoi(argv[1]);
        a = atof(argv[2]);
        chprintf(chp, "Going to x: %dmm y: %dmm a: %.1fdeg\r\n", x, y, a);

        add_rectangular_obstacle(750, 1150, 350, 200);

        /* Request a path to the planner */
        point_t *p;

        /* Sets starting and ending point of the path. */
        oa_start_end_points(
            position_get_x_s16(&robot.pos), position_get_x_s16(&robot.pos),
            x, y);

        /* Computes the path */
        oa_process();
        int len = oa_get_path(&p);
        chprintf(chp, "Found path of length %d\r\n", len);

        /* Checks if a path was found. */
        if(len < 0) {
            chprintf(chp, "Cannot find a suitable path.\r\n");
            return;
        } else {
            chprintf(chp, "Path found contains %d points\r\n", len);
        }

        /* For all the points in the path. */
        for (int i = 0; i < len; i++) {
            /* Goes to the point. */
            trajectory_goto_forward_xy_abs(&robot.traj, p->x, p->y);
            chprintf(chp, "Going to x: %.1fmm y: %.1fmm\r\n", p->x, p->y);

            /* Waits for the completion of the trajectory. */
            chThdSleepMilliseconds(100);
            while (trajectory_finished(&robot.traj) == 0) {
                chThdSleepMilliseconds(10);
            }

            /* Increments pointer to load next point. */
            p++;
        }

        trajectory_a_abs(&robot.traj, a);
    } else {
        chprintf(chp, "Usage: path x y a\r\n");
    }
}

static void cmd_create_static_obstacle(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 3) {
        /* Get obstacle properties */
        float x, y, half_size;
        x = atof(argv[0]);
        y = atof(argv[1]);
        half_size = atof(argv[2]);

        /* Create obstacle */
        add_rectangular_obstacle(x, y, half_size, half_size);

        chprintf(chp, "Created square obstacle at x: %.1fmm y: %.1fmm of half size: %.1fmm\r\n", x, y, half_size);
        oa_dump();
    } else {
        chprintf(chp, "Usage: obs x y size\r\n");
    }
}

static void cmd_pid(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 2) {
        float kp, ki, kd, ilim;
        pid_get_gains(&robot.angle_pid.pid, &kp, &ki, &kd);
        ilim = pid_get_integral_limit(&robot.angle_pid.pid);

        if (strcmp("p", argv[0]) == 0) {
            kp = atof(argv[1]);
        } else if (strcmp("i", argv[0]) == 0) {
            ki = atof(argv[1]);
        } else if (strcmp("d", argv[0]) == 0) {
            kd = atof(argv[1]);
        } else if (strcmp("l", argv[0]) == 0) {
            ilim = atof(argv[1]);
        } else {
            chprintf(chp, "Usage: pid {p,i,d,l} value\r\n");
            return;
        }

        pid_set_gains(&robot.angle_pid.pid, kp, ki, kd);
        pid_set_integral_limit(&robot.angle_pid.pid, ilim);

        chprintf(chp, "New PID config: p %.2f i %.2f d %.2f ilim %.2f\r\n", kp, ki, kd, ilim);
    } else {
        chprintf(chp, "Usage: pid {p,i,d,l} value\r\n");
    }
}

static void cmd_blocking_detection_config(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 3) {
        uint32_t err_th = atoi(argv[1]);
        uint16_t cpt_th = atoi(argv[2]);
        if (strcmp("angle", argv[0]) == 0) {
            bd_set_thresholds(&robot.angle_bd, err_th, cpt_th);
        } else if (strcmp("distance", argv[0]) == 0) {
            bd_set_thresholds(&robot.distance_bd, err_th, cpt_th);
        }
    } else {
        chprintf(chp, "Usage: bdconf \"angle\"/\"distance\" err_th cpt_th\r\n");
    }
}

static void cmd_wheel_calibration(BaseSequentialStream *chp, int argc, char *argv[])
{
    int32_t start_angle, start_distance;
    int32_t delta_angle, delta_distance;
    float factor, left_gain, right_gain;

    int count;
    if(argc < 1) {
        count = 1;
    } else {
        count = atoi(argv[0]);
    }

    /* Configure robot to be slower and more sensitive to collisions */
    bd_set_thresholds(&robot.distance_bd, 20000, 2);
    trajectory_set_acc(&robot.traj,
            acc_mm2imp(&robot.traj, 150.),
            acc_rd2imp(&robot.traj, 1.57));
    trajectory_set_speed(&robot.traj,
            speed_mm2imp(&robot.traj, 100.),
            speed_rd2imp(&robot.traj, 0.75));

    /* Go backwards until we hit the wall */
    robot.mode = BOARD_MODE_DISTANCE_ONLY;
    trajectory_d_rel(&robot.traj, -2000.);
    while(!bd_get(&robot.distance_bd)) {
        chThdSleepMilliseconds(1);
    }
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);
    chprintf(chp, "I just hit the wall\n");

    /* Take reference at the wall */
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    start_angle = rs_get_angle(&robot.rs);
    start_distance = rs_get_distance(&robot.rs);

    /* Start calibration sequence and do it N times */
    while(count--) {
        chprintf(chp, "%d left !\n", count);
        trajectory_d_rel(&robot.traj, 1200.);
        while(!trajectory_finished(&robot.traj)) {
            chThdSleepMilliseconds(1);
        }
        trajectory_a_rel(&robot.traj, 180.);
        while(!trajectory_finished(&robot.traj)) {
            chThdSleepMilliseconds(1);
        }
        trajectory_d_rel(&robot.traj, 1100.);
        while(!trajectory_finished(&robot.traj)) {
            chThdSleepMilliseconds(1);
        }
        trajectory_a_rel(&robot.traj, -180.);
        while(!trajectory_finished(&robot.traj)) {
            chThdSleepMilliseconds(1);
        }
    }

    trajectory_d_rel(&robot.traj, -75.);
    while(!trajectory_finished(&robot.traj)) {
        chThdSleepMilliseconds(1);
    }

    /* Go backwards until we reach a wall */
    robot.mode = BOARD_MODE_DISTANCE_ONLY;
    trajectory_d_rel(&robot.traj, -2000.);
    while(!bd_get(&robot.distance_bd)){
        chThdSleepMilliseconds(1);
    }
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);

    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    /* Compute correction factors */
    delta_angle = start_angle - rs_get_angle(&robot.rs);
    delta_distance = start_distance - rs_get_distance(&robot.rs);

    factor = (float)(delta_angle) / (float)(delta_distance);
    left_gain = (1. + factor) * robot.rs.left_ext_gain;
    right_gain = (1. - factor) * robot.rs.right_ext_gain;

    chprintf(chp, "Angle difference : %f\n", DEGREES(pos_imp2rd(&robot.traj, delta_angle)));
    chprintf(chp, "Suggested factors :\n");
    chprintf(chp, "Left : %.8f (old gain was %f)\n", left_gain, robot.rs.left_ext_gain);
    chprintf(chp, "Right : %.8f (old gain was %f)\n", right_gain, robot.rs.right_ext_gain);
}

static void cmd_wheel_correction(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 2) {
        if (!strcmp("left", argv[0])) {
            float left = atof(argv[1]);
            rs_set_left_ext_encoder(&robot.rs, cvra_encoder_get_left_ext, NULL, left);
        } else if (!strcmp("right", argv[0])) {
            float right = atof(argv[1]);
            rs_set_right_ext_encoder(&robot.rs, cvra_encoder_get_right_ext, NULL, right);
        }
    } else {
        chprintf(chp, "Usage: wheel_corr {left|right} factor\r\n");
    }
}

static void print_fn(void *arg, const char *fmt, ...)
{
    BaseSequentialStream *chp = (BaseSequentialStream *)arg;
    va_list ap;
    va_start(ap, fmt);
    chvprintf(chp, fmt, ap);
    va_end(ap);
}

static void cmd_trace(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "Usage: trace dump|clear\r\n");
        return;
    }
    if (strcmp("dump", argv[0]) == 0) {
        trace_print(print_fn, chp);
    } else if (strcmp("clear", argv[0]) == 0) {
        trace_clear();
    }
}

const ShellCommand commands[] = {
    {"crashme", cmd_crashme},
    {"config_tree", cmd_config_tree},
    {"encoders", cmd_encoders},
    {"forward", cmd_traj_forward},
    {"ip", cmd_ip},
    {"mem", cmd_mem},
    {"node", cmd_node},
    {"node_reboot", cmd_uavcan_node_reboot},
    {"pos", cmd_position},
    {"pos_reset", cmd_position_reset},
    {"reboot", cmd_reboot},
    {"rotate", cmd_traj_rotate},
    {"rpc_client_demo", cmd_rpc_client_test},
    {"threads", cmd_threads},
    {"time", cmd_time},
    {"topics", cmd_topics},
    {"pid", cmd_pid},
    {"goto", cmd_traj_goto},
    {"path", cmd_pathplanner},
    {"obs", cmd_create_static_obstacle},
    {"bdconf", cmd_blocking_detection_config},
    {"wheel_calib", cmd_wheel_calibration},
    {"wheel_corr", cmd_wheel_correction},
    {"trace", cmd_trace},
    {NULL, NULL}
};
