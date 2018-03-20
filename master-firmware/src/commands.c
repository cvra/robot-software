#include <lwip/api.h>
#include <simplerpc/service_call.h>
#include <ff.h>
#include <lwip/netif.h>
#include <hal.h>
#include <chprintf.h>
#include <string.h>
#include <shell.h>
#include "rpc_server.h"
#include "config.h"
#include "commands.h"
#include "panic_log.h"
#include "unix_timestamp.h"
#include "timestamp/timestamp.h"
#include "bus_enumerator.h"
#include "uavcan_node.h"
#include <stdio.h>
#include "msgbus/messagebus.h"
#include "main.h"
#include "aversive_port/cvra_motors.h"
#include "base/encoder.h"
#include "base/base_controller.h"
#include "base/base_helpers.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "robot_helpers/motor_helpers.h"
#include "robot_helpers/math_helpers.h"
#include "scara/scara.h"
#include "scara/scara_utils.h"
#include "scara/scara_trajectories.h"
#include "arms/arms_controller.h"
#include "hand/hand.h"
#include "strategy.h"
#include <trace/trace.h>
#include "pca9685_pwm.h"
#include "lever/lever_module.h"

const ShellCommand commands[];

#if SHELL_USE_HISTORY == TRUE
static char sc_histbuf[SHELL_MAX_HIST_BUFF];
#endif

static ShellConfig shell_cfg = {
    NULL,
    commands,
#if SHELL_USE_HISTORY == TRUE
    sc_histbuf,
    sizeof(sc_histbuf),
#endif
};

void shell_spawn(BaseSequentialStream *stream)
{
    static THD_WORKING_AREA(shell_wa, 2048);
    static thread_t *shelltp = NULL;

    if (!shelltp) {
        shell_cfg.sc_channel = stream;
        shelltp = chThdCreateStatic(&shell_wa, sizeof(shell_wa), USB_SHELL_PRIO,
                                    shellThread, (void *)&shell_cfg);
        chRegSetThreadNameX(shelltp, "shell");
    } else if (chThdTerminatedX(shelltp)) {
        chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
        shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[])
{
    static const char *states[] = {CH_STATE_NAMES};
    thread_t *tp;

    (void)argv;
    if (argc > 0) {
        shellUsage(chp, "threads");
        return;
    }
    chprintf(chp,
             "stklimit    stack     addr refs prio     state       time         name\r\n"SHELL_NEWLINE_STR);
    tp = chRegFirstThread();
    do {
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
        uint32_t stklimit = (uint32_t)tp->wabase;
#else
        uint32_t stklimit = 0U;
#endif
        chprintf(chp, "%08lx %08lx %08lx %4lu %4lu %9s %10lu %12s"SHELL_NEWLINE_STR,
                 stklimit, (uint32_t)tp->ctx.sp, (uint32_t)tp,
                 (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
                 (uint32_t)tp->time, tp->name == NULL ? "" : tp->name);
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

    ERROR("You asked for it!, uptime=%d ms", ST2MS(chVTGetSystemTime()));
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

                case _PARAM_TYPE_STRING: {
                    static char buf[50];
                    parameter_string_get(p, buf, sizeof(buf));
                    chprintf(out, "%s: \"%s\"\r\n", p->id, buf);
                    break;
                }

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

static void cmd_config_set(BaseSequentialStream *chp, int argc, char **argv)
{
    parameter_t *param;
    int value_i;
    float value_f;

    if (argc != 2) {
        chprintf(chp, "Usage: config_set /parameter/url value.\r\n");
        return;
    }

    param = parameter_find(&global_config, argv[0]);

    if (param == NULL) {
        chprintf(chp, "Could not find parameter \"%s\"\r\n", argv[0]);
        return;
    }

    switch (param->type) {
        case _PARAM_TYPE_INTEGER:
            if (sscanf(argv[1], "%d", &value_i) == 1) {
                parameter_integer_set(param, value_i);
            } else {
                chprintf(chp, "Invalid value for integer parameter.\r\n");
            }
            break;

        case _PARAM_TYPE_SCALAR:
            if (sscanf(argv[1], "%f", &value_f) == 1) {
                parameter_scalar_set(param, value_f);
            } else {
                chprintf(chp, "Invalid value for scalar parameter.\r\n");
            }
            break;


        case _PARAM_TYPE_BOOLEAN:
            if (!strcmp(argv[1], "true")) {
                parameter_boolean_set(param, true);
            } else if (!strcmp(argv[1], "false")) {
                parameter_boolean_set(param, false);
            } else {
                chprintf(chp, "Invalid value for boolean parameter, must be true or false.\r\n");
            }
            break;

        case _PARAM_TYPE_STRING:
            if (argc == 2) {
                parameter_string_set(param, argv[1]);
            } else {
                chprintf(chp, "Invalid value for string parameter, must not use spaces.\r\n");
            }
            break;

        default:
            chprintf(chp, "%s: unknown type %d\r\n", param->id, param->type);
            break;
    }
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

        chprintf(chp, "New pos x: %f [mm]\r\ny: %f [mm]\r\na: %f [deg]\r\n", x, y, a);
    } else {
        chprintf(chp, "Usage: pos_reset x[mm] y[mm] a[deg]\r\n");
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
        int32_t x, y, a;
        x = atoi(argv[0]);
        y = atoi(argv[1]);
        a = atoi(argv[2]);
        chprintf(chp, "Going to x: %d [mm], y: %d [mm], a: %d [deg]\r\n", x, y, a);

        trajectory_move_to(x, y, a);
    } else {
        chprintf(chp, "Usage: goto x y a\r\n");
    }
}

static void cmd_goto_avoid(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 3) {
        int32_t x = atoi(argv[0]);
        int32_t y = atoi(argv[1]);
        int32_t a = atoi(argv[2]);

        strategy_goto_avoid(x, y, a, TRAJ_FLAGS_ALL);
    } else {
        chprintf(chp, "Usage: goto_avoid x y a\r\n");
    }
}


void add_rectangular_obstacle(int x, int y, int half_dx, int half_dy)
{
    poly_t *obstacle = oa_new_poly(4);

    oa_poly_set_point(obstacle, x - half_dx, y - half_dy, 3);
    oa_poly_set_point(obstacle, x - half_dx, y + half_dy, 2);
    oa_poly_set_point(obstacle, x + half_dx, y + half_dy, 1);
    oa_poly_set_point(obstacle, x + half_dx, y - half_dy, 0);
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

        /* Compute a path */
        oa_reset();
        oa_start_end_points(
            position_get_x_s16(&robot.pos), position_get_x_s16(&robot.pos),
            x, y);
        oa_process();

        /* Fetch the computed path */
        point_t *p;
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
            trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

            /* Increments pointer to load next point. */
            p++;
        }

        trajectory_a_abs(&robot.traj, a);
    } else {
        chprintf(chp, "Usage: path x y a\r\n");
    }
}

static void cmd_oa_dump(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)chp;
    (void)argc;
    (void)argv;

    oa_dump();
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

static void cmd_pid_tune(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    chprintf(chp, "pid tuner: press q or CTRL-D to quit\n");

    long index;
    static char line[20];
    parameter_namespace_t *ns;
    motor_driver_t *motors;
    uint16_t len, i;

    /* list all motors */
    motor_manager_get_list(&motor_manager, &motors, &len);
    chprintf(chp, "id: name\n");
    for (i = 0; i < len; i++) {
        chprintf(chp, "%2u: %s\n", i+1, motors[i].id);
    }

    const char *extra[] = {
        "master/aversive/control/angle",
        "master/aversive/control/distance",
        "master/main_arm/control/x",
        "master/main_arm/control/y",
        "master/main_arm/control/heading",
    };
    const size_t extra_len = sizeof(extra)/sizeof(char *);
    for (i = 0; i < extra_len; i++) {
        chprintf(chp, "%2u: %s\n", len+i+1, extra[i]);
    }

    while (1) {
        chprintf(chp, "choose [1-%u]: ", len + extra_len);
        if (shellGetLine(&shell_cfg, line, sizeof(line), NULL) || line[0] == 'q') {
            /* CTRL-D was pressed */
            return;
        }

        index = strtol(line, NULL, 10);
        if (index > 0 && (unsigned long) index <= len + extra_len) {
            break;
        }
        chprintf(chp, "invalid index\n");
    }

    /* index starting from 0 */
    index -= 1;

    if (index  < len) {
        chprintf(chp, "tune %s\n", motors[index].id);
        chprintf(chp, "1: current\n2: velocity\n3: position\n> ");
        if (shellGetLine(&shell_cfg, line, sizeof(line), NULL) || line[0] == 'q') {
            /* q or CTRL-D was pressed */
            return;
        }
        char *type = line;
        if (!strcmp(type, "1")) {
            type = "current";
        } else if (!strcmp(type, "2")) {
            type = "velocity";
        } else if (!strcmp(type, "3")) {
            type = "position";
        }
        ns = parameter_namespace_find(&motors[index].config.control, line);
    } else {
        chprintf(chp, "tune %s\n", extra[index-len]);
        ns = parameter_namespace_find(&global_config, extra[index-len]);
    }

    if (ns == NULL) {
        chprintf(chp, "not found\n");
        return;
    }

    /* interactive command line */
    chprintf(chp, "select:\n> kp|ki|kd|ilimit value\n");
    while (true) {
        chprintf(chp, "> ");
        if (shellGetLine(&shell_cfg, line, sizeof(line), NULL) || line[0] == 'q') {
            /* q or CTRL-D was pressed */
            return;
        }
        char *p = strchr(line, ' ');
        if (p == NULL) {
            chprintf(chp, "invalid value\n");
            continue;
        }
        *p = '\0';
        float val = strtof(p + 1, NULL);

        /* shortcut for kp ki kd */
        char *name = line;
        if (!strcmp(name, "i")) {
            name = "ki";
        } else if (!strcmp(name, "p")) {
            name = "kp";
        } else if (!strcmp(name, "d")) {
            name = "kd";
        }

        parameter_t *param;
        param = parameter_find(ns, name);
        if (param == NULL) {
            chprintf(chp, "parameter not found\n");
            continue;
        }
        chprintf(chp, "%s = %f\n", name, val);
        parameter_scalar_set(param, val);
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
    int count;
    if(argc < 1) {
        count = 1;
    } else {
        count = atoi(argv[0]);
    }

    /* Configure robot to be slower and less sensitive to collisions */
    trajectory_set_mode_aligning(&robot.mode, &robot.traj, &robot.distance_bd, &robot.angle_bd);

    /* Take reference at the wall */
    trajectory_align_with_wall();
    chprintf(chp, "I just hit the wall\n");

    int32_t start_angle = rs_get_angle(&robot.rs);
    int32_t start_distance = rs_get_distance(&robot.rs);

    /* Start calibration sequence and do it N times */
    while(count--) {
        chprintf(chp, "%d left !\n", count);
        trajectory_d_rel(&robot.traj, - robot.calibration_direction * 1200.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
        trajectory_a_rel(&robot.traj, 180.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
        trajectory_d_rel(&robot.traj, - robot.calibration_direction * 1100.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
        trajectory_a_rel(&robot.traj, -180.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
    }

    trajectory_d_rel(&robot.traj, robot.calibration_direction * 75.);
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Take reference again at the wall */
    trajectory_align_with_wall();

    /* Compute correction factors */
    int32_t delta_angle = start_angle - rs_get_angle(&robot.rs);
    int32_t delta_distance = start_distance - rs_get_distance(&robot.rs);

    float factor = (float)(delta_angle) / (float)(delta_distance);
    float left_gain = (1. + factor) * robot.rs.left_ext_gain;
    float right_gain = (1. - factor) * robot.rs.right_ext_gain;

    /* Stop polar control */
    trajectory_d_rel(&robot.traj, - robot.calibration_direction * 75.);

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

static void cmd_track_calibration(BaseSequentialStream *chp, int argc, char *argv[])
{
    int count;
    if(argc < 1) {
        count = 1;
    } else {
        count = atoi(argv[0]);
    }

    /* Configure robot to be slower and less sensitive to collisions */
    trajectory_set_mode_aligning(&robot.mode, &robot.traj, &robot.distance_bd, &robot.angle_bd);

    /* Take reference with wall */
    trajectory_align_with_wall();
    chprintf(chp, "I just hit the wall\n");
    float start_angle = pos_imp2rd(&robot.traj, rs_get_angle(&robot.rs));

    /* Start calibration sequence and do it N times */
    trajectory_d_rel(&robot.traj, - robot.calibration_direction * 200.);
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
    for (int i = 0; i < count; i++) {
        chprintf(chp, "%d left !\n", i);
        trajectory_a_rel(&robot.traj, 360.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
    }
    trajectory_d_rel(&robot.traj, robot.calibration_direction * 180.);
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Take reference at the wall */
    trajectory_align_with_wall();
    float end_angle = pos_imp2rd(&robot.traj, rs_get_angle(&robot.rs));

    /* Compute correction factors */
    float delta_angle = angle_delta(0., end_angle - start_angle);
    float track_calibrated = (float)robot.pos.phys.track_mm * \
                                (1 + (delta_angle / (2. * M_PI * (float)count)));

    chprintf(chp, "Start angle %f, End angle : %f\n", DEGREES(start_angle), DEGREES(end_angle));
    chprintf(chp, "Angle difference : %f\n", DEGREES(delta_angle));
    chprintf(chp, "Suggested track : %.8f mm\n", track_calibrated);
}

static void cmd_track_correction(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc < 1) {
        chprintf(chp, "Usage: track_corr factor\r\n");
        return;
    }
    float track = atof(argv[0]);
    position_set_physical_params(&robot.pos, track, robot.pos.phys.distance_imp_per_mm);
}


static void cmd_autopos(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc < 4) {
        chprintf(chp, "Usage: autopos {yellow|blue} x y a\r\n");
        return;
    }

    enum strat_color_t color = BLUE;
    if (strcmp("blue", argv[0]) == 0) {
        color = BLUE;
    } else if (strcmp("yellow", argv[0]) == 0) {
        color = YELLOW;
    } else {
        chprintf(chp, "Unknown color, please chose either yellow or blue\r\n");
        return;
    }

    int32_t x, y, a;
    x = atoi(argv[1]);
    y = atoi(argv[2]);
    a = atoi(argv[3]);
    chprintf(chp, "Positioning robot to x: %d[mm], y: %d[mm], a: %d[deg]\r\n", x, y, a);

    strategy_auto_position(x, y, a, color);
}

static void cmd_motors(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    motor_driver_t *motors;
    uint16_t len, i;
    motor_manager_get_list(&motor_manager, &motors, &len);
    chprintf(chp, "CAN_ID: NAME\n");
    for (i = 0; i < len; i++) {
        chprintf(chp, "   %3u: %s\n", motors[i].can_id, motors[i].id);
    }
}

static void cmd_motor_pos(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc < 2) {
        chprintf(chp, "Usage: motor_pos motor_name position\r\n");
        return;
    }
    float position = atof(argv[1]);
    chprintf(chp, "Setting motor %s position to %f\r\n", argv[0], position);
    motor_manager_set_position(&motor_manager, argv[0], position);
}

static void cmd_motor_voltage(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc < 2) {
        chprintf(chp, "Usage: motor_voltage motor_name voltage\r\n");
        return;
    }
    float voltage = atof(argv[1]);
    chprintf(chp, "Setting motor %s voltage to %f\r\n", argv[0], voltage);
    motor_manager_set_voltage(&motor_manager, argv[0], voltage);
}

static void cmd_motor_index(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc < 3) {
        chprintf(chp, "Usage: motor_index motor_name direction speed\r\n");
        return;
    }
    int motor_dir = atoi(argv[1]);
    float motor_speed = atof(argv[2]);

    motor_driver_t* motor = bus_enumerator_get_driver(motor_manager.bus_enumerator, argv[0]);
    if (motor == NULL) {
        chprintf(chp, "Motor %s doesn't exist\r\n", argv[0]);
        return;
    }

    chprintf(chp, "Searching for index of motor %s\r\n", argv[0]);

    float index = motor_auto_index(motor, motor_dir, motor_speed);
    chprintf(chp, "Average index is %.4f\r\n", index);
}

static void cmd_scara_goto(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 5) {
        chprintf(chp, "Usage: scara_goto frame x y z max_vel\r\n");
        return;
    }
    position_3d_t pos = {.x = atof(argv[1]), .y = atof(argv[2]), .z = atof(argv[3])};

    chprintf(chp, "Moving arm to %f %f %f heading 0 in %s frame\r\n", pos.x, pos.y, pos.z, argv[0]);

    scara_t* arm = &main_arm;
    float _max_vel = atof(argv[4]);
    velocity_3d_t max_vel = {.x=_max_vel, .y=_max_vel, .z=_max_vel};

    if (strcmp("robot", argv[0]) == 0) {
        scara_goto(arm, pos, COORDINATE_ROBOT, max_vel);
    } else if (strcmp("table", argv[0]) == 0) {
        scara_goto(arm, pos, COORDINATE_TABLE, max_vel);
    } else {
        scara_goto(arm, pos, COORDINATE_ARM, max_vel);
    }
}


static void cmd_scara_mode(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "Usage: scara_mode {joint,cartesian,free}\r\n");
        return;
    }

    if (strcmp("joint", argv[0]) == 0) {
        scara_control_mode_joint(&main_arm);
    } else if (strcmp("cartesian", argv[0]) == 0) {
        scara_control_mode_cartesian(&main_arm);
    } else {
        scara_control_mode_disabled(&main_arm);
    }
}

static void cmd_scara_angles(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    scara_joint_positions_t angles = scara_hw_read_joint_positions(&main_arm.hw_interface);

    chprintf(chp, "Arm joint shoulder: %f elbow: %f z: %f\r\n", angles.shoulder, angles.elbow, angles.z);
}


static void cmd_scara_mv(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 2) {
        chprintf(chp, "Usage: scara_mv x y\r\n");
        return;
    }
    scara_t* arm = &main_arm;

    scara_trajectory_t trajectory;

    position_3d_t pos = scara_position(arm, COORDINATE_TABLE);
    velocity_3d_t max_vel = {.x = 500.f, .y = 500.f, .z = 1000.f};
    scara_trajectory_init(&trajectory);
    scara_trajectory_append_point(&trajectory, pos, COORDINATE_TABLE, max_vel, arm->length);

    pos.x = atof(argv[0]);
    pos.y = atof(argv[1]);
    scara_trajectory_append_point(&trajectory, pos, COORDINATE_TABLE, max_vel, arm->length);
    scara_do_trajectory(arm, &trajectory);

    chprintf(chp, "Moving arm to %f %f %f in table frame\r\n", pos.x, pos.y, pos.z);
}

static void cmd_scara_z(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "Usage: scara_z z\r\n");
        return;
    }
    scara_t* arm = &main_arm;
    float z = atof(argv[0]);

    scara_move_z(arm, z, COORDINATE_ROBOT, 1);

    chprintf(chp, "Moving arm z to %f\r\n", z);
}

static void cmd_scara_pos(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "Usage: scara_pos frame\r\n");
        return;
    }

    scara_t* arm = &main_arm;

    position_3d_t pos;
    if (strcmp("robot", argv[0]) == 0) {
        pos = scara_position(arm, COORDINATE_ROBOT);
    } else if (strcmp("table", argv[0]) == 0) {
        pos = scara_position(arm, COORDINATE_TABLE);
    } else {
        pos = scara_position(arm, COORDINATE_ARM);
    }

    chprintf(chp, "Position of arm is %f %f %f in %s frame\r\n",
             pos.x, pos.y, pos.z, argv[0]);
}

static void cmd_scara_hold(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "Usage: scara_hold frame\r\n");
        return;
    }

    scara_t* arm = &main_arm;

    if (strcmp("robot", argv[0]) == 0) {
        scara_hold_position(arm, COORDINATE_ROBOT);
    } else if (strcmp("table", argv[0]) == 0) {
        scara_hold_position(arm, COORDINATE_TABLE);
    } else {
        scara_hold_position(arm, COORDINATE_ARM);
    }

    chprintf(chp, "Holding current arm position in %s frame\r\n", argv[0]);
}


static void cmd_scara_traj(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    chprintf(chp, "scara trajectory editor: press q or CTRL-D to quit\n");

    static char line[128];
    scara_t* arm = &main_arm;

    /* interactive command line */
    chprintf(chp, "enter trajectory points, press x to execute, q to abort and exit\n");
    chprintf(chp, "input:\n> coord x y z dt\n");

    scara_trajectory_t trajectory;
    scara_trajectory_init(&trajectory);
    unsigned i = 0;
    char* token = NULL;
    char* coord = NULL;
    long point[4]; // x, y, z, dt;
    scara_coordinate_t system = COORDINATE_ARM;
    const unsigned point_len = sizeof(point) / sizeof(long) + 1;

    while (true) {
        while (i < SCARA_TRAJ_MAX_NUM_FRAMES) {
            chprintf(chp, "> ");
            if (shellGetLine(&shell_cfg, line, sizeof(line), NULL) || line[0] == 'q') {
                chprintf(chp, "q or CTRL-D was pressed. Exiting...\n");
                return;
            } else if (strcmp("x", line) == 0) {
                chprintf(chp, "x pressed. Executing trajectory...\n");
                break;
            }

            unsigned j = 0;
            coord = strtok(line, " ");

            if (coord != NULL) {
                if (strcmp("robot", coord) == 0) {
                    system = COORDINATE_ROBOT;
                } else if (strcmp("table", coord) == 0) {
                    system = COORDINATE_TABLE;
                } else {
                    system = COORDINATE_ARM;
                }
                j++;
            }

            token = strtok(NULL, " ");
            while (j < point_len) {
                if (token != NULL) {
                    /* If the token was "_" we keep the coordinate from previous point. */
                    if (strcmp(token, "_") != 0) {
                        point[j - 1] = strtol(token, NULL, 10);
                    }

                    j++;
                    token = strtok(NULL, " ");
                } else {
                    break;
                }
            }

            if (j == point_len) {
                position_3d_t position = {.x = point[0], .y = point[1], .z = point[2]};
                velocity_3d_t max_vel = {.x = point[3], .y = point[3], .z = point[3]};
                scara_trajectory_append_point(&trajectory, position,
                        system, max_vel, arm->length);
                i++;

                chprintf(chp, "Point %d coord:%s x:%d y:%d z:%d max_vel:%d added successfully.\n",
                        i, coord, point[0], point[1], point[2], point[3]);
            }
        }
        scara_do_trajectory(arm, &trajectory);
    }
}

static void cmd_scara_pause(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    scara_t* arm = &main_arm;

    scara_pause(arm);

    chprintf(chp, "Arm paused trajectory\r\n");
}

static void cmd_scara_continue(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    scara_t* arm = &main_arm;

    scara_continue(arm);

    chprintf(chp, "Arm restoring from pause\r\n");
}

static void cmd_base_mode(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "Usage: base_mode {all,angle,distance,free}\r\n");
        return;
    }

    if (strcmp("all", argv[0]) == 0) {
        robot.mode = BOARD_MODE_ANGLE_DISTANCE;
    } else if (strcmp("angle", argv[0]) == 0) {
        robot.mode = BOARD_MODE_ANGLE_ONLY;
    } else if (strcmp("distance", argv[0]) == 0) {
        robot.mode = BOARD_MODE_DISTANCE_ONLY;
    } else {
        robot.mode = BOARD_MODE_FREE;
    }
}

static void cmd_state(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    chprintf(chp, "Current robot state:\r\n");

    chprintf(chp, "Position of robot is %d %d %d\r\n",
             position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));

    position_3d_t pos = scara_position(&main_arm, COORDINATE_TABLE);
    chprintf(chp, "Position of main arm is %.1f %.1f %.1f in table frame\r\n",
             pos.x, pos.y, pos.z);
}

static void cmd_hand(BaseSequentialStream *chp, int argc, char * argv[])
{
    if (argc != 1) {
        chprintf(chp, "Usage: hand on|off\r\n");
        return;
    }

    pump_state_t pump_state;
    if (strcmp("on", argv[0]) == 0) {
        pump_state = PUMP_ON;
    } else {
        pump_state = PUMP_OFF;
    }

    hand_set_pump(&main_hand, pump_state);
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

static void cmd_servo(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 2) {
        chprintf(chp, "Usage: servo N PULSE_MS\r\n");
        return;
    }
    unsigned int n = atoi(argv[0]);
    float pw = atof(argv[1]);

    pca9685_pwm_set_pulse_width(n, pw/1000);
}

static void cmd_lever(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 2) {
        chprintf(chp, "Usage: lever right|left deploy|retract|pickup|deposit\r\n");
        return;
    }

    lever_t* lever;
    if      (strcmp("right", argv[0]) == 0) { lever = &right_lever; }
    else if (strcmp("left", argv[0]) == 0)  { lever = &left_lever; }
    else                                    { chprintf(chp, "Invalid lever side %s", argv[0]); return; }

    se2_t robot_pose = base_get_robot_pose(&robot.pos);
    se2_t lever_offset;
    if (strcmp("right", argv[0]) == 0) {
        lever_offset = se2_create_xya(
            parameter_scalar_get(parameter_find(&master_config, "lever/right/offset_x")),
            parameter_scalar_get(parameter_find(&master_config, "lever/right/offset_y")),
            parameter_scalar_get(parameter_find(&master_config, "lever/right/offset_a")));
    } else {
        lever_offset = se2_create_xya(
            parameter_scalar_get(parameter_find(&master_config, "lever/left/offset_x")),
            parameter_scalar_get(parameter_find(&master_config, "lever/left/offset_y")),
            parameter_scalar_get(parameter_find(&master_config, "lever/left/offset_a")));
    }
    se2_t blocks_pose = se2_chain(robot_pose, lever_offset);

    if (strcmp("deploy", argv[1]) == 0)       { lever_deploy(lever); }
    else if (strcmp("retract", argv[1]) == 0) { lever_retract(lever); }
    else if (strcmp("pickup", argv[1]) == 0)  { lever_pickup(lever, robot_pose, blocks_pose); }
    else if (strcmp("deposit", argv[1]) == 0) { blocks_pose = lever_deposit(lever, robot_pose); }
    else                                      { chprintf(chp, "Invalid command: %s", argv[1]); }

    chprintf(chp, "Blocks at x:%f y:%f a:%f\r\n",
        blocks_pose.translation.x, blocks_pose.translation.y, blocks_pose.rotation.angle);
}

static void cmd_pick_cube(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 3) {
        chprintf(chp, "Usage: pick x y z_start\r\n");
        return;
    }
    point_t xy;
    xy.x = atof(argv[0]);
    xy.y = atof(argv[1]);
    float z_start = atof(argv[2]);

    chprintf(chp, "Picking cube at x:%f y:%f z:65(%f)\r\n", xy.x, xy.y, z_start);
    strat_pick_cube(xy, z_start);
}

static void cmd_deposit_cube(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 3) {
        chprintf(chp, "Usage: deposit x y num_cubes\r\n");
        return;
    }
    float x = atof(argv[0]);
    float y = atof(argv[1]);
    int num_cubes = atoi(argv[2]);

    chprintf(chp, "Depositing cube on tower x:%f y:%f holding %f cubes\r\n", x, y, num_cubes);
    strat_deposit_cube(x, y, num_cubes);
    chprintf(chp, "Tower now has %f cubes\r\n", num_cubes + 1);
}


static void cmd_push_y(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 4) {
        chprintf(chp, "Usage: push_y x y z y_push\r\n");
        return;
    }
    float x = atof(argv[0]);
    float y = atof(argv[1]);
    float z = atof(argv[2]);
    float y_push = atof(argv[3]);

    chprintf(chp, "Pushing from x:%f y:%f z:%f to x:%f y:%f z:%f \r\n", x, y, z, x, y_push, z);
    strat_push_switch_on(x, y, z, y_push);
}

#include "can/can_io_driver.h"

static void cmd_canio(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 3) {
        chprintf(chp, "Usage: canio name channel pwm\r\n");
        return;
    }

    int channel = atoi(argv[1]);
    float pwm = atof(argv[2]);
    can_io_set_pwm(argv[0], channel, pwm);
    chprintf(chp, "Set CAN-IO %s Channel %d PWM %f\r\n", argv[0], channel, pwm);
}

static void cmd_hand_dist(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    messagebus_topic_t *topic;
    float dist;

    topic = messagebus_find_topic_blocking(&bus, "/hand_distance");
    if (messagebus_topic_read(topic, &dist, sizeof(dist))) {
        chprintf(chp, "hand_distance: %f\r\n", dist);
    } else {
        chprintf(chp, "topic was never published\r\n");
    }
}

#include "arms/arm_trajectory_manager.h"

static void cmd_arm_bd(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 3) {
        chprintf(chp, "Usage: arm_bd xy|z error_threshold_mm error_count_threshold\r\n");
        return;
    }

    int error_threshold_mm = atoi(argv[1]);
    int error_count_threshold = atoi(argv[2]);

    if (strcmp("xy", argv[0]) == 0) {
        arm_traj_manager_set_blocking_detection_xy(&main_arm_traj_manager, error_threshold_mm, error_count_threshold);
        chprintf(chp, "Set %s Arm blocking detector to error_threshold_mm: %d error_count_threshold: %d\r\n", argv[0],
                 error_threshold_mm, error_count_threshold);
    } else if (strcmp("z", argv[0]) == 0) {
        arm_traj_manager_set_blocking_detection_z(&main_arm_traj_manager, error_threshold_mm, error_count_threshold);
        chprintf(chp, "Set %s Arm blocking detector to error_threshold_mm: %d error_count_threshold: %d\r\n", argv[0],
                 error_threshold_mm, error_count_threshold);
    } else {
        chprintf(chp, "Unknown Arm blocking detection %s only xy or z supported\r\n", argv[0]);
    }
}

const ShellCommand commands[] = {
    {"crashme", cmd_crashme},
    {"config_tree", cmd_config_tree},
    {"config_set", cmd_config_set},
    {"encoders", cmd_encoders},
    {"forward", cmd_traj_forward},
    {"hand", cmd_hand},
    {"ip", cmd_ip},
    {"node", cmd_node},
    {"pos", cmd_position},
    {"pos_reset", cmd_position_reset},
    {"reboot", cmd_reboot},
    {"rotate", cmd_traj_rotate},
    {"rpc_client_demo", cmd_rpc_client_test},
    {"threads", cmd_threads},
    {"time", cmd_time},
    {"topics", cmd_topics},
    {"pid", cmd_pid},
    {"pid_tune", cmd_pid_tune},
    {"goto", cmd_traj_goto},
    {"path", cmd_pathplanner},
    {"goto_avoid", cmd_goto_avoid},
    {"oa_dump", cmd_oa_dump},
    {"obs", cmd_create_static_obstacle},
    {"bdconf", cmd_blocking_detection_config},
    {"wheel_calib", cmd_wheel_calibration},
    {"wheel_corr", cmd_wheel_correction},
    {"track_calib", cmd_track_calibration},
    {"track_corr", cmd_track_correction},
    {"autopos", cmd_autopos},
    {"motor_pos", cmd_motor_pos},
    {"motor_voltage", cmd_motor_voltage},
    {"motor_index", cmd_motor_index},
    {"motors", cmd_motors},
    {"scara_angles", cmd_scara_angles},
    {"scara_mode", cmd_scara_mode},
    {"scara_goto", cmd_scara_goto},
    {"scara_mv", cmd_scara_mv},
    {"scara_z", cmd_scara_z},
    {"scara_pos", cmd_scara_pos},
    {"scara_hold", cmd_scara_hold},
    {"scara_traj", cmd_scara_traj},
    {"scara_pause", cmd_scara_pause},
    {"scara_continue", cmd_scara_continue},
    {"base_mode", cmd_base_mode},
    {"state", cmd_state},
    {"trace", cmd_trace},
    {"servo", cmd_servo},
    {"lever", cmd_lever},
    {"pick", cmd_pick_cube},
    {"deposit", cmd_deposit_cube},
    {"push_y", cmd_push_y},
    {"canio", cmd_canio},
    {"dist", cmd_hand_dist},
    {"arm_bd", cmd_arm_bd},
    {NULL, NULL}
};
