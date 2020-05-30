#include <lwip/api.h>

#include <aversive/trajectory_manager/trajectory_manager.h>
extern "C" {
#include <aversive/robot_system/robot_system.h>
#include <aversive/trajectory_manager/trajectory_manager_utils.h>
}

#include <string>
#include <fatfs/ff.h>
#include <lwip/netif.h>
#include <hal.h>
#include <chprintf.h>
#include <string.h>
#include <shell.h>
#include "config.h"
#include "commands.h"
#include "debug/panic_log.h"
#include "unix_timestamp.h"
#include <timestamp/timestamp.h>
#include "can/bus_enumerator.h"
#include "can/electron_starter.hpp"
#include "can/uavcan_node.h"
#include <stdio.h>
#include "msgbus_protobuf.h"
#include "main.h"
#include "base/rs_port.h"
#include "base/encoder.h"
#include "base/base_controller.h"
#include "base/base_helpers.h"
#include "base/map_server.h"
#include "robot_helpers/beacon_helpers.h"
#include "protobuf/beacons.pb.h"
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>
#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "robot_helpers/motor_helpers.h"
#include "strategy.h"
#include "strategy/goals.h"
#include "strategy/state.h"
#include "strategy/score_counter.h"
#include <trace/trace.h>
#include <error/error.h>
#include "pca9685_pwm.h"
#include "protobuf/sensors.pb.h"
#include "protobuf/encoders.pb.h"
#include "usbconf.h"
#include "shell_commands.h"

SHELL_COMMAND_END();

#if SHELL_USE_HISTORY == TRUE
static char sc_histbuf[SHELL_MAX_HIST_BUFF];
#endif

static ShellConfig shell_cfg = {
    NULL,
    __shell_commands_start,
#if SHELL_USE_HISTORY == TRUE
    sc_histbuf,
    sizeof(sc_histbuf),
#endif
};

void shell_spawn(BaseSequentialStream* stream)
{
    static THD_WORKING_AREA(shell_wa, 2048);
    static thread_t* shelltp = NULL;

    if (!shelltp) {
        shell_cfg.sc_channel = stream;
        shelltp = chThdCreateStatic(&shell_wa, sizeof(shell_wa), USB_SHELL_PRIO,
                                    shellThread, (void*)&shell_cfg);
        chRegSetThreadNameX(shelltp, "shell");
    } else if (chThdTerminatedX(shelltp)) {
        chThdRelease(shelltp); /* Recovers memory of the previous shell.   */
        shelltp = NULL; /* Triggers spawning of a new shell.        */
    }
}

SHELL_COMMAND(threads, chp, argc, argv)
{
    static const char* states[] = {CH_STATE_NAMES};
    thread_t* tp;

    (void)argv;
    if (argc > 0) {
        shellUsage(chp, "threads");
        return;
    }
    chprintf(chp,
             "stklimit    stack     addr refs prio     state       time         name\r\n" SHELL_NEWLINE_STR);
    tp = chRegFirstThread();
    do {
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
        uint32_t stklimit = (uint32_t)tp->wabase;
#else
        uint32_t stklimit = 0U;
#endif
        chprintf(chp, "%08lx %08lx %08lx %4lu %4lu %9s %10lu %12s" SHELL_NEWLINE_STR,
                 stklimit, (uint32_t)tp->ctx.sp, (uint32_t)tp,
                 (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
                 (uint32_t)tp->time, tp->name == NULL ? "" : tp->name);
        tp = chRegNextThread(tp);
    } while (tp != NULL);
}

SHELL_COMMAND(stack, chp, argc, argv)
{
#if (CH_DBG_FILL_THREADS != TRUE) || (CH_CFG_USE_REGISTRY != TRUE) || (CH_DBG_ENABLE_STACK_CHECK != TRUE)
#error "Requires: CH_DBG_FILL_THREADS CH_CFG_USE_REGISTRY CH_DBG_ENABLE_STACK_CHECK"
#endif

    (void)argv;
    (void)argc;

    const uint32_t STACK_FILL = 0x55555555;
    uint32_t p, sp, wabase;
    const char* name;
    thread_t* tp;

    chprintf(chp, "stackptr  stacktop  stklimit  free   name\n");

    tp = chRegFirstThread();
    while (tp) {
        sp = (uint32_t)tp->ctx.sp;
        wabase = (uint32_t)tp->wabase;
        name = tp->name == NULL ? "NULL" : tp->name;

        uint32_t limit = wabase + sizeof(thread_t);

        for (p = limit; p < sp; p += 4) {
            if (STACK_FILL != *(uint32_t*)p) {
                break;
            }
        }

        chprintf(chp, "%08lx  %08lx  %08lx  %5lu  %s\n", sp, p, limit, p - limit, name);

        tp = chRegNextThread(tp);
    }
}

SHELL_COMMAND(ip, chp, argc, argv)
{
    (void)argv;
    (void)argc;

    struct netif* n; /* used for iteration. */

    for (n = netif_list; n != NULL; n = n->next) {
        /* Converts the IP adress to a human readable format. */
        char ip[17], gw[17], nm[17];
        ipaddr_ntoa_r(&n->ip_addr, ip, 17);
        ipaddr_ntoa_r(&n->netmask, nm, 17);
        ipaddr_ntoa_r(&n->gw, gw, 17);

        chprintf(chp, "%s%d: %s, nm: %s, gw:%s\r\n", n->name, n->num, ip, nm, gw);
    }
}

SHELL_COMMAND(crashme, chp, argc, argv)
{
    (void)argv;
    (void)argc;
    (void)chp;

    // ERROR("You asked for it!, uptime=%d ms", TIME_I2MS(chVTGetSystemTime()));
}

SHELL_COMMAND(reboot, chp, argc, argv)
{
    (void)argv;
    (void)argc;
    (void)chp;
    NVIC_SystemReset();
}

SHELL_COMMAND(time, chp, argc, argv)
{
    (void)argv;
    (void)argc;

    unix_timestamp_t ts;
    int h, m;

    /* Get current time */
    int now = timestamp_get();
    ts = timestamp_local_us_to_unix(now);
    chprintf(chp, "Current scheduler tick:      %12ld\r\n", now);
    chprintf(chp, "Current UNIX timestamp:      %12ld\r\n", ts.s);
    chprintf(chp, "current ChibiOS time (ms):   %12ld\r\n", TIME_I2MS(chVTGetSystemTime()));
    chprintf(chp, "current timestamp time (us): %12ld\r\n", timestamp_get());

    /* Get time since start of day */
    ts.s = ts.s % (24 * 60 * 60);

    h = ts.s / 3600;
    ts.s = ts.s % 3600;

    m = ts.s / 60;
    ts.s = ts.s % 60;

    chprintf(chp, "Current time: %02d:%02d:%02d\r\n", h, m, ts.s);
}

static void tree_indent(BaseSequentialStream* out, int indent)
{
    int i;
    for (i = 0; i < indent; ++i) {
        chprintf(out, "  ");
    }
}

static void show_config_tree(BaseSequentialStream* out, parameter_namespace_t* ns, int indent)
{
    parameter_t* p;

    tree_indent(out, indent);
    chprintf(out, "%s\r\n", ns->id);

    for (p = ns->parameter_list; p != NULL; p = p->next) {
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

                case _PARAM_TYPE_BOOLEAN:
                    chprintf(out, "%s: %s\r\n", p->id, parameter_boolean_get(p) ? "true" : "false");
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

SHELL_COMMAND(config_tree, chp, argc, argv)
{
    parameter_namespace_t* ns;
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

SHELL_COMMAND(config_set, chp, argc, argv)
{
    parameter_t* param;
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

SHELL_COMMAND(node, chp, argc, argv)
{
    if (argc != 1) {
        chprintf(chp, "usage: node node_name.\r\n");
        chprintf(chp, "or node -a to show all nodes");
        return;
    }

    if (!strcmp(argv[0], "-a")) {
        for (int i = 0; i < 128; i++) {
            const char* s = bus_enumerator_get_str_id(&bus_enumerator, i);
            if (s) {
                chprintf(chp, "%02d: %s\n", i, s);
            }
        }
    } else {
        uint8_t id = bus_enumerator_get_can_id(&bus_enumerator, argv[0]);
        chprintf(chp, "Node ID: %s = %d.\r\n", argv[0], id);
    }
}

SHELL_COMMAND(topics, chp, argc, argv)
{
    (void)argc;
    (void)argv;

    chprintf(chp, "available topics:\r\n");

    MESSAGEBUS_TOPIC_FOREACH (&bus, topic) {
        messagebus_topic_stats_t stats;
        messagebus_topic_stats_get(topic, &stats);
        chprintf(chp, "%s:%d messages\r\n", topic->name, stats.messages);
    }
}

SHELL_COMMAND(encoders, chp, argc, argv)
{
    (void)argc;
    (void)argv;

    messagebus_topic_t* encoders_topic;
    WheelEncodersPulse values;

    encoders_topic = messagebus_find_topic_blocking(&bus, "/encoders");
    messagebus_topic_wait(encoders_topic, &values, sizeof(values));

    chprintf(chp, "left: %ld\r\nright: %ld\r\n", values.left, values.right);
}

/* position */
SHELL_COMMAND(pos, chp, argc, argv)
{
    (void)argc;
    (void)argv;

    float x, y, a;

    x = position_get_x_float(&robot.pos);
    y = position_get_y_float(&robot.pos);
    a = position_get_a_rad_float(&robot.pos);

    chprintf(chp, "x: %f [mm]\r\ny: %f [mm]\r\na: %f [rad]\r\n", x, y, a);
}

/* position_reset */
SHELL_COMMAND(pos_reset, chp, argc, argv)
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

/* ally_position */
SHELL_COMMAND(ally_pos, chp, argc, argv)
{
    (void)argc;
    (void)argv;
    point_t pos;
    messagebus_topic_t* topic;
    const char* topic_name = "/ally_pos";

    topic = messagebus_find_topic(&bus, topic_name);

    if (topic == NULL) {
        chprintf(chp, "Could not find topic %s\r\n", topic_name);
        return;
    }

    if (messagebus_topic_read(topic, &pos, sizeof(pos))) {
        chprintf(chp, "Allied robot position: %.3f %.3f\r\n", pos.x, pos.y);
    } else {
        chprintf(chp, "No data published on %s\r\n", topic_name);
    }
}

/* traj_forward */
SHELL_COMMAND(forward, chp, argc, argv)
{
    if (argc == 1) {
        robot.mode = BOARD_MODE_ANGLE_DISTANCE;

        int32_t distance;
        distance = atoi(argv[0]);
        trajectory_d_rel(&robot.traj, distance);
        int end_reason = trajectory_wait_for_end(
            TRAJ_END_GOAL_REACHED | TRAJ_END_COLLISION | TRAJ_END_OPPONENT_NEAR | TRAJ_END_ALLY_NEAR);
        trajectory_hardstop(&robot.traj);
        chprintf(chp, "End reason %d\r\n", end_reason);

        robot.mode = BOARD_MODE_ANGLE_DISTANCE;
    } else {
        chprintf(chp, "Usage: forward distance\r\n");
    }
}

SHELL_COMMAND(rotate, chp, argc, argv)
{
    if (argc == 1) {
        robot.mode = BOARD_MODE_ANGLE_DISTANCE;

        int32_t angle;
        angle = atoi(argv[0]);
        trajectory_a_rel(&robot.traj, angle);

        robot.mode = BOARD_MODE_ANGLE_DISTANCE;
    } else {
        chprintf(chp, "Usage: rotate angle\r\n");
    }
}

/* traj_goto */
SHELL_COMMAND(goto, chp, argc, argv)
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

SHELL_COMMAND(reset_timer, chp, argc, argv)
{
    (void)chp;
    (void)argc;
    (void)argv;
    trajectory_game_timer_reset();
}

SHELL_COMMAND(pid, chp, argc, argv)
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

SHELL_COMMAND(pid_tune, chp, argc, argv)
{
    (void)argc;
    (void)argv;
    chprintf(chp, "pid tuner: press q or CTRL-D to quit\n");

    long index;
    static char line[20];
    parameter_namespace_t* ns;
    motor_driver_t* motors;
    uint16_t len, i;

    /* list all motors */
    motor_manager_get_list(&motor_manager, &motors, &len);
    chprintf(chp, "id: name\n");
    for (i = 0; i < len; i++) {
        chprintf(chp, "%2u: %s\n", i + 1, motors[i].id);
    }

    const char* extra[] = {
        "master/aversive/control/angle",
        "master/aversive/control/distance",
    };
    const size_t extra_len = sizeof(extra) / sizeof(char*);
    for (i = 0; i < extra_len; i++) {
        chprintf(chp, "%2u: %s\n", len + i + 1, extra[i]);
    }

    while (1) {
        chprintf(chp, "choose [1-%u]: ", len + extra_len);
        if (shellGetLine(&shell_cfg, line, sizeof(line), NULL) || line[0] == 'q') {
            /* CTRL-D was pressed */
            return;
        }

        index = strtol(line, NULL, 10);
        if (index > 0 && (unsigned long)index <= len + extra_len) {
            break;
        }
        chprintf(chp, "invalid index\n");
    }

    /* index starting from 0 */
    index -= 1;

    if (index < len) {
        chprintf(chp, "tune %s\n", motors[index].id);
        chprintf(chp, "1: current\n2: velocity\n3: position\n> ");
        if (shellGetLine(&shell_cfg, line, sizeof(line), NULL) || line[0] == 'q') {
            /* q or CTRL-D was pressed */
            return;
        }
        ns = parameter_namespace_find(&motors[index].config.control, line);
    } else {
        chprintf(chp, "tune %s\n", extra[index - len]);
        ns = parameter_namespace_find(&global_config, extra[index - len]);
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
        char* p = strchr(line, ' ');
        if (p == NULL) {
            chprintf(chp, "invalid value\n");
            continue;
        }
        *p = '\0';
        float val = strtof(p + 1, NULL);

        /* shortcut for kp ki kd */
        std::string name = line;
        if (!strcmp(name.c_str(), "i")) {
            name = "ki";
        } else if (!strcmp(name.c_str(), "p")) {
            name = "kp";
        } else if (!strcmp(name.c_str(), "d")) {
            name = "kd";
        }

        parameter_t* param;
        param = parameter_find(ns, name.c_str());
        if (param == NULL) {
            chprintf(chp, "parameter not found\n");
            continue;
        }
        chprintf(chp, "%s = %f\n", name.c_str(), val);
        parameter_scalar_set(param, val);
    }
}

/* blocking_detection_config */
SHELL_COMMAND(bdconf, chp, argc, argv)
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

/* wheel_calibration */
SHELL_COMMAND(wheel_calib, chp, argc, argv)
{
    int count;
    if (argc < 1) {
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

    trajectory_d_rel(&robot.traj, -robot.calibration_direction * 100.);
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Start calibration sequence and do it N times */
    while (count--) {
        chprintf(chp, "%d left !\n", count);
        trajectory_d_rel(&robot.traj, -robot.calibration_direction * 800.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
        trajectory_a_rel(&robot.traj, 180.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
        trajectory_d_rel(&robot.traj, -robot.calibration_direction * 800.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
        trajectory_a_rel(&robot.traj, -180.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
    }

    //trajectory_d_rel(&robot.traj, robot.calibration_direction * 75.);
    //trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Take reference again at the wall */
    trajectory_set_mode_aligning(&robot.mode, &robot.traj, &robot.distance_bd, &robot.angle_bd);
    trajectory_align_with_wall();

    /* Compute correction factors */
    int32_t delta_angle = start_angle - rs_get_angle(&robot.rs);
    int32_t delta_distance = start_distance - rs_get_distance(&robot.rs);

    float factor = (float)(delta_angle) / (float)(delta_distance);
    float left_gain = (1. + factor) * robot.rs.left_ext_gain;
    float right_gain = (1. - factor) * robot.rs.right_ext_gain;

    /* Stop polar control */
    trajectory_d_rel(&robot.traj, -robot.calibration_direction * 75.);

    chprintf(chp, "Angle difference : %f\n", DEGREES(pos_imp2rd(&robot.traj, delta_angle)));
    chprintf(chp, "Suggested factors :\n");
    chprintf(chp, "Left : %.8f (old gain was %f)\n", left_gain, robot.rs.left_ext_gain);
    chprintf(chp, "Right : %.8f (old gain was %f)\n", right_gain, robot.rs.right_ext_gain);

    static char line[2];
    chprintf(chp, "Press y to apply, any other key to discard\r\n");
    if (shellGetLine(&shell_cfg, line, sizeof(line), NULL) || line[0] != 'y') {
        /* CTRL-D was pressed or any key that is not 'y' */
        return;
    }

    parameter_scalar_set(PARAMETER("master/odometry/left_wheel_correction_factor"), left_gain);
    parameter_scalar_set(PARAMETER("master/odometry/right_wheel_correction_factor"), right_gain);
    chprintf(chp, "New wheel correction factors set\r\n");
}

/* track_calibration */
SHELL_COMMAND(track_calib, chp, argc, argv)
{
    int count;
    if (argc < 1) {
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
    trajectory_d_rel(&robot.traj, -robot.calibration_direction * 200.);
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
    for (int i = 0; i < count; i++) {
        chprintf(chp, "%d left !\n", i);
        trajectory_a_rel(&robot.traj, 360.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
    }
    trajectory_d_rel(&robot.traj, robot.calibration_direction * 180.);
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Take reference at the wall */
    trajectory_set_mode_aligning(&robot.mode, &robot.traj, &robot.distance_bd, &robot.angle_bd);

    trajectory_align_with_wall();
    float end_angle = pos_imp2rd(&robot.traj, rs_get_angle(&robot.rs));

    /* Compute correction factors */
    float delta_angle = angle_delta(0., end_angle - start_angle);
    float track_calibrated = (float)robot.pos.phys.track_mm * (1 + (delta_angle / (2. * M_PI * (float)count)));

    trajectory_d_rel(&robot.traj, -robot.calibration_direction * 50);

    chprintf(chp, "Start angle %f, End angle : %f\n", DEGREES(start_angle), DEGREES(end_angle));
    chprintf(chp, "Angle difference : %f\n", DEGREES(delta_angle));
    chprintf(chp, "Suggested track : %.8f mm\n", track_calibrated);

    static char line[2];
    chprintf(chp, "Press y to apply, any other key to discard\r\n");
    if (shellGetLine(&shell_cfg, line, sizeof(line), NULL) || line[0] != 'y') {
        /* CTRL-D was pressed or any key that is not 'y' */
        return;
    }

    parameter_scalar_set(PARAMETER("master/odometry/external_track_mm"), track_calibrated);
    chprintf(chp, "New track set\r\n");
}

SHELL_COMMAND(autopos, chp, argc, argv)
{
    if (argc < 4) {
        chprintf(chp, "Usage: autopos {yellow|violet} x y a\r\n");
        return;
    }

    enum strat_color_t color = YELLOW;
    if (strcmp("violet", argv[0]) == 0) {
        color = VIOLET;
    } else if (strcmp("yellow", argv[0]) == 0) {
        color = YELLOW;
    } else {
        chprintf(chp, "Unknown color, please chose either yellow or violet\r\n");
        return;
    }

    int32_t x, y, a;
    x = atoi(argv[1]);
    y = atoi(argv[2]);
    a = atoi(argv[3]);
    chprintf(chp, "Positioning robot to x: %d[mm], y: %d[mm], a: %d[deg]\r\n", x, y, a);

    strategy_auto_position(x, y, a, color);
}

SHELL_COMMAND(motors, chp, argc, argv)
{
    (void)argc;
    (void)argv;
    motor_driver_t* motors;
    uint16_t len, i;
    motor_manager_get_list(&motor_manager, &motors, &len);
    chprintf(chp, "CAN_ID: NAME\n");
    for (i = 0; i < len; i++) {
        chprintf(chp, "   %3u: %s\n", motors[i].can_id, motors[i].id);
    }
}

SHELL_COMMAND(motor_pos, chp, argc, argv)
{
    if (argc < 2) {
        chprintf(chp, "Usage: motor_pos motor_name position\r\n");
        return;
    }
    float position = atof(argv[1]);
    chprintf(chp, "Setting motor %s position to %f\r\n", argv[0], position);
    motor_manager_set_position(&motor_manager, argv[0], position);
}

SHELL_COMMAND(motor_voltage, chp, argc, argv)
{
    if (argc < 2) {
        chprintf(chp, "Usage: motor_voltage motor_name voltage\r\n");
        return;
    }
    float voltage = atof(argv[1]);
    chprintf(chp, "Setting motor %s voltage to %f\r\n", argv[0], voltage);
    motor_manager_set_voltage(&motor_manager, argv[0], voltage);
}

SHELL_COMMAND(motor_index_sym, chp, argc, argv)
{
    if (argc < 3) {
        chprintf(chp, "Usage: motor_index_sym motor_name direction speed\r\n");
        return;
    }
    int motor_dir = atoi(argv[1]);
    float motor_speed = atof(argv[2]);

    motor_driver_t* motor = (motor_driver_t*)bus_enumerator_get_driver(motor_manager.bus_enumerator, argv[0]);
    if (motor == NULL) {
        chprintf(chp, "Motor %s doesn't exist\r\n", argv[0]);
        return;
    }

    chprintf(chp, "Searching for index of motor %s\r\n", argv[0]);

    float index = motor_auto_index_sym(motor, motor_dir, motor_speed);
    chprintf(chp, "Average index is %.4f\r\n", index);
}

SHELL_COMMAND(motor_index, chp, argc, argv)
{
    if (argc < 3) {
        chprintf(chp, "Usage: motor_index motor_name direction speed\r\n");
        return;
    }
    int motor_dir = atoi(argv[1]);
    float motor_speed = atof(argv[2]);

    chprintf(chp, "Searching for index of motor %s...\r\n", argv[0]);
    float index = motor_auto_index(argv[0], motor_dir, motor_speed);
    motor_manager_set_torque(&motor_manager, argv[0], 0);
    chprintf(chp, "Index at %.4f\r\n", index);
}

SHELL_COMMAND(base_mode, chp, argc, argv)
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

SHELL_COMMAND(state, chp, argc, argv)
{
    (void)argc;
    (void)argv;

    chprintf(chp, "Current robot state:\r\n");

    chprintf(chp, "Position of robot is %d %d %d\r\n",
             position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));
}

static void print_fn(void* arg, const char* fmt, ...)
{
    BaseSequentialStream* chp = (BaseSequentialStream*)arg;
    va_list ap;
    va_start(ap, fmt);
    chvprintf(chp, fmt, ap);
    va_end(ap);
}

SHELL_COMMAND(trace, chp, argc, argv)
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

SHELL_COMMAND(servo, chp, argc, argv)
{
    if (argc != 2) {
        chprintf(chp, "Usage: servo N PULSE_MS\r\n");
        return;
    }
    unsigned int n = atoi(argv[0]);
    float pw = atof(argv[1]);

    pca9685_pwm_set_pulse_width(n, pw / 1000);
}

#include "can/can_io_driver.h"

SHELL_COMMAND(canio, chp, argc, argv)
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

SHELL_COMMAND(motor_sin, chp, argc, argv)
{
    if (argc != 4) {
        chprintf(chp, "Usage: motor_sin motor amplitude period times\r\n");
        return;
    }

    float amplitude = atof(argv[1]);
    float period = atof(argv[2]);
    float dt = 0.02; // 50Hz update
    float dx = dt * 2 * M_PI / period;
    float times = atof(argv[3]);
    int num_points = period * times / dt;

    for (int i = 0; i <= num_points; i++) {
        float voltage = amplitude * sinf(i * dx);
        motor_manager_set_voltage(&motor_manager, argv[0], voltage);
        chprintf(chp, "%f\r\n", voltage);
        chThdSleepMilliseconds(dt * 1000);
    }
}

SHELL_COMMAND(speed, chp, argc, argv)
{
    if (argc != 1) {
        chprintf(chp, "Usage: speed init|slow|fast\r\n");
        return;
    }

    if (strcmp("init", argv[0]) == 0) {
        robot.base_speed = BASE_SPEED_INIT;
    } else if (strcmp("slow", argv[0]) == 0) {
        robot.base_speed = BASE_SPEED_SLOW;
    } else if (strcmp("fast", argv[0]) == 0) {
        robot.base_speed = BASE_SPEED_FAST;
    } else {
        chprintf(chp, "Invalid base speed: %s", argv[0]);
    }
}

/* panel_status */
SHELL_COMMAND(panel, chp, argc, argv)
{
    (void)argc;
    (void)argv;

    messagebus_topic_t* topic;
    topic = messagebus_find_topic(&bus, "/panel_contact_us");

    if (!topic) {
        chprintf(chp, "Could not find topic.\r\n");
        return;
    }

    uint32_t last_contact_time;
    if (messagebus_topic_read(topic, &last_contact_time, sizeof(last_contact_time))) {
        uint32_t current_time_us = timestamp_get();
        uint32_t delta = current_time_us - last_contact_time;
        chprintf(chp, "Last seen the panel %.2f seconds ago.\r\n", delta / 1e6);
    } else {
        chprintf(chp, "never seen that panel mate.\r\n");
    }
}

/* proximity_beacon */
SHELL_COMMAND(beacon, chp, argc, argv)
{
    (void)argc;
    (void)argv;

    BeaconSignal beacon_signal;
    messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");

    messagebus_topic_wait(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));
    chprintf(chp, "beacon signal: range: %4.1fmm %3.1fdeg\r\n",
             beacon_signal.range.range.distance * 1000.f,
             beacon_signal.range.angle * (180.f / 3.1415f));
}

SHELL_COMMAND(beacon_calib, chp, argc, argv)
{
    (void)argc;
    (void)argv;
    float r_meas;

    if (argc == 1) {
        r_meas = atof(argv[0]);
    } else {
        chprintf(chp, "Usage: beacon_calib d[mm]\r\n");
        return;
    }

    robot.mode = BOARD_MODE_FREE; // move robot by hand

    BeaconSignal beacon_signal;
    messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");

    float x, y, a, tx, ty;
    x = position_get_x_float(&robot.pos); // [mm]
    y = position_get_y_float(&robot.pos); // [mm]
    a = position_get_a_rad_float(&robot.pos); // [rad]

    messagebus_topic_wait(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));

    tx = x + r_meas * cosf(a + beacon_signal.range.angle);
    ty = y + r_meas * sinf(a + beacon_signal.range.angle);

    chprintf(chp, "Target position [mm]: %5.1f, %5.1f\n", tx, ty);

    chprintf(chp, "Distance [mm]\n");
    chprintf(chp, "odom, beacon\n");
    for (int i = 0; i < 100; i++) {
        float dx, dy, d, r;

        messagebus_topic_wait(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));

        dx = tx - position_get_x_float(&robot.pos);
        dy = ty - position_get_y_float(&robot.pos);
        d = sqrt(dx * dx + dy * dy);
        r = 1000 * beacon_signal.range.range.distance;

        chprintf(chp, "%5.1f, %5.1f\n", d, r);
        chThdSleepMilliseconds(50);
    }
}
