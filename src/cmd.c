#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <hal.h>
#include <chprintf.h>
#include <shell.h>

#include <parameter_flash_storage/parameter_flash_storage.h>

#include "main.h"
#include "imu_thread.h"
#include "ahrs_thread.h"
#include "ranging_thread.h"
#include "uwb_protocol.h"
#include "state_estimation_thread.h"
#include "anchor_position_cache.h"

#define TEST_WA_SIZE  THD_WORKING_AREA_SIZE(256)
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static void cmd_reboot(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) chp;
    (void) argc;
    (void) argv;
    NVIC_SystemReset();
}

static void cmd_topics(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    const char *usage = "usage:\r\n"
                        "topics list -- Lists all available topics.\r\n"
                        "topics hz topic_name -- Displays the rate of the"
                        "topic over a 5 second window.";

    if (argc < 1) {
        chprintf(chp, "%s\r\n", usage);
        return;
    }

    if (!strcmp(argv[0], "list")) {
        chprintf(chp, "available topics:\r\n");

        MESSAGEBUS_TOPIC_FOREACH(&bus, topic) {
            chprintf(chp, "%s\r\n", topic->name);
        }
    } else if (!strcmp(argv[0], "hz")) {
        if (argc != 2) {
            chprintf(chp, "%s\r\n", usage);
            return;
        }

        messagebus_topic_t *topic = messagebus_find_topic(&bus, argv[1]);
        if (topic == NULL) {
            chprintf(chp, "Cannot find topic \"%s\".\r\n", argv[1]);
            return;
        }

        chprintf(chp, "Waiting for publish for 5 seconds...\r\n");

        systime_t start = chVTGetSystemTime();
        unsigned int message_counter = 0;

        while (chVTGetSystemTime() < start + MS2ST(5000)) {
            chMtxLock(topic->lock);
            if (chCondWaitTimeout(topic->condvar, MS2ST(10)) != MSG_TIMEOUT) {
                message_counter ++;
                chMtxUnlock(topic->lock);
            }
        }

        if (message_counter == 0) {
            chprintf(chp, "No messages.\r\n");
        } else {
            chprintf(chp, "Average rate: %.2f Hz\r\n", message_counter / 5.);
        }
    } else {
        chprintf(chp, "%s\r\n", usage);
        return;
    }
}

static void cmd_imu(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;

    imu_msg_t msg;
    messagebus_topic_t *imu_topic;

    imu_topic = messagebus_find_topic(&bus, "/imu");

    if (imu_topic == NULL) {
        chprintf(chp, "Could not find IMU topic.\r\n");
        return;
    }

    if (!messagebus_topic_read(imu_topic, &msg, sizeof(msg))) {
        chprintf(chp, "No IMU data available.\r\n");
        return;
    }

    msg.gyro.x *= 180 / 3.14;
    msg.gyro.y *= 180 / 3.14;
    msg.gyro.z *= 180 / 3.14;


    chprintf(chp, "timestamp: %.3f s\r\n", msg.timestamp * 1e-6);
    chprintf(chp, "acc [m/s^2] :\t%.2f %.2f %.2f\r\n", msg.acc.x, msg.acc.y, msg.acc.z);
    chprintf(chp, "gyro [deg/s]:\t%.2f %.2f %.2f\r\n", msg.gyro.x, msg.gyro.y, msg.gyro.z);
    chprintf(chp, "mag [uT] :\t%.2f %.2f %.2f\r\n", msg.mag.x, msg.mag.y, msg.mag.z);
}

static void cmd_temp(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;

    temperature_msg_t msg;
    messagebus_topic_t *temperature_topic;

    temperature_topic = messagebus_find_topic(&bus, "/imu/temperature");

    if (temperature_topic == NULL) {
        chprintf(chp, "Could not find temperature topic.\r\n");
        return;
    }

    if (!messagebus_topic_read(temperature_topic, &msg, sizeof(msg))) {
        chprintf(chp, "No temperature data available.\r\n");
        return;
    }

    chprintf(chp, "timestamp: %.3f s\r\n", msg.timestamp * 1e-6);
    chprintf(chp, "IMU temperature: %d\r\n", (int)msg.temperature);
}

static void cmd_ahrs(BaseSequentialStream *chp, int argc, char **argv)
{
    if (argc < 1) {
        chprintf(chp, "usage: ahrs cal\r\n");
        return;
    }

    if (!strcmp(argv[0], "cal")) {
        chprintf(chp, "calibrating gyro, do not move the board...\r\n");
        ahrs_calibrate_gyro();
    }
}

static void cmd_range(BaseSequentialStream *chp, int argc, char **argv)
{
    const char *usage =  "usage: range rx\r\n";

    if (argc < 1) {
        chprintf(chp, usage);
        return;
    }

    if (!strcmp(argv[0], "rx")) {
        const char *topic_name = "/range";
        messagebus_topic_t *topic;
        range_msg_t msg;

        topic = messagebus_find_topic(&bus, topic_name);
        if (topic == NULL) {
            chprintf(chp, "could not find topic \"%s\"\r\n", topic_name);
            return;
        }

        messagebus_topic_wait(topic, &msg, sizeof(msg));
        chprintf(chp, "got a ToF to anchor 0x%x : %.3f\r\n", msg.anchor_addr, msg.range);
    } else {
        chprintf(chp, usage);
    }
}

static void cmd_anchors(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;

    anchor_position_msg_t *pos;
    uint16_t id;

    for (id = 0; id < 0xffff; id++) {
        pos = anchor_position_cache_get(id);
        if (pos) {
            chprintf(chp, "%d:\r\n", pos->anchor_addr);
            chprintf(chp, "  x: %.2f\r\n", pos->x);
            chprintf(chp, "  y: %.2f\r\n", pos->y);
            chprintf(chp, "  z: %.2f\r\n", pos->z);
        }
    }
}

static void cmd_state(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;

    const char *topic_name = "/ekf/state";
    messagebus_topic_t *topic;
    position_estimation_msg_t msg;

    topic = messagebus_find_topic(&bus, topic_name);

    if (topic == NULL) {
        chprintf(chp, "Cound not find topic \"%s\".\r\n", topic_name);
        return;
    }

    if (messagebus_topic_read(topic, &msg, sizeof(msg)) == false) {
        chprintf(chp, "No published data.\r\n");
        return;
    }

    chprintf(chp, "mu: (%.3f; %.3f)\r\n", msg.x, msg.y);
    chprintf(chp, "sigma: (%.3f; %.3f)\r\n", msg.variance_x, msg.variance_y);
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
    char string_buf[64];

    tree_indent(out, indent);
    chprintf(out, "%s:\r\n", ns->id);

    for (p = ns->parameter_list; p != NULL; p = p->next) {
        tree_indent(out, indent + 1);
        if (parameter_defined(p)) {
            switch (p->type) {
                case _PARAM_TYPE_SCALAR:
                    chprintf(out, "%s: %f\r\n", p->id, parameter_scalar_get(p));
                    break;

                case _PARAM_TYPE_INTEGER:
                    chprintf(out, "%s: %d\r\n", p->id, parameter_integer_get(p));
                    break;

                case _PARAM_TYPE_BOOLEAN:
                    chprintf(out, "%s: %s\r\n", p->id, parameter_boolean_get(p) ? "true" : "false");
                    break;

                case _PARAM_TYPE_STRING:
                    parameter_string_get(p, string_buf, sizeof(string_buf));
                    chprintf(out, "%s: %s\r\n", p->id, string_buf);
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
        ns = &parameter_root;
    } else {
        ns = parameter_namespace_find(&parameter_root, argv[0]);
        if (ns == NULL) {
            chprintf(chp, "Cannot find subtree.\r\n");
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

    param = parameter_find(&parameter_root, argv[0]);

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

static void cmd_config_erase(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;
    (void) chp;
    parameter_flash_storage_erase(&_config_start);
}

static void cmd_config_save(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;
    size_t len = (size_t)(&_config_end - &_config_start);
    bool success;

    // First write the config to flash
    parameter_flash_storage_save(&_config_start, len, &parameter_root);

    // Second try to read it back, see if we failed
    success = parameter_flash_storage_load(&parameter_root, &_config_start);

    if (success) {
        chprintf(chp, "OK.\r\n");
    } else {
        chprintf(chp, "Save failed.\r\n");
    }
}

static void cmd_config_load(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;
    bool success;

    success = parameter_flash_storage_load(&parameter_root, &_config_start);

    if (success) {
        chprintf(chp, "OK.\r\n");
    } else {
        chprintf(chp, "Load failed.\r\n");
    }
}


static ShellConfig shell_cfg;
const ShellCommand shell_commands[] = {
    {"reboot", cmd_reboot},
    {"topics", cmd_topics},
    {"imu", cmd_imu},
    {"ahrs", cmd_ahrs},
    {"temp", cmd_temp},
    {"range", cmd_range},
    {"anchors", cmd_anchors},
    {"state", cmd_state},
    {"config_tree", cmd_config_tree},
    {"config_set", cmd_config_set},
    {"config_save", cmd_config_save},
    {"config_load", cmd_config_load},
    {"config_erase", cmd_config_erase},
    {NULL, NULL}
};

static THD_FUNCTION(shell_spawn_thd, p)
{
    BaseSequentialStream *io = (BaseSequentialStream *)p;
    thread_t *shelltp = NULL;

    shell_cfg.sc_channel = io;
    shell_cfg.sc_commands = shell_commands;

    shellInit();

    while (TRUE) {
        if (!shelltp) {
            shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
        } else {
            if (chThdTerminatedX(shelltp)) {
                chThdRelease(shelltp);
                shelltp = NULL;
            }
        }
        chThdSleepMilliseconds(500);
    }
}

void shell_start(BaseSequentialStream *io)
{
    static THD_WORKING_AREA(wa, SHELL_WA_SIZE);
    chThdCreateStatic(wa, sizeof(wa), NORMALPRIO, shell_spawn_thd, io);
}
