#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <hal.h>
#include <chprintf.h>
#include <shell.h>

#include "main.h"
#include "imu_thread.h"

#define TEST_WA_SIZE        THD_WORKING_AREA_SIZE(256)
#define SHELL_WA_SIZE       THD_WORKING_AREA_SIZE(2048)

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


    chprintf(chp, "acc [m/s^2] :\t%.2f %.2f %.2f\r\n", msg.acc.x, msg.acc.y, msg.acc.z);
    chprintf(chp, "gyro [deg/s]:\t%.2f %.2f %.2f\r\n", msg.gyro.x, msg.gyro.y, msg.gyro.z);
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

    chprintf(chp, "IMU temperature: %d\r\n", (int)msg.temperature);
}

static ShellConfig shell_cfg;
const ShellCommand shell_commands[] = {
    {"reboot", cmd_reboot},
    {"topics", cmd_topics},
    {"imu", cmd_imu},
    {"temp", cmd_temp},
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
