#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <hal.h>
#include <chprintf.h>
#include <shell.h>

#include "main.h"
#include "imu_thread.h"
#include "ahrs_thread.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"

#define TEST_WA_SIZE  THD_WORKING_AREA_SIZE(256)
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

//#define DW1000_SPI_DUMP

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

BaseSequentialStream *output;

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength,
                uint8 *readBuffer)
{
    spiSelect(&SPID1);
    spiSend(&SPID1, headerLength, headerBuffer);
    spiReceive(&SPID1, readlength, readBuffer);
    spiUnselect(&SPID1);

#ifdef DW1000_SPI_DUMP
    chprintf(output, "%s -> ", __FUNCTION__);
    for (int i = 0; i < headerLength; i++) {
        chprintf(output, "0x%02x ", headerBuffer[i]);
    }
    chprintf(output, "| ");
    for (int i = 0; i < readlength; i++) {
        chprintf(output, "0x%02x ", readBuffer[i]);
    }
    chprintf(output, "\r\n");
#endif
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer,
               uint32 bodyLength, const uint8 *bodyBuffer)
{
#ifdef DW1000_SPI_DUMP
    chprintf(output, "%s -> ", __FUNCTION__);
    for (int i = 0; i < headerLength; i++) {
        chprintf(output, "0x%02x ", headerBuffer[i]);
    }
    chprintf(output, "| ");

    for (int i = 0; i < bodyLength; i++) {
        chprintf(output, "0x%02x ", bodyBuffer[i]);
    }
    chprintf(output, "\r\n");
#endif

    spiSelect(&SPID1);
    spiSend(&SPID1, headerLength, headerBuffer);
    spiSend(&SPID1, bodyLength, bodyBuffer);
    spiUnselect(&SPID1);
}

void deca_sleep(unsigned int time_ms)
{
    chThdSleepMilliseconds(time_ms);
}

decaIrqStatus_t decamutexon(void)
{
    return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
}

static void cmd_dwm(BaseSequentialStream *chp, int argc, char **argv)
{
    if (argc < 1) {
        chprintf(chp, "usage: dwm rx|tx\r\n");
        return;
    }

    /* Configuration example taken straight from decawave's example. */
    static dwt_config_t config = {
        2,               /* Channel number. */
        DWT_PRF_64M,     /* Pulse repetition frequency. */
        DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
        DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_110K,     /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    };

    static SPIConfig spi_cfg = {
        .end_cb = NULL,
        .ssport = GPIOA,
        .sspad = GPIOA_UWB_CS_N,
        .cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_0
    };
    spiStart(&SPID1, &spi_cfg);

    output = chp;
    uint32_t id = dwt_readdevid();
    chprintf(chp, "id=0x%x\r\n", id);

    chprintf(chp, "Configuring DW1000\r\n");
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        chprintf(chp, "INIT FAILED\r\n");
        return;
    }


    /* Configure GPIOs to show TX/RX activity. */
    dwt_setlnapamode(1, 1);
    dwt_setleds(DWT_LEDS_ENABLE);
    dwt_configure(&config);


    if (!strcmp(argv[0], "tx")) {
        /* DElay from end of transmission to activation of reception in
         * microseconds. */
        //dwt_setrxaftertxdelay(60);
        while (1) {
            chprintf(chp, "Sending packet\r\n");
            const char *tx_msg = "ping";
            dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

            /* Start transmission. */
            dwt_starttx(DWT_START_TX_IMMEDIATE);

            /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
             * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
             * function to access it.*/
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
                chThdSleepMilliseconds(100);
            }


            /* Clear TX frame sent event. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

            chprintf(chp, "done\r\n");
            chThdSleepMilliseconds(1000);
        }
    } else if (!strcmp(argv[0], "rx")) {
        while (1) {
            static char rx_buffer[100];
            memset(rx_buffer, 0, sizeof(rx_buffer));
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 4 below.
             * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
             * function to access it. */
            uint32_t status_reg, frame_len;
            do {
                status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                //chprintf(chp, "status reg:0x%x\r\n", status_reg);
            } while (!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)));

            /* Did we receive a valid frame? */
            if (status_reg & SYS_STATUS_RXFCG) {
                /* A frame has been received, copy it to our local buffer. */
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

                if (frame_len <= sizeof(rx_buffer)) {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                    chprintf(chp, "Received a frame of length %d \"%s\"\r\n", frame_len, rx_buffer);
                }
                /* Clear good RX frame event in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            } else {
                /* Clear RX error events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            }
        }
    }
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


static ShellConfig shell_cfg;
const ShellCommand shell_commands[] = {
    {"reboot", cmd_reboot},
    {"topics", cmd_topics},
    {"imu", cmd_imu},
    {"ahrs", cmd_ahrs},
    {"temp", cmd_temp},
    {"dwm", cmd_dwm},
    {"config_tree", cmd_config_tree},
    {"config_set", cmd_config_set},
    {NULL, NULL}
};

static THD_FUNCTION(shell_spawn_thd, p)
{
    BaseSequentialStream *io = (BaseSequentialStream *)p;
    thread_t *shelltp = NULL;

    shell_cfg.sc_channel = io;
    shell_cfg.sc_commands = shell_commands;

    shellInit();

#if 0
    const char arg[] = "tx";
    const char **argv[] = {arg};
    cmd_dwm(io, 1, argv);
#endif

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
