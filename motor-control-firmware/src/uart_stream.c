#include <string.h>
#include <ch.h>
#include <hal.h>
#include <cmp_mem_access/cmp_mem_access.h>
#include <serial-datagram/serial_datagram.h>
#include "encoder.h"
#include "control.h"

static void _stream_sndfn(void *arg, const void *p, size_t len)
{
    if (len > 0) {
        chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)p, len);
    }
}

THD_WORKING_AREA(uart_stream_task_wa, 256);
THD_FUNCTION(uart_stream_task, arg)
{
    BaseSequentialStream *dev = (BaseSequentialStream *)arg;
    chRegSetThreadName("print data");
    static char dtgrm[200];
    static cmp_mem_access_t mem;
    static cmp_ctx_t cmp;
    while (1) {
        cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
        bool err = false;
        err = err || !cmp_write_map(&cmp, 3);
        const char *enc_id = "enc";
        err = err || !cmp_write_str(&cmp, enc_id, strlen(enc_id));
        err = err || !cmp_write_u32(&cmp, encoder_get_primary());
        const char *pos_id = "pos";
        err = err || !cmp_write_str(&cmp, pos_id, strlen(pos_id));
        err = err || !cmp_write_float(&cmp, control_get_position());
        const char *vel_id = "vel";
        err = err || !cmp_write_str(&cmp, vel_id, strlen(vel_id));
        err = err || !cmp_write_float(&cmp, control_get_velocity());
        // const char *batt_voltage_id = "batt_voltage";
        // err = err || !cmp_write_str(&cmp, batt_voltage_id, strlen(batt_voltage_id));
        // err = err || !cmp_write_float(&cmp, analog_get_battery_voltage());
        //const char *velocity_id = "velocity";
        //err = err || !cmp_write_str(&cmp, velocity_id, strlen(velocity_id));
        //err = err || !cmp_write_float(&cmp, control_get_velocity());
        //const char *vel_ctrl_id = "vel_ctrl";
        //err = err || !cmp_write_str(&cmp, vel_ctrl_id, strlen(vel_ctrl_id));
        //err = err || !cmp_write_float(&cmp, control_get_vel_ctrl_out());
        if (!err) {
            serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_sndfn, dev);
        }
        chThdSleepMilliseconds(10);
    }
}

void uart_stream_start(BaseSequentialStream *dev)
{
    chThdCreateStatic(uart_stream_task_wa, sizeof(uart_stream_task_wa), LOWPRIO, uart_stream_task, dev);
}
