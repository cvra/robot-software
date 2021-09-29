#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <string.h>
#include <stdio.h>
#include <trace/trace.h>
#include <error/error.h>
#include <cmp/cmp.h>
#include <cmp_mem_access/cmp_mem_access.h>
#include <parameter_flash_storage/parameter_flash_storage.h>
#include <parameter/parameter_msgpack.h>

#include "main.h"
#include "usbconf.h"
#include "cmd.h"
#include "exti.h"
#include "imu_thread.h"
#include "ahrs_thread.h"
#include "./uavcan/uavcan_node.h"
#include "ranging_thread.h"
#include "state_estimation_thread.h"
#include "anchor_position_cache.h"
#include "log.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root;

static void blink_start(void);

/** Late init hook, called before c++ static constructors. */
void __late_init(void)
{
    /* C++ Static initializer requires working chibios. */
    halInit();
    chSysInit();

    trace_init();
    trace_enable();
}

static void _parameter_error(void* /*arg */, const char* /* id */, const char* /*err*/)
{
}

void config_load(void)
{
    /* First, we start with the default config, then we load board-specific
     * overlays (see board_config.yaml) */
    extern unsigned char msgpack_board_config[];
    extern const size_t msgpack_board_config_size;
    cmp_ctx_t cmp;
    cmp_mem_access_t mem;
    cmp_mem_access_ro_init(&cmp, &mem, msgpack_board_config,
                           msgpack_board_config_size);

    uint8_t uid[12];
    char* uid_device = (char*)UID_BASE;
    memcpy(uid, uid_device, sizeof(uid));

    char uid_as_str[25];
    memset(uid_as_str, 0, sizeof(uid_as_str));
    for (int i = 0; i < 12; i++) {
        sprintf(&uid_as_str[2 * i], "%02x", uid[i]);
    }

    uint32_t map_size;
    if (cmp_read_map(&cmp, &map_size)) {
        char key[64];
        for (uint32_t i = 0; i < map_size; i++) {
            memset(key, 0, sizeof(key));
            int res;
            uint32_t size = sizeof(key);
            res = cmp_read_str(&cmp, key, &size);
            if (!res) {
                // We encountered something not a string, skip the value
                // and continue
                cmp_skip_object_no_limit(&cmp);
                continue;
            }
            if (strncmp(key, uid_as_str, size)) {
                // Wrong key, skip the value
                cmp_skip_object_no_limit(&cmp);
                continue;
            }
            parameter_msgpack_read_cmp(&parameter_root, &cmp, _parameter_error, NULL);
        }
    }

    /* Then, load the saved config on top on the default config. */
    parameter_flash_storage_load(&parameter_root, &_config_start);
}

int main(void)
{
    /* Starts USB, this takes about 1 second, as we have to disconnect and
     * reconnect the device. */
    usb_start(42);
    log_init();

    NOTICE("boot");

    messagebus_init(&bus, &bus_lock, &bus_condvar);
    parameter_namespace_declare(&parameter_root, NULL, NULL);

    blink_start();
    exti_start();
    imu_start(); // disabled so that the EKF does not run
    ahrs_start();
    ranging_start();
    anchor_position_cache_start();
    state_estimation_start();

    uavcan_node_start(42, "uwb-beacon");

    shell_start((BaseSequentialStream*)&SDU1);

    /* All services should be initialized by now, we can load the config. */
    chThdSleepMilliseconds(1000);
    config_load();

    while (true) {
        chThdSleepMilliseconds(1000);
    }
}

static void blink_thd(void* p)
{
    (void)p;

    while (1) {
        board_led_toggle(BOARD_LED_STATUS);
        chThdSleepMilliseconds(200);
    }
}

static void blink_start(void)
{
    static THD_WORKING_AREA(blink_wa, 256);
    chThdCreateStatic(blink_wa, sizeof(blink_wa), LOWPRIO, blink_thd, NULL);
}
