#include <lwip/api.h>
#include "msgbus_protobuf.h"
#include "protobuf/ally_position.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "base/base_controller.h"
#include "ally_position_service.h"

#include "parameter/parameter.h"
#include "main.h"

static parameter_t ally_ip;
char ally_ip_buffer[IP4ADDR_STRLEN_MAX];

static int ip_from_parameter(parameter_t* p, ip4_addr_t* addr)
{
    char buffer[IP4ADDR_STRLEN_MAX];
    parameter_string_get(p, buffer, sizeof(buffer));
    int success = ip4addr_aton(buffer, addr);
    if (success == 0) {
        WARNING("\"%s\" is not a valid IP.", buffer);
    }

    return success;
}

static void position_send_thread(void* p)
{
    (void)p;

    chRegSetThreadName(__FUNCTION__);

    struct netconn* conn;
    conn = netconn_new(NETCONN_UDP);

    uint8_t object_buf[AllyPosition_size];

    while (true) {
        AllyPosition pos;
        struct netbuf* buf;
        pb_ostream_t stream;

        pos.x = position_get_x_float(&robot.pos);
        pos.y = position_get_y_float(&robot.pos);
        pos.a = position_get_a_rad_float(&robot.pos);

        /* Encode the message as Protobuf and send it */
        stream = pb_ostream_from_buffer(object_buf, sizeof(object_buf));
        pb_encode(&stream, AllyPosition_fields, &pos);

        buf = netbuf_new();

        netbuf_ref(buf, object_buf, sizeof(object_buf));

        /* TODO: Take that from parameter tree */
        ip_addr_t addr;
        if (parameter_defined(&ally_ip)) {
            ip_from_parameter(&ally_ip, &addr);

            const int port = 3000;

            netconn_sendto(conn, buf, &addr, port);
        }
        netbuf_delete(buf);

        /* Publish at 10 Hz */
        chThdSleepMilliseconds(100);
    }
}

void ally_position_start(void)
{
    static THD_WORKING_AREA(send_wa, 2048);
    parameter_namespace_t* ip_ns = parameter_namespace_find(&global_config, "/ip");
    parameter_string_declare(&ally_ip, ip_ns, "ally_address", ally_ip_buffer, sizeof(ally_ip_buffer));
    chThdCreateStatic(send_wa, sizeof(send_wa), HIGHPRIO, position_send_thread, NULL);
}
