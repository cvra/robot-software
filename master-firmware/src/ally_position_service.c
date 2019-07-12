#include <lwip/api.h>
#include <msgbus_protobuf.h>
#include "protobuf/ally_position.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "base/base_controller.h"
#include "ally_position_service.h"

#include <parameter/parameter.h>
#include "main.h"
#include "control_panel.h"

#define ALLY_POSITION_PORT 3000

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

    static uint8_t object_buf[AllyPosition_size];

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

            netconn_sendto(conn, buf, &addr, ALLY_POSITION_PORT);
        }
        netbuf_delete(buf);

        /* Publish at 10 Hz */
        chThdSleepMilliseconds(100);
    }
}

TOPIC_DECL(ally_position_topic, AllyPosition);

static void position_receive_thread(void* p)
{
    (void)p;
    chRegSetThreadName(__FUNCTION__);

    struct netconn* conn;

    messagebus_advertise_topic(&bus, &ally_position_topic.topic, "/ally_pos");

    conn = netconn_new(NETCONN_UDP);
    chDbgAssert(conn != NULL, "Cannot create a connection object");
    netconn_bind(conn, IPADDR_ANY, ALLY_POSITION_PORT);

    while (true) {
        struct netbuf* buf;
        static uint8_t pos_buf[AllyPosition_size];
        AllyPosition pos;
        pb_istream_t istream;

        /* Read a datagram */
        if (netconn_recv(conn, &buf) != ERR_OK) {
            continue;
        }

        netbuf_copy(buf, &pos_buf, sizeof(pos_buf));

        istream = pb_istream_from_buffer(pos_buf, sizeof(pos_buf));
        pb_decode(&istream, AllyPosition_fields, &pos);

        control_panel_toggle(LED_READY);
        messagebus_topic_publish(&ally_position_topic.topic, &pos, sizeof(pos));

        netbuf_delete(buf);
    }
}

void ally_position_start(void)
{
    static THD_WORKING_AREA(send_wa, 2048);
    static THD_WORKING_AREA(receive_wa, 2048);
    parameter_namespace_t* ip_ns = parameter_namespace_find(&global_config, "/ip");
    parameter_string_declare(&ally_ip, ip_ns, "ally_address", ally_ip_buffer, sizeof(ally_ip_buffer));
    chThdCreateStatic(send_wa, sizeof(send_wa), NORMALPRIO, position_send_thread, NULL);
    chThdCreateStatic(receive_wa, sizeof(receive_wa), NORMALPRIO, position_receive_thread, NULL);
}
