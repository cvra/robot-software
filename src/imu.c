#include <timestamp/timestamp.h>
#include <lwip/ip_addr.h>
#include <rpc_server.h>
#include <simplerpc/message.h>
#include <serial-datagram/serial_datagram.h>
#include <string.h>
#include <hal.h>
#include <ch.h>
#include "priorities.h"
#include "imu.h"

static void imu_publish(const void *data, size_t len, void *arg)
{
    (void)arg;
    timestamp_t t = timestamp_get();
    unix_timestamp_t now = timestamp_local_us_to_unix((int32_t)t);
    // stream
    ip_addr_t server;
    ODOMETRY_PUBLISHER_HOST(&server);

    static uint8_t buffer[100];
    static cmp_ctx_t ctx;
    static cmp_mem_access_t mem;
    message_write_header(&ctx, &mem, buffer, sizeof(buffer), "imu");
    cmp_write_array(&ctx, 2);
    cmp_write_array(&ctx, 2);
    cmp_write_sint(&ctx, now.s);
    cmp_write_sint(&ctx, now.us);
    size_t pos = cmp_mem_access_get_pos(&mem);
    if (len + pos > sizeof(buffer) || len == 0) {
        return;
    }
    memcpy(&buffer[pos], data, len);
    len = len + pos;
    message_transmit(buffer, len, &server, STREAM_PORT);
}

#define IMU_UART_BAUDRATE 921600
static const SerialConfig imu_serial_config = {
    .speed = IMU_UART_BAUDRATE,
    .cr1 = 0,
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    .cr3 = 0
};

void imu_thread_main(void *arg)
{
    (void)arg;
    chRegSetThreadName("imu");
    serial_datagram_rcv_handler_t rcv;
    static char datagram_buf[40];
    serial_datagram_rcv_handler_init(
        &rcv,
        datagram_buf,
        sizeof(datagram_buf),
        (serial_datagram_cb_t)imu_publish,
        NULL);
    while (1) {
        uint8_t buf[32];
        size_t n = sdReadTimeout(&SD6, buf, sizeof(buf), MS2ST(1));
        if (n > 0) {
            serial_datagram_receive(&rcv, buf, n);
        }
    }
}

void imu_init(void)
{
    sdStart(&SD6, &imu_serial_config);
    static THD_WORKING_AREA(imu_thread, 2048);
    chThdCreateStatic(imu_thread,
                      2048,
                      IMU_PRIO,
                      imu_thread_main,
                      NULL);
}
