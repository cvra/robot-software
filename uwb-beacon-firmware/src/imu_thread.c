#include <ch.h>
#include <hal.h>

#include <msgbus/messagebus.h>

#include "imu_thread.h"
#include "exti.h"
#include "mpu9250.h"
#include "main.h"

#define IMU_INTERRUPT_EVENT (0x01)

static void imu_init_hardware(mpu9250_t* mpu)
{
    /*
     * SPI1 configuration structure for MPU9250.
     * SPI1 is on APB2 @ 84MHz / 128 = 656.25kHz
     * CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
     */
    static SPIConfig spi_cfg = {
        .end_cb = NULL,
        .ssport = GPIOB,
        .sspad = GPIOB_IMU_CS_N,
        .cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA};
    spiStart(&SPID2, &spi_cfg);

    mpu9250_init(mpu, &SPID2);

    mpu9250_reset(mpu);

    do {
        chThdSleepMilliseconds(100);
    } while (!mpu9250_ping(mpu));

    mpu9250_configure(mpu);
    mpu9250_enable_magnetometer(mpu);
}

static void imu_reader_thd(void* p)
{
    (void)p;
    mpu9250_t mpu;

    /* Starts waiting for the external interrupt. */
    event_listener_t imu_int;
    chEvtRegisterMask(&imu_event, &imu_int, (eventmask_t)IMU_INTERRUPT_EVENT);

    chRegSetThreadName("IMU");


    imu_init_hardware(&mpu);

    /* Creates the IMU topic */
    messagebus_topic_t imu_topic;
    MUTEX_DECL(imu_topic_lock);
    CONDVAR_DECL(imu_topic_condvar);
    imu_msg_t imu_topic_content;

    messagebus_topic_init(&imu_topic, &imu_topic_lock, &imu_topic_condvar,
                          &imu_topic_content, sizeof(imu_topic_content));
    messagebus_advertise_topic(&bus, &imu_topic, "/imu");

    /* Create the temperature topic. */
    messagebus_topic_t temperature_topic;
    MUTEX_DECL(temperature_topic_lock);
    CONDVAR_DECL(temperature_topic_condvar);
    temperature_msg_t temperature_topic_content;

    messagebus_topic_init(&temperature_topic,
                          &temperature_topic_lock, &temperature_topic_condvar,
                          &temperature_topic_content, sizeof(temperature_topic_content));
    messagebus_advertise_topic(&bus, &temperature_topic, "/imu/temperature");

    int temperature_pub_prescaler = 0;
    while (1) {
        imu_msg_t msg;

        /* Wait for a measurement ready interrupt */
        chEvtWaitAny(IMU_INTERRUPT_EVENT);

        /* Read data from the IMU. */
#if 0
        /* TODO: For some reason the macro ST2US creates an overflow. */
        uint32_t ts = ST2US(chVTGetSystemTime());
#else
        uint32_t ts = chVTGetSystemTime() * (1000000 / CH_CFG_ST_FREQUENCY);
#endif
        mpu9250_gyro_read(&mpu, &msg.gyro.x, &msg.gyro.y, &msg.gyro.z);
        mpu9250_acc_read(&mpu, &msg.acc.x, &msg.acc.y, &msg.acc.z);
        mpu9250_mag_read(&mpu, &msg.mag.x, &msg.mag.y, &msg.mag.z);

        msg.timestamp = ts;

        /* Publish the data. */
        messagebus_topic_publish(&imu_topic, &msg, sizeof(msg));

        /* Read the temperature, but not too often, as its useless. */
        if (temperature_pub_prescaler++ >= 100) {
            temperature_msg_t msg;
            msg.temperature = mpu9250_temp_read(&mpu);
            msg.timestamp = ts;
            messagebus_topic_publish(&temperature_topic, &msg, sizeof(msg));
            temperature_pub_prescaler = 0;
        }

        /* Signals the MPU that we are ready for another interrupt. */
        mpu9250_interrupt_read_and_clear(&mpu);
    }
}

void imu_start(void)
{
    static THD_WORKING_AREA(imu_reader_thd_wa, 2048);
    chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa),
                      HIGHPRIO, imu_reader_thd, NULL);
}
