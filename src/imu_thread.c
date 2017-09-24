#include <ch.h>
#include <hal.h>

#include <msgbus/messagebus.h>

#include "imu_thread.h"
#include "exti.h"
#include "mpu9250.h"
#include "main.h"

#define IMU_INTERRUPT_EVENT (0x01)

static void imu_init_hardware(mpu9250_t *mpu)
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
        .cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
    };
    spiStart(&SPID2, &spi_cfg);

    mpu9250_init(mpu, &SPID2);

    mpu9250_reset(mpu);

    do {
        chThdSleepMilliseconds(100);
    } while (!mpu9250_ping(mpu));

    mpu9250_configure(mpu);
}

static void imu_reader_thd(void *p)
{
    (void) p;
    mpu9250_t mpu;

    /* Starts waiting for the external interrupt. */
    event_listener_t imu_int;
    chEvtRegisterMaskWithFlags(&exti_events, &imu_int,
                               (eventmask_t)IMU_INTERRUPT_EVENT,
                               (eventflags_t)EXTI_EVENT_IMU_INT);

    imu_init_hardware(&mpu);

    /* Creates the IMU topic */
    messagebus_topic_t imu_topic;
    MUTEX_DECL(imu_topic_lock);
    CONDVAR_DECL(imu_topic_condvar);
    imu_msg_t imu_topic_content;

    messagebus_topic_init(&imu_topic, &imu_topic_lock, &imu_topic_condvar,
                          &imu_topic_content, sizeof(imu_topic_content));
    messagebus_advertise_topic(&bus, &imu_topic, "/imu");

    while (1) {
        imu_msg_t msg;

        /* Wait for a measurement ready interrupt */
        chEvtWaitAny(IMU_INTERRUPT_EVENT);

        /* Read data from the IMU. */
        mpu9250_gyro_read(&mpu, &msg.gyro.x, &msg.gyro.y, &msg.gyro.z);
        mpu9250_acc_read(&mpu, &msg.acc.x, &msg.acc.y, &msg.acc.z);

        /* Signals the MPU that we are ready for another interrupt. */
        mpu9250_interrupt_read_and_clear(&mpu);

        /* Publish the data. */
        messagebus_topic_publish(&imu_topic, &msg, sizeof(msg));
    }
}

void imu_start(void)
{
    static THD_WORKING_AREA(imu_reader_thd_wa, 2048);
    chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa),
                      HIGHPRIO, imu_reader_thd, NULL);
}
