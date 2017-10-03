#include <ch.h>
#include "main.h"
#include "MadgwickAHRS.h"
#include "ahrs_thread.h"
#include "imu_thread.h"


static void ahrs_thd(void *p)
{
    (void) p;
    chRegSetThreadName("AHRS");

    messagebus_topic_t *imu_topic;

    imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

    /* Create the attitude topic. */
    messagebus_topic_t attitude_topic;
    MUTEX_DECL(attitude_topic_lock);
    CONDVAR_DECL(attitude_topic_condvar);
    attitude_msg_t attitude_topic_content;

    messagebus_topic_init(&attitude_topic,
                          &attitude_topic_lock, &attitude_topic_condvar,
                          &attitude_topic_content, sizeof(attitude_topic_content));
    messagebus_advertise_topic(&bus, &attitude_topic, "/attitude");

    while (1) {
        imu_msg_t imu;
        messagebus_topic_wait(imu_topic, &imu, sizeof(imu));

        magdwick_filter_update(imu.gyro.x, imu.gyro.y, imu.gyro.z,
                               imu.acc.x, imu.acc.y, imu.acc.z,
                               imu.mag.x, imu.mag.y, imu.mag.z);

        attitude_msg_t msg;

        madgwick_filter_get_quaternion(&msg.q[0]);

        messagebus_topic_publish(&attitude_topic, &msg, sizeof(msg));
    }
}

void ahrs_start(void)
{
    static THD_WORKING_AREA(ahrs_thd_wa, 2048);
    chThdCreateStatic(ahrs_thd_wa, sizeof(ahrs_thd_wa),
                      NORMALPRIO, ahrs_thd, NULL);
}
