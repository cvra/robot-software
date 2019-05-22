#include <math.h>
#include <ch.h>

#include "main.h"
#include "MadgwickAHRS.h"
#include "ahrs_thread.h"
#include "imu_thread.h"
#include "board.h"

static struct {
    parameter_t beta;
    parameter_namespace_t ns;
} ahrs_params;

static void ahrs_thd(void* p)
{
    (void)p;
    chRegSetThreadName("AHRS");

    parameter_namespace_declare(&ahrs_params.ns, &parameter_root, "ahrs");
    parameter_scalar_declare_with_default(&ahrs_params.beta, &ahrs_params.ns, "beta", 0.1);

    messagebus_topic_t* imu_topic;

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

    // Prepare the filter
    madgwick_filter_t filter;
    madgwick_filter_init(&filter);
    madgwick_filter_set_gain(&filter, parameter_scalar_get(&ahrs_params.beta));

    // TODO: Measure sample frequency instead
    madgwick_filter_set_sample_frequency(&filter, 250);

    while (1) {
        if (parameter_changed(&ahrs_params.beta)) {
            madgwick_filter_set_gain(&filter, parameter_scalar_get(&ahrs_params.beta));
        }

        imu_msg_t imu;
        messagebus_topic_wait(imu_topic, &imu, sizeof(imu));


        attitude_msg_t msg;

        msg.q.w = filter.q[0];
        msg.q.x = filter.q[1];
        msg.q.y = filter.q[2];
        msg.q.z = filter.q[3];

        msg.timestamp = imu.timestamp;

        messagebus_topic_publish(&attitude_topic, &msg, sizeof(msg));
    }
}

void ahrs_start(void)
{
    static THD_WORKING_AREA(ahrs_thd_wa, 2048);
    chThdCreateStatic(ahrs_thd_wa, sizeof(ahrs_thd_wa),
                      NORMALPRIO, ahrs_thd, NULL);
}

void ahrs_calibrate_gyro(void)
{
    const int N = 1000;
    messagebus_topic_t* topic;
    parameter_t* gain;
    float x_avg = 0, y_avg = 0, z_avg = 0;
    float new_beta;

    topic = messagebus_find_topic_blocking(&bus, "/imu");

    board_led_set(BOARD_LED_ERROR);
    for (int i = 0; i < N; i++) {
        imu_msg_t msg;
        messagebus_topic_wait(topic, &msg, sizeof(msg));

        x_avg += msg.gyro.x / N;
        y_avg += msg.gyro.y / N;
        z_avg += msg.gyro.z / N;

        // Blink the LED during calibration
        if (i % 50 == 0) {
            board_led_toggle(BOARD_LED_ERROR);
        }
    }
    board_led_clear(BOARD_LED_ERROR);

    new_beta = sqrtf(3 / 4.f) * (x_avg + y_avg + z_avg) / 3.f;

    gain = parameter_find(&parameter_root, "/ahrs/beta");
    parameter_scalar_set(gain, new_beta);
}
