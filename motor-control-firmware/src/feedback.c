#include "feedback.h"
#include <math.h>
#include <rpm.h>


static int32_t compute_delta_accumulator_periodic(uint16_t encoder,
                                                  uint16_t previous,
                                                  uint16_t p)
{
    return (int32_t)(int16_t)(encoder - previous) * p;
}

static int32_t compute_delta_accumulator_bounded(uint16_t encoder,
                                                  uint16_t previous)
{
    return (int32_t)(int16_t)(encoder - previous);
}

static void periodic_accumulator_overflow(int32_t *accumulator,
                                          uint32_t ticks_per_rev,
                                          uint16_t q)
{
    /* adjust accumulator so it overflows on every revolution of the
     * working end */
    if (*accumulator >= (int32_t)ticks_per_rev * q) {
        *accumulator -= ticks_per_rev * q;
    } else if (*accumulator < 0) {
        *accumulator += ticks_per_rev * q;
    }
}

static float compute_encoder_position_periodic(int32_t accumulator,
                                               uint32_t ticks_per_rev,
                                               uint16_t q)
{
    return (float)accumulator / ticks_per_rev / q * 2 * M_PI;
}

static float compute_encoder_position_bounded(int32_t accumulator,
                                              uint32_t ticks_per_rev,
                                              uint16_t p,
                                              uint16_t q)
{
    return (float)accumulator / ticks_per_rev * p / q * 2 * M_PI;
}

static float compute_encoder_velocity_periodic(int32_t delta_accumulator,
                                               uint32_t ticks_per_rev,
                                               uint16_t q,
                                               float delta_t)
{
    return (float)delta_accumulator / ticks_per_rev / q * 2 * M_PI / delta_t;
}

static float compute_encoder_velocity_bounded(int32_t delta_accumulator,
                                               uint32_t ticks_per_rev,
                                               uint16_t p,
                                               uint16_t q,
                                               float delta_t)
{
    return (float)delta_accumulator / ticks_per_rev * p / q * 2 * M_PI / delta_t;
}


void feedback_compute(struct feedback_s *feedback)
{
    switch (feedback->input_selection) {
        case FEEDBACK_RPM : {
            /* Call other module that updates the period on each interrupt
             * (light barrier crossing) and integrates the position assuming
             * constant velocity.
             * The period is assumed being constant as long as the time since
             * the last interrupt is less than the previous period.
             * When more time than the previous period has passed (i.e. the
             * actuator is decelerating), the speed is updated to the _maximal_
             * possible speed (1 / [time since last interrupt]) and the
             * position stops moving (SBB clock style).
             */
            float position;
            rpm_get_velocity_and_position(&feedback->output.velocity,
                                          &position);
            feedback->output.position = position - feedback->rpm.phase;
            feedback->output.actuator_is_periodic = true;
            break;
        }
        case FEEDBACK_PRIMARY_ENCODER_PERIODIC : {
            // accumulate
            int32_t delta_accumulator = compute_delta_accumulator_periodic(
                    feedback->input.primary_encoder,
                    feedback->primary_encoder.previous,
                    feedback->primary_encoder.transmission_p
                    );
            feedback->primary_encoder.accumulator += delta_accumulator;
            feedback->primary_encoder.previous = feedback->input.primary_encoder;

            periodic_accumulator_overflow(&feedback->primary_encoder.accumulator,
                                          feedback->primary_encoder.ticks_per_rev,
                                          feedback->primary_encoder.transmission_q);

            // position
            feedback->output.position = compute_encoder_position_periodic(
                    feedback->primary_encoder.accumulator,
                    feedback->primary_encoder.ticks_per_rev,
                    feedback->primary_encoder.transmission_q
                    );


            // velocity
            feedback->output.velocity = compute_encoder_velocity_periodic(
                    delta_accumulator,
                    feedback->primary_encoder.ticks_per_rev,
                    feedback->primary_encoder.transmission_q,
                    feedback->input.delta_t
                    );

            feedback->output.actuator_is_periodic = true;
            break;
         }
        case FEEDBACK_PRIMARY_ENCODER_BOUNDED : {
            // accumulate
            int32_t delta_accumulator = compute_delta_accumulator_bounded(
                    feedback->input.primary_encoder,
                    feedback->primary_encoder.previous
                    );
            feedback->primary_encoder.accumulator += delta_accumulator;
            feedback->primary_encoder.previous = feedback->input.primary_encoder;

            // position
            feedback->output.position = compute_encoder_position_bounded(
                    feedback->primary_encoder.accumulator,
                    feedback->primary_encoder.ticks_per_rev,
                    feedback->primary_encoder.transmission_p,
                    feedback->primary_encoder.transmission_q
                    );

            // velocity
            feedback->output.velocity = compute_encoder_velocity_bounded(
                    delta_accumulator,
                    feedback->primary_encoder.ticks_per_rev,
                    feedback->primary_encoder.transmission_p,
                    feedback->primary_encoder.transmission_q,
                    feedback->input.delta_t
                    );

            feedback->output.actuator_is_periodic = false;
            break;
        }
        case FEEDBACK_TWO_ENCODERS_PERIODIC : {
            // accumulate
            int32_t delta_accumulator_primary = compute_delta_accumulator_periodic(
                    feedback->input.primary_encoder,
                    feedback->primary_encoder.previous,
                    feedback->primary_encoder.transmission_p
                    );
            feedback->primary_encoder.accumulator += delta_accumulator_primary;
            feedback->primary_encoder.previous = feedback->input.primary_encoder;

            periodic_accumulator_overflow(&feedback->primary_encoder.accumulator,
                                          feedback->primary_encoder.ticks_per_rev,
                                          feedback->primary_encoder.transmission_q);

            int32_t delta_accumulator_secondary = compute_delta_accumulator_periodic(
                    feedback->input.secondary_encoder,
                    feedback->secondary_encoder.previous,
                    feedback->secondary_encoder.transmission_p
                    );
            feedback->secondary_encoder.accumulator += delta_accumulator_secondary;
            feedback->secondary_encoder.previous = feedback->input.secondary_encoder;

            periodic_accumulator_overflow(&feedback->secondary_encoder.accumulator,
                                          feedback->secondary_encoder.ticks_per_rev,
                                          feedback->secondary_encoder.transmission_q);

            // position
            feedback->output.position = compute_encoder_position_periodic(
                    feedback->secondary_encoder.accumulator,
                    feedback->secondary_encoder.ticks_per_rev,
                    feedback->secondary_encoder.transmission_q
                    );


            // velocity
            feedback->output.velocity = compute_encoder_velocity_periodic(
                    delta_accumulator_primary,
                    feedback->primary_encoder.ticks_per_rev,
                    feedback->primary_encoder.transmission_q,
                    feedback->input.delta_t
                    );

            feedback->output.actuator_is_periodic = true;
            break;
        }
        case FEEDBACK_POTENTIOMETER : {
            float position =
                feedback->potentiometer.gain * feedback->input.potentiometer
                - feedback->potentiometer.zero;

            feedback->output.velocity =
                (position - feedback->output.position)
                / feedback->input.delta_t;

            feedback->output.position = position;
            feedback->output.actuator_is_periodic = false;
            break;
        }
    }
}
