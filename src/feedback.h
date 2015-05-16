/**
 * Feedback
 * ========
 *
 * This module multiplexes the measurement inputs according to the hardware
 * I/O configuration to deliver the right position and velocity information to
 * the control module.
 *
 * The inputs are the raw, unprocessed sensor values and the outputs are
 * position, velocity, and current.
 *
 */

#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum feedback_input_selection {
    FEEDBACK_RPM,
    FEEDBACK_PRIMARY_ENCODER_PERIODIC,
    FEEDBACK_PRIMARY_ENCODER_BOUNDED,
    FEEDBACK_TWO_ENCODERS_PERIODIC,
    FEEDBACK_POTENTIOMETER,
};


// note: enocders always overflow at 2**16
struct encoder_s {
    int32_t accumulator;        // accumulates encoder * p (except for bounded)
    uint16_t previous;          // previous input

    uint16_t transmission_p;    // transmission factor from motor to output
    uint16_t transmission_q;    // is p / q (i.e. working_end_pos = accumulator / q)

    uint32_t ticks_per_rev;     // one physical revolution of the encoder (datasheet)
};

struct potentiometer_s {
    float zero;
    float gain;     // pos = gain * input
};

struct rpm_s {
    float phase;    // angle in rad from zero to light barrier
};

struct feedback_s {
    enum feedback_input_selection input_selection;

    struct {
        float position; // between 0 and 2*PI for periodic actuators
        float velocity;
        bool actuator_is_periodic;
    } output;

    struct {
        float potentiometer;
        uint16_t primary_encoder;
        uint16_t secondary_encoder;
        float delta_t;
    } input;

    struct encoder_s primary_encoder;
    struct encoder_s secondary_encoder;

    struct potentiometer_s potentiometer;

    struct rpm_s rpm;
};


void feedback_compute(struct feedback_s *feedback);


#ifdef __cplusplus
}
#endif

#endif /* FEEDBACK_H */
